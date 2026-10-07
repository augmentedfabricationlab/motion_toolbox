"""Controller path JSON: metres, xyzw quaternions and relative ROS timestamps.

This module supplies reusable functions. In Grasshopper, load
examples/grasshopper_export_paths.py instead; its header documents all component
inputs and outputs, including the printed export destination on out.
"""
from datetime import datetime
import json
import math
from pathlib import Path

import numpy as np

from motion_toolbox.geometry import as_plane
from motion_toolbox.recording import recorded, event, metric

NANOSECONDS = 1_000_000_000


def _quaternion(plane):
    """Plane axes are rotation-matrix columns; output order is xyzw."""
    m = plane.matrix[:3, :3]
    trace = np.trace(m)
    if trace > 0:
        s = math.sqrt(trace+1)*2
        q = [(m[2,1]-m[1,2])/s, (m[0,2]-m[2,0])/s, (m[1,0]-m[0,1])/s, s/4]
    else:
        i = int(np.argmax(np.diag(m)))
        j, k = (i+1)%3, (i+2)%3
        s = math.sqrt(max(0., 1+m[i,i]-m[j,j]-m[k,k]))*2
        q = np.zeros(4)
        q[i], q[j], q[k], q[3] = s/4, (m[j,i]+m[i,j])/s, (m[k,i]+m[i,k])/s, (m[k,j]-m[j,k])/s
    q = np.asarray(q, dtype=float)
    return q/np.linalg.norm(q)


def _stamp(ns):
    sec, nanosec = divmod(int(ns), NANOSECONDS)
    return dict(sec=sec, nanosec=nanosec)


def _payload(planes, stamps, frame_id):
    return dict(frame_id=frame_id, poses=[dict(frame_id=frame_id, stamp=_stamp(t),
        position=dict(zip(('x','y','z'), map(float,p.origin))),
        orientation=dict(zip(('x','y','z','w'), map(float,_quaternion(p)))))
        for p,t in zip(planes,stamps)])


@recorded
def build_path_export(tcp_planes, base_planes, speed, *, model_units_to_metres=1.,
                      frame_id='vicon_world', sanity_checks=True):
    """Preserve index pairing; both paths use timing from 3D TCP origin travel.

    speed is TCP speed in cm/s. This exports supplied poses without IK or
    collision checking and does not resample or change their orientations.
    """
    speed = float(speed)
    if not math.isfinite(speed) or speed <= 0:
        raise ValueError('speed must be finite and greater than zero, in cm/s')
    if not isinstance(frame_id, str) or not frame_id.strip():
        raise ValueError('frame_id must be a nonempty string')
    def convert(values, name):
        result = []
        for i,p in enumerate(values):
            if p is None or not getattr(p, 'IsValid', True):
                raise ValueError('{}[{}] is missing or invalid'.format(name,i))
            result.append(as_plane(p, model_units_to_metres))
        if not result: raise ValueError(name+' is empty')
        return result
    tcp, bases = convert(tcp_planes, 'tcp_planes'), convert(base_planes, 'base_planes')
    if len(tcp) != len(bases):
        raise ValueError('TCP and base lists must have the same length')
    elapsed, stamps = 0., [0]
    for a,b in zip(tcp,tcp[1:]):
        elapsed += float(np.linalg.norm(b.origin-a.origin))/(speed/100.)
        if not math.isfinite(elapsed): raise ValueError('Calculated timestamp is not finite')
        stamps.append(round(elapsed*NANOSECONDS))
    arm_data, base_data = _payload(tcp, stamps, frame_id), _payload(bases, stamps, frame_id)
    heights = np.array([p['position']['z'] for p in arm_data['poses']])
    xyz = np.array([[p['position'][axis] for axis in 'xyz'] for p in base_data['poses']])
    extent = float(np.linalg.norm(np.ptp(xyz[:,:2],axis=0)))
    if sanity_checks and heights.max() <= 1.:
        raise ValueError('Export blocked: no TCP plane is above 1.0 m; check geometry and metre units')
    if sanity_checks and extent < .9:
        raise ValueError('Export blocked: base XY extent is {:.6f} m; expected at least 0.9 m. '
                         'This measures travel between base origins, not the base footprint. '
                         'For stationary or intentionally short paths, set sanity_checks=False. '
                         'Use List access for both plane inputs.'.format(extent))
    diagnostics = []
    if heights.min() < 0 or heights.max() > 2.5:
        diagnostics.append('TCP heights extend outside 0-2.5 m: {:.6f} to {:.6f} m'.format(heights.min(),heights.max()))
    if np.max(abs(xyz[:,2])) > .01:
        diagnostics.append('Some base planes are more than 1 cm from Z=0')
    if any(t==previous for previous,t in zip(stamps,stamps[1:])):
        diagnostics.append('Repeated TCP origins produce equal timestamps; origin-distance timing does not allocate rotation-only time.')
    for payload in (arm_data, base_data):
        json.dumps(payload, allow_nan=False)
    event('export.timing', strategy='shared_tcp_distance', poses=len(tcp),
          duration_seconds=stamps[-1]/NANOSECONDS, speed_cm_s=speed,
          geometry_validated=False, diagnostics=diagnostics)
    metric('export.poses',len(tcp))
    return dict(arm_data=arm_data, base_data=base_data, base_planes=bases,
                timestamps=[_stamp(t) for t in stamps], diagnostics=diagnostics,
                timing_strategy='shared_tcp_distance', duration_seconds=stamps[-1]/NANOSECONDS,
                base_xy_extent_m=extent, tcp_height_range_m=[float(heights.min()),float(heights.max())])


@recorded
def export_robot_paths(tcp_planes, base_planes, speed, *, documents_folder=None, write_files=True, **options):
    """Write arm_path.json/base_path.json in a new yyMMdd_HHmm_robot_path folder."""
    result = build_path_export(tcp_planes, base_planes, speed, **options)
    result.update(tcp_file=None, base_file=None, export_folder=None)
    if not write_files: return result
    documents = Path(documents_folder) if documents_folder is not None else Path.home()/'Documents'
    texts = [json.dumps(result[k], indent=2, allow_nan=False) for k in ('arm_data','base_data')]
    documents.mkdir(parents=True, exist_ok=True)
    name = datetime.now().strftime('%y%m%d_%H%M')+'_robot_path'
    suffix = 0
    while True:
        folder = documents/(name if suffix == 0 else name+'_{:02d}'.format(suffix))
        try:
            folder.mkdir()
            break
        except FileExistsError:
            suffix += 1
    paths = [folder/'arm_path.json', folder/'base_path.json']
    try:
        for path,text in zip(paths,texts): path.write_text(text, encoding='utf-8')
    except Exception:
        # This invocation owns the newly-created folder; remove incomplete pairs.
        for path in paths:
            if path.exists(): path.unlink()
        folder.rmdir()
        raise
    result.update(tcp_file=str(paths[0]), base_file=str(paths[1]), export_folder=str(folder))
    event('export.files', arm_path=str(paths[0]), base_path=str(paths[1]), poses=len(result['timestamps']))
    return result
