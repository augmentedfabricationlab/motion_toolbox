"""Rhino 8 Python 3: export paired arm/base paths to timestamped Documents folders.

Inputs:
  tcp_planes             List, Plane: selected TCP planes (connect planned_tcp)
  base_planes            List, Plane: corresponding planned base planes
  speed                  Item, float: TCP origin speed in cm/s
Optional:
  write_files            Item, bool: True; False previews timing without exporting
  model_units_to_metres  Item, float: 1 for metre inputs, 0.001 for millimetres
  frame_id               Item, str: vicon_world
  sanity_checks          Item, bool: True; preserve supplied exporter's height >1 m
                         and base XY extent >=0.9 m checks; False permits small paths
  documents_folder       Item, str: override Windows Documents destination
  toolbox_src            Item, str: source override; defaults to this repo's src

Outputs:
  tcp_file, base_file     Saved arm_path.json / base_path.json, or None in preview
  export_folder          New yyMMdd_HHmm_robot_path folder; numbered on collision
  time_seconds           Shared relative timestamps, one per corresponding pose
  duration_seconds       Total path duration
  diagnostics            Readable geometry/unit warnings
  status, result, version Export summary, full payload/diagnostics and package version

Every recompute with write_files=True creates a new folder. Both files always
use the same timestamps, derived from 3D TCP distances / speed. Base positions
and rotations are unchanged; base speed can differ from TCP speed. This component
does not perform IK or collision validation and does not communicate with robots.
"""
import importlib
from pathlib import Path
import sys


def _input(name, default=None):
    value = globals().get(name)
    return default if value is None else value


tcp_file, base_file, export_folder = None, None, None
time_seconds, diagnostics = [], []
duration_seconds, result = None, None
status, version = '', '0.1.43'
try:
    source = str(_input('toolbox_src', Path(__file__).resolve().parents[1]/'src'))
    if source in sys.path: sys.path.remove(source)
    sys.path.insert(0, source)
    importlib.invalidate_caches()
    import motion_toolbox
    importlib.reload(motion_toolbox)
    import motion_toolbox.recording as recording
    if recording.current_run() is None and getattr(recording,'RECORDING_VERSION',0)<8:
        importlib.reload(recording)
    module = importlib.import_module('motion_toolbox.utilities.path_export')
    importlib.reload(module)
    speed_value = _input('speed')
    if speed_value is None: raise ValueError('Provide speed in cm/s')
    write = bool(_input('write_files', True))
    folder = _input('documents_folder')
    if write and folder is None:
        import System
        folder = System.Environment.GetFolderPath(System.Environment.SpecialFolder.MyDocuments)
        if not folder: raise ValueError('Could not locate Windows Documents')
    result = module.export_robot_paths(list(_input('tcp_planes', [])), list(_input('base_planes', [])),
        speed_value, documents_folder=folder, write_files=write,
        model_units_to_metres=float(_input('model_units_to_metres', 1.)),
        frame_id=_input('frame_id', 'vicon_world'), sanity_checks=bool(_input('sanity_checks', True)))
    tcp_file, base_file, export_folder = (result[k] for k in ('tcp_file','base_file','export_folder'))
    time_seconds = [t['sec']+t['nanosec']/1e9 for t in result['timestamps']]
    duration_seconds, diagnostics = result['duration_seconds'], result['diagnostics']
    version = motion_toolbox.__version__
    status = '{} {} paired poses; {:.3f} s; arm/base timestamps identical.'.format(
        'Exported' if write else 'Previewed', len(time_seconds), duration_seconds)
    if write: status += ' Saved to: '+export_folder
except Exception as error:
    tcp_file, base_file, export_folder = None, None, None
    time_seconds, duration_seconds, result = [], None, None
    status = 'Error: '+str(error)
    diagnostics = [status]
