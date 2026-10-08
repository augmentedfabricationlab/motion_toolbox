"""Rhino 8 Python 3: export paired arm/base paths to timestamped Documents folders.

Load this file (examples/grasshopper_export_paths.py) in Grasshopper's Python 3
component. The utilities/path_export.py module is a helper, not the component.
Create inputs with these exact names, access modes and type hints.
Optional inputs can be omitted or marked Optional and left unconnected.
Keep the built-in out output and connect a Panel to see the export destination.
Add any other named outputs below as needed; output type hints are not required.

Required inputs:
  tcp_planes             List, Plane: selected TCP planes (connect planned_tcp)
  base_planes            List, Plane: corresponding planned base planes, same
                         count and order as tcp_planes
  speed                  Item, float: positive TCP origin speed in cm/s
Optional:
  write_files            Item, bool: True; False previews timing without exporting
  model_units_to_metres  Item, float: 1 for metre inputs, 0.001 for millimetres
  frame_id               Item, str: vicon_world
  sanity_checks          Item, bool: True; require at least one TCP above 1 m and
                         base XY bounding-box diagonal >=0.9 m; False allows small
                         or stationary paths (Boolean Toggle or text True/False)
  documents_folder       Item, str: parent destination; default Windows Documents
  toolbox_src            Item, str: path to repository src directory, not a file;
                         default adjacent src when file-loaded, or installed package

Outputs:
  out                    Built-in text output: printed summary, folder, file paths,
                         warnings, or an error explaining why export failed
  tcp_file               Text: full arm_path.json path; None in preview/on error
  base_file              Text: full base_path.json path; None in preview/on error
  export_folder          Text: full new yyMMdd_HHmm_robot_path folder path (numbered
                         if needed); None in preview/on error
  time_seconds           Number list: shared relative timestamps in seconds, from zero
  duration_seconds       Number: total path duration in seconds
  diagnostics            Text list: readable geometry/unit warnings or errors
  status                 Text: export/preview summary, or error message
  result                 Object: full payload and diagnostics dictionary
  version                Text: loaded motion-toolbox package version

Every recompute with write_files=True creates a new folder. Both files always
use the same timestamps, derived from 3D TCP distances / speed. Base positions
and rotations are unchanged; base speed can differ from TCP speed. Positions are
assumed to be metres unless model_units_to_metres is supplied. Use List access
for both plane inputs to export the whole path together.
Files contain metre positions, normalized xyzw quaternions, frame_id and relative
integer sec/nanosec timestamps. Repeated TCP origins get no extra time, including
rotation-only changes; diagnostics and out report this. This component does not
check IK, collisions or motion limits and does not communicate with robots.
"""
import importlib
from pathlib import Path
import sys


def _input(name, default=None):
    value = globals().get(name)
    return default if value is None else value


def _bool_input(name, default):
    value = _input(name, default)
    if isinstance(value, str):
        text = value.strip().lower()
        if text in ('true', 'false'):
            return text == 'true'
        raise ValueError('{} must be True or False; use an Item/bool input'.format(name))
    return bool(value)


tcp_file, base_file, export_folder = None, None, None
time_seconds, diagnostics = [], []
duration_seconds, result = None, None
status, version = '', '0.1.62'
try:
    source = _input('toolbox_src')
    script_file = globals().get('__file__')
    if source is None and script_file:
        candidate = Path(script_file).resolve().parent.parent/'src'
        if (candidate/'motion_toolbox'/'__init__.py').is_file():
            source = candidate
    if source is not None:
        source = str(source)
        if source in sys.path: sys.path.remove(source)
        sys.path.insert(0, source)
    importlib.invalidate_caches()
    import motion_toolbox
    importlib.reload(motion_toolbox)
    version = motion_toolbox.__version__
    import motion_toolbox.recording as recording
    if recording.current_run() is None and getattr(recording,'RECORDING_VERSION',0)<8:
        importlib.reload(recording)
    module = importlib.import_module('motion_toolbox.utilities.path_export')
    importlib.reload(module)
    speed_value = _input('speed')
    if speed_value is None: raise ValueError('Provide speed in cm/s')
    write = _bool_input('write_files', True)
    folder = _input('documents_folder')
    if write and folder is None:
        import System
        folder = System.Environment.GetFolderPath(System.Environment.SpecialFolder.MyDocuments)
        if not folder: raise ValueError('Could not locate Windows Documents')
    result = module.export_robot_paths(list(_input('tcp_planes', [])), list(_input('base_planes', [])),
        speed_value, documents_folder=folder, write_files=write,
        model_units_to_metres=float(_input('model_units_to_metres', 1.)),
        frame_id=_input('frame_id', 'vicon_world'), sanity_checks=_bool_input('sanity_checks', True))
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

# Grasshopper's built-in out parameter captures print(), not the status variable.
print(status)
if tcp_file is not None:
    print('Arm path: '+tcp_file)
    print('Base path: '+base_file)
elif result is not None:
    print('Preview only: no files written.')
for message in diagnostics:
    if message != status:
        print('WARNING: '+str(message))
