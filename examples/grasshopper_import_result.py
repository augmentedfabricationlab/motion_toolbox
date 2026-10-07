"""Rhino 8 Python 3: import an offline stationary benchmark result.json.

Load this file into a Grasshopper Python 3 component. Recomputes read the file
again; no search, collision checking, or robot motion occurs. Requires the
motion-toolbox COMPAS extra, but no robot input or collision backend.

Required input:
  result_path      Item, str: path to the stationary benchmark result.json
Optional inputs (mark Optional or remove):
  case_folder      Item, str: moved capture folder (or case.json); otherwise use
                   the result's case path. Relative paths resolve beside result.json.
  units_to_metres  Item, float (1): output model units; 0.001 for millimetres
  toolbox_src      Item, str: repository src directory, not a file

Outputs (add with these exact names; no type hints needed):
  base_plane       Single Rhino footprint plane
  base_planes      Same footprint repeated once per target, for path export
  planned_tcp      Rhino TCP planes with the saved selected TCP-Z rotations
  joint_plan       DataTree: one branch per target, six arm angles in radians
  configurations   Named COMPAS Configurations: captured fixed joints plus arm;
                   prismatic values stay in metres, regardless of model units
  path_cost        Saved joint-path cost
  path_complete    True only for a complete, structurally consistent saved path
  valid            Saved collision, joint-limit, joint-step and FK checks passed;
                   does not validate the current Rhino scene or swept transitions
  verification     JSON of saved checks; missing flags remain unknown
  diagnostics      Import/validation notes
  status           Readable summary or error (also printed to built-in out)
  result           Restored numeric data with the original JSON in source_result
  version          Loaded package version

Failed saved searches emit no trajectory. File/metadata errors clear all outputs.
Only stationary benchmark output is supported, including adaptive and heuristic
search. For grasshopper_export_paths.py, wire planned_tcp -> tcp_planes and
base_planes -> base_planes (List access); use sanity_checks=False for stationary
paths and the same model_units_to_metres as this component's units_to_metres.
"""
import importlib
import json
import math
from pathlib import Path
import sys


def _input(name, default=None):
    value = globals().get(name)
    return default if value is None else value


base_plane, joint_plan, path_cost, result = None, None, None, None
base_planes, planned_tcp, configurations, diagnostics = [], [], [], []
path_complete, valid = False, False
verification, status, version = None, '', None
try:
    source = _input('toolbox_src')
    if source is None and globals().get('__file__'):
        adjacent = Path(__file__).resolve().parent.parent / 'src'
        if (adjacent / 'motion_toolbox' / '__init__.py').is_file():
            source = adjacent
    if source:
        source = Path(str(source).strip().strip('"')).expanduser().resolve()
        if not (source / 'motion_toolbox' / '__init__.py').is_file():
            raise ValueError('toolbox_src must be the src directory containing motion_toolbox')
        loaded = sys.modules.get('motion_toolbox')
        if loaded is not None and Path(loaded.__file__).resolve().parent != source / 'motion_toolbox':
            raise RuntimeError('A different toolbox is cached; restart Rhino with the desired toolbox_src')
        if str(source) in sys.path:
            sys.path.remove(str(source))
        sys.path.insert(0, str(source))
    importlib.invalidate_caches()
    import motion_toolbox
    importlib.reload(motion_toolbox)
    version = motion_toolbox.__version__
    module = importlib.import_module('motion_toolbox.utilities.result_import')
    importlib.reload(module)
    from motion_toolbox.geometry import to_rhino
    from Grasshopper import DataTree
    from Grasshopper.Kernel.Data import GH_Path
    path = _input('result_path')
    if not path:
        raise ValueError('Provide result_path: the stationary run result.json')
    scale = float(_input('units_to_metres', 1.0))
    if not math.isfinite(scale) or scale <= 0:
        raise ValueError('units_to_metres must be positive and finite')
    result = module.load_stationary_result(path, case_folder=_input('case_folder'))
    if result['path_complete']:
        base_plane = to_rhino(result['base_plane'], 1 / scale)
        base_planes = [to_rhino(p, 1 / scale) for p in result['base_planes']]
        planned_tcp = [to_rhino(p, 1 / scale) for p in result['selected_target_planes']]
        configurations = result['configuration_objects']
        joint_plan = DataTree[float]()
        for i, q in enumerate(result['configurations']):
            for value in q:
                joint_plan.Add(float(value), GH_Path(i))
        path_cost = result['cost']
    path_complete, valid = result['path_complete'], result['valid']
    verification = json.dumps(result['verification'], sort_keys=True)
    diagnostics, status = result['diagnostics'], result['status']
except (Exception, KeyboardInterrupt) as error:
    base_plane, joint_plan, path_cost, result = None, None, None, None
    base_planes, planned_tcp, configurations = [], [], []
    path_complete, valid, verification = False, False, None
    status = '{}: {}'.format(type(error).__name__, error)
    diagnostics = [status]
print(status)
if not path_complete and 'ghenv' in globals():
    from Grasshopper.Kernel import GH_RuntimeMessageLevel
    ghenv.Component.AddRuntimeMessage(GH_RuntimeMessageLevel.Warning, status)
