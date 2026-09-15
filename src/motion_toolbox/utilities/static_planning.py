"""Edit -> collision filter -> exact shortest path, from Python or a JSON job."""
from motion_toolbox.recording import recorded, event
import argparse
from contextlib import ExitStack
import json
from pathlib import Path
from functools import partial
from ..geometry import Plane, as_plane
from ..graph import shortest_path
from ..planning import calculate_partial_trajectory, normalize_joint_ranges, _in_ranges
from .edit import edit_solutions
from .io import load_planes


@recorded
def run_all(solutions, current_pose, *, base_planes=None, collision=None,
            tolerance=None, snap_periodic=None, periodic=None, max_joint_step=2.5,
            weights=None, transition_check=None, joint_ranges=None, start_base=None, revolute_joints=None):
    """Run the retained static workflow on explicitly supplied candidate layers."""
    layers = edit_solutions(solutions, current_pose, tolerance=tolerance,
                           periodic=([False]*len(current_pose) if snap_periodic is None else snap_periodic))
    ranges = normalize_joint_ranges(joint_ranges, len(current_pose))
    layers = [[q for q in layer if _in_ranges(q, ranges)] for layer in layers]
    if base_planes is None:
        bases = [Plane.world_xy()]*len(layers)
    elif len(base_planes) == 1:
        bases = [as_plane(base_planes[0])]*len(layers)
    elif len(base_planes) == len(layers):
        bases = [as_plane(b) for b in base_planes]
    else:
        raise ValueError('One base or one base per candidate layer required')
    if collision is not None:
        layers = [[q for q in layer if collision(q, base)] for layer, base in zip(layers, bases)]
    def edge(i, a, b):
        if transition_check is None:
            return True
        q0 = current_pose if i == 0 else layers[i-1][a]
        b0 = (as_plane(start_base) if start_base is not None else bases[0]) if i == 0 else bases[i-1]
        return transition_check(q0, b0, layers[i][b], bases[i])
    result = shortest_path(layers, start=current_pose, max_step=max_joint_step,
                           count_paths=False, revolute_joints=revolute_joints,
                           weights=weights, periodic=periodic, edge_valid=edge if transition_check else None)
    return dict(configurations=result.configurations, path_length=result.cost,
                ik_solutions_per_node=layers, unreachable_points=[i for i, q in enumerate(layers) if not q])


@recorded
def run_job(job, *, root=None):
    """JSON job paths resolve relative to the job file, not the process cwd.

    Provide either solutions or targets, current_pose, optional bases, tool,
    arm_in_base, and collision={urdf, joint_names, environment, ground_z,...}.
    Output is returned; writing is explicit in the CLI.
    """
    root = Path.cwd() if root is None else Path(root)
    def path(value):
        return (root / value).resolve()
    def data(value):
        if isinstance(value, str):
            event('job.input_file', path=path(value))
        return json.loads(path(value).read_text(encoding='utf-8')) if isinstance(value, str) else value
    def planes(value):
        return load_planes(path(value)) if isinstance(value, str) else [as_plane(v) for v in value]
    if ('solutions' in job) == ('targets' in job):
        raise ValueError('Job requires exactly one of solutions or targets')
    current = job['current_pose']
    bases = planes(job['bases']) if 'bases' in job else [Plane.world_xy()]
    options = dict(job.get('planning', {}))
    collision_config = job.get('collision')
    candidate_keys = {'rotation_mode', 'rotation_angle_deg', 'rotation_steps', 'angle_cw_deg', 'angle_ccw_deg'}
    graph_keys = {'max_joint_step', 'periodic', 'weights', 'joint_ranges', 'start_base', 'revolute_joints'}
    unknown = set(options)-candidate_keys-graph_keys-{'enable_collision_check'}
    if unknown:
        raise ValueError('Unknown planning options: ' + ', '.join(sorted(unknown)))
    if options.pop('enable_collision_check', False) and not collision_config:
        raise ValueError('Collision checking requested without a collision configuration')
    if 'solutions' in job and candidate_keys.intersection(options):
        raise ValueError('Rotation sampling options require targets rather than precomputed solutions')
    with ExitStack() as stack:
        scene = None
        if collision_config:
            from ..collision import PybulletServer
            scene = stack.enter_context(PybulletServer(path(collision_config['urdf']),
                joint_names=collision_config.get('joint_names'),
                allowed_pairs=collision_config.get('allowed_pairs', []), gui=collision_config.get('gui', False),
                check_static_self_collisions=collision_config.get('check_static_self_collisions', False)))
            scene.set_fixed_joints(collision_config.get('fixed_joints', {}))
            for mesh in collision_config.get('environment', []):
                if isinstance(mesh, str):
                    scene.add_mesh(path(mesh))
                else:
                    scene.add_mesh(path(mesh['path']), plane=mesh.get('plane'), scale=mesh.get('scale', 1.0))
            if 'ground_z' in collision_config:
                scene.add_ground(collision_config['ground_z'], support_links=collision_config.get('support_links', []))
            for attachment in collision_config.get('tools', []):
                scene.attach_mesh(path(attachment['mesh']), attachment['link_name'],
                                  frame=attachment.get('frame'), touch_links=attachment.get('touch_links', []))
        if 'targets' in job:
            targets = planes(job['targets'])
            evaluated = calculate_partial_trajectory(current, targets, base_planes=bases,
                tool=job.get('tool'), arm_in_base=job.get('arm_in_base'),
                dont_build_graph=True, **{k: v for k, v in options.items() if k != 'revolute_joints'})
            solutions = evaluated['ik_solutions_per_node']
        else:
            solutions = data(job['solutions'])
        graph_options = {k: options[k] for k in graph_keys if k in options}
        if 'targets' in job:
            graph_options.setdefault('revolute_joints', tuple(range(6)))
        return run_all(solutions, current, base_planes=bases, collision=scene.is_valid if scene else None,
            transition_check=partial(scene.edge_is_valid, periodic=options.get('periodic')) if scene and collision_config.get('check_edges', False) else None,
            tolerance=job.get('edit_tolerance'), snap_periodic=job.get('snap_periodic'), **graph_options)


@recorded
def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('job', type=Path, help='Explicit JSON planning job')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    event('job.configuration_file', path=args.job)
    result = run_job(json.loads(args.job.read_text(encoding='utf-8')), root=args.job.resolve().parent)
    if result['configurations']:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(result['configurations'], indent=2, allow_nan=False), encoding='utf-8')
        event('job.output_file', path=args.output)
        print('Planned {} targets; cost {:.6g}'.format(len(result['configurations']), result['path_length']))
    else:
        parser.exit(2, 'No complete feasible path. Unreachable targets: {}\n'.format(result['unreachable_points']))


if __name__ == '__main__':
    main()
