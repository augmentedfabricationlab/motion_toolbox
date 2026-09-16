"""Compare bounded XY smoothing on captures; all artifacts go to --output."""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'src'))
import numpy as np
from motion_toolbox import __version__
from motion_toolbox.xy_averaging import compare_windows
from motion_toolbox.xy_smoothing import smooth_xy, significant_reversals
import motion_toolbox.xy_smoothing as implementation


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('case', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--deviations', type=float, nargs='+', default=[.1, .25, .5])
    parser.add_argument('--selected-deviation', type=float, default=.25)
    args = parser.parse_args()
    if not (args.case/'READY').is_file():
        raise ValueError('Capture must contain READY')
    case = args.case/'case.json'
    digest = hashlib.sha256(case.read_bytes()).hexdigest()
    if json.loads((args.case/'manifest.json').read_text()).get('case.json') != digest:
        raise ValueError('Capture hash mismatch')
    data = json.loads(case.read_text())['replay']
    if data['units'] != 'metres/radians':
        raise ValueError('Expected metres/radians')
    points = np.array([t['origin'] for t in data['targets']])
    raw = points[:, :2]
    budgets = sorted(set(args.deviations+[args.selected_deviation]))
    results = []
    for budget in budgets:
        start = time.perf_counter()
        result = smooth_xy(points, budget)
        result['elapsed_seconds'] = time.perf_counter()-start
        results.append(result)
    selected = results[budgets.index(args.selected_deviation)]
    previous = compare_windows(points)
    # PCA is only a display coordinate; the optimizer works in full world XY.
    origin = raw.mean(axis=0)
    _, _, axes = np.linalg.svd(raw-origin, full_matrices=False)
    axis = axes[0]
    if axis[np.argmax(abs(axis))] < 0:
        axis = -axis
    coordinate = (raw-origin)@axis
    smoothed_coordinate = (selected['curve']-origin)@axis
    threshold = 2.*args.selected_deviation
    turns = significant_reversals(smoothed_coordinate, threshold)
    raw_turns = significant_reversals(coordinate, threshold)
    args.output.mkdir(parents=True, exist_ok=True)
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig = plt.figure(figsize=(16, 12), layout='constrained')
    grid = fig.add_gridspec(4, 3, height_ratios=[1.3, 1.15, 1., 1.])
    ax = fig.add_subplot(grid[0, :])
    ax.plot(*raw.T, color='#c4cad0', lw=.8, label='Input projected to XY')
    ax.plot(*selected['curve'].T, color='#218358', lw=2., label='Bounded smooth line')
    ax.scatter(*selected['curve'][0], c='#218358', s=45, marker='o', label='Start', zorder=4)
    ax.scatter(*selected['curve'][-1], c='#a5456b', s=45, marker='s', label='End', zorder=4)
    ax.set(title='Selected geometry experiment: {:.2f} m maximum corresponding-target deviation | {:.2f} m line'.format(
        args.selected_deviation, selected['length']), xlabel='X (m)', ylabel='Y (m)')
    ax.set_aspect('equal', adjustable='datalim')
    ax.legend(loc='upper left', fontsize=9)
    ax.grid(alpha=.2)
    for col, result in enumerate(results[:3]):
        ax = fig.add_subplot(grid[1, col])
        ax.plot(*raw.T, color='#c4cad0', lw=.6)
        ax.plot(*result['curve'].T, color='#218358' if result is selected else '#327a91', lw=1.7)
        ax.set(title='Bound {:.2f} m | length {:.2f} m\nMax deviation {:.3f} m | {}'.format(
            result['max_deviation'], result['length'], result['measured_max_deviation'],
            'converged' if result['converged'] else 'iteration limit'), xlabel='X (m)', ylabel='Y (m)')
        ax.set_aspect('equal', adjustable='datalim')
        ax.grid(alpha=.2)
    ax = fig.add_subplot(grid[2, :2])
    ax.plot(coordinate, color='#b4bec7', lw=1., label='Input XY dominant-axis coordinate')
    ax.plot(smoothed_coordinate, color='#218358', lw=1.8, label='Selected line coordinate')
    ax.scatter(turns, smoothed_coordinate[turns], color='#bd6534', s=30, zorder=4, label='Significant reversals')
    ax.set(title='Ordered horizontal motion: {} retained reversals (confirmation excursion {:.2f} m)'.format(len(turns), threshold),
           xlabel='Original target index', ylabel='Dominant-axis coordinate (m)')
    ax.legend(fontsize=8, loc='upper left')
    ax.grid(alpha=.2)
    ax = fig.add_subplot(grid[2, 2])
    ax.plot(points[:, 2], color='#78579b', lw=1.)
    ax.set(title='Input height (not used for XY smoothing)', xlabel='Original target index', ylabel='Z (m)')
    ax.grid(alpha=.2)
    ax = fig.add_subplot(grid[3, :2])
    for result in results:
        ax.plot(result['deviations'], lw=1., label='Bound {:.2f} m'.format(result['max_deviation']))
    ax.set(title='Distance from each line point to its corresponding input target', xlabel='Original target index', ylabel='XY deviation (m)')
    ax.legend(fontsize=9)
    ax.grid(alpha=.2)
    ax = fig.add_subplot(grid[3, 2])
    ax.plot(*raw.T, color='#c4cad0', lw=.6)
    ax.plot(*previous['best_curve'].T, color='#bc763c', lw=1.7)
    old_error = float(np.linalg.norm(previous['best_curve']-raw, axis=1).max())
    ax.set(title='Previous L/D average: window {}\nMax deviation {:.2f} m (unconstrained)'.format(previous['best_window'], old_error),
           xlabel='X (m)', ylabel='Y (m)')
    ax.set_aspect('equal', adjustable='datalim')
    ax.grid(alpha=.2)
    fig.suptitle('XY smoothing | {} | {:,} targets'.format(args.case.name, len(points)), fontsize=17)
    fig.supxlabel('Whole-path squared-step + curvature smoothing within per-index XY disks. Geometry only; no robot reach or collision validation.\n'
                  'Reversal markers use a PCA display coordinate, not wall arc length. Repeated traversals overlap in the XY view.', fontsize=10)
    fig.savefig(args.output/'smoothing_comparison.png', dpi=160)
    fig.savefig(args.output/'smoothing_comparison.pdf')
    plt.close(fig)
    xyz = np.column_stack((selected['curve'], np.zeros(len(raw))))
    np.savetxt(args.output/'selected_line.csv', xyz, delimiter=',', header='x_m,y_m,z_m', comments='')
    (args.output/'selected_line.json').write_text(json.dumps(dict(units='metres', max_deviation=args.selected_deviation, points=xyz.tolist())))
    summaries = [{k:v for k,v in result.items() if k not in ('curve','deviations')} for result in results]
    summary = dict(case=str(args.case), case_sha256=digest, target_count=len(raw), toolbox_version=__version__,
                   source_sha256=hashlib.sha256(Path(implementation.__file__).read_bytes()).hexdigest(),
                   selected_deviation=args.selected_deviation, results=summaries,
                   significant_input_reversals=raw_turns, significant_line_reversals=turns,
                   previous_window=previous['best_window'], previous_max_deviation=old_error,
                   notes='Geometry only. Bound is an experiment parameter, not arm reach. One output per original target; Z=0. '
                         'Endpoints free inside disks. Index parametrization, no speed/IK/collision validation.',
                   command=[sys.executable]+sys.argv)
    (args.output/'summary.json').write_text(json.dumps(summary, indent=2))
    (args.output/'comparison_lines.json').write_text(json.dumps([
        dict(max_deviation=r['max_deviation'], points=r['curve'].tolist()) for r in results]))
    print(json.dumps(summary))


if __name__ == '__main__':
    main()
