"""Overlay saved smooth_xy lines and diagonal frame offsets for two captures."""
import argparse
import hashlib
import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'src'))
import numpy as np
from motion_toolbox import __version__
from motion_toolbox.xy_offset import offset_frames
import motion_toolbox.xy_offset as implementation


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cases', type=Path, nargs=2, required=True)
    parser.add_argument('--lines', type=Path, nargs=2, required=True, help='Prior smoothing output directories')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(1, 2, figsize=(15, 6))
    fig.subplots_adjust(left=.06, right=.98, top=.78, bottom=.27, wspace=.18)
    summaries = []
    for number, (case, lines, ax) in enumerate(zip(args.cases, args.lines, axes), 1):
        digest = hashlib.sha256((case/'case.json').read_bytes()).hexdigest()
        if not (case/'READY').exists() or json.loads((case/'manifest.json').read_text()).get('case.json') != digest:
            raise ValueError('Incomplete or mismatched capture: {}'.format(case))
        data = json.loads((case/'case.json').read_text())['replay']
        previous = json.loads((lines/'summary.json').read_text())
        saved = json.loads((lines/'selected_line.json').read_text())
        if previous['case_sha256'] != digest or saved['units'] != 'metres' or data['units'] != 'metres/radians':
            raise ValueError('Saved line provenance/units mismatch')
        targets = data['targets']
        old = np.array(saved['points'])
        raw = np.array([t['origin'] for t in targets])
        frames = offset_frames(old, [t['x_axis'] for t in targets], [t['y_axis'] for t in targets])
        new = frames['origins']
        # Correspondence is by original index: no arc-length redistribution.
        assert len(new) == len(raw)
        np.testing.assert_allclose(np.linalg.norm(new-old, axis=1), 1.5, atol=1e-12)
        ax.plot(*raw[:, :2].T, color='#cbd0d5', lw=.65, label='Original TCP projection', zorder=1)
        ax.plot(*old[:, :2].T, color='#218358', lw=2., label='Existing smooth_xy line', zorder=3)
        ax.plot(*new[:, :2].T, color='#3269b1', lw=2., label='Offset base-origin path', zorder=4)
        indices = np.unique(np.linspace(0, len(new)-1, 9, dtype=int))
        for i in indices:
            ax.plot([old[i, 0], new[i, 0]], [old[i, 1], new[i, 1]], color='#8c96a0', lw=.8, ls='--', alpha=.6)
        for axis_name, color, label in [('x_axes', '#cd5c49', 'Base +X (target Z projected to XY)'),
                                       ('y_axes', '#9671ad', 'Base +Y')]:
            vector = frames[axis_name][indices, :2]*.20
            ax.quiver(new[indices, 0], new[indices, 1], vector[:, 0], vector[:, 1],
                      angles='xy', scale_units='xy', scale=1, color=color, width=.004, label=label, zorder=5)
        ax.scatter(*new[0, :2], c='#3269b1', marker='o', s=55, zorder=6)
        ax.scatter(*new[-1, :2], c='#3269b1', marker='s', s=55, zorder=6)
        ax.annotate('start', new[0, :2], xytext=(7, 9), textcoords='offset points', fontsize=9)
        ax.annotate('end', new[-1, :2], xytext=(7, -13), textcoords='offset points', fontsize=9)
        ax.set(title='Recording {}: {}\n{:,} corresponding points / upright frames'.format(
            number, 'main oscillations in height' if number == 1 else 'horizontal sweeps', len(new)),
               xlabel='World X (m)', ylabel='World Y (m)')
        ax.set_aspect('equal', adjustable='box')
        ax.grid(alpha=.2)
        out = dict(units='metres', case_sha256=digest, target_indices=list(range(len(new))),
                   base_frames=[dict(origin=o.tolist(), x_axis=x.tolist(), y_axis=y.tolist(), z_axis=z.tolist())
                                for o, x, y, z in zip(new, frames['x_axes'], frames['y_axes'], frames['z_axes'])])
        (args.output/('recording_{}_frames.json'.format(number))).write_text(json.dumps(out))
        summaries.append(dict(case=str(case), case_sha256=digest, line=str(lines/'selected_line.json'),
                              line_sha256=hashlib.sha256((lines/'selected_line.json').read_bytes()).hexdigest(),
                              count=len(new), smoothing_bound=saved['max_deviation'],
                              x_offset=-.9, y_offset=1.2, diagonal_offset=1.5,
                              collision_checked=False))
    # Shared scale makes the two recordings directly comparable.
    xmin = min(ax.get_xlim()[0] for ax in axes); xmax = max(ax.get_xlim()[1] for ax in axes)
    ymin = min(ax.get_ylim()[0] for ax in axes); ymax = max(ax.get_ylim()[1] for ax in axes)
    for ax in axes:
        ax.set(xlim=(xmin, xmax), ylim=(ymin, ymax))
    handles, labels = axes[0].get_legend_handles_labels()
    fig.legend(handles, labels, loc='lower center', bbox_to_anchor=(.5, .085), ncol=3, fontsize=10)
    fig.suptitle('Diagonal base offset from the same smooth_xy lines\n'
                 'New origin = smooth point - 0.9 m local X + 1.2 m local Y | Z up', fontsize=17)
    fig.supxlabel('Dashed lines connect matching indices; arrows show sampled frame axes (0.20 m). Circles: start; squares: end.\n'
                  'Geometry proposal only: 0.9 m is the local-X offset, not verified wall clearance. No reach / collision validation.', fontsize=10, y=.015)
    fig.savefig(args.output/'old_and_offset_paths.png', dpi=170)
    fig.savefig(args.output/'old_and_offset_paths.pdf')
    plt.close(fig)
    summary = dict(toolbox_version=__version__, source_sha256=hashlib.sha256(Path(implementation.__file__).read_bytes()).hexdigest(),
                   recordings=summaries, command=[sys.executable]+sys.argv)
    (args.output/'summary.json').write_text(json.dumps(summary, indent=2))
    print(json.dumps(summary))


if __name__ == '__main__':
    main()
