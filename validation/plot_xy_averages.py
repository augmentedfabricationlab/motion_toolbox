"""Plot an XY-only window sweep from a captured case; writes outside Git."""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
import numpy as np
from motion_toolbox.xy_averaging import compare_windows


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('case', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--min-window', type=int, default=10)
    parser.add_argument('--max-window', type=int, default=200)
    args = parser.parse_args()
    case = args.case / 'case.json'
    if not (args.case / 'READY').is_file():
        raise ValueError('Capture must contain READY')
    digest = hashlib.sha256(case.read_bytes()).hexdigest()
    manifest = json.loads((args.case / 'manifest.json').read_text())
    if manifest.get('case.json') != digest:
        raise ValueError('case.json does not match its manifest')
    data = json.loads(case.read_text())['replay']
    if data['units'] != 'metres/radians':
        raise ValueError('Expected captured positions in metres')
    points = np.array([t['origin'] for t in data['targets']], dtype=float)
    started = time.perf_counter()
    result = compare_windows(points, range(args.min_window, args.max_window + 1))
    elapsed = time.perf_counter()-started
    args.output.mkdir(parents=True, exist_ok=True)
    windows, lengths = np.array(result['windows']), np.array(result['lengths'])
    best = result['best_window']
    raw = result['raw_xy']
    shortest = result['shortest_window']
    ratios = np.array(result['length_ratios'])
    interior = [w for w in result['local_minimum_windows'] if w >= 50]
    trough = min(interior, key=lambda w: lengths[result['windows'].index(w)]) if interior else best
    choices = sorted({int(windows[np.argmin(abs(windows-w))])
                      for w in (10,25,50,75,100,trough,best,150,shortest)})
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    rows = int(np.ceil(len(choices)/3))
    fig = plt.figure(figsize=(15, 3.4 + rows*1.8), layout='constrained')
    grid = fig.add_gridspec(rows+1, 3, height_ratios=[1.8]+[1]*rows)
    top = grid[0,:].subgridspec(1,2)
    for column, values, winner, color, title, ylabel in (
        (0,lengths,shortest,'#d99200','Raw length: favors 200 points','XY polyline length (m)'),
        (1,ratios,best,'#278457','Length / endpoint distance: selects 120 points','L / D (dimensionless)')):
        # Titles use actual results when rerun on different captures/windows.
        title = ('Raw length: shortest at {} points' if column==0 else
                 'Length / endpoint distance: selected {} points').format(winner)
        ax = fig.add_subplot(top[0,column])
        ax.plot(windows,values,color='#327a91',lw=2)
        value = values[result['windows'].index(winner)]
        ax.scatter([winner],[value],color=color,s=65,zorder=5)
        ax.axvline(winner,color=color,ls='--',alpha=.5)
        ax.text(.04,.95,'{} points | {:.4f}{}'.format(winner,value,' m' if column==0 else ''),
                transform=ax.transAxes,va='top',color=color,weight='bold',
                bbox=dict(facecolor='white',edgecolor='none',alpha=.9))
        ax.set(title=title,xlabel='Window (input points)',ylabel=ylabel)
        ax.grid(alpha=.2)
        if windows[-1]-windows[0]>=50:
            zoom = ax.inset_axes([.49,.45,.46,.42])
            mask = windows>=max(windows[0],best-20)
            zoom.plot(windows[mask],values[mask],color='#327a91',lw=1.5)
            zoom.scatter([winner],[value],color=color,s=25,zorder=4)
            zoom.set_title('Detail around the minima',fontsize=9)
            zoom.tick_params(labelsize=8)
            zoom.grid(alpha=.2)
    padding = .05
    bounds = (raw.min(axis=0)-padding, raw.max(axis=0)+padding)
    for k,w in enumerate(choices):
        ax = fig.add_subplot(grid[1+k//3, k%3])
        index = result['windows'].index(w)
        xy = result['curves'][index]
        chosen = w==best
        color = '#278457' if chosen else '#d99200' if w==shortest else '#327a91'
        ax.plot(raw[:,0], raw[:,1], color='#c8cdd2', lw=.65, label='Projected input')
        ax.plot(xy[:,0], xy[:,1], color=color, lw=2,
                label='XY moving average')
        ax.scatter(*xy[0], color='#278457', s=17, zorder=4)
        ax.scatter(*xy[-1], color='#9c5266', s=17, zorder=4)
        ax.set(xlim=(bounds[0][0],bounds[1][0]), ylim=(bounds[0][1],bounds[1][1]),
               xlabel='X (m)', ylabel='Y (m)',
               title=('MIN L/D | ' if chosen else 'MIN LENGTH | ' if w==shortest else '') + '{} points\nL = {:.4f} m | L/D = {:.4f}'.format(w,lengths[index],ratios[index]))
        ax.title.set_fontsize(10)
        ax.set_aspect('equal')
        if chosen or w==shortest:
            ax.set_facecolor('#edf8ef' if chosen else '#fff7e4')
            for spine in ax.spines.values():
                spine.set_color(color)
        ax.grid(alpha=.15)
    fig.suptitle('XY position averaging | {:,} input planes | no robot planning'.format(len(points)), fontsize=17)
    fig.supxlabel('Grey: input XY | Green panel: minimum L/D | Gold panel: minimum length | D uses each averaged line\'s own endpoints', fontsize=10)
    fig.savefig(args.output/'window_comparison.png', dpi=160)
    fig.savefig(args.output/'window_comparison.pdf')
    plt.close(fig)
    xy = result['best_curve']
    xyz = np.column_stack((xy,np.zeros(len(xy))))
    np.savetxt(args.output/'selected_line.csv', xyz, delimiter=',', header='x_m,y_m,z_m', comments='')
    np.savetxt(args.output/'window_lengths.csv', np.column_stack((windows,lengths,result['chord_lengths'],ratios)), delimiter=',',
               header='window_points,line_length_m,endpoint_distance_m,length_ratio', comments='', fmt=['%d','%.12f','%.12f','%.12f'])
    (args.output/'selected_line.json').write_text(json.dumps(dict(units='metres',window=best,metric=result['metric'],points=xyz.tolist())))
    displayed = []
    for w in choices:
        i = result['windows'].index(w)
        curve = np.column_stack((result['curves'][i], np.zeros(len(points))))
        displayed.append(dict(window=w,length_metres=lengths[i],length_ratio=ratios[i],points=curve.tolist()))
    (args.output/'comparison_lines.json').write_text(json.dumps(dict(units='metres',lines=displayed)))
    summary = {k:result[k] for k in ('metric','best_window','best_length','best_score','shortest_window','shortest_length','raw_length','local_minimum_windows')}
    import motion_toolbox.xy_averaging as module
    from motion_toolbox import __version__
    summary.update(target_count=len(points), window_range=[int(windows[0]),int(windows[-1])],
        best_at_search_boundary=best in (windows[0],windows[-1]),
        endpoint_shifts_metres=result['endpoint_shifts'].tolist(),elapsed_seconds=elapsed,
        case_sha256=digest,toolbox_version=__version__,
        averaging_source_sha256=hashlib.sha256(Path(module.__file__).read_bytes()).hexdigest(),
        notes='Centered index-space box average, clipped/renormalized at ends. Heights/orientations ignored. '
              'L/D uses each averaged line endpoint distance. Neither score proves frequency/curvature optimality. Endpoints are not fixed.')
    (args.output/'summary.json').write_text(json.dumps(summary,indent=2))
    print(json.dumps(summary))


if __name__ == '__main__':
    main()
