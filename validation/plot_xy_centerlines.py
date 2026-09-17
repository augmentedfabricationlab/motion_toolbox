"""Plot one spatial centerline for each saved smooth_xy recording."""
import argparse
import hashlib
import json
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src'))
import numpy as np
from motion_toolbox.xy_centerline import centerline_xy
import motion_toolbox.xy_centerline as implementation
from motion_toolbox import __version__

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--recordings',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    args.output.mkdir(parents=True,exist_ok=True)
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    summaries=[]
    for n,label in enumerate(['first','second'],1):
        source=args.recordings/('xy_smoothing_'+label+'_20260916')
        saved=json.loads((source/'selected_line.json').read_text())
        provenance=json.loads((source/'summary.json').read_text())
        assert saved['units']=='metres'
        case=Path(provenance['case'])
        digest=hashlib.sha256((case/'case.json').read_bytes()).hexdigest()
        assert digest==provenance['case_sha256']==json.loads((case/'manifest.json').read_text())['case.json']
        assert (case/'READY').exists()
        raw=np.array([t['origin'] for t in json.loads((case/'case.json').read_text())['replay']['targets']])
        points=np.array(saved['points'])
        result=centerline_xy(points)
        curve=result['curve']
        # Verify no longitudinal shortening, including every mapped point.
        projection=(curve-result['origin'])@result['direction']
        np.testing.assert_allclose(projection[[0,-1]], [result['longitudinal'].min(),result['longitudinal'].max()],atol=1e-12)
        np.testing.assert_allclose((result['mapped_points']-result['origin'])@result['direction'],result['longitudinal'],atol=1e-12)
        fig,axes=plt.subplots(2,1,figsize=(12,8),gridspec_kw={'height_ratios':[1.3,1]},layout='constrained')
        ax=axes[0]
        ax.plot(*raw[:,:2].T,color='#d2d7dc',lw=.6,label='Input TCP projection')
        ax.plot(*points[:,:2].T,color='#218358',lw=1.3,label='Existing smooth_xy passes')
        ax.plot(*curve.T,color='#2568b1',lw=3,label='Single spatial centerline')
        ax.scatter(*curve[[0,-1]].T,color='#2568b1',s=35,zorder=5)
        middle=curve[len(curve)//2]
        ax.quiver(*middle,*result['direction']*.55,angles='xy',scale_units='xy',scale=1,color='#bd6930',width=.005,label='Dominant XY direction')
        ax.set_aspect('equal',adjustable='datalim')
        ax.set(xlabel='World X (m)',ylabel='World Y (m)',title='Average across the passes; retain their full longitudinal extent')
        ax.grid(alpha=.2);ax.legend(fontsize=9,loc='best')
        ax=axes[1]
        ax.scatter(result['longitudinal'],result['transverse'],s=4,color='#218358',alpha=.3,label='Input smooth-line points')
        ax.plot(result['station_longitudinal'],result['station_transverse'],color='#2568b1',lw=2.5,label='Averaged transverse position')
        for limit in [result['longitudinal'].min(),result['longitudinal'].max()]:
            ax.axvline(limit,color='#bd6930',ls='--',lw=1)
        ax.set(xlabel='Position along dominant direction (m)',ylabel='Perpendicular position (m)',title='Only the perpendicular coordinate is smoothed')
        ax.grid(alpha=.2);ax.legend(fontsize=9)
        fig.suptitle('Recording {} | single centerline | preserved extent {:.4f} m'.format(n,result['longitudinal_extent']),fontsize=16)
        fig.supxlabel('Spatial averaging: 100 bins, local-linear smoothing bandwidth = 8% of longitudinal extent.\nNo offset applied. Extent is preserved relative to the existing smooth_xy line, not the raw TCP path.',fontsize=10)
        fig.savefig(args.output/f'recording_{n}_centerline.png',dpi=170)
        fig.savefig(args.output/f'recording_{n}_centerline.pdf')
        plt.close(fig)
        xyz=np.column_stack((curve,np.zeros(len(curve))))
        mapped=np.column_stack((result['mapped_points'],np.zeros(len(points))))
        payload=dict(units='metres',centerline=xyz.tolist(),per_target_points=mapped.tolist(),direction=result['direction'].tolist(),longitudinal_extent=result['longitudinal_extent'],bandwidth=result['bandwidth'])
        (args.output/f'recording_{n}_centerline.json').write_text(json.dumps(payload))
        summaries.append(dict(recording=n,source=str(source),source_sha256=hashlib.sha256((source/'selected_line.json').read_bytes()).hexdigest(),case_sha256=digest,input_count=len(points),longitudinal_extent=result['longitudinal_extent'],principal_variance_fraction=result['principal_variance_fraction'],bandwidth=result['bandwidth']))
    (args.output/'summary.json').write_text(json.dumps(dict(toolbox_version=__version__,source_sha256=hashlib.sha256(Path(implementation.__file__).read_bytes()).hexdigest(),recordings=summaries),indent=2))
    print(json.dumps(summaries))

if __name__=='__main__':
    main()
