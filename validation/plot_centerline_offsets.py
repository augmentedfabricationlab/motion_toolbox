"""Offset original smooth passes, using the centerline only for headings."""
import argparse
import hashlib
import json
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src'))
import numpy as np
from motion_toolbox.xy_offset import centerline_offset_frames
import motion_toolbox.xy_offset as implementation
from motion_toolbox import __version__


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--recordings',type=Path,required=True)
    parser.add_argument('--centerlines',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    args.output.mkdir(parents=True,exist_ok=True)
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    provenance=json.loads((args.centerlines/'summary.json').read_text())
    summaries=[]
    for n,name in enumerate(['20260916_165102_2ae63c0d','20260916_204312_5c6b046f'],1):
        case=args.recordings/name
        digest=hashlib.sha256((case/'case.json').read_bytes()).hexdigest()
        assert (case/'READY').exists()
        assert digest==json.loads((case/'manifest.json').read_text())['case.json']==provenance['recordings'][n-1]['case_sha256']
        replay=json.loads((case/'case.json').read_text())['replay']
        assert replay['units']=='metres/radians'
        targets=replay['targets']
        source=args.centerlines/f'recording_{n}_centerline.json'
        saved=json.loads(source.read_text()); assert saved['units']=='metres'
        curve=np.array(saved['centerline'])
        mapped=np.array(saved['per_target_points'])
        pass_source=Path(provenance['recordings'][n-1]['source'])/'selected_line.json'
        assert hashlib.sha256(pass_source.read_bytes()).hexdigest()==provenance['recordings'][n-1]['source_sha256']
        passes_saved=json.loads(pass_source.read_text()); assert passes_saved['units']=='metres'
        passes=np.array(passes_saved['points'])
        tcp=np.array([t['origin'] for t in targets])
        f=centerline_offset_frames(curve,mapped,[t['x_axis'] for t in targets],[t['y_axis'] for t in targets],pass_points=passes)
        base=f['origins']
        assert len(base)==len(tcp)
        np.testing.assert_array_equal(f['z_axes'],np.tile([0.,0.,1.],(len(base),1)))
        np.testing.assert_array_equal(base[:,2],0.)
        np.testing.assert_allclose(np.cross(f['x_axes'],f['y_axes']),f['z_axes'],atol=1e-12)
        shift=base-passes
        np.testing.assert_allclose(np.sum(shift*f['x_axes'],axis=1),-.9,atol=1e-12)
        np.testing.assert_allclose(np.sum(shift*f['y_axes'],axis=1),1.2,atol=1e-12)
        fig=plt.figure(figsize=(15,7),layout='constrained')
        ax=fig.add_subplot(121)
        ax.plot(*tcp[:,:2].T,c='#d2d7dc',lw=.6,label='Input TCP projection')
        ax.plot(*passes[:,:2].T,c='#218358',lw=1.3,label='Original smooth passes')
        ax.plot(*curve[:,:2].T,c='#7751a0',lw=1.5,ls='--',label='Centerline: heading reference')
        ax.plot(*base[:,:2].T,c='#2568b1',lw=2,label='Base path: -0.9 X +1.2 Y')
        indices=np.unique(np.linspace(0,len(base)-1,8,dtype=int))
        for i in indices:
            ax.plot([passes[i,0],base[i,0]],[passes[i,1],base[i,1]],c='#aab1b8',ls='--',lw=.7)
        for key,col,label in [('x_axes','#d0644c','Base +X (toward wall)'),('y_axes','#9770b0','Base +Y (tangent)')]:
            vector=f[key][indices,:2]*.2
            ax.quiver(base[indices,0],base[indices,1],vector[:,0],vector[:,1],angles='xy',scale_units='xy',scale=1,color=col,width=.004,label=label)
        ax.set_aspect('equal',adjustable='datalim');ax.grid(alpha=.2)
        ax.set(xlabel='World X (m)',ylabel='World Y (m)',title='Top view: headings follow the centerline')
        ax.legend(fontsize=9,loc='best')
        ax=fig.add_subplot(122,projection='3d',computed_zorder=False)
        ax.plot(*tcp.T,c='#cf782d',lw=.8,label='Input TCP path')
        ax.plot(*passes.T,c='#218358',lw=1.2,label='Original smooth passes')
        ax.plot(*curve.T,c='#7751a0',lw=1.5,ls='--',label='Centerline: heading reference')
        ax.plot(*base.T,c='#2568b1',lw=2,label='Offset base path')
        ax.scatter(*base[0],c='#2568b1',s=40,marker='o',depthshade=False)
        ax.scatter(*base[-1],c='#2568b1',s=40,marker='s',depthshade=False)
        bounds=np.vstack((tcp,curve,base));span=np.ptp(bounds,axis=0)
        ax.set_box_aspect(span)
        ax.view_init(elev=27,azim=-115)
        ax.set(xlabel='World X (m)',ylabel='World Y (m)',zlabel='World Z (m)',title='3D view: base Z = 0, global +Z up')
        ax.legend(fontsize=9,loc='upper left')
        fig.suptitle(f'Recording {n}: offset all passes using centerline headings',fontsize=17)
        fig.supxlabel('Normal offset 0.9 m away from wall + tangent offset 1.2 m along base +Y.\nAll pass positions retained before offset; centerline controls headings only. Geometry only: reach/collisions not validated.',fontsize=10)
        fig.savefig(args.output/f'recording_{n}_centerline_offset.png',dpi=170)
        fig.savefig(args.output/f'recording_{n}_centerline_offset.pdf')
        plt.close(fig)
        frames=[dict(origin=o.tolist(),x_axis=x.tolist(),y_axis=y.tolist(),z_axis=z.tolist()) for o,x,y,z in zip(base,f['x_axes'],f['y_axes'],f['z_axes'])]
        (args.output/f'recording_{n}_frames.json').write_text(json.dumps(dict(units='metres',case_sha256=digest,base_frames=frames)))
        summaries.append(dict(recording=n,case_sha256=digest,centerline_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),pass_source_sha256=hashlib.sha256(pass_source.read_bytes()).hexdigest(),count=len(base),x_offset=-.9,y_offset=1.2,mean_target_normal_alignment=f['mean_target_normal_alignment']))
    (args.output/'summary.json').write_text(json.dumps(dict(toolbox_version=__version__,source_sha256=hashlib.sha256(Path(implementation.__file__).read_bytes()).hexdigest(),recordings=summaries),indent=2))
    print(json.dumps(summaries))

if __name__=='__main__':
    main()
