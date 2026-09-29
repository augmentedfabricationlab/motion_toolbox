"""Plot segment-average translational speeds from exported controller JSON.

Usage: python validation/plot_export_speeds.py INPUT_FOLDER --output OUTSIDE_GIT
Requires Matplotlib for plotting. Does not infer actual measured controller speed.
"""
import argparse
import csv
import hashlib
import json
from pathlib import Path

import numpy as np


def speeds(path):
    data=json.loads(path.read_text(encoding='utf-8'))
    poses=data['poses']
    ns=[p['stamp']['sec']*10**9+p['stamp']['nanosec'] for p in poses]
    dt=np.array([b-a for a,b in zip(ns,ns[1:])],dtype=float)/1e9
    if len(dt)==0 or np.any(dt<=0):
        raise ValueError('Speed plotting requires at least two poses and strictly increasing timestamps: '+str(path))
    xyz=np.array([[p['position'][k] for k in 'xyz'] for p in poses])
    if not np.isfinite(xyz).all():raise ValueError('Nonfinite position in '+str(path))
    distance=np.linalg.norm(np.diff(xyz,axis=0),axis=1)
    cm_s=100*distance/dt
    return ns,np.array(ns,dtype=float)/1e9,cm_s,dict(
        poses=len(poses),duration_seconds=sum(dt),distance_m=float(distance.sum()),
        minimum_cm_s=float(cm_s.min()),maximum_cm_s=float(cm_s.max()),
        time_weighted_mean_cm_s=float(100*distance.sum()/sum(dt)),
        sha256=hashlib.sha256(path.read_bytes()).hexdigest())


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('input',type=Path)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    arm=speeds(args.input/'arm_path.json');base=speeds(args.input/'base_path.json')
    args.output.mkdir(parents=True,exist_ok=True)
    summary=dict(input_folder=str(args.input),timestamp_pairs_match=arm[0]==base[0],
                 arm=arm[3],base=base[3],method='3D position distance / elapsed time for each consecutive pair; cm/s')
    (args.output/'speed_summary.json').write_text(json.dumps(summary,indent=2))
    with (args.output/'segment_speeds.csv').open('w',newline='',encoding='utf-8') as stream:
        writer=csv.writer(stream);writer.writerow(['path','from_index','to_index','start_seconds','end_seconds','speed_cm_s'])
        for name,result in [('arm',arm),('base',base)]:
            for i,speed in enumerate(result[2]):writer.writerow([name,i,i+1,result[1][i],result[1][i+1],speed])
    plt.rcParams.update({'font.size':11,'axes.spines.top':False,'axes.spines.right':False})
    fig,axes=plt.subplots(2,1,figsize=(13,7.5),layout='constrained')
    start=arm[1][0]
    for ax in axes:
        ax.stairs(base[2],(base[1]-start)/60,color='#247a9e',linewidth=1.25,label='Base',baseline=None)
        ax.stairs(arm[2],(arm[1]-start)/60,color='#d55e00',linewidth=1.7,label='Arm TCP',baseline=None)
        ax.set(ylabel='Speed (cm/s)',xlabel='Time (minutes)',ylim=(0,max(arm[2].max(),base[2].max())*1.12))
        ax.grid(alpha=.22);ax.legend(loc='upper right')
    axes[0].set_xlim(0,max(arm[1][-1],base[1][-1])/60-start/60)
    axes[0].set_title('Whole path: {:.2f} minutes, {:,} poses'.format(arm[3]['duration_seconds']/60,arm[3]['poses']),loc='left')
    axes[1].set_xlim(0,min(3,arm[3]['duration_seconds']/60))
    axes[1].set_title('First three minutes',loc='left')
    fig.suptitle('Arm and base speeds from the exported path',fontsize=17,fontweight='bold')
    fig.supxlabel('Consecutive-pose average speeds, not measured robot velocity. All {:,} paired timestamps match.\n'
        'Arm TCP: {:.2f} cm/s  |  Base: {:.2f}–{:.2f} cm/s; time-weighted mean {:.2f} cm/s'.format(
            arm[3]['poses'],arm[3]['time_weighted_mean_cm_s'],base[3]['minimum_cm_s'],base[3]['maximum_cm_s'],base[3]['time_weighted_mean_cm_s'])
        if summary['timestamp_pairs_match'] else 'Consecutive-pose average speeds. Arm and base timestamps differ.',fontsize=10)
    for extension in ('png','pdf'):
        fig.savefig(args.output/('arm_base_speed.'+extension),dpi=170)
    plt.close(fig)
    print(json.dumps(summary,indent=2))


if __name__=='__main__':main()
