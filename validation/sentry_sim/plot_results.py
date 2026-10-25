"""Export actual simulator paths and complete command traces with Matplotlib."""
import argparse
import json
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
from scene import boxes


def plot(directory):
    result=json.loads((directory/'result.json').read_text())
    if not (directory/'commands.csv').stat().st_size or not (directory/'body.csv').stat().st_size:
        print('No motion data:',directory)
        return
    body=np.loadtxt(directory/'body.csv',delimiter=',',ndmin=2)
    command=np.loadtxt(directory/'commands.csv',delimiter=',',ndmin=2)
    if not len(command):
        return
    start=command[0,0]
    # Gazebo's odometry plugin can emit an initialization velocity pulse before
    # navigation. Keep it in raw CSV; plot the complete commanded interval here.
    active_body=body[(body[:,0]>=start)&(body[:,0]<=command[-1,0])]
    figure,axes=plt.subplots(2,2,figsize=(12,8),constrained_layout=True)
    path=axes[0,0]
    world=ET.parse(directory/'maps/scene.sdf').getroot().find('world')
    for matrix,size in boxes(world):
        xy=np.array([[-size[0]/2,-size[1]/2],[size[0]/2,-size[1]/2],
                     [size[0]/2,size[1]/2],[-size[0]/2,size[1]/2]])
        path.add_patch(Polygon(xy@matrix[:2,:2].T+matrix[:2,2],facecolor='#dadada',edgecolor='#777'))
    path.plot(body[:,1],body[:,2],color='#176b8f',label='Gazebo body')
    path.plot(result['original_goal']['x'],result['original_goal']['y'],'*',color='#dc6b30',markersize=12,label='Original goal')
    path.set(xlim=(-.6,4.6),ylim=(-3.1,-.9),xlabel='x (m)',ylabel='y (m)',title='Preserved room + passage fixtures')
    path.set_aspect('equal');path.legend(loc='lower left')
    speed=np.linalg.norm(command[:,1:3],axis=1)
    dt=np.diff(command[:,0])
    acceleration=np.diff(command[:,1:3],axis=0)/dt[:,None]
    jerk=np.diff(acceleration,axis=0)/((dt[1:]+dt[:-1])/2)[:,None]
    axes[0,1].plot(command[:,0]-start,speed,label='Sent speed',color='#176b8f')
    axes[0,1].plot(active_body[:,0]-start,np.linalg.norm(active_body[:,4:6],axis=1),label='Gazebo measured speed',alpha=.7,color='#dc6b30')
    axes[0,1].set(ylabel='m/s',title='Navigation speed (startup excluded)');axes[0,1].legend()
    axes[1,0].plot(command[1:,0]-start,np.linalg.norm(acceleration,axis=1),color='#176b8f')
    axes[1,0].set(ylabel='m/s²',title='Complete acceleration magnitude')
    axes[1,1].plot(command[2:,0]-start,np.linalg.norm(jerk,axis=1),color='#176b8f')
    axes[1,1].set(ylabel='m/s³',title='Complete jerk magnitude')
    for axis in (axes[0,1],axes[1,0],axes[1,1]):
        axis.set_xlabel('Time since first command (s)');axis.grid(alpha=.2)
        for event in result.get('safety_stops',[]):
            axis.axvline(event['time']-start,color='#c33',alpha=.45,linewidth=.8)
    figure.suptitle(f"{result['case']} | passage {result['width_m']*100:.0f} cm | "
                   f"passed={result['passed']} | safety stops={len(result.get('safety_stops',[]))}")
    figure.savefig(directory/'motion.png',dpi=160)
    figure.savefig(directory/'motion.svg')
    plt.close(figure)
    print(directory/'motion.png')
    throat=active_body[(active_body[:,1]>=1.)&(active_body[:,1]<=3.)]
    if len(throat):
        hull=float(world.find("./model[@name='shaoshao']/link/collision/geometry/cylinder/radius").text)
        margin=(result['width_m']/2-hull)*1000
        figure,axes=plt.subplots(2,1,figsize=(10,6),sharex=True,constrained_layout=True)
        axes[0].plot(throat[:,1],(throat[:,2]+2)*1000,color='#176b8f',label='Measured center offset')
        axes[0].axhline(0,color='#555',linestyle=':',label='Passage center')
        for sign in (-1,1):
            axes[0].axhline(sign*margin,color='#c33',linestyle='--',
                           label='Physical hull limit (zero padding)' if sign==1 else None)
        axes[0].set(ylabel='Lateral offset (mm)',title='Actual body inside passage')
        axes[0].legend()
        axes[1].plot(throat[:,1],np.linalg.norm(throat[:,4:6],axis=1),color='#dc6b30')
        axes[1].set(xlabel='Body x (m)',ylabel='Body speed (m/s)',title='Measured passage speed, including stops',
                    ylim=(0.,max(.1,float(np.linalg.norm(throat[:,4:6],axis=1).max())*1.15)))
        axes[1].ticklabel_format(axis='y',style='plain',useOffset=False)
        for axis in axes:
            axis.grid(alpha=.2)
        figure.savefig(directory/'passage.png',dpi=160)
        figure.savefig(directory/'passage.svg')
        plt.close(figure)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directories',nargs='+',type=Path)
    args=parser.parse_args()
    for directory in args.directories:
        plot(directory)
