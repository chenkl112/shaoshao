"""Offline held-out recorded scans against each uploaded saved PCD.

This is not a new MID360 recording or closed-loop hardware test. The exact
query frame is excluded from retrieval. Its recorded map pose is only used
after search, for evaluation; no current/last map pose enters search.
"""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import numpy as np

SOURCE=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(SOURCE/'web_panel'))
from pointcloud_relocalization import PointCloudMatcher
from pointcloud_index import delta,transform


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--maps-root',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();results=[]
    for manifest in sorted(args.maps_root.glob('**/keyframes/manifest.json')):
        root=manifest.parent.parent
        if not (root/'site.pcd').is_file():continue
        result={'map':str(root),'queries':[],
                'map_sha256':hashlib.sha256((root/'site.pcd').read_bytes()).hexdigest(),
                'manifest_sha256':hashlib.sha256(manifest.read_bytes()).hexdigest()}
        try:
            meta=json.loads(manifest.read_text());matcher=PointCloudMatcher(root/'site.yaml')
            result.update(frames=len(meta['frames']),pcd_points=len(matcher.points),voxel_points=matcher.target.size())
            if len(meta['frames'])<2:
                result['state']='insufficient_independent_keyframes';results.append(result);continue
            original=(matcher.index.contexts.copy(),matcher.index.poses.copy(),matcher.index.keys.copy())
            for frame_index in sorted(set([0,len(meta['frames'])//2,len(meta['frames'])-1])):
                with np.load(manifest.parent/meta['frames'][frame_index]['file'],allow_pickle=False) as data:
                    points=data['points'];recorded=data['pose']
                keep=np.arange(len(meta['frames']))!=frame_index
                matcher.index.contexts,matcher.index.poses,matcher.index.keys=[a[keep] for a in original]
                # Simulate a different orientation without injecting a pose.
                yaw=1.234;rotate=np.eye(4);c,s=np.cos(yaw),np.sin(yaw)
                rotate[:2,:2]=[[c,-s],[s,c]]
                query=transform(points,np.linalg.inv(rotate))
                found=matcher.search(query,np.eye(4),.297658)
                entry={'frame':frame_index,'held_out':True,**found}
                if found['state']=='candidate':
                    position,angle=delta(recorded@rotate,np.asarray(found['sensor_pose']))
                    entry.update(position_error_m=position,rotation_error_rad=angle,
                                 correctly_recovered=position<.15 and angle<.12)
                result['queries'].append(entry)
            result['state']='evaluated'
        except Exception as exc:result.update(state='map_error',detail=str(exc))
        results.append(result)
        print(root.name,result['state'],[q['state'] for q in result['queries']],flush=True)
    args.output.parent.mkdir(parents=True,exist_ok=True)
    sources=['web_panel/pointcloud_relocalization.py','web_panel/pointcloud_index.py','validation/sentry_sim/check_saved_clouds.py']
    args.output.write_text(json.dumps({'kind':'offline_saved_mapping_scans',
        'source_sha256':{name:hashlib.sha256((SOURCE/name).read_bytes()).hexdigest() for name in sources},
        'results':results},ensure_ascii=False,indent=2)+'\n')


if __name__=='__main__':main()
