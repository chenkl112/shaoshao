"""Physical teleport tests with blind/reset/tracked local odometry.

Relocation coordinates go only to Gazebo and the evaluation ledger. No test
injects an AMCL pose for these cases; production code searches the saved map.
"""
import argparse
import json
from pathlib import Path
import subprocess
import sys

CASES=[
    ('kidnap-blind-right',.8,['--odom-mode','blind','--relocate-to','4','-3','-1.1']),
    ('kidnap-reset-turn',.8,['--odom-mode','reset','--relocate-to','0','-.5','2.8']),
    ('kidnap-tracked-left',.8,['--odom-mode','tracked','--relocate-to','-3.8','-3.1','1.57']),
    ('kidnap-narrow65',.65,['--odom-mode','blind','--relocate-to','2','-2','1.57']),
    ('kidnap-twice',.8,['--odom-mode','blind','--relocate-to','4','-3','-1.1',
                       '--repeat-kidnap','-3.8','-3.1','1.57']),
    ('kidnap-cancel',.8,['--cancel-after-kidnap']),
    ('kidnap-no-scan',.8,['--hide-scan-after-kidnap']),
]


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();root=args.output.resolve();root.mkdir(parents=True,exist_ok=True)
    results=[]
    for name,width,extra in CASES:
        print('Running',name,flush=True)
        directory=root/name
        with (root/(name+'.stdout.log')).open('w') as log:
            process=subprocess.run(['bash',str(Path(__file__).parent/'run.sh'),'--case','kidnap','--width',str(width),
                                    '--timeout','150','--output',str(directory),*extra],stdout=log,stderr=subprocess.STDOUT)
        path=directory/'result.json'
        result=json.loads(path.read_text()) if path.is_file() else {'passed':False,'error':'No result file'}
        results.append({'name':name,'exit_code':process.returncode,**result})
        (root/'suite.json').write_text(json.dumps({'passed':all(r['passed'] for r in results),
                                                'results':results},ensure_ascii=False,indent=2)+'\n')
        print(name,'PASS' if result['passed'] else 'FAIL',result.get('error',''),flush=True)
    sys.exit(0 if all(r['passed'] for r in results) else 1)
