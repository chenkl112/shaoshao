"""Run isolated Gazebo cases sequentially and retain every result/log."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys

CASES=[('smooth80','smooth',.8,0.),('narrow75','narrow',.75,0.),('narrow65','narrow',.65,0.),
       ('narrow65-offset','narrow',.65,.04),('reject58','reject',.58,0.),
       ('stuck80','stuck',.8,0.),('localization80','localization',.8,0.),('cancel80','cancel',.8,0.),
       ('slow-status80','smooth',.8,0.)]


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();root=args.output.resolve();root.mkdir(parents=True,exist_ok=True)
    results=[]
    for name,case,width,offset in CASES:
        directory=root/name
        print('Running',name,flush=True)
        with (root/(name+'.stdout.log')).open('w') as log:
            process=subprocess.run(['bash',str(Path(__file__).parent/'run.sh'),'--case',case,'--width',str(width),
                                    '--start-offset',str(offset),'--state-delay',str(.25 if name=='slow-status80' else 0.),
                                    '--timeout','90','--output',str(directory)],stdout=log,stderr=subprocess.STDOUT)
        path=directory/'result.json'
        result=json.loads(path.read_text()) if path.is_file() else {'passed':False,'error':'No result file'}
        results.append({'name':name,'exit_code':process.returncode,**result})
        (root/'suite.json').write_text(json.dumps({'passed':all(x['passed'] for x in results),
                                                'results':results},ensure_ascii=False,indent=2)+'\n')
        print(name,'PASS' if result['passed'] else 'FAIL',result.get('error',''),flush=True)
    sys.exit(0 if all(x['passed'] for x in results) else 1)
