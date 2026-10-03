"""Opt-in local compilation of a separate exact copy. Never runs candidate code."""
from datetime import datetime,timezone
from pathlib import Path
import argparse
import json
import os
import shutil
import subprocess
from verify_evidence import ROOT,read,sha,verify

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--run',action='store_true',help='Perform the local compiler invocation after integrity checks.')
    p.add_argument('--compiler',default='gcc',help='Locally installed GCC executable; no download is performed.')
    args=p.parse_args();verify()
    recorded=read(ROOT/'evidence/compile_result.json')['translation_units'][0]
    command=[args.compiler,*recorded['command'][1:]]
    if not args.run:
        print(json.dumps({'will_compile':False,'command':command,'expected_result':'NULL undeclared; no Host execution'},indent=2));return 0
    package_root=ROOT.parent.parent
    in_replication_repo=ROOT.parent.name=='evidence' and (package_root/'MANIFEST.json').is_file()
    output_base=(package_root/'outputs/gpio_evidence_compile') if in_replication_repo else ROOT/'local_outputs'
    display_root=package_root if in_replication_repo else ROOT
    output=output_base/datetime.now(timezone.utc).strftime('compile_%Y%m%dT%H%M%S%fZ')
    output.mkdir(parents=True,exist_ok=False)
    for source in (ROOT/'candidate').iterdir():
        if source.name not in ('6_generated_code.c','6_generated_code.h'):raise ValueError('Unexpected candidate member')
        shutil.copyfile(source,output/source.name)
        if sha(source)!=sha(output/source.name):raise ValueError('Copy changed candidate bytes')
    env={k:v for k,v in os.environ.items() if k.upper() in ('PATH','SYSTEMROOT','WINDIR','TEMP','TMP','PATHEXT')}
    env['LC_ALL']='C'
    version=subprocess.run([args.compiler,'--version'],capture_output=True,text=True,errors='replace',env=env,timeout=15)
    result=subprocess.run(command,cwd=output,capture_output=True,text=True,errors='replace',env=env,timeout=60)
    combined=result.stdout+result.stderr
    matches=result.returncode!=0 and 'NULL' in combined and 'undeclared' in combined
    report={'command':command,'compiler_version':version.stdout.splitlines()[0] if version.stdout else '',
        'returncode':result.returncode,'stdout':result.stdout,'stderr':result.stderr,
        'recorded_failure_reproduced':matches,'candidate_modified':False,'host_scenarios_executed':0,
        'candidate_executed':False,'model_calls':0,'network_requests':0}
    (output/'result.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    print(json.dumps({'recorded_failure_reproduced':matches,'report':output.relative_to(display_root).as_posix()+'/result.json',
                      'meaning':'Compilation-failure reproduction, not successful compilation or Host success.'},indent=2))
    return 0 if matches else 2

if __name__=='__main__':raise SystemExit(main())
