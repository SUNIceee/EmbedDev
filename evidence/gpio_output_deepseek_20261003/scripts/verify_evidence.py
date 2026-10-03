"""Offline evidence and byte-integrity checks. No network, compilation, or execution."""
from pathlib import Path,PurePosixPath
import hashlib
import json

ROOT=Path(__file__).resolve().parents[1]

def no_duplicate_keys(pairs):
    out={}
    for key,value in pairs:
        if key in out:raise ValueError('Duplicate JSON key: '+key)
        out[key]=value
    return out

def read(p):return json.loads(Path(p).read_text(encoding='utf-8-sig'),object_pairs_hook=no_duplicate_keys)
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()

def resolve_member(name):
    part=PurePosixPath(name)
    if part.is_absolute() or '..' in part.parts or '\\' in name or ':' in name:raise ValueError('Unsafe member path')
    p=ROOT.joinpath(*part.parts)
    for q in (p,*p.parents):
        if q==ROOT.parent:break
        if q.is_symlink():raise ValueError('Symlink member')
    p.resolve().relative_to(ROOT)
    return p

def verify():
    manifest=read(ROOT/'manifest.sha256.json');files=manifest['files']
    if not files:raise ValueError('Empty manifest')
    for name,expected in files.items():
        p=resolve_member(name)
        if not p.is_file() or sha(p)!=expected:raise ValueError('Hash mismatch: '+name)
    actual={p.relative_to(ROOT).as_posix() for p in ROOT.rglob('*') if p.is_file() and
            'local_outputs' not in p.relative_to(ROOT).parts and '__pycache__' not in p.parts}
    if actual!=set(files)|{'manifest.sha256.json'}:raise ValueError('Unlisted or missing distributed file')
    packet=read(ROOT/'evidence/candidate_packet.json')
    for name,body in packet['files'].items():
        p=resolve_member('candidate/'+name)
        if p.read_bytes()!=body.encode('utf-8') or sha(p)!=packet['file_sha256'][name]:raise ValueError('Candidate bytes differ')
    provenance=read(ROOT/'provenance.json')
    for row in provenance['distributed_artifacts']:
        if sha(resolve_member(row['distributed_path']))!=row['distributed_sha256']:raise ValueError('Distributed provenance mismatch')
        if row['transformation']=='byte-identical copy' and row['original_sha256']!=row['distributed_sha256']:
            raise ValueError('Byte-identical provenance claim mismatch')
    review=read(ROOT/'review/static_review.json');checklist=read(ROOT/'review/checklist.json')
    if sha(ROOT/review['checklist_binding']['path'])!=review['checklist_binding']['sha256']:raise ValueError('Checklist binding mismatch')
    ids=['GO-R%02d'%i for i in range(1,9)]
    if [x['id'] for x in review['items']]!=ids or [x['id'] for x in checklist['items']]!=ids:raise ValueError('Requirement IDs mismatch')
    for item,requirement in zip(review['items'],checklist['items']):
        if item['requirement_original_text']!=requirement['requirement_original_text']:raise ValueError('Requirement text changed')
        if item['verdict']!='P' or item['human_reviewed'] is not False:raise ValueError('Recorded review outcome changed')
    excerpts=0
    def walk(value):
        nonlocal excerpts
        if isinstance(value,dict):
            if {'file','file_sha256','line_start_1based','line_end_1based_inclusive','text'}<=value.keys():
                p=ROOT/'candidate'/value['file']
                if sha(p)!=value['file_sha256']:raise ValueError('Review file hash mismatch')
                lines=p.read_text(encoding='utf-8').splitlines()
                excerpt='\n'.join(lines[value['line_start_1based']-1:value['line_end_1based_inclusive']])
                if excerpt!=value['text'].replace('\r\n','\n'):raise ValueError('Review excerpt mismatch')
                excerpts+=1
            for x in value.values():walk(x)
        elif isinstance(value,list):
            for x in value:walk(x)
    walk(review)
    for contract in review['contract_bindings']:
        if sha(ROOT/contract['path'])!=contract['sha256']:raise ValueError('Contract binding mismatch')
    host=read(ROOT/'evidence/host_report.json')
    if host['compile_pass'] is not False or host['executed_scenarios']!=0 or host['total_scenarios']!=10:
        raise ValueError('Recorded Host outcome changed')
    for name,digest in host['suite_sha256'].items():
        if sha(ROOT/'tests'/name)!=digest:raise ValueError('Host suite changed')
    for name,digest in host['code_sha256'].items():
        if sha(ROOT/'candidate'/name)!=digest:raise ValueError('Host candidate binding mismatch')
    summary=read(ROOT/'summary.json')
    if summary['accounting']['logical_attempt_physical_requests']!=7 or summary['accounting']['logical_attempt_reported_tokens']!=135348:
        raise ValueError('Logical-attempt accounting changed')
    return {'verified':True,'distributed_files':len(files),'static_items':8,'checked_code_excerpts':excerpts,
            'recorded_compilation':'failed','recorded_executed_host_scenarios':0,'model_calls':0,'compilations':0,'host_runs':0}

if __name__=='__main__':print(json.dumps(verify(),indent=2))
