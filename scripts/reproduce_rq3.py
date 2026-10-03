#!/usr/bin/env python3
"""Reconstruct the current six-task RQ3 means and population SD, entirely offline."""
from __future__ import annotations
import argparse
from collections import Counter
import csv
from fractions import Fraction
import hashlib
import json
import math
from pathlib import Path
import statistics

ROOT = Path(__file__).resolve().parents[1]
SPECIAL_ID = 'gpio_output__deepseek-v4-pro__C4_structured__r03'
REPLACEMENT_ID = 'crazyflie__deepseek-v4-pro__B_chatdev__r02'
TASKS = ['gpio_output','gpio_button_debounce','balance_control','discobot','onstep','crazyflie']
METHOD_KEYS = {'Direct-LLM':'Direct_Raw','MetaGPT':'metagpt','ChatDev':'chatdev','StructGen':'structgen','EmbedDev':'C4_structured'}


def read(path):
    return json.loads(path.read_text(encoding='utf-8'))


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def require(condition, message):
    if not condition:
        raise ValueError(message)


def close(a,b):
    return math.isclose(a,b,rel_tol=0,abs_tol=1e-9)


def local(relative):
    rel = Path(relative)
    require(not rel.is_absolute() and '..' not in rel.parts, 'Unsafe bundled path.')
    target = (ROOT/rel).resolve()
    require(target.is_relative_to(ROOT.resolve()), 'Bundled path leaves the release.')
    return target


def load_and_recompute():
    provenance = read(ROOT/'rq3_export_provenance.json')
    for relative,expected in provenance['exported_files'].items():
        require(sha(local(relative))==expected,'Bundled data hash mismatch: '+relative)
    data = read(ROOT/'data/rq3/display_cells.json')
    positions = read(ROOT/'data/rq3/position_scores.json')['positions']
    protocol = read(ROOT/'data/rq3/protocol.json')
    require(data['task_order']==TASKS,'Current RQ3 task order differs from the manuscript.')
    require(len(positions)==len({p['identity'] for p in positions})==90,'Expected 90 distinct positions.')
    require(len(data['cells'])==30 and len(data['methods'])==5,'Expected 30 means over six tasks.')
    source_index = read(ROOT/'data/rq3/candidate_sources/index.json')
    candidates = source_index['candidates']
    require(source_index['candidate_count']==len(candidates)==44,'Expected 44 fixed candidates for the three smaller tasks.')
    files = [f for c in candidates for f in c['files']]
    require(source_index['file_count']==len(files)==88,'Expected 88 smaller-task C/header files.')
    for item in files:
        require(sha(local(item['path']))==item['original_sha256']==item['exported_sha256'],
                'Candidate bytes differ from their fixed packet hash.')
    checklists = {task:read(ROOT/('data/rq3/requirements/'+task+'.json')) for task in TASKS[:3]}
    require(sum(len(c['items']) for c in checklists.values())==31,'Expected 31 smaller-task requirements.')
    counts = read(ROOT/'data/rq3/new_task_review_counts.json')
    require(len(counts['rows'])==45,'Expected 45 smaller-task positions.')
    by_id = {p['identity']:p for p in positions}
    judgment_total = 0
    for review in counts['rows']:
        position = by_id[review['identity']]
        if review['artifact_available']:
            require([j['id'] for j in review['judgments']]==[i['id'] for i in checklists[review['task']]['items']],
                    'Retained judgments differ from the fixed requirement order.')
            observed = Counter(j['verdict'] for j in review['judgments'])
            require({k:observed[k] for k in ('P','F','U','N/A')}==review['counts']==position['counts'],
                    'Item verdicts disagree with position counts.')
            judgment_total += len(review['judgments'])
        else:
            require(review['judgments']==[] and review['counts'] is None,
                    'A historical no-code record must not acquire fabricated judgments.')
    require(judgment_total==457,'Expected 457 retained smaller-task judgments.')
    source_counts = Counter()
    for position in positions:
        kind,score = position['source_type'],position['display_score_percent']
        source_counts[kind] += 1
        require(position['model']==data['model'],'Generation model mismatch.')
        require(math.isfinite(score) and 0<=score<=100,'Invalid position score.')
        if kind=='code_bound_static_review':
            c = position['counts']
            require(set(c)=={'P','F','U','N/A'} and all(isinstance(v,int) and v>=0 for v in c.values()),
                    'Invalid count fields.')
            den = c['P']+c['F']+c['U']
            require(den>0 and den==position['denominator'],'Review denominator mismatch.')
            require(close(score,100*c['P']/den),'Review score mismatch.')
            require(position['review_binding'] is not None and bool(position['candidate_code_sha256']),
                    'A reviewed position requires code and review bindings.')
        elif kind=='no_code_position':
            require(score==0 and not position['artifact_available'] and position['counts'] is None,
                    'Invalid no-code observation.')
        elif kind=='author_reported_rerun_score':
            require(position['identity']==SPECIAL_ID and score==100,'Unexpected author-reported score.')
            require(position['review_binding'] is None and position['counts'] is None
                    and position['locally_bound_review'] is False
                    and position['author_report']['local_verification_completed'] is False,
                    'The GPIO author-reported rerun must retain its missing local bindings.')
        else:
            raise ValueError('Unknown source: '+kind)
    require(source_counts=={'code_bound_static_review':81,'no_code_position':8,'author_reported_rerun_score':1},
            'Unexpected current source counts.')
    replacement = by_id[REPLACEMENT_ID]
    require(replacement['counts']=={'P':46,'F':47,'U':7,'N/A':0} and replacement['display_score_percent']==46,
            'The selected ChatDev replacement must use its fixed 100-case assessment.')
    binding = replacement['review_binding']
    review = read(local(binding['path']))
    require(sha(local(binding['path']))==binding['distributed_sha256'],'Replacement review hash mismatch.')
    require(len(review['cases'])==len({x['id'] for x in review['cases']})==100,'Expected 100 unique replacement cases.')
    verdicts = Counter(x['verdict'] for x in review['cases'])
    require(dict(verdicts)=={'pass':46,'fail':47,'uncertain':7},'Replacement verdicts disagree with 46/100.')
    require(len(replacement['candidate_code_sha256'])==12,'Expected twelve unchanged replacement source files.')
    for path,digest in replacement['candidate_code_sha256'].items():
        require(sha(local(path))==digest,'Replacement candidate file hash mismatch.')
    rq1 = read(ROOT/'data/rq1/identity_metrics.json')['rows']
    for p in positions:
        if p['task'] not in TASKS[3:]:
            continue
        matches = [r for r in rq1 if r['project']==p['task'] and r['method']==METHOD_KEYS[p['method']]
                   and r['model']==p['model'] and r['repeat']==p['repeat']]
        require(len(matches)==1 and close(p['display_score_percent'],float(100*Fraction(matches[0]['assisted_fraction']))),
                'RQ1 and RQ3 position values disagree: '+p['identity'])
    lookup = {(c['task'],c['method']):c for c in data['cells']}
    require(len(lookup)==30,'Duplicate task/method cell.')
    rows = []
    for task in TASKS:
        for method in data['methods']:
            fixed = lookup[task,method]
            group = sorted((p for p in positions if p['task']==task and p['method']==method),key=lambda p:p['repeat'])
            require([p['repeat'] for p in group]==[1,2,3],'Missing repeat position.')
            values = [p['display_score_percent'] for p in group]
            mean,sd = statistics.mean(values),statistics.pstdev(values)
            require(fixed['n']==3 and all(close(a,b) for a,b in zip(values,fixed['values_percent'])),
                    'Displayed positions differ from saved observations.')
            require(close(mean,fixed['mean_percent']) and close(sd,fixed['population_sd_percentage_points']),
                    'Mean/population SD differs from the fixed display.')
            rows.append({'task':task,'method':method,'protocol':fixed['protocol'],'n':3,
                         'repeat_1_percent':values[0],'repeat_2_percent':values[1],'repeat_3_percent':values[2],
                         'mean_percent':mean,'population_sd_percentage_points':sd,
                         'author_reported_positions':sum(p['identity']==SPECIAL_ID for p in group)})
    return data,protocol,positions,rows


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,default=ROOT/'outputs/rq3',help='Fresh output directory; existing results are not overwritten.')
    parser.add_argument('--plot',action='store_true',help='Also render the six-task PDF/SVG line chart with ReportLab.')
    args = parser.parse_args()
    data,protocol,positions,rows = load_and_recompute()
    output = args.output.resolve()
    names = ['rq3_table.csv','rq3_table.md','rq3_recomputed.json','rq3_validation.json']
    if args.plot:
        names += ['rq3_static_support.pdf','rq3_static_support.svg']
    require(not any((output/name).exists() for name in names),'Output exists; choose a fresh --output directory.')
    output.mkdir(parents=True,exist_ok=True)
    with (output/'rq3_table.csv').open('x',encoding='utf-8',newline='') as stream:
        writer=csv.DictWriter(stream,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
    with (output/'rq3_table.md').open('x',encoding='utf-8',newline='\n') as stream:
        stream.write('| Task | Method | Mean (%) | Population SD (pp) | n |\n|---|---|---:|---:|---:|\n')
        for row in rows:
            stream.write(f"| {row['task']} | {row['method']} | {row['mean_percent']:.2f} | {row['population_sd_percentage_points']:.2f} | 3 |\n")
    (output/'rq3_recomputed.json').write_text(json.dumps({'cells':rows,'protocol':protocol},indent=2)+'\n',encoding='utf-8')
    geometry = None
    if args.plot:
        from render_rq3 import plot
        geometry=plot(rows,data,protocol,output)
    validation = {'status':'offline_recomputed_from_bundled_scores','cells':30,'positions':90,
                  'means_and_population_sd_match_fixed_display':True,'population_sd_divisor':3,
                  'counts_checked_for_locally_bound_reviews':81,'retained_no_code_positions':8,
                  'fixed_smaller_task_candidates_verified':44,'exact_smaller_task_code_files_verified':88,
                  'fixed_whole_requirement_items_verified':31,'retained_smaller_task_item_verdicts_checked':457,
                  'replacement_testcase_verdicts_checked':100,'exact_replacement_code_files_verified':12,
                  'rq1_shared_positions_cross_checked':45,'author_reported_replacement_positions':1,
                  'author_reported_gpio_position_locally_verified':False,
                  'replacement_human_review':'author-attested; not independently witnessed by these scripts',
                  'new_model_calls':0,'new_host_runs':0,'new_semantic_judgments':0,
                  'input_sha256':{p.relative_to(ROOT).as_posix():sha(p) for p in
                     [ROOT/'data/rq3/display_cells.json',ROOT/'data/rq3/position_scores.json',ROOT/'data/rq3/protocol.json',ROOT/'rq3_export_provenance.json']},
                  'plot':geometry}
    (output/'rq3_validation.json').write_text(json.dumps(validation,indent=2)+'\n',encoding='utf-8')
    print(json.dumps({'status':validation['status'],'cells':30,'positions':90,'plot_created':args.plot,'new_experiments':0}))


if __name__=='__main__':
    main()
