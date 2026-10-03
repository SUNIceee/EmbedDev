"""Recompute RQ1, RQ2 and RQ4 from saved per-position observations; offline only."""
from pathlib import Path
from fractions import Fraction
from collections import Counter
import argparse
import csv
import hashlib
import json
import math
import statistics

ROOT=Path(__file__).resolve().parents[1]
NAMES={'Direct_Raw':'Direct-LLM','metagpt':'MetaGPT','chatdev':'ChatDev','structgen':'StructGen','C4_structured':'EmbedDev','C5_api_repair':'Direct generation + repair','C6_plain_plan':'Prose planning','A_no_state':'Without state view','A_no_behavior':'Without behavior view','A_no_trace_gate':'Without extra trace feedback'}

def read(p):return json.loads((ROOT/p).read_text(encoding='utf-8'))
def checked_file(reference):
    relative=Path(reference['path'])
    assert not relative.is_absolute() and '..' not in relative.parts
    path=ROOT/relative
    assert path.is_file(),reference['path']
    assert hashlib.sha256(path.read_bytes()).hexdigest()==reference['sha256'],reference['path']
    return path
def near(x,y):
    if not math.isclose(x,y,rel_tol=1e-11,abs_tol=1e-9):raise ValueError(f'Metric mismatch: {x} != {y}')
def save_csv(out,name,rows):
    with (out/name).open('w',encoding='utf-8',newline='') as f:
        writer=csv.DictWriter(f,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)

def requirement_coverage(rq1_rows):
    """Recompute current coverage from frozen leaf judgments, not display values."""
    coverage=read('data/rq1/requirement_coverage.json')
    rows=coverage['rows'];by_id={r['id']:r for r in rows}
    assert len(rows)==len(by_id)==135
    assert set(by_id)=={r['id'] for r in rq1_rows}
    checklists={}
    for project,reference in coverage['checklists'].items():
        checklist=json.loads(checked_file(reference).read_text(encoding='utf-8'))
        checked_file(reference['source_requirement'])
        leaves={item['id']:item for item in checklist['requirements']}
        assert len(leaves)==reference['leaf_count']
        static={key for key,item in leaves.items() if item['static_reviewability']['classification']=='static_evidence_reviewable'}
        assert len(static)==reference['static_denominator']
        checklists[project]=(leaves,static)
    checked_file(coverage['additional_candidate_bindings'])
    shares={};reviewed=0;no_code=0;leaf_judgments=0
    allowed={'fully_covered','partially_covered','not_covered','uncertain','unassessed'}
    for rq1 in rq1_rows:
        row=by_id[rq1['id']]
        assert (row['method_key'],row['model'],row['project'],row['repeat'])==(rq1['method'],rq1['model'],rq1['project'],rq1['repeat'])
        assert row['delivered']==rq1['delivered'],row['id']
        review=json.loads(checked_file(row['review']).read_text(encoding='utf-8'))
        assert review['identity']==row['id']
        items=review.get('requirements',[])
        counts=dict(Counter(item['verdict'] for item in items))
        assert set(counts)<=allowed and counts==row['counts']==review['counts'],row['id']
        for binding in row['candidate_bindings']:
            checked_file({'path':binding['path'],'sha256':binding['distributed_sha256']})
        if not row['delivered']:
            no_code+=1
            assert not items and not row['candidate_bindings'] and row['static_denominator'] is None
            share=Fraction()
        else:
            reviewed+=1
            leaves,static=checklists[row['project']]
            assert len(items)==len(leaves) and {item['id'] for item in items}==set(leaves),row['id']
            assert review['checklist_sha256']==coverage['checklists'][row['project']]['original_sha256']
            for item in items:
                assert (item['verdict']=='unassessed')==(item['id'] not in static),(row['id'],item['id'])
            denominator=sum(value for verdict,value in counts.items() if verdict!='unassessed')
            assert denominator==len(static)==row['static_denominator']==review['denominator'],row['id']
            # Full and partial remain separate verdicts. The current manuscript
            # counts either as present implementation evidence, without half credit.
            share=Fraction(counts.get('fully_covered',0)+counts.get('partially_covered',0),denominator)
            leaf_judgments+=len(items)
        assert share==Fraction(row['position_contribution_fraction']),row['id']
        near(float(100*share),row['position_contribution_percent'])
        shares[row['id']]=share
    assert (reviewed,no_code)==(coverage['reviewed_code_positions'],coverage['no_code_positions'])==(108,27)
    groups={(g['method_key'],g['model']):g for g in coverage['expected_groups']}
    assert len(groups)==15
    return shares,groups,{'reviewed_positions':reviewed,'no_code_positions':no_code,'leaf_judgments_checked':leaf_judgments}

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir',type=Path,default=ROOT/'results')
    args=parser.parse_args();out=args.output_dir;out.mkdir(parents=True,exist_ok=True)
    d=read('data/rq1/identity_metrics.json');expected=read('data/rq1/expected_statistics.json');rows=d['rows']
    assert len(rows)==135 and len({r['id'] for r in rows})==135
    for r in rows:
        if not r['delivered']:
            assert Fraction(r['assisted_fraction'])==Fraction(r['host_gated_fraction'])==0
            continue
        counts=r['fresh_static_counts'] if r['fresh_static_counts'] is not None else r['old_static_counts']
        assert counts is not None
        denom=sum(counts.get(k,0) for k in ['pass','fail','uncertain'])
        q=Fraction(counts.get('pass',0),denom) if denom else Fraction()
        assert q==Fraction(r['assisted_fraction']),r['id']
        if not r['host_all_pass']:assert Fraction(r['host_gated_fraction'])==0
    coverage_shares,coverage_groups,coverage_checks=requirement_coverage(rows)
    result=[]
    for e in expected['rows']:
        selected=[r for r in rows if (r['method'],r['model'])==(e['method'],e['model'])]
        assert len(selected)==9
        blocks={'Host':[],'Gated':[],'Static':[],'Coverage':[]}
        for repeat in [1,2,3]:
            block=[r for r in selected if r['repeat']==repeat]
            assert len(block)==3 and len({r['project'] for r in block})==3
            blocks['Host'].append(100*sum(r['host_all_pass'] is True for r in block)/3)
            for label,key in [('Gated','host_gated_fraction'),('Static','assisted_fraction')]:
                blocks[label].append(float(100*sum((Fraction(r[key]) for r in block),Fraction())/3))
            blocks['Coverage'].append(float(100*sum((coverage_shares[r['id']] for r in block),Fraction())/3))
        row={'method':NAMES[e['method']],'model':e['model'],'positions':9}
        for key,values in blocks.items():
            mean=statistics.mean(values);sd=statistics.pstdev(values)
            if key=='Coverage':
                group=coverage_groups[(e['method'],e['model'])]
                near(mean,group['mean_percent']);near(sd,group['population_sd_pp'])
                near(mean,float(Fraction(group['mean_fraction_percent'])))
                for a,b in zip(values,group['repeat_block_means_percent']):near(a,b)
            else:
                near(mean,e['mean'][key]);near(sd,e['population_sd'][key])
                for a,b in zip(values,e['repeat_values'][key]):near(a,b)
            row[key.lower()+'_mean_percent']=mean;row[key.lower()+'_population_sd_pp']=sd
        result.append(row)
    save_csv(out,'rq1_table.csv',result)
    r2=read('data/rq2/identity_metrics.json');result=[]
    assert len(r2['rows'])==54 and len({r['id'] for r in r2['rows']})==54
    for g in r2['expected_groups']:
        selected=[r for r in r2['rows'] if r['method']==g['method']]
        n=len(selected);code=sum(r['delivery'] for r in selected);host=sum(r['host']['all_pass'] is True for r in selected)
        assert (n,code,host)==(g['n'],g['delivered'],g['host_all'])
        result.append({'condition':NAMES[g['method']],'positions':n,'delivered':code,'host_all_pass':host,'delivery_percent':100*code/n,'host_success_percent':100*host/n})
    save_csv(out,'rq2_table.csv',result)
    r4=read('data/rq4/identity_endpoints.json');result=[]
    assert len(r4['rows'])==162 and len({r['id'] for r in r4['rows']})==162
    categories=['no_code','candidate_build_failed','harness_or_link_failed','executed_assertion_failed','host_all_pass']
    for g in r4['expected_groups']:
        selected=[r for r in r4['rows'] if r['method']==g['method']];counts=Counter(r['exclusive_endpoint'] for r in selected)
        assert set(counts)<=set(categories) and len(selected)==g['planned']==27
        for k in categories:assert counts[k]==g[k]
        result.append({'condition':NAMES[g['method']],'positions':27,**{k:counts[k] for k in categories}})
    save_csv(out,'rq4_failure_endpoints.csv',result)
    audit={'status':'passed','rq1_positions':135,'rq1_method_model_rows':15,'rq1_mean_sd_cells':60,'rq1_sd':'population SD across three equally weighted project-mean repeat blocks','requirement_coverage':{'metric':'(fully_covered + partially_covered) / fixed static denominator',**coverage_checks},'rq2_positions':54,'rq2_conditions':6,'rq4_positions':162,'rq4_conditions':6,'new_model_calls':0,'new_candidate_executions':0,'interpretation':'Reconstruction of saved observations, not new judgments or regeneration.'}
    (out/'table_verification.json').write_text(json.dumps(audit,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(audit))

if __name__=='__main__':main()
