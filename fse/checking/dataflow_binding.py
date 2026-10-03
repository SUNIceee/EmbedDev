"""Conservative binding of explicit S/I/B fields to a monitored obligation.

This narrow compiler accepts one object, two unconditional ordered steps and
an explicit adapter cycle mapping. It does not infer aliases or parse prose.
Acceptance means model consistency, never trusted instrumentation coverage.
"""
from dataclasses import asdict
from .dataflow_obligation import DataflowObligation

def bind_obligation(state,interface,behavior,selection):
    def unknown(reason):return dict(status='inconclusive',reason=reason)
    if any(not isinstance(x,dict) for x in (state,interface,behavior,selection)):return unknown('Expected canonical model objects')
    keys=('requirement','object','producer','consumer','output','scenario','adapter_cycle')
    if any(not isinstance(selection.get(k),str) or not selection[k] for k in keys):
        return unknown('Explicit identifiers and adapter cycle required')
    if selection.get('same_cycle') is not True:return unknown('Same-cycle interpretation not declared')
    requirement,obj,producer,consumer,output,scenario,cycle=(selection[k] for k in keys)
    def unique(rows,key,value):
        found=[r for r in rows if isinstance(r,dict) and r.get(key)==value]
        return found[0] if len(found)==1 else None
    try:
        variable=unique(state['variables'],'name',obj)
        boundary=unique(state['variables'],'name',output)
        p=unique(interface['functions'],'name',producer)
        c=unique(interface['functions'],'name',consumer)
        flow=unique(behavior['scenarios'],'name',scenario)
        if any(x is None for x in (variable,boundary,p,c,flow)):return unknown('Missing or ambiguous model identifier')
        if producer==consumer or obj==output:return unknown('In-place or single-function flows require another binding profile')
        if variable.get('writers')!=[producer]:return unknown('Producer is not the unique declared object writer')
        if boundary.get('writers')!=[consumer]:return unknown('Consumer is not the unique declared output writer')
        if boundary.get('externally_visible') is not True:return unknown('Output is not declared externally visible')
        if obj not in p.get('writes_state',[]) or obj not in c.get('reads_state',[]) or output not in c.get('writes_state',[]):
            return unknown('Interface does not declare the selected write/read/output chain')
        if any(requirement not in item.get('source_requirements',[]) for item in (variable,boundary,p,c,flow)):
            return unknown('Requirement anchor is not shared by the selected model elements')
        steps=flow['steps']
        if len(steps)!=2 or any(not isinstance(s,dict) for s in steps):return unknown('Only explicit two-step flows are supported')
        if any(type(s.get('order'))!=int or s.get('condition') or s.get('step_conditions') for s in steps):
            return unknown('Conditional or unordered steps require explicit path binding')
        ordered=sorted(steps,key=lambda s:s['order'])
        if ordered[0]['order']==ordered[1]['order'] or [s.get('function') for s in ordered]!=[producer,consumer]:
            return unknown('Producer does not strictly precede consumer')
        return dict(status='bound',obligation=asdict(DataflowObligation(requirement,producer,consumer,output)),
                    binding=dict(object=obj,scenario=scenario,adapter_cycle=cycle,same_cycle=True),
                    scope='Explicit model consistency only; adapter must independently establish complete event coverage')
    except (KeyError,TypeError,ValueError,AttributeError):return unknown('Malformed or unsupported canonical model fields')
