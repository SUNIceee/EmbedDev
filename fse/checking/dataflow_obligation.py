"""Validate a bounded producer/consumer obligation over trusted event traces.

This is a monitor, not a C alias analyser. A trusted adapter must emit every
relevant write, copy, read and publish in execution order. Missing coverage or
malformed traces are inconclusive; function names alone are never evidence.
"""
from dataclasses import dataclass
from typing import Any


@dataclass(frozen=True)
class DataflowObligation:
    requirement: str
    producer: str
    consumer: str
    output: str


def evaluate_trace(obligation: DataflowObligation, trace: dict[str, Any]) -> dict:
    issues = []
    def issue(code, index, cycle, target):
        issues.append(dict(code=code, event=index, cycle=cycle, target=target,
                           requirement=obligation.requirement))
    def unknown(reason):
        return dict(status='inconclusive', reason=reason, issues=[], cycles=0)
    if trace.get('coverage') != 'complete' or trace.get('completed') is not True:
        return unknown('Complete trusted instrumentation and normal completion are required')
    events = trace.get('events')
    if not isinstance(events, list) or not events:
        return unknown('No observed events')
    memory = {}
    reads = {}
    active = None
    seen = set()
    written = published = False
    for index, event in enumerate(events):
        if not isinstance(event, dict): return unknown('Malformed event')
        kind = event.get('kind')
        fields = {'begin': ('cycle',), 'write': ('actor', 'object'),
                  'copy': ('src', 'dst'), 'read': ('actor', 'object', 'read_id'),
                  'publish': ('actor', 'output', 'read_id'), 'end': ()}
        if kind not in fields or any(not isinstance(event.get(f), str) or not event[f]
                                     for f in fields[kind]):
            return unknown('Unknown event or missing identifier')
        if kind == 'begin':
            if active is not None or event['cycle'] in seen:
                return unknown('Nested or duplicate cycle')
            active = event['cycle']; seen.add(active)
            reads = {}; written = published = False
            continue
        if active is None: return unknown('Event outside cycle')
        if kind == 'write':
            # A write creates provenance; memory survives across cycles.
            memory[event['object']] = (active, event['actor'], index)
            if event['actor'] == obligation.producer: written = True
        elif kind == 'copy':
            memory[event['dst']] = memory.get(event['src'])
        elif kind == 'read':
            key = event['read_id']
            if key in reads: return unknown('Duplicate read identifier')
            origin = memory.get(event['object'])
            valid = origin is not None and origin[:2] == (active, obligation.producer)
            reads[key] = (event['actor'], valid)
            if event['actor'] == obligation.consumer and not valid:
                issue('consumer_missing_current_producer_data', index, active, 'I/B')
        elif kind == 'publish':
            if event['output'] != obligation.output:
                issue('wrong_output_boundary', index, active, 'B/code')
                continue
            source = reads.get(event['read_id'])
            if event['actor'] != obligation.consumer or source != (obligation.consumer, True):
                issue('output_without_valid_consumer_read', index, active, 'B/code')
            else:
                published = True
        elif kind == 'end':
            if not written: issue('missing_producer_write', index, active, 'S/I')
            if not published: issue('missing_valid_output', index, active, 'B/code')
            active = None
    if active is not None: return unknown('Unclosed cycle')
    return dict(status='fail' if issues else 'pass', issues=issues, cycles=len(seen),
                scope='Observed provenance and order only; not value correctness or timing')
