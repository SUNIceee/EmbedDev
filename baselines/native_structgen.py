"""Official StructGen entry with C extraction/execution and budgeted model hooks.

Original core loops and prompts are imported from the pinned author checkout.
This process receives no model credentials and does not execute candidate code.
"""
import argparse
import configparser
import json
from pathlib import Path
import re
import socket
import sys
from types import SimpleNamespace

from c_project import extract_files, validate_files, project_hash
from runtime_bridge import RuntimeBridge, BridgeStop, write_once


def decode_test_cases(text):
    text = re.sub(r'<think>.*?</think>', '', text, flags=re.S).strip()
    if text.startswith('```'):
        text = re.sub(r'^```(?:json)?\s*', '', text, count=1)
    start = text.find('{')
    if start < 0:
        raise ValueError('Development cases contain no JSON object')
    # Compatibility for literal newlines in C strings; never invent missing text.
    value, end = json.JSONDecoder(strict=False).raw_decode(text, start)
    if text[end:].strip() not in ('', '```'):
        raise ValueError('Unexpected content after development cases')
    cases = value.get('cases')
    if not isinstance(cases, list) or not 3 <= len(cases) <= 6:
        raise ValueError('Expected 3..6 development cases')
    ids = set()
    for case in cases:
        if not isinstance(case, dict) or not all(isinstance(case.get(k), str) for k in ('id', 'declarations', 'body')):
            raise ValueError('Invalid C development case')
        if case['id'] in ids or not case['id'] or not case['body'].strip():
            raise ValueError('Empty or duplicate development case')
        ids.add(case['id'])
    return {'cases': cases}


def run(job, repo, work, bridge):
    sys.path.insert(0, str(repo))
    import generate_feedback as native
    import utils
    def deny_network(*args, **kwargs):
        raise BridgeStop('unapproved_network_operation', 'Framework network must use the local parent bridge')
    socket.socket.connect = deny_network
    socket.create_connection = deny_network
    config = configparser.ConfigParser()
    config.read(repo / 'generate/config.ini', encoding='utf-8')
    config.set('ollama', 'base_url', 'http://127.0.0.1:9/v1')
    config.set('ollama', 'api_key', 'local-bridge-no-credential')
    config.set('eval', 'model', job['model'])
    config.set('eval', 'max_tokens', '65536')
    config.set('eval', 'temperature', '0')
    config.set('eval', 'max_workers', '1')
    config.set('feedback', 'designer_model', job['model'])
    # Initial diagram + at most three native design revisions. The parent
    # additionally enforces prior-version calls/tokens/design/code slots.
    config.set('feedback', 'max_plantUML_attempts', '3')
    config.set('feedback', 'max_function_code_attempts', '1')
    config.set('datasets', 'dataset_name', 'fse_c')
    config.set('basic', 'output_file_jsonl', str(work / 'native_completions.jsonl'))
    config.set('UML', 'uml_csv_file', str(work / 'native_results.csv'))
    config.set('prompt', 'generate_uml', str(repo / 'generate/generate_plantuml.md'))
    config.set('prompt', 'generate_function_code', str(repo / 'generate/generate_function_code.md'))
    with (work / 'runtime_config.ini').open('w', encoding='utf-8') as output:
        config.write(output)
    utils.get_config = lambda: config
    native.get_config = lambda: config

    inputs = '\n\n'.join('=== %s ===\n%s' % (key, job[key]) for key in ('requirement', 'device', 'api', 'frozen_header'))
    tests_response = bridge.complete([
        {'role': 'system', 'content': 'Prepare 3 to 6 C development test cases from the supplied requirements, device interface and fixed API. '
         'Use only this information. These are development tests. Return one JSON object with cases, each with id, declarations, body. '
         'declarations contains includes (use "6_generated_code.h") and any needed device stubs; body is inside main and uses PB_CHECK or PB_NEAR. '
         'Do not define main. PB_CHECK(expr) checks an assertion; PB_NEAR(actual, expected, tolerance) checks a finite numerical result. '
         'Use meaningful observable behavior and boundary cases. Escape newlines inside JSON strings.'},
        {'role': 'user', 'content': inputs}], 'structgen_development_tests')
    if tests_response['metadata'].get('finish_reason') in ('length', 'max_tokens'):
        raise ValueError('Development test response was truncated')
    tests = decode_test_cases(tests_response['text'])
    write_once(work / 'development_tests.json', tests)
    prompt = inputs + '\n\nImplement a complete C11 library exposing the supplied fixed API. The fixed public header is 6_generated_code.h. '
    prompt += 'Return complete C source/header files with explicit file-name headings and fenced c blocks; no GUI, package installation or network features. '
    prompt += 'Preserve all required behavior and device interfaces. A single unnamed C block means 6_generated_code.c.\n'
    prompt += '=== Development examples ===\n' + json.dumps(tests, ensure_ascii=False)
    problem = {'task_id': 'C/0', 'prompt': prompt, 'plantuml': '', 'generate_test_cases': tests['cases']}
    state = {'design': 0, 'code': 0, 'stage': '', 'kind': None, 'label': None, 'candidates': [], 'parse_error': None}

    def create(**kwargs):
        if kwargs['model'] != job['model']:
            raise BridgeStop('model_mismatch', kwargs['model'])
        response = bridge.complete(kwargs['messages'], state['stage'], state['kind'], state['label'])
        if response['metadata'].get('finish_reason') in ('length', 'max_tokens'):
            raise ValueError('Provider marked response truncated')
        return SimpleNamespace(choices=[SimpleNamespace(message=SimpleNamespace(content=response['text']))])
    client = SimpleNamespace(chat=SimpleNamespace(completions=SimpleNamespace(create=create)))
    native.get_client = lambda unused: client
    original_generate = native.generate_completion
    def generate(*args, **kwargs):
        kind = 'design' if kwargs.get('generate_type') == 'generate_uml' else 'code'
        state[kind] += 1
        state.update(kind=kind, label='structgen_%s_%02d' % (kind, state[kind]),
                     stage='structgen_%s_%02d' % (kind, state[kind]), parse_error=None)
        return original_generate(*args, **kwargs)
    native.generate_completion = generate

    def extract_c(problem_arg, response, prompt_language):
        if prompt_language != 'c':
            raise BridgeStop('language_mismatch', prompt_language)
        try:
            files = extract_files(response, default_source='6_generated_code.c')
            if not any(name.lower().endswith('.c') for name in files):
                raise ValueError('No C translation unit was returned')
            if '6_generated_code.h' not in files:
                files['6_generated_code.h'] = job['frozen_header']
            validate_files(files)
            bundle = json.dumps(files, ensure_ascii=False, sort_keys=True)
            return bundle, True
        except ValueError as exc:
            state['parse_error'] = str(exc)
            write_once(work / ('parse_error_%02d.json' % state['code']), {'error': str(exc)})
            return '# extract function code failed', False
    native.extract_function = extract_c

    def execute_c(case_list, bundle):
        if state['parse_error']:
            return 'failed: ' + state['parse_error']
        files = json.loads(bundle)
        stage = 'code_%02d' % state['code']
        record = {'stage': stage, 'files': files, 'development_passed': None,
                  'project_sha256': project_hash(files, {'cases': case_list})}
        write_once(work / (stage + '_snapshot.json'), record)
        state['candidates'].append(record)
        result = bridge.evaluate(files, {'cases': case_list}, stage)
        record.update(development_passed=result['passed'], project_sha256=result['packet_sha256'])
        write_once(work / (stage + '.json'), record)
        return 'passed' if result['passed'] else ['C compilation, linking or development cases failed; see saved execution report.']
    native.execute_public_test = execute_c

    problems = {'C/0': problem}
    try:
        # Upstream main expects a dataset CSV containing prior diagrams. Starting
        # from raw requirements requires its own original diagram entry once.
        native.generate_uml(client, problem, job['model'], config)
        native.process_eval(problems, job['model'], 'c')
        status, detail = 'native_workflow_finished', ''
    except BridgeStop as exc:
        status, detail = exc.status, exc.detail
    selected = state['candidates'][-1] if state['candidates'] else None
    return {'status': status, 'detail': detail, 'candidates': state['candidates'], 'selected_candidate': selected,
            'native_core_used': ['generate_uml', 'generate_and_validate_function_code', 'process_eval'],
            'native_feedback_behavior_preserved': True, 'design_requests': state['design'], 'code_requests': state['code']}


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--input', type=Path, required=True)
    p.add_argument('--repo', type=Path, required=True)
    p.add_argument('--work', type=Path, required=True)
    p.add_argument('--bridge', type=Path, required=True)
    args = p.parse_args()
    job = json.loads(args.input.read_text(encoding='utf-8'))
    args.work.mkdir(parents=True, exist_ok=True)
    bridge = RuntimeBridge(args.bridge, job['model'])
    try:
        result = run(job, args.repo.resolve(), args.work.resolve(), bridge)
    except BridgeStop as exc:
        result = {'status': exc.status, 'detail': exc.detail, 'selected_candidate': None}
    except Exception as exc:
        result = {'status': 'adapter_or_output_error', 'detail': str(exc), 'error_type': type(exc).__name__, 'selected_candidate': None}
    write_once(args.work / 'result.json', result)
    print(json.dumps({'status': result['status'], 'has_candidate': bool(result.get('selected_candidate'))}))


if __name__ == '__main__':
    main()
