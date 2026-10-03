"""Pinned ChatDev ChatChain/Phase/RolePlaying with audited C boundary hooks.

No credentials, candidate execution, downloads or installation occur here.
The official conversation, reflection and composed-phase loops are imported.
"""
import argparse
from collections import defaultdict
from dataclasses import dataclass
import functools
import hashlib
import json
import logging
import os
from pathlib import Path
import re
import shutil
import socket
import subprocess
import sys

from c_project import extract_files, validate_files
from runtime_bridge import RuntimeBridge, BridgeStop, write_once
from native_structgen import decode_test_cases


@dataclass(frozen=True)
class ApprovedModel:
    value: str


def extract_chatdev_files(text, default_source='6_generated_code.c'):
    # Compatibility for explicit numbered file headings such as
    # "### 1. 6_generated_code.h". Change headings only, never source bodies.
    lines, fenced = [], False
    for line in text.splitlines(keepends=True):
        if line.strip().startswith('```'):
            fenced = not fenced
        elif not fenced:
            line = re.sub(r'^(\s*#{1,6}\s+)\d+[.)]\s+', r'\1', line)
            line = re.sub(r'(?i)^(\s*(?:#{1,6}\s+)?)file(?:\s*name)?\s*:\s*', r'\1', line)
        lines.append(line)
    return extract_files(''.join(lines), default_source)


def decode_chatdev_cases(text):
    # A top-level JSON array contains the same case records as {"cases": [...]}.
    # Preserve every record; only normalize this explicit container shape.
    cleaned = re.sub(r'<think>.*?</think>', '', text, flags=re.S).strip()
    if cleaned.startswith('```'):
        cleaned = re.sub(r'^```(?:json)?\s*', '', cleaned, count=1)
    if cleaned.lstrip().startswith('['):
        value, end = json.JSONDecoder(strict=False).raw_decode(cleaned.lstrip())
        if cleaned.lstrip()[end:].strip() not in ('', '```'):
            raise ValueError('Unexpected content after development case array')
        return decode_test_cases(json.dumps({'cases': value}, ensure_ascii=False))
    return decode_test_cases(text)


def configure(repo, work):
    origin = repo / 'CompanyConfig/Default'
    paths, changes = {}, []
    for filename in ('ChatChainConfig.json', 'PhaseConfig.json', 'RoleConfig.json'):
        obj = json.loads((origin / filename).read_text(encoding='utf-8'))
        if filename == 'ChatChainConfig.json':
            for key in ('clear_structure', 'gui_design', 'git_management', 'web_spider',
                        'self_improve', 'incremental_develop', 'with_memory'):
                if obj[key] != 'False':
                    changes.append({'file': filename, 'key': key, 'before': obj[key], 'after': 'False'})
                obj[key] = 'False'
        elif filename == 'PhaseConfig.json':
            for phase, data in obj.items():
                before = list(data['phase_prompt'])
                # Python literal documentation examples are syntax-invalid in C.
                updated = [s.replace('lowercase file name', 'case-sensitive relative file name')
                           .replace("No placeholders (such as 'pass' in Python).", 'No placeholders such as TODO or an unimplemented function.')
                           .replace('which contains a unimplemented class', 'which contains an unimplemented function')
                           for s in before]
                if phase in ('Coding', 'CodeComplete', 'CodeReviewModification', 'TestModification'):
                    block = ['FILENAME', '```LANGUAGE', "'''", 'DOCSTRING', "'''", 'CODE', '```']
                    for index in range(len(updated) - len(block) + 1):
                        if updated[index:index + len(block)] == block:
                            updated[index:index + len(block)] = ['FILENAME', '```c', '/* DOCSTRING */', 'CODE', '```']
                            break
                    updated.append('This project is a C11 library with the fixed public API from the task. '
                                   'Use explicit relative .c/.h file names and complete file bodies. '
                                   'Preserve include path case. Use standard C comments, not Python docstrings. '
                                   'The public header is 6_generated_code.h; do not add a standalone main function.')
                if phase == 'LanguageChoose':
                    updated = [s.replace('If python can complete this task via Python, please answer Python; otherwise, answer another programming language (e.g., Java, C++, etc,).',
                                         'The target device interface requires C11; discuss that required language.') for s in updated]
                if phase == 'DemandAnalysis':
                    updated.append('For this task the required product modality is an embedded C library using the supplied fixed device API.')
                if phase == 'EnvironmentDoc':
                    updated = [s.replace('which is commonly used in Python projects to specify the dependencies or packages required for the project to run properly.',
                                         'which here documents the C compiler, standard libraries and build requirements.')
                               .replace('numpy==1.19.2', 'C11 compiler').replace('pandas>=1.1.4', 'C standard library') for s in updated]
                    updated.append('Document build requirements only; no package installation is performed.')
                if before != updated:
                    changes.append({'file': filename, 'phase': phase, 'before': before, 'after': updated})
                data['phase_prompt'] = updated
        path = work / 'config' / filename
        write_once(path, obj)
        paths[filename] = str(path)
    write_once(work / 'configuration_changes.json', changes)
    return paths


def run(job, repo, work, bridge):
    sys.path.insert(0, str(repo))
    # The legacy module checks existence only. This is a non-secret placeholder;
    # the custom backend routes every actual completion through the parent.
    os.environ['OPENAI_API_KEY'] = 'local-bridge-no-credential'
    os.environ['BASE_URL'] = 'http://127.0.0.1:9/v1'
    os.environ['TIKTOKEN_CACHE_DIR'] = str(Path(__file__).parent / 'cache/tiktoken')
    import ecl
    sys.path.append(str(repo / 'ecl'))
    import tiktoken
    from openai.types.chat import ChatCompletion
    from camel import model_backend
    from camel.agents import chat_agent
    from camel.utils import count_tokens_openai_chat_models
    from chatdev import chat_chain, chat_env, codes, phase, composed_phase
    encoding = tiktoken.get_encoding('cl100k_base')
    configs = configure(repo, work)

    def deny(*args, **kwargs):
        raise BridgeStop('unapproved_runtime_operation', 'Native framework must use the local bridge and isolated C execution')
    socket.socket.connect = deny
    socket.socket.connect_ex = deny
    socket.create_connection = deny
    subprocess.Popen = deny
    os.system = deny
    os.remove = deny
    os.unlink = deny
    os.rmdir = deny
    shutil.rmtree = deny

    state = {'phase': 'setup', 'kind': None, 'label': None, 'code_round': 0,
             'request': 0, 'update': 0, 'candidates': [], 'evaluations': {}, 'tests': None, 'phase_visits': []}
    model = ApprovedModel(job['model'])

    class Backend(model_backend.ModelBackend):
        def run(self, *args, **kwargs):
            state['request'] += 1
            response = bridge.complete(kwargs['messages'], 'chatdev_%s_%03d' % (state['phase'], state['request']),
                                       state['kind'], state['label'])
            finish = response['metadata'].get('finish_reason') or 'stop'
            if finish in ('length', 'max_tokens'):
                raise BridgeStop('output_truncated', 'Native response ended at the provider output limit')
            # Usage is deliberately absent: authoritative accounting is in the
            # parent ledger, not fabricated legacy OpenAI pricing statistics.
            return ChatCompletion(id='bridge_%03d' % state['request'], created=0, model=job['model'],
                object='chat.completion', choices=[{'index': 0, 'finish_reason': 'stop',
                'message': {'role': 'assistant', 'content': response['text']}}])

    def factory(model_type, model_config_dict):
        if model_type != model:
            raise BridgeStop('model_mismatch', str(model_type))
        return Backend()
    model_backend.ModelFactory.create = staticmethod(factory)
    # Legacy tables recognize only 2023 OpenAI models. Retain message windows
    # and role order; local approximate tokens guard the identity's 800k ceiling.
    # This is not a claim about any provider's actual context capacity.
    chat_agent.get_model_token_limit = lambda supplied: 800000 if supplied == model else deny()
    chat_agent.num_tokens_from_messages = lambda messages, supplied: count_tokens_openai_chat_models(messages, encoding) if supplied == model else deny()

    original_chatting = phase.Phase.chatting
    @functools.wraps(original_chatting)
    def chatting(self, *args, **kwargs):
        name = kwargs.get('phase_name', self.phase_name)
        previous = {key: state[key] for key in ('phase', 'kind', 'label')}
        state['phase'] = name
        state['phase_visits'].append(name)
        if name in ('Coding', 'CodeComplete', 'CodeReviewModification', 'TestModification'):
            state['code_round'] += 1
            state.update(kind='code', label='chatdev_code_%02d' % state['code_round'])
        elif name in ('DemandAnalysis', 'LanguageChoose'):
            state.update(kind='design', label='chatdev_design_01')
        elif name != 'Reflection':
            state.update(kind=None, label=None)
        try:
            return original_chatting(self, *args, **kwargs)
        finally:
            state.update(previous)
    phase.Phase.chatting = chatting

    class LosslessCodes(codes.Codes):
        def __init__(self, generated_content=''):
            self.directory, self.version, self.generated_content = None, 0.0, generated_content
            self.codebooks = extract_chatdev_files(generated_content) if generated_content else {}

        def _format_code(self, code):
            return code

        def _update_codes(self, generated_content):
            incoming = extract_chatdev_files(generated_content)
            merged = dict(self.codebooks)
            merged.update(incoming)
            if '6_generated_code.h' not in merged:
                merged['6_generated_code.h'] = job['frozen_header']
            self.codebooks = validate_files(merged)

        def _rewrite_codes(self, git_management, phase_info=None):
            if git_management:
                deny()
            state['update'] += 1
            files = validate_files(self.codebooks)
            if not any(name.endswith('.c') for name in files):
                raise ValueError('No C translation unit')
            stage = 'code_%02d' % state['update']
            record = {'stage': stage, 'files': files, 'phase_info': phase_info,
                      'project_sha256': hashlib.sha256(json.dumps(files, sort_keys=True).encode('utf-8')).hexdigest()}
            write_once(work / (stage + '.json'), record)
            state['candidates'].append(record)
            # A working view may be overwritten, but every prior version above
            # remains immutable. All paths have already passed validation.
            for name, body in files.items():
                path = Path(self.directory) / name
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_text(body, encoding='utf-8')
            self.version += 1
    codes.Codes = LosslessCodes
    chat_env.Codes = LosslessCodes

    software = work / 'software'
    software.mkdir(exist_ok=True)
    def set_directory(self, ignored_native_path):
        if self.env_dict['directory']:
            raise ValueError('Framework workspace initialized twice')
        self.env_dict['directory'] = str(software)
        self.codes.directory = self.requirements.directory = self.manuals.directory = str(software)
    chat_env.ChatEnv.set_directory = set_directory
    chat_env.ChatEnv.generate_images_from_codes = lambda self: None
    chat_env.ChatEnv.get_proposed_images_from_message = deny
    chat_env.ChatEnv.fix_module_not_found_error = staticmethod(deny)

    def exist_bugs(self):
        if not state['candidates']:
            return True, 'No complete C candidate was generated; tests did not run.'
        current = state['candidates'][-1]
        stage = current['stage']
        if stage not in state['evaluations']:
            result = bridge.evaluate(current['files'], state['tests'], stage)
            state['evaluations'][stage] = result
            write_once(work / (stage + '_development.json'), result)
        result = state['evaluations'][stage]
        return not result['passed'], json.dumps(result, ensure_ascii=False)
    chat_env.ChatEnv.exist_bugs = exist_bugs

    # Preserve completion cycles and retry counters, substituting C file names
    # and explicit TODO/unimplemented comments for the Python-only `pass` check.
    def completion_env(self, env):
        names = [name for name in env.codes.codebooks if name.endswith('.c')]
        self.phase_env.update(max_num_implement=5, pyfiles=names, num_tried=defaultdict(int))
    composed_phase.CodeCompleteAll.update_phase_env = completion_env
    def completion_file(self, env):
        self.phase_env.update(task=env.env_dict['task_prompt'], modality=env.env_dict['modality'],
            ideas=env.env_dict['ideas'], language=env.env_dict['language'], codes=env.get_codes(), unimplemented_file='')
        for name in self.phase_env['pyfiles']:
            content = env.codes.codebooks[name]
            if re.search(r'(?m)^\s*(?://|/\*|\*)\s*(?:TODO|UNIMPLEMENTED)\b', content) and self.phase_env['num_tried'][name] < self.phase_env['max_num_implement']:
                self.phase_env['unimplemented_file'] = name
                break
        self.phase_env['num_tried'][self.phase_env['unimplemented_file']] += 1
    phase.CodeComplete.update_phase_env = completion_file

    # Original preprocessing chooses repo/WareHouse, deletes existing folders
    # and moves logs. Adapt only this administrative boundary to a fixed local
    # workspace. Recruitment and execute_chain/execute_step remain original.
    log_path = work / 'chatdev.log'
    chat_chain.ChatChain.get_logfilepath = lambda self: ('20260909000000', str(log_path))
    logging.basicConfig(filename=log_path, level=logging.INFO, encoding='utf-8', force=True)
    inputs = '\n\n'.join('=== %s ===\n%s' % (key, job[key]) for key in ('requirement', 'device', 'api', 'frozen_header'))
    task = inputs + '\n\nImplement this fixed API as a complete C11 library. Use 6_generated_code.h as the public header, '
    task += 'explicit relative C file headings and complete fenced c blocks. No standalone main, GUI, downloads or package installation.'
    chain = chat_chain.ChatChain(config_path=configs['ChatChainConfig.json'], config_phase_path=configs['PhaseConfig.json'],
        config_role_path=configs['RoleConfig.json'], task_prompt=task, project_name=job['id'], org_name='FSE', model_type=model)
    chain.chat_env.set_directory(str(software))
    chain.chat_env.env_dict['task_prompt'] = task
    write_once(work / 'raw_task.json', {'task': task})
    chain.make_recruitment()

    try:
        # A library has no UI entry point. The C execution adapter prepares a
        # small R/D/API-only development caller in place of `python main.py`.
        # These cases never come from final evaluation or an old adaptation.
        response = bridge.complete([
            {'role': 'system', 'content': chain.role_prompts['Software Test Engineer'] + '\nPrepare 3 to 6 development cases for this C11 library from only the supplied requirements, device and fixed API. '
             'Return JSON with cases, each containing id, declarations and body strings. declarations contains includes (use "6_generated_code.h") and needed device stubs. '
             'body runs inside main and uses PB_CHECK(expr) or PB_NEAR(actual, expected, tolerance). Do not define main. Escape newlines in JSON strings.'},
            {'role': 'user', 'content': inputs}], 'chatdev_development_tests')
        if response['metadata'].get('finish_reason') in ('length', 'max_tokens'):
            raise BridgeStop('output_truncated', 'Development cases response was truncated')
        state['tests'] = decode_chatdev_cases(response['text'])
        write_once(work / 'development_tests.json', state['tests'])
        chain.execute_chain()
        status, detail = 'native_workflow_finished', ''
    except BridgeStop as exc:
        status, detail = exc.status, exc.detail
    except Exception as exc:
        status, detail = 'adapter_or_output_error', type(exc).__name__ + ': ' + str(exc)
    # No additional model request here. Evaluating already generated versions
    # does not feed results back into a terminated generation workflow.
    if state['tests']:
        for candidate in state['candidates']:
            stage = candidate['stage']
            try:
                if stage not in state['evaluations']:
                    result = bridge.evaluate(candidate['files'], state['tests'], stage)
                    state['evaluations'][stage] = result
                    write_once(work / (stage + '_development.json'), result)
                candidate['development_passed'] = state['evaluations'][stage]['passed']
            except BridgeStop as exc:
                # Keep every code snapshot even when the execution service
                # fails. An unavailable test is unknown, never a model fail.
                status, detail = exc.status, exc.detail
                candidate['development_passed'] = None
                break
    return {'status': status, 'detail': detail, 'selected_candidate': state['candidates'][-1] if state['candidates'] else None,
        'candidates': state['candidates'], 'native_core_used': ['ChatChain.execute_chain', 'ChatChain.execute_step',
        'Phase.chatting', 'Phase.self_reflection', 'RolePlaying', 'ComposedPhase.execute'],
        'phase_visits': state['phase_visits'], 'native_code_requests': state['code_round'],
        'native_model': job['model'], 'local_token_estimator': 'cl100k_base approximate; authoritative parent budget',
        'native_completion_marker_behavior_preserved': True}


def main():
    parser = argparse.ArgumentParser()
    for name in ('input', 'repo', 'work', 'bridge'):
        parser.add_argument('--' + name, type=Path, required=True)
    args = parser.parse_args()
    job = json.loads(args.input.read_text(encoding='utf-8'))
    args.work.mkdir(parents=True, exist_ok=True)
    try:
        result = run(job, args.repo.resolve(), args.work.resolve(), RuntimeBridge(args.bridge, job['model']))
    except BridgeStop as exc:
        result = {'status': exc.status, 'detail': exc.detail, 'selected_candidate': None}
    except Exception as exc:
        result = {'status': 'adapter_or_output_error', 'detail': str(exc), 'error_type': type(exc).__name__, 'selected_candidate': None}
    write_once(args.work / 'result.json', result)
    print(json.dumps({'status': result['status'], 'has_candidate': bool(result.get('selected_candidate'))}))


if __name__ == '__main__':
    main()
