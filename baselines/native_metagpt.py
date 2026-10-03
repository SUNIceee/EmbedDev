"""Pinned MetaGPT Team/SOP with C and credential-free execution boundary hooks."""
import argparse
import asyncio
from collections import defaultdict
from contextvars import ContextVar
import functools
import hashlib
import json
import os
from pathlib import Path
import re
import socket
import subprocess
import sys

from c_project import extract_files, safe_name, validate_files
from runtime_bridge import RuntimeBridge, BridgeStop, write_once
from native_structgen import decode_test_cases


async def run(job, repo, work, bridge):
    sys.path.insert(0, str(repo))
    # A fresh working view on every replay prevents old generated docs/code from
    # changing the original requirement's path through the upstream SOP.
    views = work / 'views'
    views.mkdir(exist_ok=True)
    view = views / ('v%03d' % (len(list(views.iterdir())) + 1))
    view.mkdir(exist_ok=False)
    os.environ['METAGPT_PROJECT_ROOT'] = str(view)
    os.environ['GIT_PYTHON_GIT_EXECUTABLE'] = 'C:/Program Files/Git/cmd/git.exe'
    os.environ['GIT_CONFIG_NOSYSTEM'] = '1'
    os.environ['GIT_CONFIG_GLOBAL'] = str(view / 'empty_gitconfig')
    (view / 'empty_gitconfig').write_text('', encoding='utf-8')
    os.environ['GIT_CONFIG_COUNT'] = '1'
    os.environ['GIT_CONFIG_KEY_0'] = 'core.hooksPath'
    os.environ['GIT_CONFIG_VALUE_0'] = str(view / 'no_hooks')
    os.environ['GIT_AUTHOR_NAME'] = os.environ['GIT_COMMITTER_NAME'] = 'FSE isolated experiment'
    os.environ['GIT_AUTHOR_EMAIL'] = os.environ['GIT_COMMITTER_EMAIL'] = 'fse-experiment@localhost'
    config_dir = view / 'config'
    config_dir.mkdir()
    (config_dir / 'config2.yaml').write_text('llm:\n  api_type: openai\n  api_key: local-bridge-no-credential\n'
        '  base_url: http://127.0.0.1:9/v1\n  model: ' + job['model'] + '\n  max_token: 65536\n  temperature: 0\n', encoding='utf-8')

    from metagpt import context as context_module
    from metagpt.config2 import Config
    from metagpt.provider.base_llm import BaseLLM
    from metagpt.team import Team
    from metagpt.roles import ProductManager, Architect, ProjectManager, Engineer, QaEngineer
    from metagpt.actions import WriteCode, WriteCodeReview, WriteTest, RunCode, DebugError
    from metagpt.actions.prepare_documents import PrepareDocuments
    from metagpt.actions.write_prd import WritePRD
    from metagpt.actions.design_api import WriteDesign
    from metagpt.actions.project_management import WriteTasks
    from metagpt.actions.summarize_code import SummarizeCode
    from metagpt.actions import write_code, write_code_review, run_code, debug_error, write_prd, design_api
    from metagpt.actions import write_prd_an, design_api_an, project_management_an
    from metagpt.schema import Document, Message, RunCodeContext, TestingContext
    from metagpt.utils.common import CodeParser
    from metagpt.utils.file_repository import FileRepository
    from metagpt.utils.git_repository import GitRepository
    from metagpt.utils.project_repo import ProjectRepo

    def deny(*args, **kwargs):
        raise BridgeStop('unapproved_runtime_operation', 'Use the local model bridge and isolated C execution')
    socket.socket.connect = deny
    socket.socket.connect_ex = deny
    socket.create_connection = deny
    original_popen = subprocess.Popen
    def git_only(args, *other, **kwargs):
        if isinstance(args, str) or kwargs.get('shell'):
            deny()
        if Path(str(args[0])).name.lower() not in ('git', 'git.exe'):
            deny()
        if any(str(arg) in ('clone', 'fetch', 'pull', 'push', 'submodule', 'clean', 'reset') for arg in args[1:]):
            deny()
        kwargs['creationflags'] = kwargs.get('creationflags', 0) | getattr(subprocess, 'CREATE_NO_WINDOW', 0)
        return original_popen(args, *other, **kwargs)
    subprocess.Popen = git_only
    os.system = deny
    GitRepository.delete_repository = deny
    GitRepository.rename_root = lambda self, name: None if name == self.workdir.name else deny()

    def contained(path):
        path = Path(path).resolve()
        try:
            path.relative_to(view)
        except ValueError:
            raise BridgeStop('invalid_output_path', 'Framework file path escapes this run view')
        if path.is_symlink():
            raise BridgeStop('invalid_output_path', 'Symlink in native work view')
        return path

    state = {'requests': 0, 'implementation': 0, 'design': 0, 'filename': 0,
             'files': {}, 'expected': set(), 'candidates': [], 'evaluations': {}, 'actions': [],
             'file_attempts': defaultdict(int), 'development_tests': {}, 'qa_executions': 0}
    current = ContextVar('official_metagpt_action', default=('Role', ''))
    inputs = '\n\n'.join('=== %s ===\n%s' % (key, job[key]) for key in ('requirement', 'device', 'api', 'frozen_header'))
    task = inputs + '\n\nImplement the fixed device API as a C11 library using 6_generated_code.h as its public header. '
    task += 'Use only C source/header files and the supplied device contract. No standalone main, GUI, network services or dependency installation. '
    task += 'Keep the complete original requirements, API declarations and device interface available in downstream design/task documents.'

    class BridgeLLM(BaseLLM):
        def __init__(self, config):
            if config.model != job['model']:
                raise BridgeStop('model_mismatch', str(config.model))
            self.config, self.model = config, config.model
            self.cost_manager = None

        async def acompletion_text(self, messages, stream=False, timeout=3):
            action, filename = current.get()
            state['requests'] += 1
            kind, label = None, None
            if action in ('WritePRD', 'WriteDesign', 'WriteTasks'):
                kind, label = 'design', 'metagpt_design_%02d' % max(state['design'], 1)
            is_code = action == 'WriteCode' or (action == 'WriteCodeReview' and '## Rewrite Code:' in messages[-1]['content'])
            if is_code:
                key = (state['implementation'], filename)
                ordinal = state['file_attempts'][key]
                state['file_attempts'][key] += 1
                kind, label = 'code', 'metagpt_implementation_%02d_revision_%02d' % (state['implementation'], ordinal)
            response = bridge.complete(messages, 'metagpt_%s_%03d' % (action, state['requests']), kind, label)
            if response['metadata'].get('finish_reason') in ('length', 'max_tokens'):
                raise BridgeStop('output_truncated', 'Provider output limit reached')
            return response['text']

        async def acompletion(self, messages, timeout=3):
            text = await self.acompletion_text(messages, timeout=timeout)
            return {'choices': [{'message': {'role': 'assistant', 'content': text}}]}

    context_module.create_llm_instance = lambda config: BridgeLLM(config)

    # Domain/format changes retain native ActionNode schemas and the core fill/
    # validation flow. No final evaluation material is added to these nodes.
    changes = []
    def node_change(node, instruction, example):
        changes.append({'node': node.key, 'old_instruction': node.instruction, 'new_instruction': instruction,
                        'old_example': node.example, 'new_example': example})
        node.instruction, node.example = instruction, example
    node_change(write_prd_an.PROGRAMMING_LANGUAGE, 'The supplied device and frozen API require C11.', 'C11')
    node_change(design_api_an.FILE_LIST, 'List only relative .c/.h paths for the C11 library. Include 6_generated_code.h. Do not create a main program.',
                ['6_generated_code.h', '6_generated_code.c'])
    node_change(design_api_an.DATA_STRUCTURES_AND_INTERFACES, 'Use mermaid classDiagram syntax to describe C structs, state, typed functions and their relationships. Preserve every supplied fixed API declaration.',
                'classDiagram\nclass Library {\n+int add(int a, int b)\n}')
    node_change(project_management_an.REQUIRED_PYTHON_PACKAGES, 'This is a C project. Return an empty Python package list.', [])
    node_change(project_management_an.TASK_LIST, 'List relative .c/.h filenames in dependency order, preserving the design and fixed public header.',
                ['6_generated_code.h', '6_generated_code.c'])
    node_change(project_management_an.LOGIC_ANALYSIS, 'List each C file, its structs/functions and include dependencies.',
                [['6_generated_code.h', 'Fixed public API'], ['6_generated_code.c', 'Implementation']])
    write_code.PROMPT_TEMPLATE = write_code.PROMPT_TEMPLATE.replace('```python', '```c').replace('## {filename}\n...', '/* {filename} */\n...')
    run_code.TEMPLATE_CONTEXT = run_code.TEMPLATE_CONTEXT.replace('```python', '```c')
    run_code.PROMPT_TEMPLATE = run_code.PROMPT_TEMPLATE.replace('xyz.py, or test_xyz.py', 'module.c, or test_module.c.json')
    debug_error.PROMPT_TEMPLATE = debug_error.PROMPT_TEMPLATE.replace('```python', '```c')
    write_once(work / 'node_changes.json', changes)

    async def keep_mermaid(engine, code, output_file, *args, **kwargs):
        target = contained(Path(output_file).with_suffix('.mmd'))
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_text(code, encoding='utf-8')
        return 0
    write_prd.mermaid_to_file = design_api.mermaid_to_file = keep_mermaid

    # Fixed document IDs and fresh local repositories make logical cache replay
    # reproducible while preserving all prior views and their git histories.
    def filename():
        state['filename'] += 1
        return 'doc_%03d' % state['filename']
    FileRepository.new_filename = staticmethod(filename)
    def init_repo(self):
        path = contained(view / 'library')
        if path.exists():
            raise BridgeStop('workspace_not_fresh', 'This native view already contains a repository')
        self.config.project_path = str(path)
        self.context.git_repo = GitRepository(local_path=path, auto_init=True)
        self.context.repo = ProjectRepo(self.context.git_repo)
    PrepareDocuments._init_repo = init_repo

    original_save, original_get = FileRepository.save, FileRepository.get
    async def save_file(self, filename, content, dependencies=None):
        contained(self.workdir / filename)
        if Path(filename).suffix.lower() in ('.c', '.h'):
            safe_name(str(filename).replace('\\', '/'))
        return await original_save(self, filename, content, dependencies)
    async def get_file(self, filename):
        contained(self.workdir / filename)
        return await original_get(self, filename)
    async def no_delete(self, filename):
        if contained(self.workdir / filename).exists():
            deny()
    FileRepository.save, FileRepository.get, FileRepository.delete = save_file, get_file, no_delete

    def snapshot(context):
        if not context or not context.code_doc:
            return
        if Path(context.filename).suffix.lower() not in ('.c', '.h'):
            # Native project planning may assign auxiliary requirements/API
            # documents alongside the C files. Preserve its original actions,
            # files and billing, but never treat those documents as C code.
            return
        name = safe_name(context.filename.replace('\\', '/'))
        state['files'][name] = context.code_doc.content
        if '6_generated_code.h' not in state['files']:
            state['files']['6_generated_code.h'] = job['frozen_header']
        if not state['expected'].issubset(state['files']) or not any(n.endswith('.c') for n in state['files']):
            return
        files = validate_files(state['files'])
        if state['candidates'] and files == state['candidates'][-1]['files']:
            return
        stage = 'code_%02d' % (len(state['candidates']) + 1)
        record = {'stage': stage, 'files': files, 'implementation': state['implementation'],
                  'project_sha256': hashlib.sha256(json.dumps(files, sort_keys=True).encode('utf-8')).hexdigest()}
        write_once(work / (stage + '.json'), record)
        state['candidates'].append(record)

    original_parse_code = CodeParser.parse_code
    def parse_code(block, text, *args, **kwargs):
        action, target = current.get()
        if action in ('WriteCode', 'WriteCodeReview') and target and Path(target).suffix.lower() in ('.c', '.h'):
            files = extract_files(text, target)
            if set(files) != {target}:
                raise ValueError('Native single-file response does not match its requested C filename')
            return files[target]
        return original_parse_code(block, text, *args, **kwargs)
    CodeParser.parse_code = staticmethod(parse_code)

    original_engineer = Engineer._act_write_code
    async def implementation(self):
        state['implementation'] += 1
        expected = set()
        for todo in self.code_todos:
            name = todo.i_context.filename.replace('\\', '/')
            if Path(name).suffix.lower() in ('.c', '.h'):
                expected.add(safe_name(name))
            elif Path(name).suffix.lower() in ('.md', '.txt', '.rst', '.json'):
                # Validate where the native repository will write the document;
                # its file writer performs the same containment check again.
                contained(view / 'library' / name)
            else:
                raise BridgeStop('unsupported_project_artifact', 'Native task is outside the C/document boundary: ' + name)
        state['expected'] = expected
        return await original_engineer(self)
    Engineer._act_write_code = implementation

    # Preserve native QA event routing while substituting C development callers
    # for the Python-only test-file selection and unittest script format.
    async def write_tests(self, message):
        source = self.project_repo.with_src_path(self.context.src_workspace).srcs
        names = sorted(source.changed_files)
        for name in names:
            if not name.endswith('.c') or 'test' in name:
                continue
            code_doc = await source.get(name)
            if not code_doc:
                continue
            test_name = 'test_' + name.replace('/', '_').replace('\\', '_') + '.json'
            test_doc = await self.project_repo.tests.get(test_name)
            if not test_doc:
                test_doc = Document(root_path=str(self.project_repo.tests.root_path), filename=test_name, content='')
            testing = TestingContext(filename=test_name, test_doc=test_doc, code_doc=code_doc)
            testing = await WriteTest(i_context=testing, context=self.context, llm=self.llm).run()
            await self.project_repo.tests.save_doc(doc=testing.test_doc, dependencies={code_doc.root_relative_path})
            execution = RunCodeContext(command=['isolated_c_test', test_name], code_filename=name, test_filename=test_name,
                                       working_directory=str(self.project_repo.workdir), additional_python_paths=[])
            self.publish_message(Message(content=execution.model_dump_json(), role=self.profile, cause_by=WriteTest, sent_from=self, send_to=self))
    QaEngineer._write_test = write_tests

    test_instruction = ('Return one JSON object with 3 to 6 development cases. Each case has id, declarations, body strings. '
        'Use #include "6_generated_code.h" in declarations and any needed device stubs. The body is inside main and uses PB_CHECK(expr) '
        'or PB_NEAR(actual, expected, tolerance). Do not define main. Escape newlines inside JSON strings. '
        'Use meaningful cases derived only from the supplied R/D/API and code; these are not final evaluation cases.')
    async def write_test(self, *args, **kwargs):
        testing = self.i_context
        prompt = 'You are the project QA engineer. Design robust C11 tests, preserving the fixed API.\n' + test_instruction
        prompt += '\n' + inputs + '\n=== Code under review ===\n' + testing.code_doc.content
        response = await self._aask(prompt)
        cases = decode_test_cases(response)
        testing.test_doc.content = json.dumps(cases, ensure_ascii=False)
        state['development_tests'][testing.test_doc.filename] = cases
        return testing
    WriteTest.run = write_test

    async def run_script(self, working_directory, additional_python_paths=None, command=None):
        if not state['candidates']:
            return '', 'No complete C candidate; tests not executed.'
        cases = decode_test_cases(self.i_context.test_code)
        current_candidate = state['candidates'][-1]
        stage = current_candidate['stage'] + '_test_' + hashlib.sha256(json.dumps(cases, sort_keys=True).encode('utf-8')).hexdigest()[:10]
        result = bridge.evaluate(current_candidate['files'], cases, stage)
        state['qa_executions'] += 1
        state['evaluations'][stage] = result
        write_once(work / (stage + '.json'), result)
        summary = json.dumps(result, ensure_ascii=False)
        return summary, '' if result['passed'] else summary
    RunCode.run_script = run_script
    RunCode.run_text = deny
    RunCode._install_dependencies = deny

    async def debug_test(self, *args, **kwargs):
        output_doc = await self.repo.test_outputs.get(self.i_context.output_filename)
        if not output_doc:
            return ''
        output = json.loads(output_doc.content)
        tests = await self.repo.tests.get(self.i_context.test_filename)
        code_doc = await self.repo.with_src_path(self.context.src_workspace).srcs.get(self.i_context.code_filename)
        if not tests or not code_doc:
            return ''
        prompt = debug_error.PROMPT_TEMPLATE.format(code=code_doc.content, test_code=tests.content, logs=output.get('stderr', ''))
        prompt += '\nYour assigned QA file is a C development-case JSON file. ' + test_instruction + '\n' + inputs
        response = await self._aask(prompt)
        cases = decode_test_cases(response)
        state['development_tests'][self.i_context.test_filename] = cases
        return json.dumps(cases, ensure_ascii=False)
    DebugError.run = debug_test

    def wrap_action(cls):
        original = cls.run
        @functools.wraps(original)
        async def wrapped(self, *args, **kwargs):
            name = cls.__name__
            context = getattr(self, 'i_context', None)
            target = getattr(context, 'filename', '') or ''
            if name == 'WritePRD':
                state['design'] += 1
            state['actions'].append(name)
            token = current.set((name, str(target).replace('\\', '/')))
            try:
                result = await original(self, *args, **kwargs)
                if name in ('WriteCode', 'WriteCodeReview'):
                    snapshot(result)
                return result
            except Exception as exc:
                # Upstream Team.serialize_decorator catches ordinary exceptions
                # and returns None. Propagate visible stop state instead of
                # misreporting an aborted framework as completed.
                raise BridgeStop('adapter_or_output_error', name + ': ' + type(exc).__name__ + ': ' + str(exc)) from exc
            finally:
                current.reset(token)
        cls.run = wrapped
    for cls in (PrepareDocuments, WritePRD, WriteDesign, WriteTasks, WriteCode, WriteCodeReview,
                SummarizeCode, WriteTest, RunCode, DebugError):
        wrap_action(cls)

    cfg = Config.default()
    cfg.project_name = 'library'
    cfg.project_path = ''
    cfg.inc = False
    cfg.max_auto_summarize_code = 0
    cfg.workspace.path = view / 'workspace'
    context = context_module.Context(config=cfg)
    company = Team(context=context)
    company.hire([ProductManager(context=context), Architect(context=context), ProjectManager(context=context),
                  Engineer(context=context, n_borg=5, use_code_review=True), QaEngineer(context=context)])
    company.invest(10.0)  # Legacy dollar counter is unused; parent token budget is authoritative.
    company.run_project(task)
    try:
        history = await company.run(n_round=20)
        status = 'native_workflow_finished' if history is not None else 'adapter_or_output_error'
        detail = '' if history is not None else 'Native Team returned no history; inspect preserved runtime error log'
    except BridgeStop as exc:
        status, detail = exc.status, exc.detail
    except Exception as exc:
        status, detail = 'adapter_or_output_error', type(exc).__name__ + ': ' + str(exc)
    return {'status': status, 'detail': detail, 'candidates': state['candidates'],
        'selected_candidate': state['candidates'][-1] if state['candidates'] else None,
        'native_core_used': ['Team.run', 'Environment.run', 'five original roles', 'ActionNode.fill',
                            'Engineer._act_sp_with_cr', 'WriteCodeReview.run', 'QaEngineer._act', 'RunCode.run'],
        'actions': state['actions'], 'qa_executions': state['qa_executions'],
        'development_evaluations': {key: {'passed': value['passed']} for key, value in state['evaluations'].items()},
        'native_model': job['model'], 'scheduler_ticks': 20,
        'native_Engineer_RunCode_watch_behavior_preserved': True}


def main():
    parser = argparse.ArgumentParser()
    for name in ('input', 'repo', 'work', 'bridge'):
        parser.add_argument('--' + name, type=Path, required=True)
    args = parser.parse_args()
    job = json.loads(args.input.read_text(encoding='utf-8'))
    args.work.mkdir(parents=True, exist_ok=True)
    # Allocate the Windows asyncio wakeup socket before denying network calls.
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    try:
        result = loop.run_until_complete(run(job, args.repo.resolve(), args.work.resolve(), RuntimeBridge(args.bridge, job['model'])))
    except BridgeStop as exc:
        result = {'status': exc.status, 'detail': exc.detail, 'selected_candidate': None}
    except Exception as exc:
        result = {'status': 'adapter_or_output_error', 'detail': str(exc), 'error_type': type(exc).__name__, 'selected_candidate': None}
    finally:
        loop.close()
    write_once(args.work / 'result.json', result)
    print(json.dumps({'status': result['status'], 'has_candidate': bool(result.get('selected_candidate'))}))


if __name__ == '__main__':
    main()
