"""Offline release checks: no model request, credential load, compiler or Host test."""
from __future__ import annotations

import ast
import importlib
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile

CHECKING = Path(__file__).resolve().parents[1]
PACKAGE = CHECKING.parents[1]
sys.path.insert(0, str(CHECKING))
sys.dont_write_bytecode = True


def main():
    checks = []
    for source in CHECKING.rglob('*.py'):
        ast.parse(source.read_text(encoding='utf-8'), filename=str(source))
    checks.append('all release Python modules parse')

    # Any attempt to initialize a model or search a credential file is a failure.
    from _internal import new_utils
    def forbidden(*args, **kwargs):
        raise AssertionError('Offline smoke attempted model initialization or credential loading')
    new_utils.initialize_llm = forbidden
    new_utils.load_dotenv = forbidden
    for name in ('agent_1_state_modeler', 'agent_2_interface_modeler',
                 'agent_3_behavior_modeler', 'agent_4_code_generator',
                 'agent_5_code_verifier', 'agent_1_state_repairer',
                 'agent_2_interface_repairer', 'agent_3_behavior_repairer'):
        importlib.import_module('agents.' + name)
    assert new_utils.llm._client is None
    checks.append('all active agents import without initializing clients or loading keys')

    from _internal.canonical_models import StateModel, InterfaceModel, BehaviorModel
    for schema in (StateModel, InterfaceModel, BehaviorModel):
        assert schema.model_json_schema()['type'] == 'object'
    checks.append('state/interface/behavior schemas construct')

    from _internal.design_iteration_models import ValidationResult
    from _internal.design_iteration_policy import choose_iteration_action, DesignIterationPolicy
    decision = choose_iteration_action(ValidationResult(valid=True), [], 1, DesignIterationPolicy())
    assert decision.action == 'accept'
    from config import ProjectConfig, DesignIterationProfile
    assert DesignIterationProfile().max_iterations == 4
    checks.append('design policy and release iteration defaults construct')

    from agents.agent_2_api_generator import build_request, run_branch, PROMPT_FILE
    # Deliberately retain a template-looking token in input to check one-pass substitution.
    with tempfile.TemporaryDirectory(prefix='embeddev_offline_') as temp:
        root = Path(temp)
        (root/'RE_req.txt').write_text('REQ-1: keep the literal {{PROJECT_NAME}} in source evidence.\n', encoding='utf-8')
        (root/'RE_api.txt').write_text('void controller_reset(void);\n', encoding='utf-8')
        (root/'device.txt').write_text('A monotonic host clock uses milliseconds.\n', encoding='utf-8')
        (root/'project.json').write_text(json.dumps({'project_name':'Example',
            'inputs':{'requirement':'RE_req.txt','api':'RE_api.txt','device_interface':'device.txt'}}), encoding='utf-8')
        config = ProjectConfig.load(str(root))
        prepared = build_request(config)
        assert 'keep the literal {{PROJECT_NAME}} in source evidence.' in prepared['prompt']
        assert 'monotonic host clock' in prepared['prompt']
        assert not prepared['final_tests_or_scores_loaded']
        original_api = (root/'RE_api.txt').read_bytes()
        branch = run_branch(config, root/'prepared', invoke=forbidden)
        assert branch['status'] == 'prepared' and branch['invocations'] == 0
        assert (root/'RE_api.txt').read_bytes() == original_api
        bad = root/'bad_prompt.md'
        bad.write_text(PROMPT_FILE.read_text(encoding='utf-8')+'\n{{UNKNOWN_FIELD}}', encoding='utf-8')
        try:
            build_request(config, bad)
        except ValueError as exc:
            assert 'unknown' in str(exc)
        else:
            raise AssertionError('Unknown template field was accepted')
        checks.append('English API preparation, one-pass fields and unknown-field rejection')
        child_env = {k:v for k,v in os.environ.items()
                     if not any(word in k.upper() for word in ('KEY','TOKEN','SECRET','PASSWORD'))}
        child_env.update(PYTHONDONTWRITEBYTECODE='1', PYTHONIOENCODING='utf-8',
                         EMBEDDEV_ENV_FILE=str(root/'not-present.env'),
                         EMBEDDEV_MODELS_FILE=str(PACKAGE/'fse/fse_models.json'))
        for args in (['--help'], ['inspect','--input-dir',str(root)],
                     ['run','--input-dir',str(root),'--model','openai','--dry-run'],
                     ['prepare-api','--input-dir',str(root),'--output-dir',str(root/'cli_prepared')]):
            result = subprocess.run([sys.executable,str(CHECKING/'pipeline.py'),*args],
                cwd=str(root),env=child_env,capture_output=True,text=True,encoding='utf-8',timeout=45)
            if result.returncode:
                raise AssertionError(f'Offline CLI failed {args[0]}: {result.stderr}')
        checks.append('help/inspect/dry-run/API preparation work outside the source directory')

    # The public release does not carry or consume the author notebook.
    os.environ.pop('EMBEDDEV_ENABLE_CONTEXT', None)
    from context import retrieve_domain_context, retrieve_error_lessons, record_lesson
    from plantuml_kb.retriever import get_retriever
    assert retrieve_domain_context('motor control', 'agent_1_state_modeler') == ''
    assert retrieve_error_lessons('timer') == ''
    assert get_retriever().retrieve_for_state_diagram('timer state') == ''
    assert get_retriever().retrieve_for_sequence_diagram('timer state') == ''
    assert record_lesson(agent='cross', category='example',error_pattern='example',
                         root_cause='example',successful_fix='example') == {}
    checks.append('optional context and notebook are disabled by default')
    print(json.dumps({'status':'passed','checks':checks,'model_requests':0,
                      'credential_files_read':0,'compiler_invocations':0,'host_tests':0},indent=2))


if __name__ == '__main__':
    main()
