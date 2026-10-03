"""Function API preparation branch feeding the Interface Modeler.

Reads the maintained docs prompt; never changes a frozen API or starts design.
LLM dependencies are imported only for explicitly requested generation.
"""
from __future__ import annotations

from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
from typing import Callable

PROMPT_FILE = Path(__file__).resolve().parents[3] / 'docs/API_GENERATION_PROMPT.md'


def digest(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def build_request(config, prompt_file: Path = PROMPT_FILE) -> dict:
    source = prompt_file.read_bytes()
    document = source.decode('utf-8').replace('\r\n', '\n')
    prompt = document.strip()
    req_path = config.path_for(config.requirement_file)
    req = req_path.read_bytes()
    if not req.decode('utf-8').strip():
        raise ValueError('Requirements must not be empty')
    device_path = config.path_for(config.device_interface_file)
    if config.device_interface_file and (not device_path or not device_path.is_file()):
        raise FileNotFoundError('Configured device interface is missing')
    device = device_path.read_bytes() if device_path else b''
    values = {
        'PROJECT_NAME': config.project_name,
        'ID_PREFIX': config.project_name,
        'LANGUAGE': config.build.standard,
        'PLATFORM': config.build.platform,
        'SOURCE_DOCUMENTS': config.requirement_file,
        'DEVICE_DOCUMENTS': config.device_interface_file or 'none',
        'CANDIDATE_VERSION': '0.1.0-candidate',
        'DATE': datetime.now(timezone.utc).date().isoformat(),
        'HOST_TEST': 'yes' if config.build.platform == 'host' else 'no',
        'PLATFORM_DEPENDENCIES': 'unknown; compiler=' + config.build.compiler,
        'SOURCE_DOCUMENT': req.decode('utf-8'),
        'DEVICE_INTERFACE': device.decode('utf-8') or 'none',
        'CONFIRMED_CONSTRAINTS': f'Target language: {config.build.standard}; build platform: {config.build.platform}. No other human decisions supplied; list ambiguities in the review report.',
    }
    # Substitute in one pass so text within project materials stays byte-for-byte intact.
    import re
    placeholders = set(re.findall(r'\{\{([A-Z_]+)\}\}', prompt))
    if placeholders != set(values):
        missing = sorted(set(values) - placeholders)
        unknown = sorted(placeholders - set(values))
        raise ValueError(f'API prompt template fields differ: missing={missing}, unknown={unknown}')
    prompt = re.sub(r'\{\{([A-Z_]+)\}\}', lambda m: values[m.group(1)], prompt)
    inputs = {'requirements': {'path': str(req_path), 'sha256': digest(req)}}
    if device_path:
        inputs['device_interface'] = {'path': str(device_path), 'sha256': digest(device)}
    return {'agent': 'agent_2_api_generator', 'consumer': 'agent_2_interface_modeler',
            'project': config.project_name, 'prompt': prompt, 'prompt_sha256': digest(prompt.encode('utf-8')),
            'prompt_source': {'path': str(prompt_file), 'sha256': digest(source)}, 'inputs': inputs,
            'frozen_api_reused_by_design': str(config.path_for(config.api_file)),
            'final_tests_or_scores_loaded': False}


def invoke_model(prompt: str) -> dict:
    from pydantic import BaseModel, Field
    from runtime import llm

    class ApiOutput(BaseModel):
        candidate_api: str = Field(min_length=1)
        review_report: dict

    response = llm.with_structured_output(ApiOutput, include_raw=True).invoke(prompt)
    raw = response.get('raw')
    parsed = response.get('parsed')
    return {'candidate_api': parsed.candidate_api if parsed else None,
            'review_report': parsed.review_report if parsed else None,
            'raw_response': raw.model_dump(mode='json') if raw is not None else None,
            'parsing_error': str(response['parsing_error']) if response.get('parsing_error') else None}


def run_branch(config, output_dir: Path, *, generate: bool = False,
               model_name: str | None = None, invoke: Callable | None = None,
               prompt_file: Path = PROMPT_FILE) -> dict:
    output_dir = Path(output_dir).resolve()
    if generate and not model_name:
        raise ValueError('An explicit model is required for generation')
    request = build_request(config, prompt_file)
    # Exclusive new directory: no overwrite of an API, candidate, or previous run.
    output_dir.mkdir(parents=True, exist_ok=False)
    def save(name, obj):
        (output_dir / name).write_text(json.dumps(obj, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    save('request.json', request)
    record = {'agent': request['agent'], 'consumer': request['consumer'], 'project': config.project_name,
              'status': 'prepared', 'model': model_name, 'invocations': 0,
              'frozen_api_modified': False, 'downstream_started': False,
              'review_required': True, 'started_at': datetime.now(timezone.utc).isoformat()}
    save('branch.json', record)
    if not generate:
        return record
    record.update(status='generating', invocations=1)
    save('branch.json', record)
    try:
        result = (invoke or invoke_model)(request['prompt'])
        save('response.json', result)
        candidate, review = result.get('candidate_api'), result.get('review_report')
        if not isinstance(candidate, str) or not candidate.strip() or not isinstance(review, dict):
            raise ValueError('Incomplete API candidate or review report; inspect response.json')
        if re_frozen(candidate):
            raise ValueError('Generated candidate incorrectly claims FROZEN status')
        (output_dir / 'RE_api.candidate.txt').write_text(candidate.strip() + '\n', encoding='utf-8')
        save('API_REVIEW_REPORT.json', review)
        record.update(status='candidate_requires_review', candidate_sha256=digest((output_dir/'RE_api.candidate.txt').read_bytes()))
    except Exception as exc:
        # Do not persist provider exception text, which may contain a request URL or credentials.
        record.update(status='failed', error_type=type(exc).__name__)
        raise
    finally:
        record['finished_at'] = datetime.now(timezone.utc).isoformat()
        save('branch.json', record)
    return record


def re_frozen(candidate: str) -> bool:
    import re
    return bool(re.search(r'^\s*Status\s*:\s*FROZEN\b', candidate, re.I | re.M))
