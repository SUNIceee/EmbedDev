"""Credential-free local file bridge used by the pinned framework runtimes."""
import hashlib
import json
import os
from pathlib import Path
import time


class BridgeStop(BaseException):
    """Do not let upstream generic retries swallow an accounting/service stop."""
    def __init__(self, status, detail=''):
        super().__init__(status + ': ' + detail)
        self.status, self.detail = status, detail


def content_hash(value):
    return hashlib.sha256(json.dumps(value, ensure_ascii=False, sort_keys=True).encode('utf-8')).hexdigest()


def write_once(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    data = json.dumps(value, ensure_ascii=False, indent=2).encode('utf-8')
    if path.exists():
        if path.read_bytes() != data:
            raise BridgeStop('checkpoint_mismatch', str(path.name))
        return
    temporary = path.with_suffix('.tmp')
    temporary.write_bytes(data)
    os.replace(temporary, path)


class RuntimeBridge:
    def __init__(self, directory, model, wait_seconds=2850):
        self.directory = Path(directory)
        self.model = model
        self.cursor = 0
        self.wait_seconds = wait_seconds

    def exchange(self, payload):
        self.cursor += 1
        folder = self.directory / ('message_%04d' % self.cursor)
        request = dict(payload, model=self.model)
        request['sha256'] = content_hash(request)
        write_once(folder / 'request.json', request)
        start = time.monotonic()
        while not (folder / 'response.json').exists():
            if time.monotonic() - start > self.wait_seconds:
                raise BridgeStop('bridge_wait_timeout', 'Parent bridge did not respond')
            time.sleep(0.1)
        response = json.loads((folder / 'response.json').read_text(encoding='utf-8'))
        if response['request_sha256'] != request['sha256']:
            raise BridgeStop('checkpoint_mismatch', 'Bridge response hash')
        if response['status'] != 'ok':
            raise BridgeStop(response['status'], response.get('detail', ''))
        return response['result']

    def complete(self, messages, stage, round_kind=None, round_label=None):
        return self.exchange({'op': 'complete', 'messages': messages, 'stage': stage,
                              'round_kind': round_kind, 'round_label': round_label})

    def evaluate(self, files, tests, stage):
        return self.exchange({'op': 'evaluate', 'files': files, 'tests': tests, 'stage': stage})
