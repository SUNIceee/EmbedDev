"""Lossless C file extraction and safe staging for official framework adapters."""
import hashlib
import json
from pathlib import Path, PurePosixPath
import re

MAX_FILES = 64
MAX_BYTES = 8 * 1024 * 1024
RESERVED = {'con', 'prn', 'aux', 'nul', *('com%d' % i for i in range(1, 10)), *('lpt%d' % i for i in range(1, 10))}


def file_label(line):
    """Decode explicit Markdown labels, including streamed prose + heading.

    Only text outside fences is inspected. The body of every C file stays
    byte-for-byte equivalent after UTF-8 decoding; no unlabeled file is guessed.
    """
    text = line.strip()
    # Some compatible endpoints join a commentary fragment directly to a final
    # Markdown heading. Require an explicit heading and a complete filename at
    # the end, rather than searching prose for arbitrary mentions of .c/.h.
    headings = list(re.finditer(r'#{1,6}\s+', text))
    if headings:
        text = text[headings[-1].end():].strip()
        text = re.sub(r'^\d+[.)]\s+', '', text)
    text = re.sub(r'^[Ff]ile\s*:\s*', '', text)
    text = (text[:-1] if text.endswith(':') else text).strip()
    for left, right in (('**', '**'), ('__', '__'), ('`', '`'), ('"', '"'), ("'", "'")):
        if text.startswith(left) and text.endswith(right) and len(text) > len(left) + len(right):
            text = text[len(left):-len(right)].strip()
    if re.fullmatch(r'[A-Za-z0-9_.\-/]+\.[ch]', text):
        return safe_name(text)
    return None


def safe_name(value):
    if not isinstance(value, str) or not value or len(value) > 220:
        raise ValueError('Invalid project path')
    if not re.fullmatch(r'[A-Za-z0-9_.\-/]+', value) or '\\' in value:
        raise ValueError('Unsupported characters in project path: ' + value[:120])
    path = PurePosixPath(value)
    if path.is_absolute() or any(part in ('', '.', '..') for part in value.split('/')):
        raise ValueError('Project path escapes or ambiguously addresses workspace: ' + value)
    if path.suffix.lower() not in ('.c', '.h'):
        raise ValueError('Only C source and header files may be staged')
    for part in path.parts:
        if part.startswith('-') or part.endswith('.') or part.split('.')[0].lower() in RESERVED or part.lower().startswith('__fse_'):
            raise ValueError('Reserved project path: ' + value)
    return value


def validate_files(files):
    if not isinstance(files, dict) or not files or len(files) > MAX_FILES:
        raise ValueError('Expected 1..64 C files')
    seen, size = set(), 0
    for name, body in files.items():
        safe_name(name)
        folded = name.lower()
        if folded in seen:
            raise ValueError('File paths collide on Windows: ' + name)
        if any(other.startswith(folded + '/') or folded.startswith(other + '/') for other in seen):
            raise ValueError('File and directory paths conflict: ' + name)
        seen.add(folded)
        if not isinstance(body, str) or '\x00' in body:
            raise ValueError('Invalid source content')
        size += len(body.encode('utf-8'))
    if size > MAX_BYTES:
        raise ValueError('Project exceeds staging size bound')
    return dict(files)


def extract_files(response, default_source=None):
    """Use explicit Markdown file labels without rewriting content or case.

    A sole unnamed C block may use the adapter's declared default filename.
    Malformed/ambiguous responses fail visibly; no partial code reconstruction.
    """
    if not isinstance(response, str) or not response.strip():
        raise ValueError('Empty code response')
    files, pending, language, body, opening = {}, None, None, [], None
    unnamed = []
    for line in response.splitlines(keepends=True):
        stripped = line.strip()
        if opening is not None:
            if re.fullmatch(r'`{3,}\s*', stripped):
                content = ''.join(body)
                if language in ('c', 'h', 'C', ''):
                    if pending:
                        name = safe_name(pending)
                        if name in files and files[name] != content:
                            raise ValueError('Conflicting repeated file in one response: ' + name)
                        files[name] = content
                    else:
                        unnamed.append(content)
                opening, language, body, pending = None, None, [], None
            else:
                body.append(line)
            continue
        if stripped.startswith('```'):
            header = stripped[3:].strip()
            # Explicit fence form: ```c filename=src/foo.c
            explicit = re.fullmatch(r'(c|h|C)\s+(?:filename=)?([A-Za-z0-9_.\-/]+\.[ch])', header)
            if explicit:
                language, pending = explicit.groups()
            else:
                language = header
            opening = True
            continue
        label = file_label(line)
        if label:
            pending = label
        elif stripped:
            pending = None
    if opening is not None:
        raise ValueError('Unclosed code block; incomplete code is not a candidate')
    if unnamed:
        if len(unnamed) != 1 or files or not default_source:
            raise ValueError('Code blocks need unambiguous C file names')
        files[safe_name(default_source)] = unnamed[0]
    return validate_files(files)


def project_hash(files, tests):
    payload = {'files': validate_files(files), 'tests': tests}
    return hashlib.sha256(json.dumps(payload, sort_keys=True, ensure_ascii=False).encode('utf-8')).hexdigest()


def stage_packet(directory, files, tests=None):
    directory = Path(directory).resolve()
    files = validate_files(files)
    packet = {'format': 'official_C_project_v1', 'files': files, 'tests': tests or {'cases': []}}
    packet['sha256'] = project_hash(packet['files'], packet['tests'])
    directory.mkdir(parents=True, exist_ok=True)
    target = directory / 'project.json'
    encoded = json.dumps(packet, ensure_ascii=False, indent=2).encode('utf-8')
    if target.exists():
        if target.read_bytes() != encoded:
            raise ValueError('Immutable candidate packet already exists with different content')
    else:
        target.write_bytes(encoded)
    return target
