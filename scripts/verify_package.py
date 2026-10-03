"""Verify distributed bytes and source-copy bindings without network access.

Only MANIFEST.json itself and new files below the documented results/ and
outputs/ runtime directories may be absent from the manifest. Links and
Windows reparse points are rejected even below those runtime directories.
"""
from pathlib import Path, PurePosixPath, PureWindowsPath
import hashlib
import json
import os
import re
import stat
import sys

ROOT = Path(__file__).resolve().parents[1]
ALLOWED_RUNTIME_DIRS = frozenset({"results", "outputs"})
HASH = re.compile(r"[0-9a-fA-F]{64}\Z")


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def is_link(path):
    info = path.lstat()
    return (stat.S_ISLNK(info.st_mode)
            or bool(getattr(info, "st_file_attributes", 0)
                    & getattr(stat, "FILE_ATTRIBUTE_REPARSE_POINT", 0x400)))


def checked_path(raw, errors, context):
    if not isinstance(raw, str) or not raw:
        errors.append(context + ": missing path")
        return None
    parts = raw.split("/")
    if ("\\" in raw or ":" in raw or "\x00" in raw
            or PurePosixPath(raw).is_absolute()
            or PureWindowsPath(raw).is_absolute()
            or any(part in {"", ".", ".."} or part.endswith((" ", "."))
                   for part in parts)):
        errors.append(context + ": unsafe path: " + repr(raw))
        return None
    path = ROOT
    try:
        for part in parts:
            path = path / part
            if path.exists() or path.is_symlink():
                if is_link(path):
                    errors.append(context + ": link/reparse point: " + raw)
                    return None
        path.resolve().relative_to(ROOT)
    except (OSError, ValueError, RuntimeError):
        errors.append(context + ": path is not contained in package: " + raw)
        return None
    return path


def load_json(raw, errors):
    path = checked_path(raw, errors, "Metadata")
    if path is None:
        return None
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, ValueError):
        errors.append("Missing or invalid JSON: " + raw)
        return None


def inventory(errors):
    files = set()
    for current, dirs, names in os.walk(ROOT, followlinks=False):
        current = Path(current)
        for name in list(dirs):
            path = current / name
            try:
                if is_link(path):
                    errors.append("Link/reparse point: " + path.relative_to(ROOT).as_posix())
                    dirs.remove(name)
            except OSError:
                errors.append("Unreadable directory: " + path.relative_to(ROOT).as_posix())
                dirs.remove(name)
        for name in names:
            path = current / name
            raw = path.relative_to(ROOT).as_posix()
            try:
                if is_link(path):
                    errors.append("Link/reparse point: " + raw)
                elif path.is_file():
                    files.add(raw)
                else:
                    errors.append("Non-regular file: " + raw)
            except OSError:
                errors.append("Unreadable file: " + raw)
    return files


def main():
    errors = []
    manifest = load_json("MANIFEST.json", errors)
    items = manifest.get("files") if isinstance(manifest, dict) else None
    if not isinstance(items, list):
        errors.append("Manifest files must be an array")
        items = []
    listed = set()
    folded = set()
    for item in items:
        if not isinstance(item, dict):
            errors.append("Invalid manifest entry")
            continue
        raw = item.get("path")
        path = checked_path(raw, errors, "Manifest")
        if path is None:
            continue
        if raw.casefold() in folded:
            errors.append("Duplicate manifest path: " + raw)
        folded.add(raw.casefold())
        listed.add(raw)
        expected = item.get("sha256")
        if not isinstance(expected, str) or not HASH.fullmatch(expected):
            errors.append("Invalid manifest hash: " + raw)
            continue
        try:
            if not path.is_file() or sha(path) != expected.lower():
                errors.append("Hash mismatch: " + raw)
            if "bytes" in item and path.is_file() and path.stat().st_size != item["bytes"]:
                errors.append("Size mismatch: " + raw)
        except OSError:
            errors.append("Unreadable manifest file: " + raw)
    actual = inventory(errors)
    unlisted = sorted(actual - listed - {"MANIFEST.json"})
    allowed_outputs = [raw for raw in unlisted
                       if raw.split("/", 1)[0] in ALLOWED_RUNTIME_DIRS]
    for raw in unlisted:
        if raw not in allowed_outputs:
            errors.append("Unlisted release file: " + raw)
    index = load_json("evidence/source_index.json", errors)
    if not isinstance(index, dict):
        errors.append("Source index must be an object")
        index = {}
    for value in index.values():
        if not isinstance(value, dict):
            errors.append("Invalid source-index entry")
            continue
        raw = value.get("path")
        path = checked_path(raw, errors, "Source index")
        if path is None:
            continue
        if raw not in listed:
            errors.append("Source-copy is not manifest-bound: " + raw)
        expected = value.get("distributed_sha256")
        if not isinstance(expected, str) or not HASH.fullmatch(expected):
            errors.append("Invalid source-copy hash: " + raw)
            continue
        try:
            if not path.is_file() or sha(path) != expected.lower():
                errors.append("Source-copy mismatch: " + raw)
        except OSError:
            errors.append("Unreadable source copy: " + raw)
        if value.get("translation_pending"):
            errors.append("Unfinished translation: " + raw)
    report = {
        "status": "failed" if errors else "passed",
        "manifest_files": len(items),
        "supporting_source_paths": len(index),
        "allowed_runtime_directories": sorted(ALLOWED_RUNTIME_DIRS),
        "unlisted_runtime_output_files": allowed_outputs,
        "errors": errors,
        "external_calls": 0,
    }
    print(json.dumps(report, indent=2))
    return 1 if errors else 0


if __name__ == "__main__":
    sys.exit(main())
