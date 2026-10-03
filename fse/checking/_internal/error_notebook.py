"""Lightweight, local RAG notebook for recurring design-generation errors."""

from __future__ import annotations

from collections import Counter
from datetime import datetime
import hashlib
import json
import os
from pathlib import Path
import re
from typing import Any, Dict, Iterable, List, Optional


DEFAULT_NOTEBOOK_PATH = (
    Path(__file__).resolve().parent
    / "knowledge_base"
    / "design_error_notebook.jsonl"
)


def _tokens(text: str) -> List[str]:
    lowered = text.lower()
    latin = re.findall(r"[a-z_][a-z0-9_]{1,}", lowered)
    chinese_runs = re.findall(r"[\u4e00-\u9fff]+", lowered)
    chinese = [
        run[index : index + 2]
        for run in chinese_runs
        for index in range(max(1, len(run) - 1))
        if run[index : index + 2]
    ]
    return latin + chinese


def load_entries(path: Path = DEFAULT_NOTEBOOK_PATH) -> List[Dict[str, Any]]:
    if os.getenv("EMBEDDEV_ENABLE_CONTEXT", "0") != "1":
        return []
    if not path.exists():
        return []
    entries = []
    for line_number, line in enumerate(
        path.read_text(encoding="utf-8").splitlines(), start=1
    ):
        if not line.strip():
            continue
        try:
            entry = json.loads(line)
        except json.JSONDecodeError as exc:
            print(f"[ErrorNotebook] Ignoring invalid JSONL line {line_number}: {exc}")
            continue
        if isinstance(entry, dict):
            entries.append(entry)
    return entries


def retrieve_error_lessons(
    query: str,
    agent: Optional[str] = None,
    top_k: int = 4,
    path: Path = DEFAULT_NOTEBOOK_PATH,
) -> str:
    """Return top recurring lessons using deterministic BM25-like scoring."""
    entries = load_entries(path)
    if not entries or top_k <= 0:
        return ""

    query_tokens = Counter(_tokens(query))
    scored = []
    for entry in entries:
        entry_agent = str(entry.get("agent", "cross"))
        if agent and entry_agent not in {agent, "cross"}:
            continue
        searchable = " ".join(
            str(entry.get(key, ""))
            for key in ("error_pattern", "root_cause", "successful_fix", "tags")
        )
        document_tokens = Counter(_tokens(searchable))
        overlap = sum(
            min(query_count, document_tokens[token])
            for token, query_count in query_tokens.items()
        )
        agent_bonus = 3 if agent and entry_agent == agent else 0
        recurrence_bonus = min(int(entry.get("occurrences", 1)), 5) * 0.25
        score = overlap + agent_bonus + recurrence_bonus
        if score > 0:
            scored.append((score, entry))

    scored.sort(
        key=lambda item: (
            item[0],
            int(item[1].get("occurrences", 1)),
            str(item[1].get("id", "")),
        ),
        reverse=True,
    )
    selected = scored[:top_k]
    if not selected:
        return ""

    lines = ["### Retrieved design-error lessons"]
    for score, entry in selected:
        lines.extend(
            [
                f"- Pattern: {entry.get('error_pattern', '')}",
                f"  Root cause: {entry.get('root_cause', '')}",
                f"  Proven fix: {entry.get('successful_fix', '')}",
                f"  Applies to: {entry.get('agent', 'cross')} "
                f"(score={score:.2f}, occurrences={entry.get('occurrences', 1)})",
            ]
        )
    print(
        f"[ErrorNotebook] Retrieved {len(selected)} lessons for "
        f"{agent or 'cross-agent'}"
    )
    return "\n".join(lines)


def record_lesson(
    *,
    agent: str,
    category: str,
    error_pattern: str,
    root_cause: str,
    successful_fix: str,
    tags: Iterable[str] = (),
    source: str = "runtime",
    path: Path = DEFAULT_NOTEBOOK_PATH,
) -> Dict[str, Any]:
    """Append or merge a validated lesson without duplicating fingerprints."""
    if os.getenv("EMBEDDEV_ENABLE_CONTEXT", "0") != "1":
        return {}
    fingerprint_input = "|".join(
        [agent, category, error_pattern.strip().lower(), successful_fix.strip().lower()]
    )
    lesson_id = hashlib.sha256(
        fingerprint_input.encode("utf-8")
    ).hexdigest()[:16]
    entries = load_entries(path)
    now = datetime.now().isoformat(timespec="seconds")
    replacement = {
        "id": lesson_id,
        "agent": agent,
        "category": category,
        "error_pattern": error_pattern,
        "root_cause": root_cause,
        "successful_fix": successful_fix,
        "tags": sorted(set(tags)),
        "source": source,
        "occurrences": 1,
        "first_seen": now,
        "last_seen": now,
    }
    merged = []
    found = False
    for entry in entries:
        if entry.get("id") == lesson_id:
            replacement["occurrences"] = int(entry.get("occurrences", 1)) + 1
            replacement["first_seen"] = entry.get("first_seen", now)
            found = True
            merged.append(replacement)
        else:
            merged.append(entry)
    if not found:
        merged.append(replacement)

    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(
        "".join(
            json.dumps(entry, ensure_ascii=False, sort_keys=True) + "\n"
            for entry in merged
        ),
        encoding="utf-8",
    )
    temporary.replace(path)
    return replacement
