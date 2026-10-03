"""PlantUML RAG retriever — keyword + optional embedding-based retrieval.

Retrieves relevant PlantUML syntax examples from the knowledge base
based on the user's requirements text.
"""

import re
import os
from typing import List, Tuple

from .state_diagram_examples import STATE_DIAGRAM_EXAMPLES, SYNTAX_FIXES
from .sequence_diagram_examples import SEQUENCE_DIAGRAM_EXAMPLES, SEQ_SYNTAX_FIXES


def _tokenize(text: str) -> set:
    """Extract lowercase alphanumeric tokens from text."""
    return set(re.findall(r"[a-zA-Z_][a-zA-Z0-9_]{2,}", text.lower()))


def _keyword_score(tokens: set, entry: dict) -> float:
    """Score an entry by token overlap with its keywords."""
    entry_tokens = set()
    for kw in entry.get("keywords", []):
        entry_tokens.update(_tokenize(kw))
    if not entry_tokens:
        return 0.0
    overlap = tokens & entry_tokens
    return len(overlap) / len(entry_tokens)


def _retrieve(
    query_tokens: set,
    examples: list,
    top_k: int = 5,
    min_score: float = 0.05,
) -> list:
    """Retrieve top-k examples sorted by keyword overlap score."""
    scored = []
    for entry in examples:
        score = _keyword_score(query_tokens, entry)
        if score >= min_score:
            scored.append((score, entry))
    scored.sort(key=lambda x: x[0], reverse=True)
    return [entry for _, entry in scored[:top_k]]


def _format_examples(examples: list, title: str) -> str:
    """Format retrieved examples into a prompt block."""
    if not examples:
        return ""
    lines = [f"### {title} ###"]
    for i, ex in enumerate(examples, 1):
        desc = ex.get("description", "Example")
        code = ex.get("code", "")
        lines.append(f"\n-- Example {i}: {desc} --")
        lines.append(code)
    lines.append("")
    return "\n".join(lines)


class PlantUMLRetriever:
    """Retrieve PlantUML syntax examples relevant to a requirements text."""

    def __init__(self):
        self._cache = {}  # simple cache: req_hash -> formatted result

    def retrieve_for_state_diagram(
        self,
        requirements: str,
        top_k: int = 3,
        include_fixes: bool = True,
    ) -> str:
        """Retrieve state-diagram examples relevant to requirements."""
        if os.getenv("EMBEDDEV_ENABLE_CONTEXT", "0") != "1":
            return ""
        tokens = _tokenize(requirements)
        examples = _retrieve(tokens, STATE_DIAGRAM_EXAMPLES, top_k=top_k)

        result = ""
        if examples:
            result += _format_examples(
                examples,
                "Retrieved State Diagram Syntax Examples (use these patterns)",
            )

        if include_fixes:
            # Always include the note-outside fix (most common error)
            note_fix = [f for f in SYNTAX_FIXES if "note inside" in f.get("symptom", "")]
            brace_fix = [f for f in SYNTAX_FIXES if "mismatched" in f.get("symptom", "")]
            gtlt_fix = [f for f in SYNTAX_FIXES if "< or >" in f.get("symptom", "")]
            critical_fixes = note_fix + brace_fix + gtlt_fix
            if critical_fixes:
                result += _format_fixes(critical_fixes)

        return result

    def retrieve_for_sequence_diagram(
        self,
        requirements: str,
        top_k: int = 3,
        include_fixes: bool = True,
    ) -> str:
        """Retrieve sequence-diagram examples relevant to requirements."""
        if os.getenv("EMBEDDEV_ENABLE_CONTEXT", "0") != "1":
            return ""
        tokens = _tokenize(requirements)
        examples = _retrieve(tokens, SEQUENCE_DIAGRAM_EXAMPLES, top_k=top_k)

        result = ""
        if examples:
            result += _format_examples(
                examples,
                "Retrieved Sequence Diagram Syntax Examples (use these patterns)",
            )

        if include_fixes:
            # Always include the most critical fixes
            bare_text_fix = [f for f in SEQ_SYNTAX_FIXES if "bare text" in f.get("symptom", "")]
            bare_assign_fix = [f for f in SEQ_SYNTAX_FIXES if "Bare assignment" in f.get("symptom", "")]
            orphan_fix = [f for f in SEQ_SYNTAX_FIXES if "Orphan" in f.get("symptom", "")]
            return_fix = [f for f in SEQ_SYNTAX_FIXES if "return" in f.get("symptom", "")]
            critical_fixes = bare_text_fix + bare_assign_fix + orphan_fix + return_fix
            if critical_fixes:
                result += _format_fixes(critical_fixes)

        return result


def _format_fixes(fixes: list) -> str:
    """Format syntax fixes into a prompt block."""
    lines = ["### CRITICAL — Common Mistakes to AVOID ###"]
    for i, fix in enumerate(fixes, 1):
        symp = fix.get("symptom", "")
        wrong = fix.get("wrong", "")
        right = fix.get("right", "")
        lines.append(f"\n-- Fix {i}: {symp} --")
        lines.append(f"WRONG:\n{wrong}")
        lines.append(f"RIGHT:\n{right}")
    lines.append("")
    return "\n".join(lines)


# Singleton
_retriever = None


def get_retriever() -> PlantUMLRetriever:
    global _retriever
    if _retriever is None:
        _retriever = PlantUMLRetriever()
    return _retriever
