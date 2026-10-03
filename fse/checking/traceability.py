"""Deterministic requirement-to-model-to-code traceability JSON for FSE."""

from __future__ import annotations

from collections import Counter
from datetime import datetime
import json
from pathlib import Path
import re
from typing import Any, Iterable

from _internal.canonical_models import _CALLABLE_REFERENCE, _callable_name
from _internal.c_contract import strip_c_comments


_WORD = re.compile(r"[A-Za-z_][A-Za-z0-9_]{2,}")
_FUNCTION = re.compile(
    r"(?m)^\s*(?:[A-Za-z_]\w*\s+|\*\s*)+([A-Za-z_]\w*)\s*\([^;{}]*\)\s*\{"
)
_CONTROL = {"if", "for", "while", "switch", "return", "sizeof"}


def _tokens(text: str) -> set[str]:
    return {word.lower() for word in _WORD.findall(text or "")}


def _node(node_id: str, kind: str, label: str, **data: Any) -> dict[str, Any]:
    return {"id": node_id, "kind": kind, "label": label, "data": data}


def _requirement_nodes(requirements: str) -> list[dict[str, Any]]:
    nodes: list[dict[str, Any]] = []
    for line_no, raw in enumerate(requirements.splitlines(), start=1):
        text = raw.strip()
        if text:
            nodes.append(_node(f"REQ-L{line_no:04d}", "requirement", text, line=line_no))
    if not nodes:
        nodes.append(_node("REQ-L0000", "requirement", "(empty requirement)", line=0))
    return nodes


def _best_requirement_ids(anchors: Iterable[Any], requirements: list[dict[str, Any]]) -> list[str]:
    selected: list[str] = []
    for anchor in anchors or []:
        text = str(anchor).strip()
        if not text:
            continue
        anchor_tokens = _tokens(text)
        best: tuple[float, str] | None = None
        for requirement in requirements:
            label = requirement["label"]
            # Preserve explicit testcase/requirement identifiers when present.
            if re.search(r"(?:TC|REQ)[-_ ]?\d+", text, re.I) and re.search(
                r"(?:TC|REQ)[-_ ]?\d+", label, re.I
            ):
                score = 1.0 if re.search(r"(?:TC|REQ)[-_ ]?\d+", text, re.I).group(0).lower() in label.lower() else 0.0
            else:
                overlap = len(anchor_tokens & _tokens(label))
                score = overlap / max(1, len(anchor_tokens))
            if best is None or score > best[0]:
                best = (score, requirement["id"])
        if best and best[0] >= 0.25:
            selected.append(best[1])
    return list(dict.fromkeys(selected))


def _edge(source: str, target: str, relation: str, status: str, evidence: list[str] | None = None) -> dict[str, Any]:
    return {
        "source_id": source,
        "target_id": target,
        "relation": relation,
        "status": status,
        "evidence": evidence or [],
    }


def _code_functions(code: str) -> dict[str, int]:
    functions: dict[str, int] = {}
    code = strip_c_comments(code or "")
    for match in _FUNCTION.finditer(code):
        name = match.group(1)
        if name not in _CONTROL:
            functions[name] = code.count("\n", 0, match.start(1)) + 1
    return functions


def build_traceability(
    *,
    requirements: str,
    state_model: dict[str, Any] | None,
    interface_model: dict[str, Any] | None,
    behavior_model: dict[str, Any] | None,
    generated_code: str | None = None,
) -> dict[str, Any]:
    """Build a compact, evidence-carrying graph without an additional LLM call."""

    state_model = state_model or {}
    interface_model = interface_model or {}
    behavior_model = behavior_model or {}
    requirements_nodes = _requirement_nodes(requirements)
    nodes = list(requirements_nodes)
    edges: list[dict[str, Any]] = []
    requirement_links: set[str] = set()
    interface_ids: dict[str, str] = {}
    code_functions = _code_functions(generated_code or "")

    def implementation_status(function_name: str) -> str:
        if generated_code is None:
            return "pending_code_evidence"
        return "covered" if function_name in code_functions else "missing"

    def link_requirements(target_id: str, anchors: Iterable[Any], relation: str) -> None:
        matched = _best_requirement_ids(anchors, requirements_nodes)
        for req_id in matched:
            edges.append(_edge(req_id, target_id, relation, "covered", [str(item) for item in anchors or []]))
            requirement_links.add(req_id)

    for index, variable in enumerate(state_model.get("variables") or [], start=1):
        name = str(variable.get("name") or f"variable_{index}")
        node_id = f"STATE-VAR:{name}"
        nodes.append(_node(node_id, "state_variable", name, type_name=variable.get("type_name", "")))
        link_requirements(node_id, variable.get("source_requirements", []), "refines")

    def add_transition(node_id: str, index: int, transition: dict[str, Any], **context: Any) -> None:
        label = str(transition.get("label") or transition.get("event") or f"transition_{index}")
        node = _node(node_id, "state_transition", label)
        node["data"].update(transition)
        node["data"].update(context)
        nodes.append(node)
        link_requirements(node_id, transition.get("source_requirements", []), "refines")

    # Retain legacy root-level IDs; machine-scoped IDs cannot collide with them.
    for index, transition in enumerate(state_model.get("transitions") or [], start=1):
        add_transition(f"STATE-TRANS:{index:03d}", index, transition)
    for machine_index, machine in enumerate(state_model.get("machines") or [], start=1):
        for index, transition in enumerate(machine.get("transitions") or [], start=1):
            add_transition(
                f"STATE-TRANS:M{machine_index:03d}:{index:03d}", index, transition,
                machine_name=machine.get("name", ""),
                model_path=f"machines[{machine_index - 1}].transitions[{index - 1}]",
            )

    for function in interface_model.get("functions") or []:
        reference = _CALLABLE_REFERENCE.match(str(function.get("name") or ""))
        if not reference:
            continue
        name = reference.group(1)
        node_id = f"INTERFACE-FUNC:{name}"
        interface_ids[name] = node_id
        nodes.append(_node(node_id, "interface_function", name, return_type=function.get("return_type", "")))
        anchors = list(function.get("source_requirements", [])) + list(function.get("req_anchors", []))
        link_requirements(node_id, anchors, "refines")
        for state_name in list(function.get("reads_state", [])) + list(function.get("writes_state", [])):
            target = f"STATE-VAR:{state_name}"
            if any(item["id"] == target for item in nodes):
                edges.append(_edge(node_id, target, "reads_or_writes", "covered"))

    for scenario_index, scenario in enumerate(behavior_model.get("scenarios") or [], start=1):
        scenario_name = str(scenario.get("name") or f"scenario_{scenario_index}")
        scenario_id = f"BEHAVIOR-SCEN:{scenario_index:03d}"
        nodes.append(_node(scenario_id, "behavior_scenario", scenario_name, trigger=scenario.get("trigger", "")))
        link_requirements(scenario_id, scenario.get("source_requirements", []), "refines")
        for step in scenario.get("steps") or []:
            order = step.get("order", 0)
            function_reference = str(step.get("function") or "")
            reference = _CALLABLE_REFERENCE.match(function_reference)
            declared_name = reference.group(1) if reference else None
            # A declaration/definition disambiguates names such as loop or complete.
            function_name = (declared_name if declared_name in interface_ids or declared_name in code_functions
                             else _callable_name(function_reference))
            step_id = f"BEHAVIOR-STEP:{scenario_index:03d}:{order}"
            nodes.append(_node(
                step_id, "behavior_step", function_name or function_reference or f"step_{order}",
                scenario_id=scenario_id, function_name=function_name,
                function_reference=function_reference,
            ))
            link_requirements(step_id, step.get("step_req_anchors", []), "refines")
            if function_name in interface_ids:
                edges.append(_edge(step_id, interface_ids[function_name], "invokes", "covered"))
            elif function_name:
                # Behavior models legitimately include internal orchestration and
                # platform-adapter steps that are not part of the frozen public
                # interface. Their implementation is checked by the later
                # behavior-step -> code-function edge, not by Agent 2.
                edges.append(_edge(
                    step_id,
                    f"INTERNAL-OR-PLATFORM-FUNC:{function_name}",
                    "uses_internal_or_platform",
                    "covered" if function_name in code_functions else "pending_code_evidence",
                ))

    for function_name, line in code_functions.items():
        nodes.append(_node(f"CODE-FUNC:{function_name}", "code_function", function_name, line=line))

    for function_name, interface_id in interface_ids.items():
        code_id = f"CODE-FUNC:{function_name}"
        edges.append(_edge(interface_id, code_id, "implemented_by", implementation_status(function_name)))

    for node in list(nodes):
        if node["kind"] != "behavior_step":
            continue
        function_name = node["data"]["function_name"]
        if function_name:
            code_id = f"CODE-FUNC:{function_name}"
            edges.append(_edge(node["id"], code_id, "realized_by", implementation_status(function_name)))

    uncovered_requirements = [node["id"] for node in requirements_nodes if node["id"] not in requirement_links]
    status_counts = Counter(edge["status"] for edge in edges)
    return {
        "schema_version": "1.1",
        "generated_at": datetime.now().isoformat(timespec="seconds"),
        "generator": "fse.traceability.deterministic",
        "nodes": nodes,
        "edges": edges,
        "summary": {
            "code_evidence_available": generated_code is not None,
            "implementation_evidence": "function_name_match_only",
            "node_counts": dict(Counter(node["kind"] for node in nodes)),
            "edge_counts": dict(status_counts),
            "uncovered_requirement_ids": uncovered_requirements,
            "missing_implementation_edges": sum(1 for edge in edges if edge["relation"] in {"implemented_by", "realized_by"} and edge["status"] == "missing"),
        },
    }


def write_traceability(path: Path, **kwargs: Any) -> dict[str, Any]:
    traceability = build_traceability(**kwargs)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(traceability, ensure_ascii=False, indent=2), encoding="utf-8")
    return traceability
