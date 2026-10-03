"""Deterministic validation for state, interface, and behavior design artifacts."""

from __future__ import annotations

import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import tempfile
from typing import Any, Dict, Iterable, List, Optional, Set, Tuple

from _internal.canonical_models import BehaviorModel, InterfaceModel, StateModel, validate_models
from _internal.design_iteration_models import (
    DesignIssue,
    IssueSeverity,
    ValidationResult,
)

from _internal.plantuml_validation import check_plantuml


DEFAULT_RUN_TIMEOUT_SECONDS = int(os.getenv("RUN_TIMEOUT_SECONDS", "600"))


_ARTIFACT_FIELDS = {
    "state_diagram": "1_state_diagram.puml",
    "state_model": "1_state_model.json",
    "function_skeleton": "2_function_skeleton.h",
    "interface_model": "2_interface_model.json",
    "sequence_diagram": "3_sequence_diagram.puml",
    "behavior_model": "3_behavior_model.json",
}


def _issue(
    code: str,
    agent: str,
    severity: IssueSeverity,
    category: str,
    artifact: str,
    message: str,
    **details: Any,
) -> DesignIssue:
    return DesignIssue(
        code=code,
        agent=agent,
        severity=severity,
        category=category,
        artifact=artifact,
        message=message,
        details=details,
    )


def _validate_required_artifacts(state: Dict[str, Any]) -> List[DesignIssue]:
    issues = []
    for field, artifact in _ARTIFACT_FIELDS.items():
        value = state.get(field)
        if value is None or value == "" or value == {} or value == []:
            agent = "agent_1_state_modeler" if field.startswith("state_") else (
                "agent_2_interface_modeler" if field in {"function_skeleton", "interface_model"} else "agent_3_behavior_modeler"
            )
            issues.append(
                _issue(
                    "DESIGN_MISSING_ARTIFACT",
                    agent,
                    IssueSeverity.CRITICAL,
                    "missing_artifact",
                    artifact,
                    f"Required design artifact {artifact} is empty or missing.",
                )
            )
    return issues


def _validate_plantuml(text: str, kind: str, owner: str, artifact: str, prefix: str):
    report = check_plantuml(text, kind)
    checks = {f"{kind.lower()}_plantuml": report}
    if report["status"] == "valid":
        return [], checks
    infrastructure = report["status"] == "validator_failure"
    location = f" at line {report['line']}" if report.get("line") else ""
    issue = _issue(
        f"{prefix}_PLANTUML_" + ("UNAVAILABLE" if infrastructure else "SYNTAX"),
        owner,
        IssueSeverity.HIGH if infrastructure else IssueSeverity.LOW,
        "validator_failure" if infrastructure else "uml_syntax",
        artifact,
        f"PlantUML{location}: {report['diagnostic']}",
        **report,
    )
    return [issue], checks


def _validate_state_plantuml(text: str) -> Tuple[List[DesignIssue], Dict[str, Any]]:
    return _validate_plantuml(
        text, "STATE", "agent_1_state_modeler", "1_state_diagram.puml", "A1",
    )


def _validate_sequence_plantuml(text: str) -> Tuple[List[DesignIssue], Dict[str, Any]]:
    return _validate_plantuml(
        text, "SEQUENCE", "agent_3_behavior_modeler", "3_sequence_diagram.puml", "A3",
    )


def _extract_api_symbols(api_spec: str) -> Dict[str, Set[str]]:
    from _internal.c_contract import strip_c_comments
    api_spec = strip_c_comments(api_spec)
    functions = set(
        re.findall(
            r"(?m)^\s*(?!typedef\b|extern\b|#)"
            r"(?:const\s+)?(?:struct\s+)?[A-Za-z_]\w*"
            r"(?:\s+[A-Za-z_]\w*)*\s*\**\s+"
            r"([A-Za-z_]\w*)\s*\(",
            api_spec,
        )
    )
    constants = set(
        re.findall(r"(?m)^#define\s+([A-Za-z_]\w*)(?:\s|\()", api_spec)
    )
    types = set(
        re.findall(
            r"(?ms)typedef\s+(?:struct|enum)\s*(?:\w+\s*)?"
            r"\{.*?\}\s*([A-Za-z_]\w*)\s*;",
            api_spec,
        )
    )
    types.update(
        re.findall(
            r"(?m)^typedef\s+[^;{}]*\(\*([A-Za-z_]\w*)\)\s*\([^;]*\)\s*;",
            api_spec,
        )
    )
    types.update(
        re.findall(
            r"(?m)^typedef\s+(?!struct\b|enum\b)(?![^;]*\(\*)"
            r"[^;(){}]+\s+([A-Za-z_]\w*)\s*;",
            api_spec,
        )
    )
    return {"functions": functions, "constants": constants, "types": types}


def _declaration(text: str, name: str) -> Optional[str]:
    from _internal.c_contract import strip_c_comments
    text = strip_c_comments(text)
    match = re.search(
        r"(?ms)^\s*([^\n;{}]*?\b"
        + re.escape(name)
        + r"\s*\(.*?\)\s*);",
        text,
    )
    return match.group(1) if match else None


def _normalize_declaration(value: Optional[str]) -> Optional[str]:
    if value is None:
        return None
    compact = " ".join(value.split())
    return re.sub(r"\s*([(),\[\]*])\s*", r"\1", compact)


def _normalize_type(value: str) -> str:
    compact = " ".join(value.strip().split())
    return re.sub(r"\s*([\[\]*])\s*", r"\1", compact)


def _split_parameters(parameters: str) -> List[str]:
    parts = []
    current = []
    depth = 0
    for character in parameters:
        if character in "([":
            depth += 1
        elif character in ")]":
            depth = max(0, depth - 1)
        if character == "," and depth == 0:
            parts.append("".join(current).strip())
            current = []
        else:
            current.append(character)
    if current:
        parts.append("".join(current).strip())
    return parts


def _parse_api_function_contract(
    api_spec: str,
    name: str,
) -> Optional[Dict[str, Any]]:
    raw = _declaration(api_spec, name)
    if not raw:
        return None
    match = re.match(
        r"(?s)^\s*(.*?)\b" + re.escape(name) + r"\s*\((.*)\)\s*$",
        raw,
    )
    if not match:
        return None
    return_type = _normalize_type(match.group(1))
    parameter_text = match.group(2).strip()
    if not parameter_text or parameter_text == "void":
        parameters = []
    else:
        parameters = []
        for parameter in _split_parameters(parameter_text):
            array_match = re.match(
                r"^(.*?)\s+([A-Za-z_]\w*)\s*(\[[^\]]*\])$",
                parameter,
            )
            normal_match = re.match(
                r"^(.*?[\s*])([A-Za-z_]\w*)$",
                parameter,
            )
            if array_match:
                type_name = _normalize_type(
                    array_match.group(1) + array_match.group(3)
                )
                parameter_name = array_match.group(2)
            elif normal_match:
                type_name = _normalize_type(normal_match.group(1))
                parameter_name = normal_match.group(2)
            else:
                type_name = _normalize_type(parameter)
                parameter_name = ""
            parameters.append(
                {"name": parameter_name, "type_name": type_name}
            )
    return {"return_type": return_type, "parameters": parameters}


def _validate_api_fidelity(
    interface_model: InterfaceModel,
    api_spec: str,
) -> Tuple[List[DesignIssue], Dict[str, Any]]:
    expected = _extract_api_symbols(api_spec)
    actual = {
        "functions": {function.name for function in interface_model.functions},
        "constants": set(interface_model.constants),
        "types": {type_contract.name for type_contract in interface_model.types},
    }
    issues: List[DesignIssue] = []
    checks: Dict[str, Any] = {}
    for symbol_kind in ("functions", "constants", "types"):
        missing = sorted(expected[symbol_kind] - actual[symbol_kind])
        extra = sorted(actual[symbol_kind] - expected[symbol_kind])
        checks[f"api_{symbol_kind}"] = {
            "expected": len(expected[symbol_kind]),
            "actual": len(actual[symbol_kind]),
            "missing": missing,
            "extra": extra,
        }
        if missing or extra:
            issues.append(
                _issue(
                    f"A2_API_{symbol_kind.upper()}",
                    "agent_2_interface_modeler",
                    IssueSeverity.HIGH,
                    "api_contract",
                    "2_interface_model.json",
                    f"Interface-model {symbol_kind} differ from the frozen API.",
                    missing=missing,
                    extra=extra,
                )
            )

    contract_differences = []
    model_functions = {
        function.name: function for function in interface_model.functions
    }
    for function_name in sorted(expected["functions"] & actual["functions"]):
        expected_contract = _parse_api_function_contract(
            api_spec, function_name
        )
        model_function = model_functions[function_name]
        actual_contract = {
            "return_type": _normalize_type(model_function.return_type),
            "parameters": [
                {
                    "name": parameter.name,
                    "type_name": _normalize_type(parameter.type_name),
                }
                for parameter in model_function.parameters
            ],
        }
        if expected_contract != actual_contract:
            contract_differences.append(
                {
                    "name": function_name,
                    "expected": expected_contract,
                    "actual": actual_contract,
                }
            )
    checks["api_function_contracts"] = {
        "checked": len(expected["functions"] & actual["functions"]),
        "differences": contract_differences,
    }
    if contract_differences:
        issues.append(
            _issue(
                "A2_API_FUNCTION_SIGNATURES",
                "agent_2_interface_modeler",
                IssueSeverity.HIGH,
                "api_contract",
                "2_interface_model.json",
                "Interface-model function signatures differ from the frozen API.",
                differences=contract_differences,
            )
        )
    return issues, checks


def _has_text(value: Any) -> bool:
    return isinstance(value, str) and bool(value.strip())


def _has_text_list(values: Iterable[Any]) -> bool:
    return any(_has_text(value) for value in values)



def _validate_interface_semantics(
    interface_model: InterfaceModel,
    requirements: str = "",
    state_model: Optional[StateModel] = None,
) -> Tuple[List[DesignIssue], Dict[str, Any]]:
    """Ensure Interface Modeler emits semantic contracts, not only API signatures."""

    issues: List[DesignIssue] = []
    functions = interface_model.functions
    total = len(functions)
    checks: Dict[str, Any] = {
        "interface_semantics": {
            "functions": total,
            "missing_purpose": [],
            "missing_preconditions": [],
            "missing_postconditions": [],
            "missing_error_cases": [],
            "missing_source_requirements": [],
            "empty_state_access": [],
        }
    }
    if not functions:
        return issues, checks

    missing_purpose: List[str] = []
    missing_preconditions: List[str] = []
    missing_postconditions: List[str] = []
    missing_error_cases: List[str] = []
    missing_source_requirements: List[str] = []
    empty_state_access: List[str] = []

    for function in functions:
        if not _has_text(function.purpose):
            missing_purpose.append(function.name)
        if not _has_text_list(function.preconditions):
            missing_preconditions.append(function.name)
        if not _has_text_list(function.postconditions):
            missing_postconditions.append(function.name)
        if not _has_text_list(function.error_cases):
            missing_error_cases.append(function.name)
        if not _has_text_list(function.source_requirements):
            missing_source_requirements.append(function.name)
        if not function.reads_state and not function.writes_state:
            empty_state_access.append(function.name)

    semantic_missing = {
        "missing_purpose": missing_purpose,
        "missing_preconditions": missing_preconditions,
        "missing_postconditions": missing_postconditions,
        "missing_error_cases": missing_error_cases,
        "missing_source_requirements": missing_source_requirements,
        "empty_state_access": empty_state_access,
    }
    checks["interface_semantics"].update(
        {
            key: {
                "count": len(names),
                "sample": names[:20],
            }
            for key, names in semantic_missing.items()
        }
    )

    required_semantic_missing = {
        key: names
        for key, names in semantic_missing.items()
        if key != "empty_state_access" and names
    }
    if required_semantic_missing:
        worst_count = max(len(names) for names in required_semantic_missing.values())
        severity = (
            IssueSeverity.HIGH
            if worst_count / max(total, 1) >= 0.25
            else IssueSeverity.MEDIUM
        )
        issues.append(
            _issue(
                "A2_INTERFACE_SEMANTICS_INCOMPLETE",
                "agent_2_interface_modeler",
                severity,
                "semantic_contract",
                "2_interface_model.json",
                "Interface Modeler function contracts are missing semantic fields required by downstream code generation.",
                total_functions=total,
                missing={
                    key: {
                        "count": len(names),
                        "sample": names[:20],
                    }
                    for key, names in required_semantic_missing.items()
                },
            )
        )

    if len(empty_state_access) == total and total >= 3:
        issues.append(
            _issue(
                "A2_INTERFACE_STATE_ACCESS_EMPTY",
                "agent_2_interface_modeler",
                IssueSeverity.MEDIUM,
                "semantic_contract",
                "2_interface_model.json",
                "No Interface Modeler function declares reads_state or writes_state; this is suspicious for stateful firmware and should be explicit.",
                total_functions=total,
                sample=empty_state_access[:20],
            )
        )

    return issues, checks


def _validate_header(
    header: str,
    api_spec: str,
    build_profile: Dict[str, Any],
) -> Tuple[List[DesignIssue], Dict[str, Any]]:
    issues: List[DesignIssue] = []
    checks: Dict[str, Any] = {}
    board_application = build_profile.get("target_profile") == "stm32_gpio_application"
    forbidden = [
        token for token in ("stm32f4xx_hal.h", "FreeRTOS.h", " main(")
        if token in header and not (board_application and token in ("stm32f4xx_hal.h", " main("))
    ]
    if forbidden:
        issues.append(
            _issue(
                "A2_HEADER_FORBIDDEN_CONTENT",
                "agent_2_interface_modeler",
                IssueSeverity.HIGH,
                "api_contract",
                "2_function_skeleton.h",
                "Header contains forbidden platform dependencies or a main entry point.",
                tokens=forbidden,
            )
        )

    expected_names = _extract_api_symbols(api_spec)["functions"]

    signature_differences = []
    for name in sorted(expected_names):
        expected_declaration = _normalize_declaration(
            _declaration(api_spec, name)
        )
        actual_declaration = _normalize_declaration(
            _declaration(header, name)
        )
        if expected_declaration != actual_declaration:
            signature_differences.append(
                {
                    "name": name,
                    "expected": expected_declaration,
                    "actual": actual_declaration,
                }
            )
    checks["header_api_signatures"] = {
        "expected": len(expected_names),
        "differences": signature_differences,
    }
    if signature_differences:
        issues.append(
            _issue(
                "A2_HEADER_API_SIGNATURES",
                "agent_2_interface_modeler",
                IssueSeverity.HIGH,
                "api_contract",
                "2_function_skeleton.h",
                "Header function declarations differ from the frozen API.",
                differences=signature_differences,
            )
        )

    if board_application:
        # Signatures are checked above. SDK compilation must occur through the
        # isolated ARM candidate build, never the host subprocess path below.
        checks["header_compile"] = "deferred_to_required_isolated_arm_build"
        checks["target_profile"] = "stm32_gpio_application"
        return issues, checks

    compiler_name = str(build_profile.get("compiler", "gcc"))
    compiler = shutil.which(compiler_name)
    checks["header_compiler"] = compiler or "unavailable"
    if not compiler:
        checks["header_compile"] = "skipped"
        return issues, checks

    language = str(build_profile.get("language", "c")).lower()
    standard = str(build_profile.get("standard", "c11"))
    suffix = ".cpp" if language == "cpp" else ".c"
    with tempfile.TemporaryDirectory(prefix="design-header-") as temp_dir:
        header_path = Path(temp_dir) / ("design" + suffix)
        header_path.write_text(header, encoding="utf-8")
        command = [
            compiler,
            f"-std={standard}",
            *list(build_profile.get("compile_flags", [])),
            *[
                flag
                for include_dir in build_profile.get("include_dirs", [])
                for flag in ("-I", str(include_dir))
            ],
            *[
                f"-D{define}"
                for define in build_profile.get("defines", [])
            ],
            "-x",
            "c++" if language == "cpp" else "c",
            "-fsyntax-only",
            str(header_path),
        ]
        completed = subprocess.run(
            command,
            text=True,
            capture_output=True,
            timeout=DEFAULT_RUN_TIMEOUT_SECONDS,
        )
    checks["header_compile"] = {
        "exit_code": completed.returncode,
        "stderr": completed.stderr[-6000:],
    }
    if completed.returncode != 0:
        error_lines = [
            line for line in completed.stderr.splitlines()
            if "error:" in line.lower()
        ]
        severity = (
            IssueSeverity.HIGH if len(error_lines) >= 8 else IssueSeverity.MEDIUM
        )
        category = (
            "architecture" if len(error_lines) >= 8 else "header_syntax"
        )
        issues.append(
            _issue(
                "A2_HEADER_COMPILE",
                "agent_2_interface_modeler",
                severity,
                category,
                "2_function_skeleton.h",
                "Generated function skeleton does not pass syntax-only compilation.",
                error_count=len(error_lines),
                compiler_output=completed.stderr[-6000:],
            )
        )
    return issues, checks


def validate_design_artifacts(
    state: Dict[str, Any],
    api_spec: str,
    build_profile: Optional[Dict[str, Any]] = None,
) -> ValidationResult:
    """Validate all three design artifacts and attribute each issue to an owner."""
    issues = _validate_required_artifacts(state)
    checks: Dict[str, Any] = {}
    if issues:
        return ValidationResult(valid=False, issues=issues, checks=checks)

    try:
        state_model = StateModel.model_validate(state["state_model"])
        interface_model = InterfaceModel.model_validate(state["interface_model"])
        behavior_model = BehaviorModel.model_validate(state["behavior_model"])
    except Exception as exc:
        issues.append(
            _issue(
                "DESIGN_MODEL_PARSE",
                "cross",
                IssueSeverity.CRITICAL,
                "model_schema",
                "canonical_models",
                "One or more canonical design models cannot be parsed.",
                exception=str(exc),
            )
        )
        return ValidationResult(valid=False, issues=issues, checks=checks)

    canonical_report = validate_models(
        state_model,
        interface_model,
        behavior_model,
    )
    checks["canonical_model_validation"] = canonical_report.model_dump()
    for model_issue in canonical_report.issues:
        if model_issue.severity != "HIGH":
            continue
        owner = {
            "state": "agent_1_state_modeler",
            "interface": "agent_2_interface_modeler",
            "behavior": "agent_3_behavior_modeler",
        }.get(model_issue.model, "cross")
        issues.append(
            _issue(
                "DESIGN_CANONICAL_HIGH",
                owner,
                IssueSeverity.HIGH,
                "architecture" if owner == "cross" else "model_consistency",
                f"{model_issue.model}_model",
                model_issue.message,
                symbol=model_issue.symbol,
            )
        )

    state_issues, state_checks = _validate_state_plantuml(
        str(state["state_diagram"])
    )
    sequence_issues, sequence_checks = _validate_sequence_plantuml(
        str(state["sequence_diagram"])
    )
    api_issues, api_checks = _validate_api_fidelity(interface_model, api_spec)
    interface_semantic_issues, interface_semantic_checks = (
        _validate_interface_semantics(
            interface_model,
            str(state.get("user_requirement", "")),
            state_model,
        )
    )
    header_issues, header_checks = _validate_header(
        str(state["function_skeleton"]),
        api_spec,
        build_profile or {},
    )
    issues.extend(state_issues)
    issues.extend(sequence_issues)
    issues.extend(api_issues)
    issues.extend(interface_semantic_issues)
    issues.extend(header_issues)
    checks.update(state_checks)
    checks.update(sequence_checks)
    checks.update(api_checks)
    checks.update(interface_semantic_checks)
    checks.update(header_checks)
    return ValidationResult(valid=not issues, issues=issues, checks=checks)
