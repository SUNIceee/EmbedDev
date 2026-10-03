"""Adaptive generate/repair loop for state, interface, and behavior artifacts."""

from __future__ import annotations

from dataclasses import asdict
from datetime import datetime
import hashlib
import json
from pathlib import Path
from typing import Any, Dict, List, Optional

from _internal.design_iteration_models import (
    DesignIssue,
    IssueSeverity,
    IterationHistoryEntry,
    ValidationResult,
)
from _internal.design_iteration_policy import (
    DesignIterationPolicy,
    choose_iteration_action,
)
from _internal.design_validation import validate_design_artifacts
from _internal.error_notebook import record_lesson
from _internal.pipeline_config import ProjectConfig, write_json
from traceability import build_traceability


_ARTIFACT_FILES = {
    "state_diagram": "1_state_diagram.puml",
    "state_model": "1_state_model.json",
    "function_skeleton": "2_function_skeleton.h",
    "interface_model": "2_interface_model.json",
    "sequence_diagram": "3_sequence_diagram.puml",
    "behavior_model": "3_behavior_model.json",
}
_AGENT_SEQUENCE = ["agent_1_state_modeler", "agent_2_interface_modeler", "agent_3_behavior_modeler"]


class StageGenerationError(RuntimeError):
    def __init__(self, agent: str, cause: BaseException):
        super().__init__(f"{agent} generation failed: {cause}")
        self.agent = agent
        self.cause = cause


def _empty_state(config: ProjectConfig) -> Dict[str, Any]:
    device_interface_path = config.path_for(config.device_interface_file)
    return {
        "user_requirement": config.path_for(config.requirement_file).read_text(
            encoding="utf-8"
        ),
        "device_interface": (
            device_interface_path.read_text(encoding="utf-8")
            if device_interface_path and device_interface_path.exists()
            else ""
        ),
        "api_spec": config.path_for(config.api_file).read_text(encoding="utf-8"),
        "target_language": config.build.language,
        "state_diagram": "",
        "state_model": {},
        "function_skeleton": "",
        "interface_model": {},
        "sequence_diagram": "",
        "behavior_model": {},
        "model_validation_report": {},
        "verification_spec": [],
        "validation_report": "",
        "generated_header": "",
        "generated_code": "",
        "readme_content": "",
        "tlc_verification_report": {},
        "design_model_tla": "",
        "design_model_cfg": "",
        "validation_retries": 0,
        "max_retries": config.design_iteration.max_iterations,
        "consistency_report": "",
        "error_context": "",
        "fidelity_report": "",
        "code_gen_retries": 0,
        "build_profile": asdict(config.build),
        "stage_status": {},
        "artifact_manifest": {},
        "repair_history": [],
        "error_notebook_top_k": config.design_iteration.error_notebook_top_k,
    }


def _load_existing_prefix(
    state: Dict[str, Any],
    config: ProjectConfig,
    regenerate_from: str,
) -> None:
    """Load only accepted upstream artifacts needed by a partial restart."""
    cutoff = _AGENT_SEQUENCE.index(regenerate_from)
    fields_by_agent = {
        "agent_1_state_modeler": ["state_diagram", "state_model"],
        "agent_2_interface_modeler": ["function_skeleton", "interface_model"],
        "agent_3_behavior_modeler": ["sequence_diagram", "behavior_model"],
    }
    for agent in _AGENT_SEQUENCE[:cutoff]:
        for field in fields_by_agent[agent]:
            path = config.path_for(_ARTIFACT_FILES[field])
            if not path.exists():
                raise FileNotFoundError(
                    f"Cannot restart from {regenerate_from}: missing upstream {path}"
                )
            if path.suffix == ".json":
                state[field] = json.loads(path.read_text(encoding="utf-8"))
            else:
                state[field] = path.read_text(encoding="utf-8")


def _artifact_hashes(state: Dict[str, Any]) -> Dict[str, str]:
    hashes = {}
    for field in _ARTIFACT_FILES:
        value = state.get(field)
        if isinstance(value, str):
            raw = value
        else:
            raw = json.dumps(value, ensure_ascii=False, sort_keys=True)
        hashes[field] = hashlib.sha256(raw.encode("utf-8")).hexdigest()
    return hashes


def _save_snapshot(
    iteration_dir: Path,
    state: Dict[str, Any],
    validation: ValidationResult,
    decision: Any,
    operation: str,
    repair_agents: List[str],
) -> None:
    iteration_dir.mkdir(parents=True, exist_ok=True)
    for field, filename in _ARTIFACT_FILES.items():
        value = state.get(field)
        if value in (None, "", {}, []):
            continue
        path = iteration_dir / filename
        if path.suffix == ".json":
            write_json(path, value)
        else:
            path.write_text(str(value), encoding="utf-8")
    write_json(iteration_dir / "validation.json", validation.model_dump())
    write_json(iteration_dir / "decision.json", decision.model_dump())
    write_json(
        iteration_dir / "iteration.json",
        {
            "operation": operation,
            "repair_agents": repair_agents,
            "issue_count": len(validation.issues),
            "issue_signature": validation.issue_signature(),
            "artifact_hashes": _artifact_hashes(state),
        },
    )


def _publish(
    config: ProjectConfig,
    state: Dict[str, Any],
    validation: ValidationResult,
) -> None:
    for field, filename in _ARTIFACT_FILES.items():
        path = config.path_for(filename)
        value = state[field]
        if path.suffix == ".json":
            write_json(path, value)
        else:
            path.write_text(str(value), encoding="utf-8")
    write_json(
        config.path_for(config.model_validation_file),
        {
            "valid": True,
            "issues": [],
            "checks": validation.checks,
            "validated_at": datetime.now().isoformat(timespec="seconds"),
            "validator": "adaptive-design-loop",
        },
    )


def _generation_issue(error: StageGenerationError) -> ValidationResult:
    return ValidationResult(
        valid=False,
        issues=[
            DesignIssue(
                code="DESIGN_GENERATION_EXCEPTION",
                agent=error.agent,
                severity=IssueSeverity.CRITICAL,
                category="structured_output",
                artifact=error.agent,
                message=str(error),
                details={"exception_type": type(error.cause).__name__},
            )
        ],
        checks={},
    )


def _validator_issue(error: BaseException) -> ValidationResult:
    return ValidationResult(
        valid=False,
        issues=[
            DesignIssue(
                code="DESIGN_VALIDATOR_EXCEPTION",
                agent="cross",
                severity=IssueSeverity.CRITICAL,
                category="validator_failure",
                artifact="design_validation",
                message=f"Deterministic design validation failed: {error}",
                details={"exception_type": type(error).__name__},
            )
        ],
        checks={},
    )


def _apply_traceability_validation(
    state: Dict[str, Any], validation: ValidationResult, trace_path: Path
) -> None:
    """Check design anchors; implementation evidence is deferred until code exists."""
    trace = build_traceability(
        requirements=state.get("user_requirement", ""),
        state_model=state.get("state_model", {}),
        interface_model=state.get("interface_model", {}),
        behavior_model=state.get("behavior_model", {}),
    )
    write_json(trace_path, trace)
    issues: List[DesignIssue] = []

    for variable in state.get("state_model", {}).get("variables", []):
        if not variable.get("source_requirements"):
            issues.append(DesignIssue(
                code="TRACE_STATE_REQUIREMENT_MISSING",
                agent="agent_1_state_modeler",
                severity=IssueSeverity.MEDIUM,
                category="traceability",
                artifact="1_state_model.json",
                message="State variable has no requirement anchor in traceability.json.",
                details={"variable": variable.get("name", "")},
            ))

    for function in state.get("interface_model", {}).get("functions", []):
        if not (function.get("source_requirements") or function.get("req_anchors")):
            issues.append(DesignIssue(
                code="TRACE_INTERFACE_REQUIREMENT_MISSING",
                agent="agent_2_interface_modeler",
                severity=IssueSeverity.MEDIUM,
                category="traceability",
                artifact="2_interface_model.json",
                message="Interface function has no requirement anchor in traceability.json.",
                details={"function": function.get("name", "")},
            ))

    validation.issues.extend(issues)
    validation.valid = not validation.issues
    validation.checks["traceability"] = {
        "path": str(trace_path),
        "summary": trace["summary"],
        "issue_count": len(issues),
        "deferred_behavior_calls": [
            edge for edge in trace["edges"]
            if edge["relation"] == "uses_internal_or_platform"
            and edge["status"] == "pending_code_evidence"
        ],
    }


class AdaptiveDesignRunner:
    def __init__(self, config: ProjectConfig, run_dir: Path):
        self.config = config
        self.run_dir = run_dir
        self.state = _empty_state(config)
        self.history: List[IterationHistoryEntry] = []
        self.policy = DesignIterationPolicy(
            max_iterations=config.design_iteration.max_iterations,
            max_local_repair_attempts=(
                config.design_iteration.max_local_repair_attempts
            ),
            regenerate_high_issue_count=(
                config.design_iteration.regenerate_high_issue_count
            ),
        )

    def generate_from(self, start_agent: str) -> None:
        # Import provider-backed agents only when a model operation is actually
        # requested. This keeps inspect/dry-run/offline policy tests key-free.
        from agents.agent_1_state_modeler import state_diagram_agent
        from agents.agent_2_interface_modeler import func_alignment_agent
        from agents.agent_3_behavior_modeler import sequence_diagram_agent

        start_index = _AGENT_SEQUENCE.index(start_agent)
        stages = [
            ("agent_1_state_modeler", state_diagram_agent.generate),
            ("agent_2_interface_modeler", func_alignment_agent.generate),
            ("agent_3_behavior_modeler", sequence_diagram_agent.generate),
        ]
        for agent, generate in stages[start_index:]:
            try:
                update = generate(self.state)
            except Exception as exc:
                raise StageGenerationError(agent, exc) from exc
            self.state.update(update)
            self.state["stage_status"][agent] = "succeeded"

    def repair(self, agents: List[str], issues: List[DesignIssue]) -> None:
        from agents.agent_1_state_repairer import agent_1_state_modeler_repair_agent
        from agents.agent_2_interface_repairer import agent_2_interface_modeler_repair_agent
        from agents.agent_3_behavior_repairer import agent_3_behavior_modeler_repair_agent

        repairers = {
            "agent_1_state_modeler": agent_1_state_modeler_repair_agent.repair,
            "agent_2_interface_modeler": agent_2_interface_modeler_repair_agent.repair,
            "agent_3_behavior_modeler": agent_3_behavior_modeler_repair_agent.repair,
        }
        for agent in agents:
            owned = [issue for issue in issues if issue.agent == agent]
            try:
                update = repairers[agent](self.state, owned)
            except Exception as exc:
                raise StageGenerationError(agent, exc) from exc
            summary = update.pop("repair_summary", [])
            self.state.update(update)
            self.state["repair_history"].append(
                {"agent": agent, "summary": summary}
            )

    def _record_resolved_lessons(
        self,
        previous_issues: List[DesignIssue],
        current: ValidationResult,
        operation: str,
    ) -> None:
        remaining = {issue.fingerprint for issue in current.issues}
        summary = "; ".join(
            item
            for repair in self.state.get("repair_history", [])[-3:]
            for item in repair.get("summary", [])
        )
        for issue in previous_issues:
            if issue.fingerprint in remaining:
                continue
            try:
                record_lesson(
                    agent=issue.agent,
                    category=issue.category,
                    error_pattern=issue.message,
                    root_cause=(
                        str(issue.details)[:1000]
                        or "Validator-localized issue"
                    ),
                    successful_fix=summary or f"Resolved by {operation}",
                    tags=[issue.code, issue.artifact],
                    source=f"adaptive-run:{self.run_dir.name}",
                )
            except Exception as exc:
                print(f"[ErrorNotebook] Could not record resolved lesson: {exc}")

    def run(self, initial_start: str = "agent_1_state_modeler") -> Dict[str, Any]:
        if initial_start != "agent_1_state_modeler":
            _load_existing_prefix(self.state, self.config, initial_start)
        next_operation = "generate"
        regenerate_from = initial_start
        repair_agents: List[str] = []
        previous_issues: List[DesignIssue] = []

        for iteration in range(1, self.policy.max_iterations + 1):
            print(
                f"[DESIGN ITERATION {iteration}] operation={next_operation} "
                f"from={regenerate_from if next_operation == 'generate' else repair_agents}"
            )
            try:
                operation_succeeded = True
                if next_operation == "generate":
                    self.generate_from(regenerate_from)
                else:
                    self.repair(repair_agents, previous_issues)
                validation = validate_design_artifacts(
                    self.state,
                    self.state["api_spec"],
                    self.state["build_profile"],
                )
                _apply_traceability_validation(
                    self.state, validation, self.run_dir / "traceability.json"
                )
            except StageGenerationError as exc:
                operation_succeeded = False
                validation = _generation_issue(exc)
            except Exception as exc:
                operation_succeeded = False
                validation = _validator_issue(exc)

            if previous_issues and operation_succeeded:
                self._record_resolved_lessons(
                    previous_issues, validation, next_operation
                )
            decision = choose_iteration_action(
                validation,
                self.history,
                iteration,
                self.policy,
            )
            entry = IterationHistoryEntry(
                iteration=iteration,
                operation=next_operation,
                repaired_agents=repair_agents if next_operation == "repair" else [],
                decision=decision,
                issue_count=len(validation.issues),
                issue_signature=validation.issue_signature(),
                artifact_hashes=_artifact_hashes(self.state),
            )
            self.history.append(entry)
            _save_snapshot(
                self.run_dir / "design_iterations" / f"iteration-{iteration:02d}",
                self.state,
                validation,
                decision,
                next_operation,
                entry.repaired_agents,
            )
            print(
                f"[DESIGN ITERATION {iteration}] issues={len(validation.issues)} "
                f"decision={decision.action} reason={decision.reason}"
            )

            if decision.action == "accept":
                _publish(self.config, self.state, validation)
                result = {
                    "status": "succeeded",
                    "iterations": iteration,
                    "published": True,
                    "history": [item.model_dump() for item in self.history],
                }
                write_json(self.run_dir / "design_result.json", result)
                return result
            if decision.action == "stop":
                result = {
                    "status": "failed",
                    "iterations": iteration,
                    "published": False,
                    "remaining_issues": [
                        issue.model_dump() for issue in validation.issues
                    ],
                    "history": [item.model_dump() for item in self.history],
                }
                write_json(self.run_dir / "design_result.json", result)
                return result

            previous_issues = validation.issues
            if decision.action == "repair":
                next_operation = "repair"
                repair_agents = decision.repair_agents
            else:
                next_operation = "generate"
                regenerate_from = decision.regenerate_from or "agent_1_state_modeler"
                repair_agents = []

        raise RuntimeError("Unreachable design iteration state")
