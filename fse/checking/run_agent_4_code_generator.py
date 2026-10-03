#!/usr/bin/env python3
"""Run implementation generation and verification with repair feedback."""

from __future__ import annotations

import argparse
import copy
from dataclasses import asdict
from datetime import datetime
import hashlib
import json
import os
from pathlib import Path
import shutil
import sys
from typing import Any, Dict, Optional, Tuple
import uuid

sys.path.insert(0, str(Path(__file__).resolve().parent))

from config import (
    ArtifactStatus,
    ProjectConfig,
    PublicationStatus,
    RunStatus,
    StageStatus,
    write_json,
)
from model_config import model_names
from traceability import write_traceability


REQUIRED_RELEASE_STAGES = (
    "agent_4_code_generator",
    "agent_5_code_verifier_compile",
    "agent_5_code_verifier_fidelity",
)
VERIFICATION_POLICY = "agent_5_code_verifier_release_policy_v2"
REPAIR_NOTEBOOK_NAME = "agent_4_code_generator_repair_history.json"
MAX_STAGNANT_REPAIRS = 6


class PublicationError(RuntimeError):
    """Raised when a release-ready candidate cannot be published safely."""


def read_utf8(path: Optional[Path], required: bool = True) -> str:
    if path is None:
        if required:
            raise FileNotFoundError("Required input path is not configured.")
        return ""
    if not path.exists():
        if required:
            raise FileNotFoundError(f"Required file not found: {path}")
        print(f"  [SKIP] Optional input not found: {path.name}")
        return ""
    try:
        content = path.read_text(encoding="utf-8")
    except UnicodeDecodeError as exc:
        raise ValueError(f"Input is not valid UTF-8: {path}") from exc
    print(f"  [OK] {path} ({len(content)} chars)")
    return content


def read_json_utf8(path: Optional[Path], required: bool = True) -> dict:
    content = read_utf8(path, required=required)
    if not content:
        return {}
    try:
        value = json.loads(content)
    except json.JSONDecodeError as exc:
        raise ValueError(f"Invalid JSON artifact: {path}: {exc}") from exc
    if not isinstance(value, dict):
        raise ValueError(f"JSON artifact must contain an object: {path}")
    return value


def load_inputs(config: ProjectConfig) -> Dict[str, Any]:
    print(f"\n[PROJECT] {config.project_name}")
    print(f"[DIR] {config.input_dir}")
    return {
        "user_requirement": read_utf8(config.path_for(config.requirement_file)),
        "device_interface": read_utf8(config.path_for(config.device_interface_file), required=False),
        "api_spec": read_utf8(config.path_for(config.api_file)),
        "state_diagram": read_utf8(config.path_for(config.state_diagram_file)),
        "state_model": read_json_utf8(config.path_for(config.state_model_file)),
        "sequence_diagram": read_utf8(config.path_for(config.sequence_diagram_file), required=False),
        "behavior_model": read_json_utf8(config.path_for(config.behavior_model_file)),
        "function_skeleton": read_utf8(config.path_for(config.skeleton_file)),
        "interface_model": read_json_utf8(config.path_for(config.interface_model_file)),
        "model_validation_report": read_json_utf8(config.path_for(config.model_validation_file)),
        "tla_context": read_utf8(config.path_for(config.tla_file), required=False),
    }


def archive_existing(config: ProjectConfig) -> Optional[Path]:
    output = config.output_path
    if not output.exists():
        return None
    config.archive_path.mkdir(parents=True, exist_ok=True)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    destination = config.archive_path / f"Implementation Generator_{timestamp}"
    counter = 1
    while destination.exists():
        destination = config.archive_path / f"Implementation Generator_{timestamp}_{counter}"
        counter += 1
    shutil.move(str(output), str(destination))
    print(f"[ARCHIVE] Previous output moved to {destination}")
    return destination


def _sha256(content: bytes) -> str:
    return hashlib.sha256(content).hexdigest()


def is_release_ready(state: dict) -> bool:
    """Return true only when every required release gate explicitly succeeded."""

    stages = state.get("stage_status", {})
    return (
        all(stages.get(name) == StageStatus.SUCCEEDED.value for name in REQUIRED_RELEASE_STAGES)
        and not str(state.get("error_context", "")).strip()
    )


def highest_completed_gate(state: dict) -> str:
    stages = state.get("stage_status", {})
    if stages.get("agent_5_code_verifier_fidelity") == StageStatus.SUCCEEDED.value:
        return "fidelity_audited"
    if stages.get("agent_5_code_verifier_compile") == StageStatus.SUCCEEDED.value:
        # Implementation Verifier currently performs a compiler syntax check, not a link/run gate.
        return "syntax_checked"
    if stages.get("agent_4_code_generator") == StageStatus.SUCCEEDED.value:
        return "generated"
    return "none"


def derive_outcome(state: dict) -> Tuple[RunStatus, ArtifactStatus]:
    """Derive the run/artifact outcome from explicit stage evidence."""

    if is_release_ready(state):
        return RunStatus.SUCCEEDED, ArtifactStatus.RELEASE_READY

    stages = state.get("stage_status", {})
    stage_values = {stages.get(name) for name in REQUIRED_RELEASE_STAGES}
    if StageStatus.BLOCKED.value in stage_values:
        return RunStatus.BLOCKED, ArtifactStatus.VERIFICATION_BLOCKED
    if stages.get("agent_4_code_generator") == StageStatus.FAILED.value:
        return RunStatus.FAILED, ArtifactStatus.GENERATION_FAILED
    if StageStatus.FAILED.value in stage_values:
        return RunStatus.FAILED, ArtifactStatus.VERIFICATION_FAILED
    if stages.get("agent_4_code_generator") == StageStatus.SUCCEEDED.value:
        return RunStatus.BLOCKED, ArtifactStatus.GENERATED
    return RunStatus.FAILED, ArtifactStatus.GENERATION_FAILED


def _manifest(
    state: dict,
    config: ProjectConfig,
    artifacts: Dict[str, Dict[str, Any]],
    run_status: RunStatus,
    artifact_status: ArtifactStatus,
    publication_status: PublicationStatus,
) -> Dict[str, Any]:
    return {
        "schema_version": 2,
        "project": config.to_manifest_dict(),
        "finished_at": datetime.now().isoformat(timespec="seconds"),
        "run_status": run_status.value,
        "artifact_status": artifact_status.value,
        "publication_status": publication_status.value,
        "highest_completed_gate": highest_completed_gate(state),
        "verification_policy": VERIFICATION_POLICY,
        "fidelity_audit_skipped": bool(state.get("fidelity_audit_skipped", False)),
        "stage_status": state.get("stage_status", {}),
        "code_generation_retries": state.get("code_gen_retries", 0),
        "remaining_error": state.get("error_context", ""),
        "repair_history": state.get("repair_history", []),
        "error_history": state.get("error_history", []),
        "selected_attempt": state.get("selected_attempt"),
        "selected_candidate_score": state.get("selected_candidate_score"),
        "artifacts": artifacts,
    }


def save_output(
    state: dict,
    config: ProjectConfig,
    output: Path,
    run_status: RunStatus,
    artifact_status: ArtifactStatus,
    publication_status: PublicationStatus = PublicationStatus.NOT_PUBLISHED,
) -> Dict[str, Any]:
    """Write a complete candidate bundle outside ``trusted_output``."""

    resolved_output = output.resolve()
    trusted_output = config.output_path
    try:
        resolved_output.relative_to(trusted_output)
    except ValueError:
        pass
    else:
        raise ValueError(
            "Candidate writer refuses to write inside trusted_output; use publish_output "
            "after all release gates succeed."
        )

    output.mkdir(parents=True, exist_ok=True)
    extension = config.build.source_extension

    artifacts = {}
    values = [
        ("generated_header", "6_generated_code.h"),
        ("generated_code", "6_generated_code" + extension),
        ("fidelity_report", "6_5_fidelity_report.txt"),
    ]
    for state_key, filename in values:
        content = state.get(state_key, "")
        if not content:
            continue
        path = output / filename
        encoded = content.encode("utf-8")
        path.write_bytes(encoded)
        artifacts[state_key] = {
            "path": filename,
            "bytes": len(encoded),
            "sha256": _sha256(encoded),
        }
        print(f"  [FILE] {path}")

    trace_path = output / "traceability.json"
    trace = write_traceability(
        trace_path,
        requirements=state.get("user_requirement", ""),
        state_model=state.get("state_model", {}),
        interface_model=state.get("interface_model", {}),
        behavior_model=state.get("behavior_model", {}),
        generated_code=state.get("generated_code", ""),
    )
    trace_bytes = trace_path.read_bytes()
    artifacts["traceability"] = {
        "path": trace_path.name,
        "bytes": len(trace_bytes),
        "sha256": _sha256(trace_bytes),
        "missing_implementation_edges": trace["summary"]["missing_implementation_edges"],
    }
    print(f"  [FILE] {trace_path}")

    manifest = _manifest(
        state,
        config,
        artifacts,
        run_status,
        artifact_status,
        publication_status,
    )
    write_json(output / "manifest.json", manifest)
    return manifest


def _copy_first_round_bundle(
    state: dict,
    config: ProjectConfig,
    candidate_output: Path,
    run_id: str,
) -> Path:
    """Preserve the best available implementation with accepted Agent 1--3 artifacts.

    This is a diagnostic/research result bundle, not a trusted release.  It is
    intentionally written even when Agent 5 fidelity verification fails.
    """

    model_name = os.environ.get("FSE_MODEL", "unknown-model").strip() or "unknown-model"
    run_date = run_id[:8] if len(run_id) >= 8 else datetime.now().strftime("%Y%m%d")
    round_number = 1
    while (config.input_dir / f"{model_name}_{run_date}_round{round_number}").exists():
        round_number += 1
    bundle = config.input_dir / f"{model_name}_{run_date}_round{round_number}"
    bundle.mkdir(parents=True, exist_ok=False)

    input_files = [
        (config.state_diagram_file, "1_state_diagram.puml"),
        (config.state_model_file, "1_state_model.json"),
        (config.skeleton_file, "2_function_skeleton.h"),
        (config.interface_model_file, "2_interface_model.json"),
        (config.sequence_diagram_file, "3_sequence_diagram.puml"),
        (config.behavior_model_file, "3_behavior_model.json"),
        (config.model_validation_file, "3_5_model_validation.json"),
    ]
    copied = []
    for configured_name, output_name in input_files:
        source = config.path_for(configured_name)
        if source is not None and source.exists():
            shutil.copy2(source, bundle / output_name)
            copied.append(output_name)

    candidate_files = [
        ("6_generated_code.h", "4_generated_code.h"),
        ("6_generated_code" + config.build.source_extension, "4_generated_code" + config.build.source_extension),
        ("6_5_fidelity_report.txt", "5_fidelity_report.txt"),
        ("traceability.json", "traceability.json"),
    ]
    for source_name, output_name in candidate_files:
        source = candidate_output / source_name
        if source.exists():
            shutil.copy2(source, bundle / output_name)
            copied.append(output_name)

    write_json(
        bundle / "first_round_manifest.json",
        {
            "schema_version": 1,
            "project": config.project_name,
            "model": model_name,
            "round": round_number,
            "run_date": run_date,
            "run_id": run_id,
            "source_candidate": str(candidate_output),
            "result_kind": "best_available_candidate",
            "trusted_release": False,
            "run_status": state.get("run_status"),
            "artifact_status": state.get("artifact_status"),
            "publication_status": state.get("publication_status"),
            "stage_status": state.get("stage_status", {}),
            "remaining_error": state.get("error_context", ""),
            "files": copied,
        },
    )
    print(f"[FIRST ROUND] Best available result bundle: {bundle}")
    return bundle


def _mark_staged_manifest_published(staging: Path, output: Path) -> None:
    manifest_path = staging / "manifest.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    manifest["publication_status"] = PublicationStatus.PUBLISHED.value
    manifest["published_at"] = datetime.now().isoformat(timespec="seconds")
    manifest["published_path"] = str(output)
    write_json(manifest_path, manifest)


def _record_candidate_publication(candidate: Path, published: Path) -> None:
    """Record the publication result in the retained run candidate manifest."""

    published_manifest = published / "manifest.json"
    write_json(
        candidate / "manifest.json",
        json.loads(published_manifest.read_text(encoding="utf-8")),
    )


def _validate_release_candidate(candidate: Path, config: ProjectConfig) -> None:
    """Fail closed unless the retained candidate manifest proves release readiness."""

    resolved_candidate = candidate.resolve()
    trusted_output = config.output_path
    try:
        resolved_candidate.relative_to(trusted_output)
    except ValueError:
        pass
    else:
        raise PublicationError("A publication candidate cannot be read from trusted_output.")

    manifest_path = resolved_candidate / "manifest.json"
    try:
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise PublicationError(f"Candidate manifest is unavailable or invalid: {exc}") from exc

    stages = manifest.get("stage_status", {})
    if not isinstance(stages, dict):
        stages = {}
    missing_gates = [
        name
        for name in REQUIRED_RELEASE_STAGES
        if stages.get(name) != StageStatus.SUCCEEDED.value
    ]
    expected_artifacts = {
        "generated_header": "6_generated_code.h",
        "generated_code": "6_generated_code" + config.build.source_extension,
        "fidelity_report": "6_5_fidelity_report.txt",
        "traceability": "traceability.json",
    }
    required_artifacts = {"generated_header", "generated_code", "fidelity_report", "traceability"}
    artifact_map = manifest.get("artifacts", {})
    if not isinstance(artifact_map, dict):
        artifact_map = {}
    missing_artifacts = sorted(required_artifacts - set(artifact_map))
    unknown_artifacts = sorted(set(artifact_map) - set(expected_artifacts))
    failures = []
    if manifest.get("schema_version") != 2:
        failures.append("schema_version must be 2")
    if manifest.get("run_status") != RunStatus.SUCCEEDED.value:
        failures.append("run_status must be succeeded")
    if manifest.get("artifact_status") != ArtifactStatus.RELEASE_READY.value:
        failures.append("artifact_status must be release_ready")
    if manifest.get("publication_status") != PublicationStatus.NOT_PUBLISHED.value:
        failures.append("publication_status must be not_published")
    if str(manifest.get("remaining_error", "")).strip():
        failures.append("remaining_error must be empty")
    if missing_gates:
        failures.append("missing successful gates: " + ", ".join(missing_gates))
    if missing_artifacts:
        failures.append("missing required artifacts: " + ", ".join(missing_artifacts))
    if unknown_artifacts:
        failures.append("unknown artifacts: " + ", ".join(unknown_artifacts))
    used_paths = set()
    for artifact_name, metadata in artifact_map.items():
        if not isinstance(metadata, dict):
            failures.append(f"artifact metadata is invalid: {artifact_name}")
            continue
        relative_name = metadata.get("path")
        if not isinstance(relative_name, str) or not relative_name:
            failures.append(f"artifact path is invalid: {artifact_name}")
            continue
        expected_name = expected_artifacts.get(artifact_name)
        if expected_name is not None and relative_name != expected_name:
            failures.append(
                f"artifact path does not match its contract: {artifact_name} -> {relative_name}"
            )
        if relative_name in used_paths:
            failures.append(f"artifact path is duplicated: {relative_name}")
        used_paths.add(relative_name)
        artifact_path = (resolved_candidate / relative_name).resolve()
        try:
            artifact_path.relative_to(resolved_candidate)
        except ValueError:
            failures.append(f"artifact path escapes candidate: {artifact_name}")
            continue
        try:
            content = artifact_path.read_bytes()
        except OSError:
            failures.append(f"artifact file is missing or unreadable: {artifact_name}")
            continue
        if metadata.get("bytes") != len(content):
            failures.append(f"artifact byte count does not match: {artifact_name}")
        if metadata.get("sha256") != _sha256(content):
            failures.append(f"artifact SHA-256 does not match: {artifact_name}")
    if failures:
        raise PublicationError("Candidate is not release-ready: " + "; ".join(failures))


def publish_output(candidate: Path, config: ProjectConfig, archive: bool) -> Path:
    """Atomically publish a release-ready candidate, preserving the old release."""

    _validate_release_candidate(candidate, config)
    output = config.output_path
    archive_path = config.archive_path
    if output.exists() and not archive:
        raise PublicationError(
            "Refusing to replace an existing trusted output with --no-archive; "
            "enable archiving so the previous trusted release can be preserved."
        )

    staging_parent = config.input_dir / ".pipeline" / "publish_staging"
    staging_parent.mkdir(parents=True, exist_ok=True)
    staging = staging_parent / ("trusted_output_" + uuid.uuid4().hex)
    archived: Optional[Path] = None
    try:
        shutil.copytree(candidate, staging)
        # Revalidate the copied snapshot to close the validation/copy race before
        # the existing trusted release is moved.
        _validate_release_candidate(staging, config)
        _mark_staged_manifest_published(staging, output)
        if output.exists():
            # Resolve the validated archive path before moving the current release.
            _ = archive_path
            archived = archive_existing(config)
        try:
            staging.replace(output)
        except Exception as exc:
            if archived is not None and not output.exists():
                try:
                    archived.replace(output)
                except Exception as restore_exc:
                    raise PublicationError(
                        "Publishing failed and the previous trusted output could not be restored: "
                        f"publish={exc}; restore={restore_exc}"
                    ) from restore_exc
            raise PublicationError(f"Publishing candidate failed: {exc}") from exc
    except PublicationError:
        raise
    except Exception as exc:
        raise PublicationError(f"Preparing candidate publication failed: {exc}") from exc
    finally:
        if staging.exists():
            # This is an unpublished staging directory and therefore safe to discard.
            shutil.rmtree(staging, ignore_errors=True)

    print(f"[PUBLISH] Trusted output updated at {output}")
    return output


def resolve_run_dir(config: ProjectConfig, run_dir: Optional[Path] = None) -> Path:
    runs_root = (config.input_dir / ".pipeline" / "runs").resolve()
    if run_dir is None:
        run_id = datetime.now().strftime("%Y%m%d_%H%M%S") + "_" + uuid.uuid4().hex[:8]
        resolved = runs_root / run_id
    else:
        resolved = Path(run_dir).expanduser().resolve()
    try:
        resolved.relative_to(runs_root)
    except ValueError as exc:
        raise ValueError(f"Run directory must stay under {runs_root}: {resolved}") from exc
    resolved.mkdir(parents=True, exist_ok=True)
    return resolved


def make_initial_state(inputs: Dict[str, Any], config: ProjectConfig) -> NewGraphState:
    from runtime import NewGraphState
    validation = inputs["model_validation_report"]
    if not validation.get("valid", False):
        raise ValueError("Canonical models have not passed 3_5_model_validation.json; rerun Agents 1-3.")
    build_profile = asdict(config.build)
    build_profile["temp_dir"] = str(config.input_dir / ".pipeline_tmp")
    return NewGraphState(
        user_requirement=inputs["user_requirement"],
        device_interface=inputs.get("device_interface", ""),
        api_spec=inputs["api_spec"],
        target_language="C++" if config.build.language == "cpp" else "C",
        state_diagram=inputs["state_diagram"],
        state_model=inputs["state_model"],
        function_skeleton=inputs["function_skeleton"],
        interface_model=inputs["interface_model"],
        sequence_diagram=inputs["sequence_diagram"],
        behavior_model=inputs["behavior_model"],
        model_validation_report=inputs["model_validation_report"],
        verification_spec=[],
        validation_report=inputs["tla_context"],
        generated_header="",
        generated_code="",
        tlc_verification_report={},
        design_model_tla="",
        design_model_cfg="",
        readme_content="",
        validation_retries=0,
        max_retries=0,
        consistency_report="",
        error_context="",
        fidelity_report="",
        code_gen_retries=0,
        build_profile=build_profile,
        stage_status={},
        artifact_manifest={},
        repair_history=[],
        error_history=[],
        error_notebook_top_k=8,
    )


def _error_signature(error: str) -> str:
    """Normalize issue identity so wording changes do not hide stagnation."""
    import re
    issue_keys = re.findall(
        r"(?m)^-\s*\[([^\]]+)\]\s+([^:]+):",
        str(error or ""),
    )
    if issue_keys:
        return "|".join(
            sorted({f"{kind.strip().lower()}::{owner.strip().lower()}" for kind, owner in issue_keys})
        )[:2000]
    text = re.sub(r"[A-Za-z]:\\[^\r\n:]+", "<path>", str(error or ""))
    text = re.sub(r"/[^\r\n:]+", "<path>", text)
    text = re.sub(r"\s+", " ", text).strip().lower()
    return text[:2000]


def _candidate_score(state: dict) -> Tuple[int, int, int, int, int]:
    """Score a candidate without weakening the release gate.

    The score is used only to retain the least-bad diagnostic candidate when
    verification fails. A candidate still needs every release gate to publish.
    """

    import re

    report = str(state.get("fidelity_report", ""))
    high_count = len(re.findall(r"(?m)^\[HIGH\]", report))
    issue_count = len(re.findall(r"(?m)^\[(?:HIGH|MEDIUM|LOW)\]", report))
    audit_completed = int(
        state.get("stage_status", {}).get("agent_5_code_verifier_fidelity")
        in {StageStatus.SUCCEEDED.value, StageStatus.FAILED.value}
        and not report.startswith("AUDIT_BLOCKED:")
    )
    compile_ok = int(
        state.get("stage_status", {}).get("agent_5_code_verifier_compile")
        == StageStatus.SUCCEEDED.value
    )
    fidelity_ok = int(is_release_ready(state))
    return (fidelity_ok, compile_ok, audit_completed, -high_count, -issue_count)


def _save_attempt_snapshot(
    state: dict,
    config: ProjectConfig,
    run_dir: Path,
    attempt: int,
) -> Path:
    """Persist every generated candidate for audit and best-version selection."""

    snapshot = run_dir / "attempts" / f"attempt_{attempt:02d}"
    snapshot.mkdir(parents=True, exist_ok=False)
    run_status, artifact_status = derive_outcome(state)
    save_output(state, config, snapshot, run_status, artifact_status)
    write_json(
        snapshot / "candidate_score.json",
        {
            "attempt": attempt,
            "score": list(_candidate_score(state)),
            "run_status": run_status.value,
            "artifact_status": artifact_status.value,
        },
    )
    return snapshot


def _load_repair_notebook(config: ProjectConfig) -> list[dict]:
    path = config.input_dir / ".pipeline" / REPAIR_NOTEBOOK_NAME
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError):
        return []
    return data if isinstance(data, list) else []


def _save_repair_notebook(config: ProjectConfig, history: list[dict]) -> None:
    path = config.input_dir / ".pipeline" / REPAIR_NOTEBOOK_NAME
    path.parent.mkdir(parents=True, exist_ok=True)
    write_json(path, history[-50:])


def run(
    config: ProjectConfig,
    max_retries: int = 3,
    archive: bool = True,
    resume_output: bool = False,
    initial_error: str = "",
    run_dir: Optional[Path] = None,
) -> dict:
    # Model clients are intentionally imported only for an actual run. Commands such as
    # --help and configuration inspection must not initialize network-backed providers.
    from agents.agent_4_code_generator import code_generator_agent
    from agents.agent_5_code_verifier import code_verifier_agent

    if max_retries < 0:
        raise ValueError("max_retries cannot be negative.")

    resolved_run_dir = resolve_run_dir(config, run_dir)
    candidate_output = resolved_run_dir / "candidate_output"
    if candidate_output.exists():
        raise ValueError(f"Candidate output already exists for this run: {candidate_output}")

    inputs = load_inputs(config)
    state = dict(make_initial_state(inputs, config))
    state["error_history"] = _load_repair_notebook(config)
    run_error_signatures: list[str] = []
    best_state: Optional[dict] = None
    best_score: Optional[Tuple[int, int, int, int, int]] = None
    best_attempt: Optional[int] = None
    if resume_output:
        output = config.output_path
        extension = config.build.source_extension
        state["generated_header"] = read_utf8(
            output / "6_generated_code.h"
        )
        state["generated_code"] = read_utf8(
            output / ("6_generated_code" + extension)
        )
        state["error_context"] = initial_error.strip()
        if not state["error_context"]:
            raise ValueError(
                "--resume-output requires non-empty external error context."
            )

    for attempt in range(max_retries + 1):
        mode = "repair" if state.get("error_context") else "initial"
        print(
            f"\n[ATTEMPT] {attempt + 1}/{max_retries + 1} ({mode})",
            flush=True,
        )

        before_error = state.get("error_context", "")
        state["stage_status"] = {
            **state.get("stage_status", {}),
            "agent_4_code_generator": StageStatus.RUNNING.value,
            "agent_5_code_verifier_compile": StageStatus.PENDING.value,
            "agent_5_code_verifier_fidelity": StageStatus.PENDING.value,
        }
        print(
            "[HEARTBEAT] "
            f"{datetime.now().isoformat(timespec='seconds')} "
            f"attempt {attempt + 1}: starting Implementation Generator model request",
            flush=True,
        )
        previous_header_for_audit = state.get("generated_header", "")
        previous_code_for_audit = state.get("generated_code", "")
        generated = code_generator_agent.generate(state)
        generation_error = str(generated.get("error_context", "")).strip()
        state.update(generated)
        if generation_error:
            state["error_context"] = generation_error
            state["code_gen_retries"] = state.get("code_gen_retries", 0) + 1
            state["stage_status"] = {
                **state.get("stage_status", {}),
                "agent_4_code_generator": StageStatus.FAILED.value,
                "agent_5_code_verifier_compile": StageStatus.SKIPPED.value,
                "agent_5_code_verifier_fidelity": StageStatus.SKIPPED.value,
            }
            state.setdefault("repair_history", []).append({
                "attempt": attempt + 1,
                "input_error": before_error,
                "generation_error": generation_error,
            })
            state["error_history"].append({
                "attempt": attempt + 1,
                "signature": _error_signature(generation_error),
                "error": generation_error,
            })
            print(f"[GENERATION] FAILED: {generation_error[:500]}")
            if attempt == max_retries:
                print("[RESULT] Retry limit reached after Implementation Generator generation failure")
            continue

        # A successful generation result supersedes the previous repair instruction.
        state["error_context"] = ""
        state["previous_generated_header"] = previous_header_for_audit
        state["previous_generated_code"] = previous_code_for_audit
        state["stage_status"] = {
            **state.get("stage_status", {}),
            "agent_4_code_generator": StageStatus.SUCCEEDED.value,
        }

        print(
            "[HEARTBEAT] "
            f"{datetime.now().isoformat(timespec='seconds')} "
            f"attempt {attempt + 1}: starting Implementation Verifier verification request",
            flush=True,
        )
        verified = code_verifier_agent.verify(state)
        state.update(verified)

        # Retain every candidate and select the best one independently of the
        # latest repair. This prevents a later full-file regeneration from
        # replacing a previously better implementation.
        attempt_number = attempt + 1
        attempt_score = _candidate_score(state)
        state["current_candidate_score"] = list(attempt_score)
        _save_attempt_snapshot(state, config, resolved_run_dir, attempt_number)
        if best_score is None or attempt_score > best_score:
            best_state = copy.deepcopy(state)
            best_score = attempt_score
            best_attempt = attempt_number
            print(
                f"[BEST] attempt {attempt_number} selected with score {list(attempt_score)}",
                flush=True,
            )
        else:
            print(
                f"[BEST] retained attempt {best_attempt}; current score {list(attempt_score)}",
                flush=True,
            )
        if is_release_ready(state):
            print("[RESULT] Verification succeeded")
            break

        blocked_gates = [
            name
            for name in REQUIRED_RELEASE_STAGES
            if state.get("stage_status", {}).get(name) == StageStatus.BLOCKED.value
        ]
        if blocked_gates:
            state.setdefault("repair_history", []).append({
                "attempt": attempt + 1,
                "input_error": before_error,
                "verification_error": state.get("error_context", ""),
                "blocked_gates": blocked_gates,
            })
            print(
                "[RESULT] Verification blocked by environment/provider gate(s): "
                + ", ".join(blocked_gates)
            )
            break

        if not str(state.get("error_context", "")).strip():
            missing = [
                name
                for name in REQUIRED_RELEASE_STAGES
                if state.get("stage_status", {}).get(name) != StageStatus.SUCCEEDED.value
            ]
            state["error_context"] = (
                "Verification did not produce explicit success evidence for required gates: "
                + ", ".join(missing)
            )

        state.setdefault("repair_history", []).append({
            "attempt": attempt + 1,
            "input_error": before_error,
            "verification_error": state.get("error_context", ""),
        })
        current_error = state.get("error_context", "")
        if current_error:
            signature = _error_signature(current_error)
            state["error_history"].append({
                "attempt": attempt + 1,
                "signature": signature,
                "error": current_error,
            })
            # Only current-run failures control early stopping. Persisted history
            # remains context for the model but must not consume this run's budget.
            run_error_signatures.append(signature)
            recent = run_error_signatures[-MAX_STAGNANT_REPAIRS:]
            if len(recent) == MAX_STAGNANT_REPAIRS and len(set(recent)) == 1:
                state["error_context"] = (
                    "Repair halted after repeated identical verification failures. "
                    "Use the recorded error history to make a targeted structural fix.\n"
                    + current_error
                )
                print("[RESULT] Repair halted: no progress across repeated error signature")
                break
        if attempt == max_retries:
            print("[RESULT] Retry limit reached with unresolved errors")

    # Publish/retain the best verified candidate, while preserving the full
    # repair history from the latest orchestration state.
    if best_state is not None:
        latest_history = state.get("repair_history", [])
        latest_error_history = state.get("error_history", [])
        latest_retries = state.get("code_gen_retries", 0)
        selected = copy.deepcopy(best_state)
        selected["repair_history"] = latest_history
        selected["error_history"] = latest_error_history
        selected["code_gen_retries"] = latest_retries
        selected["selected_attempt"] = best_attempt
        selected["selected_candidate_score"] = list(best_score or ())
        state = selected

    run_status, artifact_status = derive_outcome(state)
    if not is_release_ready(state) and not str(state.get("error_context", "")).strip():
        state["error_context"] = "Candidate is not release-ready; required verification evidence is missing."

    state["run_status"] = run_status.value
    state["artifact_status"] = artifact_status.value
    state["publication_status"] = PublicationStatus.NOT_PUBLISHED.value
    state["candidate_output"] = str(candidate_output)
    save_output(
        state,
        config,
        candidate_output,
        run_status,
        artifact_status,
    )
    first_round_bundle = _copy_first_round_bundle(
        state,
        config,
        candidate_output,
        resolved_run_dir.name,
    )
    _save_repair_notebook(config, state.get("error_history", []))
    write_json(
        config.input_dir / ".pipeline" / "latest_candidate.json",
        {
            "run_id": resolved_run_dir.name,
            "candidate_output": str(candidate_output),
            "first_round_bundle": str(first_round_bundle),
            "run_status": run_status.value,
            "artifact_status": artifact_status.value,
            "publication_status": state["publication_status"],
            "updated_at": datetime.now().isoformat(timespec="seconds"),
        },
    )

    if artifact_status == ArtifactStatus.RELEASE_READY:
        try:
            published = publish_output(candidate_output, config, archive)
        except PublicationError as exc:
            state["error_context"] = str(exc)
            state["run_status"] = RunStatus.BLOCKED.value
            state["publication_status"] = PublicationStatus.NOT_PUBLISHED.value
            save_output(
                state,
                config,
                candidate_output,
                RunStatus.BLOCKED,
                artifact_status,
            )
            print(f"[PUBLISH] BLOCKED: {exc}")
        else:
            state["publication_status"] = PublicationStatus.PUBLISHED.value
            state["published_output"] = str(published)
            write_json(
                config.input_dir / ".pipeline" / "latest_candidate.json",
                {
                    "run_id": resolved_run_dir.name,
                    "candidate_output": str(candidate_output),
                    "published_output": str(published),
                    "run_status": state["run_status"],
                    "artifact_status": state["artifact_status"],
                    "publication_status": state["publication_status"],
                    "fidelity_audit_skipped": bool(state.get("fidelity_audit_skipped", False)),
                    "updated_at": datetime.now().isoformat(timespec="seconds"),
                },
            )
            try:
                _record_candidate_publication(candidate_output, published)
            except Exception as exc:
                # Publication already completed atomically; failure to mirror metadata
                # into the diagnostic candidate must not invalidate that release.
                print(f"[WARN] Could not update candidate publication metadata: {exc}")
    else:
        print(
            "[PUBLISH] SKIPPED: candidate is not release-ready; "
            f"trusted output remains unchanged at {config.output_path}"
        )
    return state


def main() -> int:
    parser = argparse.ArgumentParser(description="Run constrained implementation generation and verification")
    parser.add_argument("--input-dir", default=".", help="Project directory")
    parser.add_argument("--project-config", default=None, help="Optional explicit project.json path")
    parser.add_argument("--model", required=True, choices=model_names())
    parser.add_argument(
        "--max-retries",
        type=int,
        default=3,
        help="Maximum repair retries after the initial generation (default: 3; 4 total attempts).",
    )
    parser.add_argument(
        "--resume-output",
        action="store_true",
        help=(
            "Use the current trusted_output as repair input. A failed repair never replaces it; "
            "a successful repair is published as a new trusted release."
        ),
    )
    parser.add_argument(
        "--initial-error-file",
        default=None,
        help="UTF-8 external validation error supplied to the first repair attempt.",
    )
    parser.add_argument(
        "--no-archive",
        action="store_true",
        help=(
            "Publish only when trusted_output does not already exist. Replacing an existing "
            "trusted release without preserving it is refused."
        ),
    )
    parser.add_argument(
        "--run-dir",
        default=None,
        help="Pipeline run directory under <project>/.pipeline/runs (normally supplied by pipeline.py).",
    )
    args = parser.parse_args()
    os.environ["FSE_MODEL"] = args.model

    try:
        config = ProjectConfig.load(args.input_dir, args.project_config)
        if not (config.input_dir / "project.json").exists() and not args.project_config:
            print("[WARN] project.json not found; using strict legacy C defaults inside input-dir only.")
        initial_error = (
            read_utf8(Path(args.initial_error_file).resolve())
            if args.initial_error_file
            else ""
        )
        state = run(
            config,
            max_retries=args.max_retries,
            archive=not args.no_archive,
            resume_output=args.resume_output,
            initial_error=initial_error,
            run_dir=Path(args.run_dir) if args.run_dir else None,
        )
        print(f"\n[CANDIDATE] {state.get('candidate_output', '(not written)')}")
        if state.get("publication_status") == PublicationStatus.PUBLISHED.value:
            print(f"[OUTPUT] {config.output_path}")
            return 0
        print(f"[OUTPUT] Trusted output was not changed: {config.output_path}")
        return 3 if state.get("run_status") == RunStatus.BLOCKED.value else 1
    except (FileNotFoundError, ValueError) as exc:
        print(f"[INPUT ERROR] {exc}")
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
