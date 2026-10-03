#!/usr/bin/env python3
"""Single command-line entry point for the embedded code-generation pipeline."""

from __future__ import annotations

import argparse
from datetime import datetime
import json
import os
from pathlib import Path
import subprocess
import sys
from typing import Dict, List
import uuid

from config import ProjectConfig, write_json
from model_config import model_names


SUPPORTED_STAGES = {"1", "2", "3", "4", "5"}
_LEGACY_STAGE_NAMES = {"6": "4", "6.5": "5"}
DEFAULT_FSE_CRAZYFLIE_DIR = Path(__file__).resolve().parents[2] / "examples" / "crazyflie"


def parse_stages(text: str, config: ProjectConfig) -> List[str]:
    raw_requested = [item.strip() for item in text.split(",") if item.strip()] if text else config.stages
    requested = [_LEGACY_STAGE_NAMES.get(stage, stage) for stage in raw_requested]
    unknown = sorted(set(requested) - SUPPORTED_STAGES)
    if unknown:
        raise ValueError(f"Unsupported stages: {', '.join(unknown)}")
    if any(stage in requested for stage in ["1", "2", "3"]) and not all(
        stage in requested for stage in ["1", "2", "3"]
    ):
        raise ValueError("The design phase is atomic: state, interface, and behavior modeling must run together.")
    if any(stage in requested for stage in ["4", "5"]) and not all(
        stage in requested for stage in ["4", "5"]
    ):
        raise ValueError("Code generation and verification are coupled; select 6,6.5 together.")
    return requested


def inspect_project(config: ProjectConfig) -> Dict[str, object]:
    files = {
        "requirement": config.path_for(config.requirement_file),
        "device_interface": config.path_for(config.device_interface_file),
        "api": config.path_for(config.api_file),
        "state_diagram": config.path_for(config.state_diagram_file),
        "state_model": config.path_for(config.state_model_file),
        "function_skeleton": config.path_for(config.skeleton_file),
        "interface_model": config.path_for(config.interface_model_file),
        "sequence_diagram": config.path_for(config.sequence_diagram_file),
        "behavior_model": config.path_for(config.behavior_model_file),
        "model_validation": config.path_for(config.model_validation_file),
    }
    return {
        "project": config.project_name,
        "input_dir": str(config.input_dir),
        "build": config.to_manifest_dict()["build"],
        "design_iteration": config.to_manifest_dict()["design_iteration"],
        "files": {
            name: {"path": str(path), "exists": bool(path and path.exists())}
            for name, path in files.items()
        },
    }


def ensure_design_valid(config: ProjectConfig) -> None:
    path = config.path_for(config.model_validation_file)
    if not path or not path.exists():
        raise ValueError("Missing canonical model validation report; run stages 1,2,3 first.")
    report = json.loads(path.read_text(encoding="utf-8"))
    if not report.get("valid", False):
        raise ValueError("Canonical model validation failed; Implementation Generator is blocked until design issues are repaired.")


def run_command(command: List[str], cwd: Path, dry_run: bool, model_name: str) -> int:
    print("[COMMAND] " + subprocess.list2cmdline(command))
    if dry_run:
        return 0
    child_env = os.environ.copy()
    child_env["FSE_MODEL"] = model_name
    result = subprocess.run(command, cwd=str(cwd), env=child_env)
    return result.returncode


def run_pipeline(
    config: ProjectConfig,
    stages: List[str],
    dry_run: bool,
    no_archive: bool,
    model_name: str,
    design_start_agent: str = "agent_1_state_modeler",
    code_repairs: int = 3,
    project_config_path: str | None = None,
) -> int:
    if code_repairs < 0:
        raise ValueError("code_repairs must be nonnegative")
    checking_dir = Path(__file__).resolve().parent
    run_id = datetime.now().strftime("%Y%m%d_%H%M%S") + "_" + uuid.uuid4().hex[:8]
    run_dir = config.input_dir / ".pipeline" / "runs" / run_id
    run_record = {
        "run_id": run_id,
        "project": config.project_name,
        "started_at": datetime.now().isoformat(timespec="seconds"),
        "requested_stages": stages,
        "status": "running",
        "stage_results": {},
    }
    if not dry_run:
        write_json(run_dir / "run.json", run_record)

    if all(stage in stages for stage in ["1", "2", "3"]):
        command = [
            sys.executable,
            str(checking_dir / "run_design_pipeline.py"),
            "--input-dir",
            str(config.input_dir),
            "--project-config",
            str(Path(project_config_path).resolve()) if project_config_path else str(config.input_dir / "project.json"),
            "--run-dir",
            str(run_dir),
            "--start-agent",
            design_start_agent,
            "--model",
            model_name,
        ]
        code = run_command(command, checking_dir, dry_run, model_name)
        run_record["stage_results"]["1-3"] = {"exit_code": code}
        if code != 0:
            run_record["status"] = "failed"
            run_record["finished_at"] = datetime.now().isoformat(timespec="seconds")
            if not dry_run:
                write_json(run_dir / "run.json", run_record)
                write_json(config.input_dir / ".pipeline" / "latest.json", {
                    "run_id": run_id,
                    "status": run_record["status"],
                    "record": str(run_dir / "run.json"),
                })
            return code

    if all(stage in stages for stage in ["4", "5"]):
        if not dry_run:
            try:
                ensure_design_valid(config)
            except (ValueError, FileNotFoundError, json.JSONDecodeError) as exc:
                run_record["status"] = "blocked"
                run_record["finished_at"] = datetime.now().isoformat(timespec="seconds")
                run_record["stage_results"]["4-5"] = {
                    "exit_code": 3,
                    "error": str(exc),
                }
                write_json(run_dir / "run.json", run_record)
                write_json(config.input_dir / ".pipeline" / "latest.json", {
                    "run_id": run_id,
                    "status": run_record["status"],
                    "record": str(run_dir / "run.json"),
                })
                print(f"[PIPELINE BLOCKED] {exc}")
                return 3
        command = [
            sys.executable,
            str(checking_dir / "run_agent_4_code_generator.py"),
            "--max-retries",
            str(code_repairs),
            "--input-dir",
            str(config.input_dir),
            "--project-config",
            str(Path(project_config_path).resolve()) if project_config_path else str(config.input_dir / "project.json"),
            "--run-dir",
            str(run_dir),
            "--model",
            model_name,
        ]
        if no_archive:
            command.append("--no-archive")
        code = run_command(command, checking_dir, dry_run, model_name)
        candidate_manifest_path = run_dir / "candidate_output" / "manifest.json"
        candidate_summary: Dict[str, object] = {"exit_code": code}
        if not dry_run and candidate_manifest_path.exists():
            candidate_manifest = json.loads(candidate_manifest_path.read_text(encoding="utf-8"))
            candidate_summary.update({
                "run_status": candidate_manifest.get("run_status"),
                "artifact_status": candidate_manifest.get("artifact_status"),
                "publication_status": candidate_manifest.get("publication_status"),
                "candidate_manifest": str(candidate_manifest_path),
            })
        run_record["stage_results"]["4-5"] = candidate_summary
        if code != 0:
            run_record["status"] = "blocked" if code == 3 else "failed"
            run_record["finished_at"] = datetime.now().isoformat(timespec="seconds")
            if not dry_run:
                write_json(run_dir / "run.json", run_record)
                write_json(config.input_dir / ".pipeline" / "latest.json", {
                    "run_id": run_id,
                    "status": run_record["status"],
                    "record": str(run_dir / "run.json"),
                })
            return code

    run_record["status"] = "dry_run" if dry_run else "succeeded"
    run_record["finished_at"] = datetime.now().isoformat(timespec="seconds")
    if not dry_run:
        write_json(run_dir / "run.json", run_record)
        write_json(config.input_dir / ".pipeline" / "latest.json", {
            "run_id": run_id,
            "status": run_record["status"],
            "record": str(run_dir / "run.json"),
        })
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description="Embedded code-generation pipeline")
    subparsers = parser.add_subparsers(dest="command", required=True)

    inspect_parser = subparsers.add_parser("inspect", help="Inspect resolved configuration and artifacts")
    inspect_parser.add_argument(
        "--input-dir",
        default=str(DEFAULT_FSE_CRAZYFLIE_DIR),
        help="Project input directory (default: examples/crazyflie).",
    )
    inspect_parser.add_argument("--project-config", default=None)

    api_parser = subparsers.add_parser("prepare-api", help="Prepare the function API branch beside Agent2")
    api_parser.add_argument("--input-dir", required=True)
    api_parser.add_argument("--project-config", default=None)
    api_parser.add_argument("--output-dir", required=True, help="New directory for the branch request and candidate")
    api_parser.add_argument("--generate", action="store_true", help="Explicitly call the model; otherwise prepare locally only")
    api_parser.add_argument("--model", choices=model_names(), default=None)

    run_parser = subparsers.add_parser("run", help="Run selected pipeline stages")
    run_parser.add_argument(
        "--input-dir",
        default=str(DEFAULT_FSE_CRAZYFLIE_DIR),
        help="Project input directory (default: examples/crazyflie).",
    )
    run_parser.add_argument("--project-config", default=None)
    run_parser.add_argument("--model", required=True, choices=model_names())
    run_parser.add_argument("--stages", default="", help="Comma-separated stages, e.g. 1,2,3,4,5")
    run_parser.add_argument("--dry-run", action="store_true")
    run_parser.add_argument("--no-archive", action="store_true")
    run_parser.add_argument("--code-repairs", type=int, default=3, help="Maximum code repairs after initial generation (default: 3)")
    run_parser.add_argument(
        "--design-start-agent",
        choices=["agent_1_state_modeler", "agent_2_interface_modeler", "agent_3_behavior_modeler"],
        default="agent_1_state_modeler",
        help="Preserve accepted upstream design artifacts and regenerate from this agent.",
    )

    args = parser.parse_args()
    try:
        config = ProjectConfig.load(args.input_dir, args.project_config)
        if args.command == "inspect":
            print(json.dumps(inspect_project(config), ensure_ascii=False, indent=2))
            return 0
        if args.command == "prepare-api":
            from agents.agent_2_api_generator import run_branch
            if args.model:
                os.environ["FSE_MODEL"] = args.model
            result = run_branch(config, Path(args.output_dir), generate=args.generate, model_name=args.model)
            print(json.dumps(result, ensure_ascii=False, indent=2))
            return 0
        stages = parse_stages(args.stages, config)
        return run_pipeline(
            config,
            stages,
            args.dry_run,
            args.no_archive,
            args.model,
            args.design_start_agent,
            args.code_repairs,
            args.project_config,
        )
    except (ValueError, FileNotFoundError, json.JSONDecodeError) as exc:
        print(f"[PIPELINE ERROR] {exc}")
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
