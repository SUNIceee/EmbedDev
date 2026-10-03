#!/usr/bin/env python3
"""Generate a frozen API candidate from REQ + device interface.

This is a dataset-preparation helper, not a runtime pipeline agent.
It writes:
  - RE_api.candidate.txt
  - API_REVIEW_REPORT.json
  - optional frozen RE_api.txt backup + replacement
"""

from __future__ import annotations

import argparse
from datetime import datetime
import json
from pathlib import Path
import shutil
import sys
from typing import Any, Dict

from pydantic import BaseModel, Field

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from runtime import llm  # noqa: E402
from config import ProjectConfig  # noqa: E402


class ApiGenerationOutput(BaseModel):
    candidate_api: str = Field(description="Complete RE_api.candidate.txt content")
    review_report: Dict[str, Any] = Field(
        description="Complete API_REVIEW_REPORT.json content as a JSON object"
    )


def read_text(path: Path, required: bool = True) -> str:
    if not path or not path.exists():
        if required:
            raise FileNotFoundError(path)
        return ""
    return path.read_text(encoding="utf-8")


def build_prompt(
    project_name: str,
    req: str,
    device_interface: str,
    previous_api: str,
) -> str:
    today = datetime.now().strftime("%Y-%m-%d")
    previous_section = (
        previous_api
        if previous_api.strip()
        else "No previous API is available."
    )
    return f"""
You are a senior C11 API designer, embedded systems engineer, and requirements auditor.

Generate a revised {project_name} software API candidate from the supplied requirements
and device/environment interface. This is a dataset-preparation task, not production code
generation and not an Interface Modeler redesign.

Evidence priority:
1. RE_req.txt is authoritative for required behavior, edge cases, timing, safety rules,
   algorithms, and testcase semantics.
2. RE_device_interface.txt is authoritative for environment facts: hardware, protocols,
   RTOS tick/task/queue behavior, sensor/actuator I/O, mock boundaries, platform adapter
   boundaries, and externally visible host-test observable locations.
3. Previous RE_api.txt is a compatibility baseline. Preserve public names, types, constants,
   and host-test observable symbols when they are still consistent with REQ + device interface.
   Change them only when the device interface exposes a clear naming/unit/boundary problem.

Hard rules:
- Output a complete C11 API contract document, not implementation code.
- Do not include main(), test harness code, or generated production implementation.
- Separate Public Production API, Platform Adapter API, Internal Interface notes, and Host-Test Adapter.
- Device-interface facts may define adapter boundaries, units, mock ownership, and observable
  symbol ownership. They must not create business behavior that is absent from REQ.
- Keep host-test adapter symbols explicit when REQ requires host verification of otherwise
  unobservable internal state.
- Avoid saying the source is a test JSON. The source is RE_req.txt plus RE_device_interface.txt;
  the previous API is only a compatibility baseline.
- Every public symbol must have a stable API id, source requirements, origin, parameter units/ranges,
  ownership/nullability rules, and execution-context notes where relevant.
- The review report must be parseable JSON and include at least:
  schema_version, project, generated_at, ready_for_freeze, inputs, decisions_required,
  environment_catalog, environment_gaps, naming_conflicts, public_symbols, host_test_adapter_symbols,
  requirement_traceability, unresolved_high_issue_count, reviewer_notes.
- If you believe human review is still needed for a decision, include it in decisions_required.
  For this experimental run, still produce a coherent complete candidate.

Project:
- project_name: {project_name}
- target_language: C11
- target_platform: host-testable Crazyflie firmware core with embedded-platform adapter boundaries
- date: {today}
- candidate_version: 0.2.0-device-interface-candidate

Return only structured fields:
- candidate_api: complete RE_api.candidate.txt
- review_report: JSON object

=== RE_req.txt ===
{req}

=== RE_device_interface.txt ===
{device_interface or "No device interface provided."}

=== Previous RE_api.txt compatibility baseline ===
{previous_section}
"""


def freeze_candidate(candidate: str, project_name: str, report_name: str) -> str:
    today = datetime.now().strftime("%Y-%m-%d")
    header = (
        f"Status: FROZEN\n"
        f"API Version: 0.2.0-device-interface\n"
        f"Frozen Date: {today}\n"
        f"Project: {project_name}\n"
        f"Source SRS: RE_req.txt\n"
        f"Device Interface: RE_device_interface.txt\n"
        f"Review Report: {report_name}\n\n"
    )
    stripped = candidate.strip()
    if stripped.startswith("Status:"):
        lines = stripped.splitlines()
        while lines and (
            lines[0].startswith("Status:")
            or lines[0].startswith("API Version:")
            or lines[0].startswith("Frozen Date:")
            or lines[0].startswith("Project:")
            or lines[0].startswith("Source SRS:")
            or lines[0].startswith("Device Interface:")
            or lines[0].startswith("Review Report:")
            or not lines[0].strip()
        ):
            lines.pop(0)
        stripped = "\n".join(lines).lstrip()
    return header + stripped + "\n"


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input-dir", required=True)
    parser.add_argument("--project-config", default=None)
    parser.add_argument("--freeze", action="store_true")
    args = parser.parse_args()

    config = ProjectConfig.load(args.input_dir, args.project_config)
    req = read_text(config.path_for(config.requirement_file))
    device = read_text(config.path_for(config.device_interface_file), required=False)
    api_path = config.path_for(config.api_file)
    previous_api = read_text(api_path, required=False)

    print(f"[API GEN] project={config.project_name}")
    print(f"[API GEN] req={len(req)} chars device={len(device)} chars previous_api={len(previous_api)} chars")

    chain = llm.with_structured_output(ApiGenerationOutput)
    result: ApiGenerationOutput = chain.invoke(
        build_prompt(config.project_name, req, device, previous_api)
    )
    if not result or not result.candidate_api.strip():
        raise RuntimeError("LLM returned empty API candidate.")

    candidate_path = config.input_dir / "RE_api.candidate.txt"
    review_path = config.input_dir / "API_REVIEW_REPORT.json"
    candidate_path.write_text(result.candidate_api.strip() + "\n", encoding="utf-8")
    review_path.write_text(
        json.dumps(result.review_report, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    print(f"[API GEN] wrote {candidate_path}")
    print(f"[API GEN] wrote {review_path}")

    if args.freeze:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        if api_path and api_path.exists():
            backup_path = api_path.with_name(f"RE_api.previous_{timestamp}.txt")
            shutil.copy2(api_path, backup_path)
            print(f"[API GEN] backed up previous API to {backup_path}")
        frozen = freeze_candidate(result.candidate_api, config.project_name, review_path.name)
        api_path.write_text(frozen, encoding="utf-8")
        print(f"[API GEN] froze candidate to {api_path}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
