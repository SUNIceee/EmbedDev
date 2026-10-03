#!/usr/bin/env python3
"""CLI entry point for the adaptive three-stage design pipeline."""

from __future__ import annotations

import argparse
from datetime import datetime
import json
import os
from pathlib import Path
import uuid

from config import ProjectConfig
from design import AdaptiveDesignRunner
from model_config import model_names
from traceability import write_traceability


def main() -> int:
    parser = argparse.ArgumentParser(description="Run adaptive state/interface/behavior design")
    parser.add_argument("--input-dir", required=True)
    parser.add_argument("--project-config", default=None)
    parser.add_argument("--run-dir", default=None)
    parser.add_argument("--model", required=True, choices=model_names())
    parser.add_argument(
        "--start-agent",
        choices=["agent_1_state_modeler", "agent_2_interface_modeler", "agent_3_behavior_modeler"],
        default="agent_1_state_modeler",
        help="Preserve accepted upstream artifacts and regenerate from this stage.",
    )
    args = parser.parse_args()
    os.environ["FSE_MODEL"] = args.model

    config = ProjectConfig.load(args.input_dir, args.project_config)
    run_dir = (
        Path(args.run_dir).resolve()
        if args.run_dir
        else config.input_dir
        / ".pipeline"
        / "design_runs"
        / (
            datetime.now().strftime("%Y%m%d_%H%M%S")
            + "_"
            + uuid.uuid4().hex[:8]
        )
    )
    result = AdaptiveDesignRunner(config, run_dir).run(args.start_agent)
    if result["status"] == "succeeded":
        write_traceability(
            run_dir / "traceability.json",
            requirements=config.path_for(config.requirement_file).read_text(encoding="utf-8"),
            state_model=json.loads(config.path_for(config.state_model_file).read_text(encoding="utf-8")),
            interface_model=json.loads(config.path_for(config.interface_model_file).read_text(encoding="utf-8")),
            behavior_model=json.loads(config.path_for(config.behavior_model_file).read_text(encoding="utf-8")),
        )
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0 if result["status"] == "succeeded" else 1


if __name__ == "__main__":
    raise SystemExit(main())
