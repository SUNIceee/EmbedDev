"""Deterministic project configuration and artifact resolution for the pipeline."""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
from enum import Enum
import json
from pathlib import Path
from typing import Any, Dict, List, Optional


class StageStatus(str, Enum):
    PENDING = "pending"
    RUNNING = "running"
    SUCCEEDED = "succeeded"
    FAILED = "failed"
    SKIPPED = "skipped"
    DEGRADED = "degraded"
    BLOCKED = "blocked"


class RunStatus(str, Enum):
    """Outcome of one orchestration run."""

    RUNNING = "running"
    SUCCEEDED = "succeeded"
    FAILED = "failed"
    BLOCKED = "blocked"
    DRY_RUN = "dry_run"


class ArtifactStatus(str, Enum):
    """Verification maturity of an Implementation Generator candidate artifact set."""

    GENERATED = "generated"
    GENERATION_FAILED = "generation_failed"
    VERIFICATION_FAILED = "verification_failed"
    VERIFICATION_BLOCKED = "verification_blocked"
    RELEASE_READY = "release_ready"


class PublicationStatus(str, Enum):
    """Whether a candidate artifact set was published as trusted output."""

    NOT_PUBLISHED = "not_published"
    PUBLISHED = "published"


@dataclass(frozen=True)
class BuildProfile:
    language: str = "c"
    standard: str = "c11"
    compiler: str = "gcc"
    compile_flags: List[str] = field(default_factory=lambda: ["-Wall", "-Wextra"])
    include_dirs: List[str] = field(default_factory=list)
    defines: List[str] = field(default_factory=list)
    platform: str = "host"

    @property
    def source_extension(self) -> str:
        return ".cpp" if self.language == "cpp" else ".c"

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "BuildProfile":
        language = str(data.get("language", "c")).lower()
        if language not in {"c", "cpp"}:
            raise ValueError(f"Unsupported build language: {language}")
        default_compiler = "g++" if language == "cpp" else "gcc"
        default_standard = "c++17" if language == "cpp" else "c11"
        return cls(
            language=language,
            standard=str(data.get("standard", default_standard)),
            compiler=str(data.get("compiler", default_compiler)),
            compile_flags=list(data.get("compile_flags", ["-Wall", "-Wextra"])),
            include_dirs=list(data.get("include_dirs", [])),
            defines=list(data.get("defines", [])),
            platform=str(data.get("platform", "host")),
        )


@dataclass(frozen=True)
class DesignIterationProfile:
    max_iterations: int = 4
    max_local_repair_attempts: int = 2
    regenerate_high_issue_count: int = 4
    error_notebook_top_k: int = 4

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "DesignIterationProfile":
        profile = cls(
            max_iterations=int(data.get("max_iterations", 4)),
            max_local_repair_attempts=int(
                data.get("max_local_repair_attempts", 2)
            ),
            regenerate_high_issue_count=int(
                data.get("regenerate_high_issue_count", 4)
            ),
            error_notebook_top_k=int(data.get("error_notebook_top_k", 4)),
        )
        if profile.max_iterations < 1:
            raise ValueError("max_iterations must be at least 1")
        if profile.max_local_repair_attempts < 0:
            raise ValueError("max_local_repair_attempts cannot be negative")
        if profile.regenerate_high_issue_count < 1:
            raise ValueError("regenerate_high_issue_count must be at least 1")
        return profile


@dataclass(frozen=True)
class ProjectConfig:
    project_name: str
    input_dir: Path
    requirement_file: str = "RE_req.txt"
    device_interface_file: Optional[str] = None
    api_file: str = "RE_api.txt"
    state_diagram_file: str = "1_state_diagram.puml"
    state_model_file: str = "1_state_model.json"
    sequence_diagram_file: str = "3_sequence_diagram.puml"
    behavior_model_file: str = "3_behavior_model.json"
    skeleton_file: str = "2_function_skeleton.h"
    interface_model_file: str = "2_interface_model.json"
    model_validation_file: str = "3_5_model_validation.json"
    tla_file: Optional[str] = None
    output_dir: str = "trusted_output"
    archive_dir: str = "archives"
    build: BuildProfile = field(default_factory=BuildProfile)
    design_iteration: DesignIterationProfile = field(
        default_factory=DesignIterationProfile
    )
    stages: List[str] = field(default_factory=lambda: ["1", "2", "3", "4", "5"])

    @classmethod
    def load(cls, input_dir: str, config_path: Optional[str] = None) -> "ProjectConfig":
        root = Path(input_dir).expanduser().resolve()
        path = Path(config_path).expanduser().resolve() if config_path else root / "project.json"

        if path.exists():
            data = json.loads(path.read_text(encoding="utf-8"))
            inputs = data.get("inputs", {})
            pipeline = data.get("pipeline", {})
            return cls(
                project_name=str(data.get("project_name", root.name)),
                input_dir=root,
                requirement_file=str(inputs.get("requirement", "RE_req.txt")),
                device_interface_file=inputs.get("device_interface"),
                api_file=str(inputs.get("api", "RE_api.txt")),
                state_diagram_file=str(inputs.get("state_diagram", "1_state_diagram.puml")),
                state_model_file=str(inputs.get("state_model", "1_state_model.json")),
                sequence_diagram_file=str(inputs.get("sequence_diagram", "3_sequence_diagram.puml")),
                behavior_model_file=str(inputs.get("behavior_model", "3_behavior_model.json")),
                skeleton_file=str(inputs.get("function_skeleton", "2_function_skeleton.h")),
                interface_model_file=str(inputs.get("interface_model", "2_interface_model.json")),
                model_validation_file=str(inputs.get("model_validation", "3_5_model_validation.json")),
                tla_file=inputs.get("tla"),
                output_dir=str(data.get("output_dir", "trusted_output")),
                archive_dir=str(data.get("archive_dir", "archives")),
                build=BuildProfile.from_dict(data.get("build", {})),
                design_iteration=DesignIterationProfile.from_dict(
                    pipeline.get("design_iteration", {})
                ),
                stages=list(pipeline.get("stages", ["1", "2", "3", "4", "5"])),
            )

        # Backward-compatible legacy mode is explicit in logs and never searches outside input_dir.
        return cls(project_name=root.name, input_dir=root)

    def path_for(self, filename: Optional[str]) -> Optional[Path]:
        if not filename:
            return None
        candidate = (self.input_dir / filename).resolve()
        try:
            candidate.relative_to(self.input_dir)
        except ValueError as exc:
            raise ValueError(f"Input path escapes project directory: {filename}") from exc
        return candidate

    def _project_path(self, configured: str, label: str) -> Path:
        candidate = (self.input_dir / configured).resolve()
        try:
            relative = candidate.relative_to(self.input_dir)
        except ValueError as exc:
            raise ValueError(f"{label} escapes project directory: {configured}") from exc
        if not relative.parts:
            raise ValueError(f"{label} cannot be the project root: {configured}")
        return candidate

    @property
    def output_path(self) -> Path:
        return self._project_path(self.output_dir, "output_dir")

    @property
    def archive_path(self) -> Path:
        candidate = self._project_path(self.archive_dir, "archive_dir")
        output = self.output_path
        try:
            candidate.relative_to(output)
        except ValueError:
            pass
        else:
            raise ValueError("archive_dir cannot be inside output_dir")
        try:
            output.relative_to(candidate)
        except ValueError:
            pass
        else:
            raise ValueError("output_dir cannot be inside archive_dir")
        return candidate

    def to_manifest_dict(self) -> Dict[str, Any]:
        result = asdict(self)
        result["input_dir"] = str(self.input_dir)
        return result


def write_json(path: Path, data: Dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding="utf-8")
    temporary.replace(path)
