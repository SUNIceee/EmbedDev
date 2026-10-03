"""Implementation Verifier: deterministic compilation plus fail-closed fidelity audit."""

from __future__ import annotations

import json
import os
import re
import shutil
import subprocess
import tempfile
import uuid
from pathlib import Path
from typing import Any, Dict, List, Literal

from langchain_core.prompts import ChatPromptTemplate
from pydantic import BaseModel, ConfigDict, Field, StrictBool, field_validator

from runtime import NewGraphState
from runtime import DEFAULT_RUN_TIMEOUT_SECONDS, llm
from traceability import build_traceability


def compile_check(header_content: str, code_content: str, build_profile: dict = None) -> dict:
    if not header_content or not code_content:
        return {
            "success": False,
            "status": "failed",
            "errors": "Header or production code is empty.",
            "command": [],
        }

    profile = build_profile or {}
    language = str(profile.get("language", "c")).lower()
    compiler = str(profile.get("compiler", "g++" if language == "cpp" else "gcc"))
    standard = str(profile.get("standard", "c++17" if language == "cpp" else "c11"))
    extension = ".cpp" if language == "cpp" else ".c"
    compiler_path = shutil.which(compiler)
    if not compiler_path and Path(compiler).is_file():
        compiler_path = str(Path(compiler).resolve())
    if not compiler_path and compiler in {"gcc", "g++", "gcc.exe", "g++.exe"}:
        # The repository ships a Windows MinGW toolchain. Some managed shells
        # expose both ``Path`` and ``PATH`` environment entries, causing child
        # Python processes to miss it even though PowerShell can resolve gcc.
        executable = compiler if compiler.endswith(".exe") else f"{compiler}.exe"
        bundled = (
            Path(__file__).resolve().parents[2]
            / ".tools"
            / "mingw64"
            / "bin"
            / executable
        )
        if bundled.is_file():
            compiler_path = str(bundled)
    if not compiler_path:
        return {
            "success": False,
            "status": "blocked",
            "errors": f"Required compiler not found: {compiler}",
            "command": [],
        }

    temp_root = profile.get("temp_dir")
    temp_root = temp_root or tempfile.gettempdir()
    os.makedirs(temp_root, exist_ok=True)
    # tempfile.mkdtemp can inherit unusable ACLs in some managed Windows environments.
    # A normal directory creation under the configured root is predictable and still unique.
    tmpdir = os.path.join(temp_root, "agent_4_code_generator5_" + uuid.uuid4().hex)
    os.makedirs(tmpdir)
    header_path = os.path.join(tmpdir, "6_generated_code.h")
    code_path = os.path.join(tmpdir, "6_generated_code" + extension)
    command: List[str] = []
    try:
        with open(header_path, "w", encoding="utf-8") as stream:
            stream.write(header_content)
        with open(code_path, "w", encoding="utf-8") as stream:
            stream.write(code_content)

        command = [compiler_path, f"-std={standard}"]
        command.extend(profile.get("compile_flags", ["-Wall", "-Wextra"]))
        command.extend(["-fsyntax-only", "-I", tmpdir])
        for include_dir in profile.get("include_dirs", []):
            command.extend(["-I", str(include_dir)])
        for define in profile.get("defines", []):
            command.append(f"-D{define}")
        command.append(code_path)

        result = subprocess.run(
            command,
            capture_output=True,
            text=True,
            timeout=DEFAULT_RUN_TIMEOUT_SECONDS,
        )
        return {
            "success": result.returncode == 0,
            "status": "succeeded" if result.returncode == 0 else "failed",
            "errors": result.stderr.strip(),
            "command": command,
        }
    except subprocess.TimeoutExpired:
        return {
            "success": False,
            "status": "failed",
            "errors": (
                "Compilation timed out after "
                f"{DEFAULT_RUN_TIMEOUT_SECONDS} seconds."
            ),
            "command": command,
        }
    except Exception as exc:
        return {
            "success": False,
            "status": "failed",
            "errors": f"Compile check exception: {exc}",
            "command": command,
        }
    finally:
        shutil.rmtree(tmpdir, ignore_errors=True)


class FidelityIssue(BaseModel):
    function_name: str = Field(description="Affected symbol")
    issue_type: str = Field(description="ADDED_BEHAVIOR, CONTROL_FLOW_CHANGE, TYPE_MISMATCH, or MISSING_LOGIC")
    description: str = Field(description="Specific difference and expected behavior")
    severity: Literal["HIGH", "MEDIUM", "LOW"] = Field(description="HIGH, MEDIUM, or LOW")

    @field_validator("severity", mode="before")
    @classmethod
    def _normalize_severity(cls, value: Any) -> Any:
        return value.strip().upper() if isinstance(value, str) else value


class FidelityAuditOutput(BaseModel):
    model_config = ConfigDict(extra="forbid")

    reasoning: str = Field(min_length=1, description="Concise evidence-based comparison")
    issues: List[FidelityIssue]
    is_faithful: StrictBool

    @field_validator("reasoning")
    @classmethod
    def _require_reasoning(cls, value: str) -> str:
        stripped = value.strip()
        if not stripped:
            raise ValueError("reasoning must not be blank")
        return stripped


class CodeVerifierAgent:
    def __init__(self):
        self.audit_prompt = ChatPromptTemplate.from_messages([
            ("system", (
                "You audit embedded production code against its declared design. Compare public symbols, "
                "types, formulas, state transitions, execution order, and required behavior. Report only "
                "evidence-supported differences. HIGH means missing or behavior-changing logic; MEDIUM means "
                "a boundary/type risk; LOW means non-functional style. is_faithful is true only when there are "
                "no HIGH issues. Treat Canonical Interface/State/Behavior Model sections as JSON contracts. Use the "
                "Traceability Summary as mandatory evidence: every missing `implemented_by` or "
                "`realized_by` edge identifies an interface function or behavior step with no matching "
                "production-code function and must be reported as a HIGH MISSING_LOGIC issue unless the "
                "trace evidence itself is demonstrably incorrect. "
                "device/environment interface as authoritative evidence for platform adapter boundaries, "
                "hardware/protocol/tick/queue behavior, units, and mock/observable ownership, but never "
                "as a reason to change frozen public API declarations. In repair mode, compare the current "
                "candidate with the previous candidate as well: verify that the requested local fix is present "
                "and that unrelated functions, shared state, timers, queues, API symbols, and cross-function "
                "invariants were not regressed. Report any regression as HIGH. A local repair may change a "
                "dependent helper or call site only when the dependency is necessary and behaviorally justified."
            )),
            ("human", (
                "=== Requirements ===\n{req}\n\n"
                "=== Device / Environment Interface ===\n{device_interface}\n\n"
                "=== API Interface ===\n{api_spec}\n\n"
                "=== Function Skeleton ===\n{skeleton}\n\n"
                "=== Canonical State Model ===\n{state_model}\n\n"
                "=== Canonical Interface Model ===\n{interface_model}\n\n"
                "=== Canonical Behavior Model ===\n{behavior_model}\n\n"
                "=== State Diagram ===\n{state_uml}\n\n"
                "=== Sequence Diagram ===\n{sequence_uml}\n\n"
                "=== Traceability Summary ===\n{traceability}\n\n"
                "=== Generated Header ===\n{generated_header}\n\n"
                "=== Previous Candidate Header (repair comparison; may be empty) ===\n{previous_header}\n\n"
                "=== Generated Production Code ===\n{generated_code}\n"
                "=== Previous Candidate Production Code (repair comparison; may be empty) ===\n{previous_code}\n"
            )),
        ])
        self.audit_chain = self.audit_prompt | llm.with_structured_output(FidelityAuditOutput)

    @staticmethod
    def _status(state: NewGraphState, **updates: str) -> Dict[str, str]:
        return {**state.get("stage_status", {}), **updates}

    @staticmethod
    def _render_json(value: object) -> str:
        return json.dumps(value or {}, ensure_ascii=False, indent=2)

    def verify(self, state: NewGraphState) -> Dict[str, Any]:
        print("\n" + "=" * 60)
        print("--- AGENT 5: SYNTAX AND FIDELITY VERIFICATION ---")
        print("=" * 60)

        header = state.get("generated_header", "")
        code = state.get("generated_code", "")
        compile_result = compile_check(header, code, state.get("build_profile", {}))
        if not compile_result["success"]:
            error = compile_result["errors"]
            print(f"[COMPILE] {compile_result['status'].upper()}: {error[:500]}")
            return {
                "fidelity_report": f"COMPILE_{compile_result['status'].upper()}:\n{error}",
                "error_context": f"Compilation must be repaired:\n{error}",
                "code_gen_retries": state.get("code_gen_retries", 0) + 1,
                "stage_status": self._status(
                    state,
                    agent_5_code_verifier_compile=compile_result["status"],
                    agent_5_code_verifier_fidelity="skipped",
                ),
            }

        print("[COMPILE] SUCCEEDED")
        try:
            traceability = build_traceability(
                requirements=state.get("user_requirement", ""),
                state_model=state.get("state_model", {}),
                interface_model=state.get("interface_model", {}),
                behavior_model=state.get("behavior_model", {}),
                generated_code=code,
            )
            missing_implementation = [
                edge for edge in traceability["edges"]
                if edge["relation"] in {"implemented_by", "realized_by"}
                and edge["status"] == "missing"
            ]
            traceability_focus = {
                "summary": traceability["summary"],
                "missing_implementation_edges": missing_implementation[:200],
            }
            audit: FidelityAuditOutput = self.audit_chain.invoke({
                "req": state.get("user_requirement", "N/A"),
                "device_interface": state.get("device_interface", "") or "(not provided)",
                "api_spec": state.get("api_spec", "(not provided)"),
                "skeleton": state.get("function_skeleton", "(not provided)"),
                "state_model": self._render_json(state.get("state_model", {})),
                "interface_model": self._render_json(state.get("interface_model", {})),
                "behavior_model": self._render_json(state.get("behavior_model", {})),
                "state_uml": state.get("state_diagram", "(not provided)"),
                "sequence_uml": state.get("sequence_diagram", "(not provided)"),
                "traceability": self._render_json(traceability_focus),
                "generated_header": header,
                "previous_header": state.get("previous_generated_header", "") or "(none)",
                "generated_code": code,
                "previous_code": state.get("previous_generated_code", "") or "(none)",
            })
            if not audit:
                return {
                    "fidelity_report": "AUDIT_BLOCKED: no model result",
                    "error_context": "Implementation Verifier fidelity audit returned no result.",
                    "code_gen_retries": state.get("code_gen_retries", 0) + 1,
                    "traceability": traceability,
                    "stage_status": self._status(
                        state,
                        agent_5_code_verifier_compile="succeeded",
                        agent_5_code_verifier_fidelity="blocked",
                    ),
                }

            high_issues = [issue for issue in audit.issues if issue.severity == "HIGH"]
            audit_passed = audit.is_faithful is True and not high_issues
            report_lines = [f"Fidelity Audit: {'PASSED' if audit_passed else 'FAILED'}"]
            report_lines.append(f"Declared faithful: {str(audit.is_faithful).lower()}")
            report_lines.append(f"Reasoning: {audit.reasoning}")
            for issue in audit.issues:
                report_lines.append(
                    f"[{issue.severity}] {issue.issue_type} in {issue.function_name}: {issue.description}"
                )
            report = "\n".join(report_lines)

            if not audit_passed:
                repairs = ["Fidelity audit rejected this candidate; it is not publishable."]
                if audit.is_faithful is not True:
                    repairs.append("- The audit explicitly returned is_faithful=false.")
                if high_issues:
                    repairs.append("Repair these HIGH fidelity issues while preserving unaffected code:")
                repairs.extend(
                    f"- [{issue.issue_type}] {issue.function_name}: {issue.description}"
                    for issue in high_issues
                )
                medium_issues = [issue for issue in audit.issues if issue.severity == "MEDIUM"]
                if medium_issues:
                    repairs.append(
                        "Regression watch items (preserve or repair these after addressing HIGH issues):"
                    )
                    repairs.extend(
                        f"- [{issue.issue_type}] {issue.function_name}: {issue.description}"
                        for issue in medium_issues
                    )
                if not high_issues:
                    repairs.append(
                        "- No HIGH issue details were supplied; regenerate or repair the code and "
                        "obtain an explicit faithful audit before publication."
                    )
                return {
                    "fidelity_report": report,
                    "error_context": "\n".join(repairs),
                    "code_gen_retries": state.get("code_gen_retries", 0) + 1,
                    "traceability": traceability,
                    "stage_status": self._status(
                        state,
                        agent_5_code_verifier_compile="succeeded",
                        agent_5_code_verifier_fidelity="failed",
                    ),
                }

            return {
                "fidelity_report": report,
                "error_context": "",
                "traceability": traceability,
                "stage_status": self._status(
                    state,
                    agent_5_code_verifier_compile="succeeded",
                    agent_5_code_verifier_fidelity="succeeded",
                ),
            }
        except Exception as exc:
            import traceback
            traceback.print_exc()
            return {
                "fidelity_report": f"AUDIT_BLOCKED: {exc}",
                "error_context": f"Implementation Verifier audit exception: {exc}",
                "code_gen_retries": state.get("code_gen_retries", 0) + 1,
                "traceability": locals().get("traceability", state.get("traceability", {})),
                "stage_status": self._status(
                    state,
                    agent_5_code_verifier_compile="succeeded",
                    agent_5_code_verifier_fidelity="blocked",
                ),
            }


code_verifier_agent = CodeVerifierAgent()
