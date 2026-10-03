"""Implementation Generator: constrained production-code generation and targeted repair."""

from typing import Dict
import json
import re

from langchain_core.prompts import ChatPromptTemplate
from pydantic import BaseModel, Field

from context import retrieve_domain_context
from runtime import NewGraphState
from runtime import llm


class CodeOutput(BaseModel):
    reasoning: str = Field(default="", description="Concise implementation reasoning")
    implementation_blueprint: str = Field(
        default="",
        description=(
            "Global implementation blueprint: state map, observable map, timer/tick map, "
            "function responsibility map, and cross-function invariants used before writing code"
        ),
    )
    self_check: str = Field(default="", description="Coverage and fidelity self-check")
    generated_header: str = Field(default="", description="Complete public header")
    generated_code: str = Field(
        default="",
        description="Complete production implementation without main() or test harness",
    )


class BoardCodeOutput(CodeOutput):
    generated_code: str = Field(
        default="", description="Complete STM32 GPIO application including main, HAL_GPIO_EXTI_Callback and EXTI0_IRQHandler; no test harness"
    )


class CodeGeneratorAgent:
    def __init__(self, *, target_profile: str = "host_library"):
        if target_profile not in ("host_library", "stm32_gpio_application"):
            raise ValueError("Unsupported code generation target profile")
        self.target_profile = target_profile
        production_rules = (
            "- Production implementation must include only 6_generated_code.h plus required system headers.\n"
            "- Production implementation MUST NOT contain main() or an embedded test harness.\n"
        ) if target_profile == "host_library" else (
            "- Target is the STM32F4-Discovery GPIO application, not a host library.\n"
            "- Emit main(void), HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin), and EXTI0_IRQHandler(void) "
            "with their exact platform signatures. Internal helper names are unrestricted.\n"
            "- Source includes 6_generated_code.h; that header includes the supplied main.h SDK header. "
            "Do not redefine HAL/BSP SDK types, constants, functions or substitute host mocks.\n"
            "- Fixed scaffolding provides startup, system initialization, SysTick and other exception handlers. "
            "Do not duplicate those definitions. Implement only the application and EXTI0 handler.\n"
            "- No test harness, test-output printing, or changes to the fixed SDK/scaffolding.\n"
        )
        self.prompt = ChatPromptTemplate.from_messages([
            ("system", (
                "You are a senior embedded software engineer. Generate code from the supplied "
                "design artifacts without changing the declared public contract.\n\n"
                "Evidence priority:\n"
                "1. Frozen API and function skeleton are authoritative for public declarations, "
                "types, constants, extern symbols, linkage, and function signatures.\n"
                "2. Requirements are authoritative for behavior: edge cases, state transitions, "
                "timing rules, safety conditions, algorithms, and testcase-specific semantics.\n"
                "3. Device/environment interface is authoritative for external environment facts: "
                "hardware/protocol/RTOS/tick/queue/sensor/actuator/mock boundaries, adapter I/O, "
                "units, and externally visible platform state. It never overrides frozen public "
                "API declarations or requirement behavior.\n"
                "4. Canonical State Modeler/2/3 artifacts are structured design aids and semantic indexes. "
                "Use them to organize state, function contracts, and execution order, but they do "
                "not replace the requirements.\n"
                "5. Retrieved domain patterns are reference only and never override API/REQ/design.\n\n"
                "Rules:\n"
                "- Use the exact public names, types, constants, and function signatures in the skeleton.\n"
                "- Do NOT use module-by-module code generation followed by later stitching. For strongly "
                "stateful embedded programs, splitting implementation by module often creates inconsistent "
                "shared state, aliases, linkage, and timers. Generate one coherent header "
                "and one coherent implementation file with a single global view of shared state.\n"
                "- Before writing source code, build a GLOBAL IMPLEMENTATION BLUEPRINT in "
                "`implementation_blueprint`. This blueprint is mandatory and must contain:\n"
                "  1) State map: each State Modeler/Interface Modeler state or flag -> exact C storage symbol -> readers/writers.\n"
                "  2) State-alias map: each semantic state/alias -> storage symbol -> readers, writers, and synchronization rule.\n"
                "  3) Timer/tick map: each required timer, sentinel tick, timeout, hysteresis window, rate gate, "
                "or counter -> storage -> update/reset points -> comparison formula.\n"
                "  4) Function responsibility map: for every public function, list the REQ anchors, required "
                "state updates, required timers, observables touched, and cross-function dependencies.\n"
                "  5) Cross-function invariants: shared-state rules that must remain true across commander, "
                "supervisor, stabilizer, health, motor, protocol, queue, and adapter-style functions.\n"
                "  6) Helper/call-chain evidence map: if a required behavior is implemented in a helper "
                "instead of directly in the public function named by the contract/testcase, record the public "
                "entry point, helper name, shared state passed or touched, and the source evidence that the "
                "entry point always reaches the helper on the required branch.\n"
                "- Treat this blueprint as an internal ledger for whole-file generation. It is not a request "
                "to split final code into separately generated modules; the final code must use one consistent "
                "set of names and storage across all functions.\n"
                "- Treat Canonical Interface/State/Behavior Model sections as JSON, not prose. Implement "
                "each public function from its contract fields: purpose, preconditions, postconditions, "
                "reads_state, writes_state, error_cases, source_requirements, req_anchors, "
                "semantic_requirements, required_state_updates, required_timers, "
                "canonical_observables, allowed_aliases, and required_code_patterns. For BehaviorStep entries also implement "
                "step_conditions, step_state_updates, step_req_anchors, and "
                "step_required_code_patterns.\n"
                "- Function signatures come from the frozen API and function skeleton. Function behavior "
                "must be reconciled from both the requirements and the canonical interface/state/behavior "
                "models. If structured artifacts omit behavior that is explicit in REQ/API, implement the "
                "REQ/API behavior. If structured artifacts conflict with REQ/API, follow REQ/API and record "
                "the conflict in reasoning or self_check.\n"
                "- Before writing each public function, perform REQ reconciliation: locate the declaration "
                "in the frozen API, locate its function contract in the interface model, then scan the "
                "requirements for related testcase IDs, constants, boundary cases, timing formulas, state "
                "updates, and expected observable source evidence. Implement the union of structured "
                "contract and requirements; use requirements to fill gaps in State Modeler/2/3 artifacts.\n"
                "- Also perform device-interface reconciliation for hardware-facing code: "
                "check the supplied Device / Environment Interface for tick source, queue/protocol packet "
                "ownership, sensor/actuator units, adapter/mock boundaries, and externally visible platform "
                "state before choosing internal storage, globals, or adapter behavior.\n"
                "- MANDATORY Interface Modeler-to-REQ backtracking: whenever you read an Interface Modeler function contract "
                "immediately go back to the corresponding `req_anchors` "
                "or `source_requirements` in the Requirements section and re-read the original REQ text "
                "before implementing. Never implement solely from Interface Modeler's paraphrase. If Interface Modeler gives "
                "only a broad testcase range, scan that REQ/testcase region yourself and extract the "
                "precise behavior before writing code.\n"
                "- MANDATORY per-function checklist: before completing each public function, verify "
                "one-by-one that every listed `required_state_updates`, `required_timers`, "
                "`canonical_observables`, `allowed_aliases`, and `required_code_patterns` from its "
                "Interface Modeler contract is either implemented exactly or explicitly reconciled with a stronger "
                "REQ/API rule in self_check. Missing one item is a fidelity failure.\n"
                "- MANDATORY cross-function consistency rule: if one function writes a state/timer/observable "
                "that another function reads, both functions must use the same storage symbol or an explicit "
                "synchronization helper. Do not create near-synonym variables for the same concept unless "
                "the observable map declares a synchronization rule between them.\n"
                "- MANDATORY helper responsibility rule: implementing required behavior in a helper is allowed "
                "only when the public API function still exposes a clear call chain to that helper and preserves "
                "the required branch guards, state updates, timers, and observable synchronization. Do not move "
                "a requirement into an unrelated helper merely to simplify a public function. If a testcase or "
                "REQ anchor names a specific public function, either implement the behavior there or leave "
                "source-visible call-chain evidence from that function to the helper.\n"
                "- MANDATORY observable/alias rule: if Interface Modeler declares a `canonical_observable` with "
                "`must_be_externally_observable=true`, emit/link that canonical symbol or an explicitly "
                "declared adapter symbol and keep it synchronized according to `sync_semantics`. "
                "Do not silently rename it to an internal variable. If Interface Modeler lists allowed aliases, "
                "use those aliases only with explicit synchronization at function entry/exit.\n"
                "- MANDATORY alias synchronization rule: before a function makes a branch decision from any "
                "externally writable observable, synchronize every declared alias for that same concept into "
                "one canonical value. After the function changes the canonical value, update every externally "
                "observable alias before returning. This applies to flags, priority/arbitration state, pending "
                "requests, queue counters, mode/status enums, and packed bitfields. Prefer small explicit "
                "sync helpers when several functions share the same aliases.\n"
                "- MANDATORY timer rule: implement every required timer/sentinel literally enough for "
                "source-contract audits to recognize it. Handle zero-sentinel ticks, start ticks, "
                "timeout comparisons, hysteresis windows, rate gates, and counter resets explicitly.\n"
                "- Prefer explicit, source-contract-friendly code for required_code_patterns and "
                "step_required_code_patterns. Do not replace required formulas/branches with clever "
                "equivalent expressions when tests or audits expect source evidence.\n"
                "- Preserve state-transition and execution order semantics.\n"
                "- Do not invent behavior when both the requirements and structured artifacts are silent; "
                "use conservative defaults only when required for compilation or safe no-op behavior, and "
                "mark the gap in self_check.\n"
                "- The header must be self-contained.\\n"
                + production_rules +
                "- Output pure source text in structured fields, without Markdown fences.\n"
                "- In repair mode, do not repair by regenerating isolated modules. Classify each listed issue "
                "by contract type first: linkage/observable missing, observable synchronization, state update, "
                "timer/tick, priority arbitration, queue/FIFO, FSM transition, REQ behavior, source-contract "
                "pattern, compile error, or test-brittle/ambiguous. Then repair the coherent whole-file code "
                "while preserving unaffected public interfaces and behavior.\n"
                "- In repair mode, the response must still contain a complete header and complete source, "
                "but the edit scope must remain local: modify only the named affected functions and the "
                "minimum shared declarations/call sites required by those functions. Preserve unrelated "
                "function bodies and cross-function behavior from the previous implementation.\n"
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
                "=== Formal Verification Context ===\n{val_report}\n\n"
                "=== Build Profile ===\n{build_profile}\n\n"
                "=== Repair Context ===\n{repair_context}\n\n"
                "=== Repair Error History (avoid repeating these failures) ===\n{repair_history_context}\n\n"
                "=== Targeted Repair Constraints (apply only to the current error) ===\n{targeted_repair_constraints}\n\n"
                "=== Previous Header (repair mode only) ===\n{previous_header}\n\n"
                "=== Previous Implementation (repair mode only) ===\n{previous_code}\n\n"
                "{domain_context}\n"
                "Generate a fresh whole-file implementation when Repair Context is '(none)'. Otherwise repair "
                "the previous whole-file implementation specifically. In both modes, do not generate modules "
                "separately for later stitching; first produce the global blueprint, then produce coherent "
                "header/source fields."
            )),
        ])
        self.output_schema = BoardCodeOutput if target_profile == "stm32_gpio_application" else CodeOutput
        self.structured_llm = llm.with_structured_output(self.output_schema)
        self.chain = self.prompt | self.structured_llm

    @staticmethod
    def _clean_code(code: str) -> str:
        if not code:
            return ""
        code = re.sub(r'^```(?:cpp|c\+\+|c|h|python)?\s*\n?', '', code, flags=re.MULTILINE)
        code = re.sub(r'\n?```\s*$', '', code, flags=re.MULTILINE)
        return code.strip()

    @staticmethod
    def _render_json(value: object) -> str:
        return json.dumps(value or {}, ensure_ascii=False, indent=2)

    def generate(self, state: NewGraphState) -> Dict[str, str]:
        print("\n" + "=" * 60)
        print("--- AGENT 4: CODE GENERATION / REPAIR ---")
        print("=" * 60)

        requirements = state.get("user_requirement", "")
        device_interface = state.get("device_interface", "")
        repair_context = state.get("error_context", "").strip()
        history = state.get("error_history", [])[-state.get("error_notebook_top_k", 8):]
        repair_history_context = self._render_json(history) if history else "(none)"
        targeted_repair_constraints = "(none)"
        if repair_context:
            affected_functions = sorted(
                set(
                    re.findall(
                        r"(?m)^\s*-\s*\[[^\]]+\]\s+([A-Za-z_]\w*)\s*:",
                        repair_context,
                    )
                )
            )
            if affected_functions:
                targeted_repair_constraints = (
                    "This is a targeted repair. Return the complete header and source files, "
                    "but locally regenerate only the affected functions listed below and the "
                    "minimum shared-state declarations/call sites required to make those fixes "
                    "consistent:\n"
                    + "- Affected functions: " + ", ".join(affected_functions) + "\n"
                    + "- Preserve every unaffected function, public declaration, constant, global, "
                    "timer, queue, and algorithm exactly unless the current issue proves it must change.\n"
                    + "- Do not redesign or rewrite unrelated modules.\n"
                    + "- Before returning, compare the previous implementation against the new one "
                    "and verify that no unrelated function was regenerated or behaviorally changed."
                )
        if "istumbledcheck" in repair_context.lower() and "isfreefalling" in repair_context.lower():
            targeted_repair_constraints += (
                "\n"
                "Current error is scoped to function isTumbledCheck only. "
                "Keep the function signature unchanged: bool *isFreeFalling. "
                "Do not generate isFreeFalling = true; write *isFreeFalling = true. "
                "Do not change unrelated functions or interfaces."
            )
        previous_header = state.get("generated_header", "") if repair_context else ""
        previous_code = state.get("generated_code", "") if repair_context else ""

        print(f"  Mode: {'repair' if repair_context else 'initial generation'}")
        print(f"  Requirements: {len(requirements)} chars")
        print(f"  Device interface: {len(device_interface)} chars")
        print(f"  Previous code: {len(previous_code)} chars")

        domain_context = retrieve_domain_context(requirements, "agent_4_code_generator")
        domain_section = ""
        if domain_context:
            domain_section = (
                "=== Retrieved Domain Patterns (non-authoritative) ===\n"
                f"{domain_context}\n"
                "Use only patterns compatible with the declared interface and behavior."
            )

        try:
            print(
                "[HEARTBEAT] Implementation Generator prompt assembled; waiting for model response...",
                flush=True,
            )
            result: CodeOutput = self.chain.invoke({
                "req": requirements or "N/A",
                "device_interface": device_interface or "(not provided)",
                "api_spec": state.get("api_spec", "") or "(not provided)",
                "skeleton": state.get("function_skeleton", "") or "(not provided)",
                "state_model": self._render_json(state.get("state_model", {})),
                "interface_model": self._render_json(state.get("interface_model", {})),
                "behavior_model": self._render_json(state.get("behavior_model", {})),
                "state_uml": state.get("state_diagram", "") or "(not provided)",
                "sequence_uml": state.get("sequence_diagram", "") or "(not provided)",
                "val_report": state.get("validation_report", "") or "(not provided)",
                "build_profile": self._render_json(state.get("build_profile", {})),
                "repair_context": repair_context or "(none)",
                "repair_history_context": repair_history_context,
                "targeted_repair_constraints": targeted_repair_constraints,
                "previous_header": previous_header or "(none)",
                "previous_code": previous_code or "(none)",
                "domain_context": domain_section,
            })
            if not result or not result.generated_code:
                return {
                    "generated_header": previous_header,
                    "generated_code": previous_code,
                    "error_context": "Implementation Generator returned empty production code.",
                }

            header = self._clean_code(result.generated_header)
            code = self._clean_code(result.generated_code)

            if header and "#pragma once" not in header and "#ifndef" not in header:
                header = "#pragma once\n\n" + header
            if code and '#include "6_generated_code.h"' not in code:
                code = '#include "6_generated_code.h"\n\n' + code

            print(f"[SUCCESS] Header: {len(header)} chars")
            print(f"[SUCCESS] Production code: {len(code)} chars")
            return {
                "generated_header": header,
                "generated_code": code,
            }
        except Exception as exc:
            import traceback
            traceback.print_exc()
            return {
                "generated_header": previous_header,
                "generated_code": previous_code,
                "error_context": f"Implementation Generator exception: {exc}",
            }


code_generator_agent = CodeGeneratorAgent()
