# agents/agent_2_interface_modeler.py
import json

from langchain_core.prompts import ChatPromptTemplate
from pydantic import BaseModel, Field, model_validator
from typing import Any, Dict

from runtime import NewGraphState
from runtime import llm
from context import retrieve_domain_context
from context import retrieve_error_lessons
from design import InterfaceModel
from runtime import coerce_nested_json_fields, with_coercing_structured_output
from _internal.design_validation import _parse_api_function_contract


def render_frozen_c_api(api_spec: str) -> str:
    """Render only the C declarations present in the frozen API document."""
    from _internal.c_contract import render_frozen_c_api as render
    return render(api_spec)


def _legacy_render_frozen_c_api(api_spec: str) -> str:
    """Historical renderer retained for diagnosing old run artifacts only."""
    import re

    lines = api_spec.splitlines()
    extracted = []
    function_start = re.compile(
        r"^\s*(?!typedef\b|extern\b|#)"
        r"(?:const\s+)?(?:struct\s+)?[A-Za-z_]\w*"
        r"(?:\s+[A-Za-z_]\w*)*\s*\**\s+"
        r"[A-Za-z_]\w*\s*\("
    )
    index = 0
    while index < len(lines):
        line = lines[index]
        stripped = line.strip()
        if stripped.startswith("#include ") or stripped.startswith("#define "):
            extracted.append(stripped)
            index += 1
            continue

        is_declaration = (
            stripped.startswith("typedef ")
            or stripped.startswith("extern ")
            or bool(function_start.match(stripped))
        )
        if not is_declaration:
            index += 1
            continue

        block = [line.rstrip()]
        brace_depth = line.count("{") - line.count("}")
        while not (brace_depth == 0 and block[-1].rstrip().endswith(";")):
            index += 1
            if index >= len(lines):
                raise ValueError("Unterminated declaration in frozen API")
            current = lines[index]
            block.append(current.rstrip())
            brace_depth += current.count("{") - current.count("}")
        extracted.extend(block)
        index += 1

    return (
        "#ifndef CRAZYFLIE_H\n"
        "#define CRAZYFLIE_H\n\n"
        "/* Deterministic rendering of the public declarations in RE_api.txt. */\n"
        + "\n".join(extracted)
        + "\n\n#endif /* CRAZYFLIE_H */\n"
    )


def extract_frozen_c_constants(api_spec: str) -> Dict[str, str]:
    """Extract object-like and function-like macros from the frozen C API."""
    import re

    constants: Dict[str, str] = {}
    lines = api_spec.splitlines()
    index = 0
    macro = re.compile(
        r"^\s*#define\s+([A-Za-z_]\w*)"
        r"(?:\s*\([^)]*\))?\s*(.*)$"
    )
    while index < len(lines):
        match = macro.match(lines[index])
        if not match:
            index += 1
            continue

        name = match.group(1)
        value_parts = [match.group(2).rstrip()]
        while value_parts[-1].endswith("\\") and index + 1 < len(lines):
            value_parts[-1] = value_parts[-1][:-1].rstrip()
            index += 1
            value_parts.append(lines[index].strip())
        constants[name] = " ".join(part for part in value_parts if part).strip()
        index += 1
    return constants


def apply_frozen_function_signatures(
    interface_model: Dict[str, Any],
    api_spec: str,
) -> Dict[str, Any]:
    """Make public signature fields match the frozen API exactly.

    The model remains responsible for every behavioral/traceability field.  C
    parameter names, however, are part of the experiment's frozen API and must
    not be freely renamed to ``*_in``/``*_out`` by a model.
    """
    functions = interface_model.get("functions")
    if not isinstance(functions, list):
        return interface_model
    for function in functions:
        if not isinstance(function, dict):
            continue
        name = str(function.get("name") or "")
        frozen = _parse_api_function_contract(api_spec, name)
        if frozen is not None:
            function["return_type"] = frozen["return_type"]
            function["parameters"] = frozen["parameters"]
    return interface_model


class FunctionSkeletonOutput(BaseModel):
    interface_model: InterfaceModel = Field(
        default_factory=InterfaceModel,
        description="Authoritative machine-readable types, constants, functions, parameters, and state access contracts",
    )
    reasoning: str = Field(default="", description="Design rationale: identify all C signatures in the interface document; group functions by responsibility; identify state persisting across calls (static locals and global structs) and plan its fields.")
    function_skeleton: str = Field(default="", description="Standard C header skeleton with include guards, struct definitions and function declarations, without implementations.")

    @model_validator(mode="before")
    @classmethod
    def _coerce_stringified_nested_json(cls, data: Any) -> Any:
        return coerce_nested_json_fields(data, ("interface_model",))

class FuncAlignmentAgent:
    def __init__(self):
        self.prompt = ChatPromptTemplate.from_messages([
            ("system", (
                "You are a professional embedded C firmware architect. Your task is to transform unstructured interface documents into standard, structured C header file (.h) skeletons."

                "#### Embedded C Code Structure Guidelines:"
                "1. **Struct-based State Encapsulation**: Identify all variables that need to persist across function calls (global variables, static locals), group them by module, and encapsulate them as `typedef struct`. Each logical module gets its own state struct with relevant fields."
                "2. **Header File (.h) Standard Format**: Output MUST include:"
                "   - `#ifndef __MODULE_H` / `#define __MODULE_H` include guard"
                "   - Necessary `#include` (e.g., `<stdint.h>`, `<stdbool.h>`, platform headers as needed)"
                "   - State struct definitions with fixed-width types (`int16_t`, `uint32_t`, `float`)"
                "   - Public function declarations with complete parameter lists and return types"
                "   - STRICTLY NO implementation code in .h files (no function bodies, no `{{}}`)"
                "3. **Naming Convention**: Use `Module_Action()` snake_case convention. Names should reflect the domain — derive module prefixes from the API document and state diagram. Use descriptive action verbs (Init, Read, Write, Compute, Parse, Execute, Get, Set, Start, Stop)."
                "4. **Interrupt & Callback Patterns**: If the target platform has a HAL/BSP layer, map callbacks to application functions. If bare-metal, map ISR vectors directly. Always distinguish ISR context (short, non-blocking) from main-loop context."
                "5. **Fixed-Width Type Fidelity**: Use `uint8_t`/`uint16_t`/`uint32_t` for register values and counters, `int16_t` for signed sensor data, `float` for computation. Never implicitly widen or narrow types across function boundaries."

                "#### Alignment with Preceding Design:"
                "The user will provide State Modeler's complete structured state model, its domain state diagram (PlantUML), and original requirements. Use the structured model for state identities, transitions, invariants, and requirement anchors; the diagram is a readable rendering. The generated header should align module/function names and struct fields with these states, transitions, and guard semantics. If conflicts exist with the API document, explain trade-offs in `reasoning`."

                "#### Function Contract Semantics:"
                "The `interface_model.functions` array is the semantic contract consumed by downstream code generation. It must not be a name-only index."
                "For every function contract, populate:"
                "- `purpose`: one precise sentence explaining the behavior required by the requirements/API."
                "- `parameters`: exact API parameter names, types, and units when known."
                "- `return_type`: exact API return type."
                "- `preconditions`: concrete caller obligations, required state, valid ranges, priority rules, or `none` when the API/requirements explicitly impose no precondition."
                "- `postconditions`: concrete effects on return value, output parameters, persistent state, events, timing, or safety behavior."
                "- `reads_state`: persistent state or externally visible state read by the function; leave empty only for truly stateless functions."
                "- `writes_state`: persistent state or externally visible state written by the function; leave empty only for truly read-only/stateless functions."
                "- `error_cases`: null-pointer handling, invalid state, rejected priority, timeout, disabled mode, bounds, or `none` when no error behavior is specified."
                "- `source_requirements`: requirement IDs, test IDs, section names, or exact source anchors from the provided requirements that justify the contract."
                "- `req_anchors`: precise original requirement anchors, preferably `RE_req.txt:<line-range>` or `RE_api.txt:<line-range>` plus a short behavior snippet. Do not use only broad ranges like `TC-020..TC-035` when a specific sentence/bullet supports the function."
                "- `semantic_requirements`: formula-level and branch-level facts extracted from the original REQ/API. Include signs, constants, timeout durations, sentinel values, return semantics, priority comparisons, and edge cases."
                "- `semantic_confidence`: high/medium/low. Use high only when REQ/API/device evidence clearly determines the behavior. Use low when semantics are ambiguous, underspecified, or multiple implementations could satisfy the documents."
                "- `open_questions`: explicit uncertainties Implementation Generator must resolve by rereading req_anchors/API/device interface. Do not hide uncertainty by inventing a fake precise contract."
                "- `required_state_updates`: concrete updates that must happen on each important branch, e.g. copy setpoint, set timestamp, reset integral, set/clear safety flags, update last tick, reset queue counters."
                "- `required_timers`: every tick, timeout, hysteresis, rate gate, timer-start, timer-clear, and sentinel-tick rule required by this function, e.g. `latestLandingTick == 0 -> no landing timeout`, `currentTick - lastStatsTick >= 500 -> compute and clear stats`."
                "- `canonical_observables`: every externally visible state or log symbol the function reads or writes. For each one include `canonical_name`, `type_name` when known, `allowed_aliases`, `access`, `must_be_externally_observable`, `sync_semantics`, and `source_requirements`."
                "- `allowed_aliases`: function-level alias rules such as `commanderSetpoint := activeSetpoint` or `lastUpdateTick := commanderLastUpdate`. Do not allow aliases for public API names unless the alias is explicitly safe and synchronized."
                "- `required_code_patterns`: explicit source-evidence patterns when tests or requirements imply code shape, e.g. `control->yaw = -yawOut`, `systemTick - lastStatsTick >= 500`, `txHead = 0; txTail = 0; txCount = 0`."
                "- `function_responsibility`: when requirements/test anchors name a function, state whether the behavior must be implemented directly in that function or may be delegated to a helper. If delegated, identify the required public-entry-to-helper call chain and the shared state/parameters that carry the behavior."
                "Do not invent public API elements. If a semantic detail is absent from the frozen API and requirements, state that gap in `reasoning` and do not turn it into a fake postcondition."
                "Broad source ranges are not enough. For formulas and stateful firmware logic, extract exact implementation facts: sign conventions, accept/reject branch semantics, timeout state guards, timestamp writes, RTOS/queue initialization observability, watchdog/hysteresis tick variables, and externally writable host symbols."
                "For stateful safety/control/protocol code, never collapse externally visible flags into only an enum unless you also define an explicit canonical observable alias and sync rule. Record exact set/clear behavior for observable flags, pending work, queue counts, state-machine indices, and timer sentinel values."
                "If you cannot determine whether a flag is physically separate or derived from another state, set semantic_confidence='low' and write an open_question such as: `Requirement mentions recovery but does not specify whether the observable fault flag is separate or derived from the mode enum`."

                "#### Comprehensive Semantic Analysis Checklist:"
                "For every module and every function, actively scan the requirements/API/device interface for the following semantic carriers. If any is relevant, represent it explicitly in the function contract; do not bury it in prose:"
                "1. Observable flags: armed/crashed/tumbled/flying/freefall/initialized/calibrated/locked/pending/requested/connected/valid/done/pass/fail."
                "2. Timers and sentinel ticks: current tick, last update tick, start tick, latest tick, zero meaning never-started, timeout durations, hysteresis windows, periodic rate gates, and counter reset moments."
                "3. Priority and arbitration rules: greater/equal/lower priority behavior, rejection behavior, tie behavior, timestamp writes, trajectory cancellation, pending setpoint handoff."
                "4. Queue/ring-buffer state: created/initialized flags, head/tail/count/free capacity, overflow/drop behavior, callback dispatch, nop/null link behavior, blocking vs non-blocking variants."
                "5. State-machine indices and sample counters: motor index, sample count, battery test tick, noise-floor sample count, prop test iteration, per-motor pass/fail bit update."
                "6. Safety gates and branch effects: canFly, motorsAllowed, emergency stop, watchdog warning/timeout, preflight/landing/spinup timeout, crash/tumble/freefall recovery accept/reject."
                "7. Numeric formulas and sign conventions: frame mixing signs, yaw sign flip, pitch gyro negation, unit conversions, clamp ranges, exact constants, divide/multiply ordering."
                "8. Platform/log integration: log structs, external adapters, and mock link/sensor ownership."
                "9. Conditional behavior variants: compile-time flags, feature switches, algorithm variants, and mode guards. If a requirement names a conditional variant, record the guard condition, selected branch behavior, and required formulas/state updates instead of collapsing to only the default path."
                "10. Enum/union/input variants: if a function consumes a typed variant, enum, message kind, measurement type, packet type, or command mode, enumerate every variant that the requirements/API mention and record whether it is handled, ignored, rejected, or delegated."
                "11. Pending/request-triggered state transitions: when a request/pending flag starts work, record both the flag clear and the state transition that begins the work; do not represent it as only a boolean return condition."
                "12. Function responsibility and delegation: if a behavior named for a public function can be implemented in a helper, record the expected call chain and source-visible evidence required for Implementation Generator/test auditors to verify that delegation."
                "When one of these appears, fill the matching fields: canonical_observables, required_timers, required_state_updates, required_code_patterns, allowed_aliases, semantic_confidence, and open_questions."

                "#### Task Execution Steps (CoT):"
                "1. **Reasoning**: Scan the API document, identify all functions → group by logical module (sensor acquisition, control algorithm, driver output, communication, configuration, etc.) → identify state variables that need cross-call persistence → map to struct fields."
                "2. **Generation**: Output a standard C header file skeleton with include guard, struct definitions, function declarations. Declarations only, no implementations."
                "3. **Structured Contract**: Populate interface_model with every type, constant, and function in the header. The header is only a rendering; names and types must match exactly. Function contracts must include semantic fields sufficient for Implementation Generator to implement code without re-deriving behavior from raw prose."
            )),
            ("human", "{input}"),
        ])
        self.structured_llm = with_coercing_structured_output(
            llm, FunctionSkeletonOutput, nested_fields=("interface_model",)
        )
        self.chain = self.prompt | self.structured_llm

    def generate(self, state: NewGraphState) -> Dict[str, str]:
        target_lang = state.get('target_language', 'Python')
        print(f"---AGENT 2: EXTRACTING {target_lang.upper()} FUNCTION SKELETON (with CoT)---")
        state_diagram = state.get("state_diagram") or "(no state diagram yet)"
        user_requirement = state.get("user_requirement") or ""
        device_interface = state.get("device_interface", "") or "(not provided)"

        # ── Domain RAG: retrieve domain-specific API/struct patterns ──
        domain_context = retrieve_domain_context(user_requirement, "agent_2_interface_modeler")
        if domain_context:
            domain_section = (
                "\n\n### Domain-Specific API & Struct Patterns (from knowledge base):\n"
                f"{domain_context}\n"
                "Use the above patterns as reference for module naming, struct design, and function signatures.\n"
            )
        else:
            domain_section = ""
        error_lessons = retrieve_error_lessons(
            user_requirement + " frozen API header interface model",
            agent="agent_2_interface_modeler",
            top_k=int(state.get("error_notebook_top_k", 3)),
        )

        input_text = (
            "Extract a C function skeleton and a semantic function-contract model from the API document aligned with the state diagram.\n\n"
            f"**User Requirements:**\n{user_requirement}\n\n"
            f"**Device / Environment Interface:**\n{device_interface}\n\n"
            f"**State Modeler Structured State Model (JSON):**\n{json.dumps(state.get('state_model') or {}, ensure_ascii=False, indent=2)}\n\n"
            f"**State Modeler Domain State Diagram (PlantUML):**\n{state_diagram}\n\n"
            f"**API Document:**\n{state['api_spec']}\n\n"
            "Device-interface use rule: treat it as environment/platform evidence for "
            "adapter boundaries, units, tick/queue/protocol behavior, hardware I/O, and "
            "platform adapter ownership. The frozen API remains authoritative for "
            "public names, types, constants, declarations, and ordering. Do not add public "
            "API elements merely because hardware or protocol details appear in the device interface.\n"
            f"{domain_section}"
            f"{error_lessons}\n"
            "\nFor interface_model.functions, do not leave semantic fields empty. "
            "Every function needs purpose, preconditions, postconditions, error_cases, "
            "source_requirements, req_anchors, semantic_requirements, required_state_updates, "
            "required_timers, canonical_observables, allowed_aliases, required_code_patterns "
            "when source evidence matters, and required_code_patterns when source evidence matters. Use reads_state/writes_state when the function "
            "state. Use reads_state/writes_state when the function "
            "observes or mutates persistent state.\n"
            "Also set semantic_confidence and open_questions. It is acceptable to mark a "
            "contract as low confidence; it is not acceptable to silently invent precise "
            "state/flag/timer semantics when the evidence is ambiguous.\n"
            "Alias discipline: if a function uses an internal name that differs from a "
            "test/log/API-visible concept, record the mapping in canonical_observables.allowed_aliases "
            "or function-level allowed_aliases, and state the synchronization rule. If no safe alias "
            "exists, require Implementation Generator to use the canonical observable name exactly.\n"
            "Before finalizing the interface_model, run the comprehensive semantic checklist: "
            "flags, timers/sentinel ticks, priority/arbitration, queues, sample counters, "
            "safety gates, numeric formulas/sign conventions, and host/log observability. "
            "For each relevant item, ensure it is represented as a structured field rather "
            "than only in purpose/reasoning prose.\n"
        )
        
        result: FunctionSkeletonOutput = self.chain.invoke({
            "target_language": target_lang,
            "input": input_text
        })
        
        if result and result.function_skeleton:
            print(f"Reasoning: {result.reasoning[:50]}...")
            model = result.interface_model.model_dump() if hasattr(result.interface_model, "model_dump") else result.interface_model.dict()
            skeleton = result.function_skeleton
            if target_lang.upper() == "C":
                skeleton = render_frozen_c_api(state["api_spec"])
                model = apply_frozen_function_signatures(
                    model, state["api_spec"]
                )
                # Macro names are a frozen API contract. Providers frequently
                # omit function-like macros such as RATE_DO_EXECUTE from the
                # structured constants map, so derive this field directly.
                model["constants"] = extract_frozen_c_constants(
                    state["api_spec"]
                )
            return {"function_skeleton": skeleton, "interface_model": model}
        # Fallback: try to extract code from reasoning field
        if result and result.reasoning and not result.function_skeleton:
            import re
            m = re.search(r'```(?:c|h)?\s*\n(.*?)```', result.reasoning, re.DOTALL)
            if m:
                print("[WARN] function_skeleton was empty, extracted from reasoning code block")
                return {"function_skeleton": m.group(1).strip(), "interface_model": {}, "error_context": "Interface Modeler omitted interface_model."}
            # Try to find C header/skeleton patterns directly
            m = re.search(r'(#ifndef\s+\w+.*?#endif)', result.reasoning, re.DOTALL)
            if m:
                print("[WARN] function_skeleton was empty, extracted header from reasoning")
                return {"function_skeleton": m.group(1).strip(), "interface_model": {}, "error_context": "Interface Modeler omitted interface_model."}
            print("[WARN] Could not extract function_skeleton from reasoning")

        return {
            "function_skeleton": "// Error: Interface Modeler Failed",
            "interface_model": {},
            "error_context": "Interface Modeler failed to produce an interface model.",
        }


func_alignment_agent = FuncAlignmentAgent()
