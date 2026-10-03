# agents/agent_3_behavior_modeler.py
import json

from langchain_core.prompts import ChatPromptTemplate
from pydantic import AliasChoices, BaseModel, ConfigDict, Field, model_validator
from typing import Any, Dict, List, Literal
import re
from runtime import NewGraphState
from runtime import llm
from plantuml_kb.retriever import get_retriever
from context import retrieve_domain_context
from context import retrieve_error_lessons
from design import BehaviorModel, BehaviorScenario, BehaviorStep
from runtime import coerce_nested_json_fields, with_coercing_structured_output

class SubDiagram(BaseModel):
    name: str = Field(description="Subdiagram name, for example 'borrow_book_flow'")
    plantuml_code: str = Field(description="PlantUML sequence-diagram code for this subflow.")

class SequenceDiagramOutput(BaseModel):
    model_config = ConfigDict(populate_by_name=True)

    behavior_model: BehaviorModel = Field(
        alias="canonical_behaviors",
        validation_alias=AliasChoices("canonical_behaviors", "behavior_model"),
        serialization_alias="behavior_model",
        description="Authoritative machine-readable scenarios, execution modes, ordered calls, and lifecycle functions",
    )
    """Sequence-diagram output with design rationale"""
    reasoning: str = Field(default="", description="Design rationale: identify and split the core workflows; establish function-call order and control flow; annotate state-variable changes.")
    sequence_diagrams: List[SubDiagram] = Field(
        alias="flow_diagrams",
        validation_alias=AliasChoices("flow_diagrams", "sequence_diagrams", "diagrams"),
        serialization_alias="sequence_diagrams",
        description="Non-empty list of core behavior PlantUML sequence fragments",
    )

    @model_validator(mode="before")
    @classmethod
    def _coerce_stringified_nested_json(cls, data: Any) -> Any:
        return coerce_nested_json_fields(
            data, ("canonical_behaviors", "behavior_model")
        )

    @model_validator(mode="after")
    def require_complete_behavior_output(self):
        if not self.sequence_diagrams:
            raise ValueError("Behavior Modeler must emit at least one flow diagram")
        if not self.behavior_model.scenarios:
            raise ValueError("Behavior Modeler must emit at least one canonical behavior scenario")
        return self


class CompactFlow(BaseModel):
    """Flat provider-facing flow; converted deterministically to both artifacts."""

    name: str
    trigger: str
    mode: Literal["synchronous", "asynchronous", "periodic", "interrupt"]
    ordered_functions: List[str] = Field(
        description="Function calls in exact execution order"
    )
    plantuml_code: str = Field(
        description="Pure PlantUML fragment without @startuml or @enduml"
    )
    source_requirements: List[str] = Field(default_factory=list)


class CompactStep(BaseModel):
    function: str = Field(description="Function name matching Interface Modeler/frozen API")
    step_conditions: List[str] = Field(default_factory=list)
    step_state_updates: List[str] = Field(default_factory=list)
    step_req_anchors: List[str] = Field(default_factory=list)
    step_required_code_patterns: List[str] = Field(default_factory=list)


class CompactSemanticFlow(BaseModel):
    """Preferred flow schema carrying per-step semantic contracts."""

    name: str
    trigger: str
    mode: Literal["synchronous", "asynchronous", "periodic", "interrupt"]
    ordered_functions: List[str] = Field(default_factory=list)
    ordered_steps: List[CompactStep] = Field(default_factory=list)
    plantuml_code: str = Field(
        description="Pure PlantUML fragment without @startuml or @enduml"
    )
    source_requirements: List[str] = Field(default_factory=list)

    @model_validator(mode="after")
    def require_steps_or_functions(self):
        if not self.ordered_steps and not self.ordered_functions:
            raise ValueError("Flow must include ordered_steps or ordered_functions")
        return self


class CompactSequenceOutput(BaseModel):
    reasoning: str = Field(default="", description="Concise flow decomposition")
    flows: List[CompactSemanticFlow] = Field(
        min_length=1,
        description="Core firmware flows, each carrying its diagram and ordered calls",
    )

class SequenceDiagramAgent:
    def __init__(self):
        self.prompt = ChatPromptTemplate.from_messages([
            ("system", (
                "You are a professional embedded firmware sequence analyst, specializing in ISR chains, hardware interaction timing, and real-time control loops."

                "#### Embedded Execution Modeling Guidelines:"
                "1. **ISR & Callback Chain Modeling**: Explicitly model the interrupt callback chain. Show: hardware trigger source (timer overflow, EXTI pin, DMA transfer complete) → ISR entry → HAL handler (if applicable) → user callback → application logic. Distinguish ISR context (short, non-blocking) from main-loop context. ISRs must not contain long-blocking operations."
                "2. **Execution Order (CRITICAL)**: Model the exact execution order described in requirements. Conditional blocks (`opt`) for periodic/polled execution go FIRST. Common post-processing (counter increments, interpolation, filtering, output formatting) that runs every cycle MUST be placed OUTSIDE `alt`/`opt` blocks."
                "3. **Sensor/Input Data Must Be Read Before Use**: Any sensor variable or input signal used in computation must have an explicit read step shown before the computation that uses it."
                "4. **State Backup Before Computation**: Inside `opt` branches, BACKUP operations must appear BEFORE compute operations. Correct order: backup old values → compute new values → clamp output → reset counter. Backup messages must appear before computation messages."
                "5. **Integer Computation Precision**: In sequence diagram notes describing formulas, reflect 'multiply before divide' order to preserve precision, e.g., `(A - B) * count / Period` not `(A - B) * (count / Period)`. Critical for MCUs without FPU."

                "#### Common Embedded Interaction Patterns:"
                "- Timer-driven: ISR → sensor read → computation → output update"
                "- External interrupt: GPIO edge → ISR → debounce → state machine update"
                "- Serial communication: UART RX ISR → buffer fill → protocol parse → command dispatch"
                "- DMA: transfer complete ISR → buffer swap → data processing"
                "- Main loop: SysTick counter → task scheduling → periodic execution"

                "#### PlantUML Syntax Constraints — ABSOLUTE (any violation = parse error):"
                ""
                "**A. Participant Rules**"
                "1. All `participant \"Name\" as ALIAS` declarations go ONLY in the FIRST sub-diagram. No participant declarations in sub-diagrams 2, 3, ... N. The first diagram is the 'registry'. Later diagrams just use the aliases."
                ""
                "**B. Message Rules**"
                "2. Every line inside a sub-diagram must be either:"
                "   - `Alias -> Alias : message` (solid arrow)"
                "   - `Alias --> Alias : message` (dashed arrow)"
                "   - `Alias -> Alias` (arrow without label)"
                "   - `note over/left/right of Alias : text` or multi-line `note ... end note`"
                "   - `activate Alias` / `deactivate Alias`"
                "   - `alt ... else ... end` / `opt ... end` / `loop ... end`"
                "   - `group`, `ref`, `break`, `critical` block starters/enders"
                "3. NEVER write bare assignments like `field = value` or `count = 0` as standalone lines. Use `Alias -> Alias : field = value` or `note over Alias : field = value`."
                "4. Arrow direction: Only `->` (solid) or `-->` (dashed). NEVER use `<-`, `<--`, or `<=` in arrow lines. Use `Alias2 --> Alias1` instead of `Alias1 <-- Alias2`."
                ""
                "**C. Forbidden Keywords — use these alternatives**"
                "5. FORBIDDEN: `return` anywhere in a message label or as a standalone line. It is a PlantUML reserved word."
                "   WRONG: `A --> B : return result`     WRONG: `A --> B : return (done)`"
                "   RIGHT: `A --> B : result`           RIGHT: `A --> B : done`"
                "   RIGHT: `A --> B : exit`            RIGHT: `A --> B : completed`"
                "6. FORBIDDEN: `caption` (not supported in sequence diagrams). Use `note over` instead."
                "7. FORBIDDEN: `title` inside sub-diagrams. The framework adds it."
                "8. FORBIDDEN: `== section ==` separator lines. The framework adds them."
                "9. FORBIDDEN: `@startuml` / `@enduml` in sub-diagram code. The framework wraps everything."
                "10. First line of each sub-diagram code must be valid PlantUML (not bare text, not a section name)."
                ""
                "**D. activate/deactivate — SELF-CONTAINED per sub-diagram**"
                "11. Every `activate Alias` MUST have exactly one matching `deactivate Alias` WITHIN THE SAME sub-diagram. You may NOT activate in sub-diagram 2 and deactivate in sub-diagram 3. If a participant is active across multiple sections, deactivate and re-activate at section boundaries."
                "12. Before writing `deactivate Alias`, verify: was `activate Alias` already written in THIS sub-diagram? If not, add the activate first."

                "**E. Self-Verification Checklist (mentally verify before output)**"
                "  ✓ All participant declarations are in the FIRST sub-diagram only."
                "  ✓ No bare `field = value` lines — every line has an arrow or is a note/block keyword."
                "  ✓ No `return` word anywhere in any message label."
                "  ✓ activate/deactivate count matches WITHIN each sub-diagram."
                "  ✓ No `<-` or `<--` arrows — use `-->` with swapped source/target."
                "  ✓ No `title`, `caption`, `== ==`, `@startuml`, `@enduml` in sub-diagram code."

                "#### Task Execution Steps (CoT):"
                "**Phase 1: Flow Decomposition**"
                "- Identify core flows: distinguish ISR, main-loop periodic tasks, event-driven tasks. Label each flow's trigger source and period."
                "- Function mapping: ensure message names fully match Interface Modeler's C function framework."
                "- **Alignment with structured models**: Use the complete State Modeler JSON for state identities, transitions, invariants, and requirement anchors, and the complete Interface Modeler JSON for function contracts, state reads/writes, preconditions, and postconditions. The state diagram and function skeleton are readable renderings. Align main flows, periodic/event triggers, guards, and state updates with these models. Do not introduce conflicting lifecycles or function contracts."
                "**Phase 2: Modeling & Variable Tracking**"
                "- Use `opt` blocks for optionally-executed algorithm updates."
                "- Use PlantUML `note` blocks to describe state variable changes (previous errors, integrals, old outputs). Ensure backup operations precede compute operations."
                "- Behavior Modeler scope: model flow step conditions and behavior, not implementation bodies. For every ordered function that has a branch, guard, safety gate, timer, queue action, or externally visible update, emit an `ordered_steps[]` item with `function`, `step_conditions`, `step_state_updates`, `step_req_anchors`, and `step_required_code_patterns`."
                "- Required step semantics include calibration waits, sensor acquisition, estimator update, commander arbitration, supervisor update, collision avoidance, canFly zeroing, motorsAllowed motor stop, health-test branch, high-level setpoint handoff, motor output ordering, queue dispatch, and tick/rate gates."
                "**Phase 3: Verification & Merge**"
                "- Check flow: after algorithm update completes, is the counter reset? Is output globally smoothed and clamped? Are hardware boundary limits applied?"
                "- Output format: each sub-diagram's `plantuml_code` must be pure PlantUML fragment — no `@startuml/@enduml`, `title`, `caption`, `== ==` separators, or `return` statements."
            )),
            ("human", "{input}"),
        ])
        # Keep the provider-facing schema flat; retain the shared structured
        # output recovery layer for stringified tool-call arguments.
        self.structured_llm = with_coercing_structured_output(
            llm, CompactSequenceOutput, nested_fields=("flows",)
        )
        self.chain = self.prompt | self.structured_llm

    def generate(self, state: NewGraphState) -> Dict[str, str]:
        print("---AGENT 3: GENERATING FUNCTION SEQUENCE DIAGRAMS (with CoT, Split, RAG)---")
        state_diagram = state.get("state_diagram") or "(no state diagram yet)"
        requirements = state['user_requirement']
        device_interface = state.get("device_interface", "") or "(not provided)"

        # ── RAG: retrieve relevant PlantUML sequence-diagram examples ──
        retriever = get_retriever()
        rag_context = retriever.retrieve_for_sequence_diagram(requirements, top_k=4)
        if rag_context:
            print(f"[RAG] Retrieved {rag_context.count('-- Example')} sequence-diagram examples")
        else:
            print("[RAG] No examples matched (using base rules only)")

        # ── Domain RAG: retrieve domain-specific sequence patterns ──
        domain_context = retrieve_domain_context(requirements, "agent_3_behavior_modeler")
        if domain_context:
            domain_section = (
                "\n\n### Domain-Specific Sequence Patterns (from knowledge base):\n"
                f"{domain_context}\n"
                "Use the above patterns as reference for execution order, ISR chains, and data flow.\n"
            )
        else:
            domain_section = ""
        error_lessons = retrieve_error_lessons(
            requirements + " behavior sequence diagram",
            agent="agent_3_behavior_modeler",
            top_k=int(state.get("error_notebook_top_k", 3)),
        )

        input_text = (
            "Generate pure PlantUML sequence diagram fragments list.\n\n"
            "Also populate behavior_model with the same flows. Mark each scenario as synchronous, asynchronous, periodic, or interrupt and list function calls in exact execution order.\n\n"
            "Preferred structured flow schema: use `ordered_steps` for each flow. Each step must include function plus step_conditions, step_state_updates, step_req_anchors, and step_required_code_patterns when behavior is required by REQ/API/tests. You may also include ordered_functions as a compact mirror, but ordered_steps is authoritative for step-level semantics.\n\n"
            "CRITICAL RULES (your output will FAIL if violated):\n"
            "- participant declarations only in the first sub-diagram. Subsequent sub-diagrams just use aliases.\n"
            "- Every line must be valid PlantUML: arrow, note, activate/deactivate, alt/opt/loop/end.\n"
            "- NO bare assignment lines ('field = value'). Put them in notes or attach an arrow.\n"
            "- NO 'return' keyword in any message label. Use 'done'/'exit'/'result' instead.\n"
            "- Every activate/deactivate must be self-paired WITHIN the same sub-diagram.\n"
            "- NO '<-' or '<--' arrows. Only '->' and '-->'.\n"
            "- NO '@startuml/@enduml', 'title', 'caption', '== ==' in sub-diagram code.\n\n"
            f"{domain_section}"
            f"{rag_context}"
            f"{error_lessons}\n\n"
            f"**Requirements document:**\n{requirements}\n\n"
            f"**Device / Environment Interface:**\n{device_interface}\n\n"
            f"**State Modeler Structured State Model (JSON):**\n{json.dumps(state.get('state_model') or {}, ensure_ascii=False, indent=2)}\n\n"
            f"**State Modeler domain state diagram (PlantUML):**\n{state_diagram}\n\n"
            f"**Interface Modeler Structured Interface Model (JSON):**\n{json.dumps(state.get('interface_model') or {}, ensure_ascii=False, indent=2)}\n\n"
            f"**Interface Modeler function skeleton (extracted):**\n{state.get('function_skeleton', 'N/A')}\n\n"
            "Use the device/environment interface to anchor external actors, periodic ticks, "
            "queues, protocol packets, sensors, actuators, and mock/observable boundaries in "
            "the sequence flows. It must not change frozen API calls or override requirement behavior.\n"
            "For stabilizer/control-loop style flows, do not only list call order. Capture step-level guards and effects: calibration wait, canFly=false setpoint zeroing, motorsAllowed=false motor stop, health-test replacement branch, high-level setpoint pending handoff, and exact motor output ordering."
        )
        compact_result: CompactSequenceOutput = self.chain.invoke({"input": input_text})
        if compact_result:
            diagrams = [
                SubDiagram(name=flow.name, plantuml_code=flow.plantuml_code)
                for flow in compact_result.flows
            ]
            scenarios = []
            for flow in compact_result.flows:
                if flow.ordered_steps:
                    steps = []
                    for index, step in enumerate(flow.ordered_steps, start=1):
                        steps.append(
                            BehaviorStep(
                                order=index,
                                function=step.function,
                                condition="; ".join(step.step_conditions),
                                step_conditions=step.step_conditions,
                                state_updates=step.step_state_updates,
                                step_state_updates=step.step_state_updates,
                                step_req_anchors=step.step_req_anchors,
                                step_required_code_patterns=step.step_required_code_patterns,
                            )
                        )
                else:
                    steps = [
                        BehaviorStep(order=index, function=function_name)
                        for index, function_name in enumerate(flow.ordered_functions, start=1)
                    ]
                scenarios.append(
                    BehaviorScenario(
                        name=flow.name,
                        trigger=flow.trigger,
                        mode=flow.mode,
                        steps=steps,
                        source_requirements=flow.source_requirements,
                    )
                )
            result = SequenceDiagramOutput(
                behavior_model=BehaviorModel(scenarios=scenarios),
                sequence_diagrams=diagrams,
                reasoning=compact_result.reasoning,
            )
        else:
            result = None
        
        if result and result.sequence_diagrams:
            print(f"Reasoning: {result.reasoning[:50]}...")

            # Combine PlantUML code and clean invalid syntax in each subdiagram
            combined_uml = "@startuml\n\n"
            seen_participants = set()  # Track declared participant aliases
            # Match participant lines and extract their aliases
            participant_re = re.compile(
                r'^\s*(?:actor|boundary|control|entity|database|collections|queue|participant)\s+'
                r'(?:"[^"]*"|"[^"]*"\s+as\s+(\w+)|\s*(\w+))\s*$',
                re.MULTILINE | re.IGNORECASE
            )

            for sub_diag in result.sequence_diagrams:
                content = sub_diag.plantuml_code
                # Remove diagram delimiters; orchestration supplies the envelope
                content = re.sub(r'(@startuml|@enduml)', '', content, flags=re.IGNORECASE)
                # Remove caption lines
                content = re.sub(r'^\s*caption\s+.*$', '', content, flags=re.MULTILINE | re.IGNORECASE)
                # Remove title lines
                content = re.sub(r'^\s*title\s+.*$', '', content, flags=re.MULTILINE | re.IGNORECASE)
                # Remove section separators inside subdiagrams
                content = re.sub(r'^==\s+.*\s+==\s*$', '', content, flags=re.MULTILINE)
                # Remove standalone return statements
                content = re.sub(r'^\s*return\s+.*$', '', content, flags=re.MULTILINE | re.IGNORECASE)
                # Replace the reserved PlantUML keyword 'return' in message labels
                content = re.sub(
                    r'(\w+\s*--?>\s*\w+\s*:\s*)return\b',
                    r'\1done',
                    content, flags=re.IGNORECASE
                )
                # Remove bare identifiers duplicating a section name
                escaped_name = re.escape(sub_diag.name.strip())
                content = re.sub(rf'^\s*{escaped_name}\s*$', '', content, flags=re.MULTILINE)

                # --- Postprocessing 1: remove duplicate participant declarations ---
                lines = content.split('\n')
                deduped_lines = []
                for line in lines:
                    m = participant_re.match(line)
                    if m:
                        alias = m.group(1) or m.group(2)
                        if alias:
                            alias_lower = alias.lower()
                            if alias_lower in seen_participants:
                                continue  # Skip duplicate declarations
                            seen_participants.add(alias_lower)
                    deduped_lines.append(line)
                content = '\n'.join(deduped_lines)

                # --- Postprocessing 2: repair bare assignments lacking an arrow ---
                lines = content.split('\n')
                fixed_lines = []
                last_arrow_alias = None  # Remember the most recent message sender
                arrow_re = re.compile(r'^\s*(\w+)\s*(->|-->|<-|<--)\s*(\w+)\s*:')
                bare_assign_re = re.compile(r'^\s*(\w[\w.]*\w)\s*=\s*.+$')  # Match "field = value"

                for line in lines:
                    m = arrow_re.match(line)
                    if m:
                        last_arrow_alias = m.group(1)  # Remember the sender
                        fixed_lines.append(line)
                    elif bare_assign_re.match(line) and last_arrow_alias:
                        # Supply a self-message using the previous sender
                        fixed_lines.append(f"{last_arrow_alias} -> {last_arrow_alias} : {line.strip()}")
                    else:
                        fixed_lines.append(line)
                content = '\n'.join(fixed_lines)

                # --- Postprocessing 2b: convert a single-line note followed by bare text into a multiline note ---
                # LLM often writes: note over X : first_line\nbare text\nbare text
                # These bare lines are meant to be part of the note but aren't parseable.
                lines = content.split('\n')
                fixed2_lines = []
                in_conversion = False
                note_indent = ''
                for i, line in enumerate(lines):
                    stripped = line.strip()
                    # Match single-line note: note over/left/right/of X : text
                    m_single_note = re.match(r'^(\s*)note\s+(over|left|right|top|bottom)(\s+of\s+\w+)?\s*:\s*(.*)', stripped)
                    if m_single_note:
                        # Check if next line is bare text (not valid PlantUML)
                        if i + 1 < len(lines):
                            next_stripped = lines[i + 1].strip()
                            if next_stripped and not re.match(
                                r'^\s*('
                                r'(activate|deactivate)\s+\w+|'
                                r'\w+\s*(->|-->)\s*\w+|'
                                r'note\s+|end\s+note|'
                                r'(alt|else|opt|loop|group|ref|break|critical|par|end)\b|'
                                r'@startuml|@enduml|==\s+.*\s+=='
                                r')\s*:?',
                                next_stripped
                            ):
                                # Next line is bare text — convert this to multi-line note
                                in_conversion = True
                                note_indent = m_single_note.group(1)
                                note_scope = m_single_note.group(2)
                                note_of = m_single_note.group(3) or ''
                                first_text = m_single_note.group(4)
                                fixed2_lines.append(f'{note_indent}note {note_scope}{note_of}')
                                if first_text.strip():
                                    fixed2_lines.append(f'{note_indent}    {first_text.strip()}')
                                continue
                        # Normal single-line note
                        fixed2_lines.append(line)
                    elif in_conversion and stripped == '':
                        # End conversion on blank line
                        fixed2_lines.append(f'{note_indent}end note')
                        fixed2_lines.append(line)
                        in_conversion = False
                    elif in_conversion:
                        # Check if this line is now valid PlantUML (end of bare text)
                        if re.match(
                            r'^\s*('
                            r'(activate|deactivate)\s+\w+|'
                            r'\w+\s*(->|-->)\s*\w+|'
                            r'note\s+|end\s+note|'
                            r'(alt|else|opt|loop|group|ref|break|critical|par|end)\b|'
                            r'@startuml|@enduml|==\s+.*\s+=='
                            r')\s*:?',
                            stripped
                        ):
                            # Close the note, then process this line normally
                            fixed2_lines.append(f'{note_indent}end note')
                            in_conversion = False
                            fixed2_lines.append(line)
                        else:
                            # Still bare text — part of note content
                            fixed2_lines.append(f'{note_indent}    {stripped}')
                    else:
                        fixed2_lines.append(line)
                if in_conversion:
                    fixed2_lines.append(f'{note_indent}end note')
                content = '\n'.join(fixed2_lines)

                content = content.strip()
                if not content:
                    continue
                combined_uml += f"== {sub_diag.name} ==\n"
                combined_uml += content + "\n\n"
            combined_uml += "@enduml"

            # --- Postprocessing 3: repair unmatched activation pairs across subdiagrams ---
            lines = combined_uml.split('\n')
            active_set = set()
            fixed_lines = []
            for i, line in enumerate(lines):
                stripped = line.strip()
                # Detect activate Alias
                m_act = re.match(r'^\s*activate\s+(\w+)', stripped)
                if m_act:
                    active_set.add(m_act.group(1))
                    fixed_lines.append(line)
                    continue
                # Detect deactivate Alias
                m_deact = re.match(r'^\s*deactivate\s+(\w+)', stripped)
                if m_deact:
                    alias = m_deact.group(1)
                    if alias in active_set:
                        active_set.discard(alias)
                        fixed_lines.append(line)
                    else:
                        # Insert an activation before an orphan deactivation
                        print(f"[WARN] Orphan deactivate {alias} at combined line {i+1}, inserting activate before it")
                        indent = line[:len(line) - len(line.lstrip())]
                        fixed_lines.append(f"{indent}activate {alias}")
                        fixed_lines.append(line)
                    continue
                fixed_lines.append(line)
            # Close remaining activations
            for alias in active_set:
                print(f"[WARN] Unclosed activate {alias}, appending deactivate at end")
                fixed_lines.append(f"deactivate {alias}")
            combined_uml = '\n'.join(fixed_lines)

            # Close malformed multi-line notes before executable messages or
            # control-flow keywords. Providers often write ``end`` where
            # PlantUML requires ``end note``; consume that token as the note
            # closer instead of letting it corrupt the surrounding alt block.
            lines = combined_uml.split('\n')
            note_fixed = []
            in_note = False
            structural = re.compile(
                r'^\s*(?:'
                r'\w+\s*(?:->|-->)\s*\w+|'
                r'alt\b|else\b|opt\b|loop\b|group\b|par\b|'
                r'critical\b|break\b|ref\b|activate\b|deactivate\b|'
                r'==|@enduml'
                r')'
            )
            for line in lines:
                stripped = line.strip()
                if in_note:
                    if stripped == 'end note':
                        note_fixed.append(line)
                        in_note = False
                        continue
                    if stripped == 'end':
                        note_fixed.append('end note')
                        in_note = False
                        continue
                    if structural.match(stripped):
                        note_fixed.append('end note')
                        in_note = False
                elif stripped == 'end note':
                    print("[WARN] Removed orphan end note")
                    continue
                if re.match(
                    r'^\s*note\s+(?:over|left|right|top|bottom)(?:\s+of)?\s+\w+\s*$',
                    stripped,
                ):
                    in_note = True
                note_fixed.append(line)
            if in_note:
                note_fixed.append('end note')

            # Remove only genuinely orphaned generic ``end`` tokens. Keep a
            # small stack for PlantUML control blocks.
            balanced = []
            block_stack = []
            block_start = re.compile(
                r'^\s*(alt|opt|loop|group|par|critical|break|ref)\b'
            )
            for line in note_fixed:
                match = block_start.match(line)
                if match:
                    block_stack.append(match.group(1))
                    balanced.append(line)
                elif line.strip() == 'end':
                    if block_stack:
                        block_stack.pop()
                        balanced.append(line)
                    else:
                        print("[WARN] Removed orphan sequence block end")
                else:
                    balanced.append(line)
            while block_stack:
                block_name = block_stack.pop()
                print(f"[WARN] Auto-closed unclosed sequence block {block_name}")
                balanced.insert(-1, 'end')
            combined_uml = '\n'.join(balanced)

            model = result.behavior_model.model_dump() if hasattr(result.behavior_model, "model_dump") else result.behavior_model.dict()
            return {"sequence_diagram": combined_uml, "behavior_model": model}

        # Fallback: try to extract sequence diagrams from reasoning field
        if result and result.reasoning and not result.sequence_diagrams:
            m = re.search(r'@startuml.*?@enduml', result.reasoning, re.DOTALL)
            if m:
                print("[WARN] sequence_diagrams was empty, extracted from reasoning field")
                return {"sequence_diagram": m.group(0), "behavior_model": {}, "error_context": "Behavior Modeler omitted behavior_model."}
            print("[WARN] Could not extract sequence_diagram from reasoning")

        return {
            "sequence_diagram": "@startuml\n' Behavior Modeler Error\n@enduml",
            "behavior_model": {},
            "error_context": "Behavior Modeler failed to produce a behavior model.",
        }

sequence_diagram_agent = SequenceDiagramAgent()
