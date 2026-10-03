# agents/agent_1_state_modeler.py
from langchain_core.prompts import ChatPromptTemplate
from pydantic import BaseModel, Field, model_validator
from typing import Any, Dict
import re

from runtime import NewGraphState
from runtime import llm
from plantuml_kb.retriever import get_retriever
from context import retrieve_domain_context
from context import retrieve_error_lessons
from design import StateModel
from runtime import coerce_nested_json_fields, with_coercing_structured_output

class StateDiagramOutput(BaseModel):
    reasoning: str = Field(default="", description="CoT reasoning")
    plantuml_code: str = Field(default="", description="PlantUML state diagram code")
    state_model: StateModel = Field(
        default_factory=StateModel,
        description="Machine-readable state variables, state machines, transitions, guards, and action functions",
    )

    @model_validator(mode="before")
    @classmethod
    def _coerce_stringified_nested_json(cls, data: Any) -> Any:
        return coerce_nested_json_fields(data, ("state_model",))

class StateDiagramAgent:
    def __init__(self):
        self.prompt = ChatPromptTemplate.from_messages([
            ("system", (
                "You are a senior embedded firmware architect. Extract rigorous domain state machines from unstructured firmware requirements."

                "#### Embedded State Modeling Principles:"
                "1. **Peripheral / Hardware State Tracking**: Identify states of hardware peripherals (GPIO, TIM, ADC, USART, SPI, I2C, DMA, etc.) relevant to the system. Model their initialization, idle, active, and error states explicitly. Use state notes to document register-level constraints and valid value ranges."
                "2. **Interrupt-Driven Transitions**: Distinguish between polling-based, ISR-triggered, and DMA-completion-driven transitions. Label interrupt sources clearly (e.g., EXTI line, TIMx_IRQHandler, DMAx_Streamx_IRQHandler, UART RX ISR)."
                "3. **Persistent State via Static/Global Variables**: Identify `static` local variables and global `struct` instances that maintain state across function calls — filter histories, integrator accumulators, error terms, circular buffer indices, previous output backups."
                "4. **Hardware Constraint Invariants**: Document valid ranges inside state nodes (e.g., PWM duty range, integral term bounds, sensor reading physical limits, output clamp thresholds)."
                "5. **Non-Exclusive Flow — Branch Convergence (CRITICAL)**: In embedded C code, patterns like decimation/subsampling have a conditional block followed by common post-processing that ALWAYS executes regardless of which branch was taken. The state diagram MUST show both branches CONVERGING into a shared post-processing state. NEVER let branches end at separate terminal states. Always draw: CheckState → (true branch) → MergeState AND CheckState → (false branch) → MergeState, then MergeState → Continue. The common logic (counter increment, interpolation, output clamping) belongs AFTER the merge point, not inside any single branch."

                "#### PlantUML Syntax Rules — ABSOLUTE (any violation = parse error):"
                "1. @startuml MUST be alone on its line."
                "2. FORBIDDEN keywords (never use): `card`, `rectangle`, `actor`, `usecase`, `component`, `interface`, `package`, `cloud`."
                "3. FORBIDDEN characters: `^` (caret), `::` (double colon for nested references)."
                "4. **Every** `state Xxx {{` MUST have a matching `}}` at the same indent level. Every `note right/left of X` that opens a multi-line block MUST have a matching `end note` on its own line — ONE `end note` for EACH `note`."
                "5. **Transition label character ban**: NEVER use `<` or `>` characters in transition labels (after `:`). PlantUML will misinterpret them as HTML. Use WORDS instead: write `count_below_N` not `count < N`, write `speed_exceeds` not `speed >`, write `angle_above` not `angle >`, write `dt_under` not `dt < 2500ms`."
                "6. Each `Source --> Target : label` line must appear exactly ONCE. Never duplicate."
                "7. **CRITICAL: NEVER place `note right ... end note` INSIDE a `state Xxx {{ ... }}` block. PlantUML PARSER FAILS on inline notes inside state braces.** Use `note right of StateName ... end note` OUTSIDE the closing `}}` instead. Example: `state Outer {{ ... }}` on one line, then `note right of Outer ... end note` on the NEXT lines OUTSIDE. ALL notes must live OUTSIDE any state block."
                "8. skinparam attributes use PascalCase: StateFontSize, DefaultFontSize."
                "9. Transition label format: `Source --> Target : short_label`. No formulas, no C code, no `&&` logic in labels. Put details in `note` blocks."
                "10. Every conditional branch (if/else) MUST have BOTH paths drawn explicitly. After a split, both paths MUST reconverge at a merge state."

                "#### CRITICAL — Self-Verification Before Output:"
                "Before writing `plantuml_code`, mentally verify:"
                "  ** Count `state Xxx {{` vs `}}` — must be equal."
                "  ** Count `note right/left` vs `end note` — must be equal."
                "  ** Scan all `--> Xxx :` labels — no `<` and no `>` characters."
                "  ** Scan all lines — no `^`, no `::`, no `card`, no `rectangle`."
                "  ** Every `-->` line is unique — no duplicate source->target pairs."

                "#### Correct vs Wrong Examples:"
                "WRONG: `Check --> DoWork : count < N`       (the `<` will break PlantUML)"
                "RIGHT: `Check --> DoWork : count_below_N`   (use words for comparisons)"
                "WRONG: `state Foo {{ note right ... end note }}`  (note inside braces CRASHES PlantUML)"
                "RIGHT: `state Foo {{ ... }}` then `note right of Foo ... end note` (note OUTSIDE braces)"
                "WRONG: `note right of X ... (no end note)`  (swallows everything after)"
                "RIGHT: `note right of X ... end note`      (always close multi-line notes)"

                "#### CRITICAL — Output Format:"
                "You MUST output the PlantUML code DIRECTLY into the `plantuml_code` field. Do NOT embed it only inside a markdown code block in the `reasoning` field. "
                "You MUST also populate `state_model`; PlantUML is a rendering, while state_model is the authoritative machine-readable result."
                "Every object in state_model.machines[].transitions MUST contain all three required string fields: `source`, `target`, and `event`. "
                "`target` is the destination state name, is NEVER optional, and must exactly match one of that machine's `states`. "
                "Before responding, inspect every transition object and reject your own output if any lacks `target`."
                "State variable contract: every important state_model.variables[] entry must include the exact `source_requirements`, `type_name`, `initial_value` when known, `writers`, `externally_visible`, `allowed_aliases`, and `invariants`. Mark host/log/API/test-observable symbols as externally_visible. Do not hide observable flags inside prose."
                "Transition contract: every state_model transition must include precise `source_requirements`, `guard`, `action_functions`, `required_state_updates`, `required_timers`, and `externally_visible_state_changes` when relevant. Guards/actions must explicitly mention timeout, sentinel tick, flag set/clear, recovery accept/reject, and externally visible state changes when the requirement implies them."

                "#### CRITICAL — Transition Rules:"
                "1. Transition labels MUST be SHORT (ideally 3-15 chars). Use concise word labels: `done`, `triggered`, `timeout`, `count_met`, `count_below`, `safe`, `unsafe`, `command_received`, `motion_complete`."
                "2. NEVER put `<` or `>` in ANY transition label. Use words: `below`, `above`, `under`, `over`, `exceeds`, `less_than`, `greater_than`, `at_least`, `at_most`."
                "3. NEVER put formulas, C expressions, or comparison operators (`&&`, `||`, `==`, `!=`) in transition labels. Put ALL formulas and logic in `note` blocks attached to states. A transition label is just a NAME, not an explanation."
                "4. Every transition MUST have exactly ONE `-->`. Format: `SourceState --> TargetState : label`."
                "5. ALL branch conditions require BOTH paths drawn explicitly. After branching, always reconverge at a merge state."

                "#### CRITICAL — State Structure Guidelines:"
                "Use only ONE level of sub-states per module. Complex operations are atomic states with `note` blocks explaining their logic — do NOT expand inner sub-states beyond one level."
                "Draw flat top-level states first, then add ONE level of sub-states with short transition labels."
                "**MANDATORY: Every `state Xxx {{` MUST have exactly one matching `}}` at the same indent level. Count your braces before outputting.**"

                "#### Workflow:"
                "Phase 1: Identify core entities, operational modes (init/idle/active/fault), and persistent C variables (static locals, global structs) from the requirements."
                "Phase 2: Draw flat top-level states first, then add ONE level of sub-states with short transition labels. Put all register constraints, formulas, invariants inside `note` blocks only."
                "Phase 3: Verify every transition has a short label, no duplicate transitions, no ^, no card, no rectangle, no ::. Put the final PlantUML in the `plantuml_code` field."
            )),
            ("human", "{input}"),
        ])
        self.structured_llm = with_coercing_structured_output(
            llm, StateDiagramOutput, nested_fields=("state_model",)
        )
        self.chain = self.prompt | self.structured_llm

    def generate(self, state: NewGraphState) -> Dict[str, str]:
        print("---AGENT 1: GENERATING STATE DIAGRAM (with CoT, Enhanced UML, RAG)---")
        requirements = state['user_requirement']
        device_interface = state.get("device_interface", "") or "(not provided)"

        # ── RAG: retrieve relevant PlantUML syntax examples ──
        retriever = get_retriever()
        rag_context = retriever.retrieve_for_state_diagram(requirements, top_k=4)
        if rag_context:
            print(f"[RAG] Retrieved {rag_context.count('-- Example')} state-diagram examples")
        else:
            print("[RAG] No examples matched (using base rules only)")

        # ── Domain RAG: retrieve domain-specific state patterns ──
        domain_context = retrieve_domain_context(requirements, "agent_1_state_modeler")
        if domain_context:
            domain_section = (
                "\n\n### Domain-Specific State Patterns (from knowledge base):\n"
                f"{domain_context}\n"
                "Use the above domain patterns as reference for state names, transitions, and operational modes.\n"
            )
        else:
            domain_section = ""
        error_lessons = retrieve_error_lessons(
            requirements + " state diagram canonical model",
            agent="agent_1_state_modeler",
            top_k=int(state.get("error_notebook_top_k", 3)),
        )

        input_text = (
            "Generate a domain state diagram with full UML transition syntax.\n\n"
            "CRITICAL RULES (your output will FAIL if violated):\n"
            "- NO '<' or '>' in any transition label. Use words: 'below' not '<', 'above' not '>'.\n"
            "- NO '^', '::', 'card', 'rectangle' anywhere.\n"
            "- NO formulas or C code in transition labels. Labels are short names only.\n"
            "- EVERY 'note' that spans multiple lines MUST have 'end note'.\n"
            "- EVERY 'state Xxx {' MUST have matching '}'.\n"
            "- NO duplicate Source --> Target transitions.\n"
            "- Notes INSIDE composite state {} are FORBIDDEN. Place ALL notes OUTSIDE using 'note right of StateName'.\n\n"
            "- The authoritative JSON state_model is mandatory: every transition object MUST be exactly populated with source, target, and event strings. "
            "The target value must be a declared state name in its machine; do not use action_functions as a substitute for target.\n\n"
            "- State Modeler scope: model state variables and transitions, not function bodies. However, every externally visible flag/timer/state must appear in state_model.variables with exact source_requirements, type_name, initial_value if known, writers, externally_visible=true, allowed_aliases if safe, and invariants/sentinel meanings.\n"
            "- For every transition, fill guard/action semantics in the JSON: required_state_updates, required_timers, and externally_visible_state_changes. Include timeout rules, sentinel tick behavior, flag set/clear, recovery accept/reject, and externally visible state changes. Keep PlantUML labels short; put details in JSON and notes.\n\n"
            f"{domain_section}"
            f"{rag_context}"
            f"{error_lessons}\n\n"
            f"Requirements:\n{requirements}\n\n"
            f"Device / Environment Interface:\n{device_interface}\n\n"
            "Use the device/environment interface only as environment evidence: hardware, "
            "protocol, RTOS/task/queue/tick, sensor/actuator, mock, and externally visible "
            "I/O boundaries. It may justify platform/environment states and transition events, "
            "but it must not override requirement behavior or invent public software API."
        )
        result: StateDiagramOutput = self.chain.invoke({"input": input_text})
        plantuml = ""
        reasoning = ""
        if result:
            plantuml = result.plantuml_code or ""
            reasoning = result.reasoning or ""
            # Fallback: if plantuml_code is empty, try to extract @startuml...@enduml from reasoning
            if not plantuml and reasoning:
                # First try: extract from markdown code block (```plantuml or ```puml or bare ```)
                m = re.search(r'```(?:plantuml|puml)?\s*\n(@startuml.*?@enduml)\s*\n```', reasoning, re.DOTALL | re.IGNORECASE)
                if not m:
                    # Second try: find ALL @startuml...@enduml blocks and take the LAST complete one
                    all_blocks = re.findall(r'@startuml.*?@enduml', reasoning, re.DOTALL)
                    if all_blocks:
                        plantuml = all_blocks[-1]  # Take the LAST (most refined) block
                        print(f"[WARN] plantuml_code was empty, extracted last of {len(all_blocks)} block(s) from reasoning")
                    else:
                        print("[WARN] plantuml_code was empty, no @startuml...@enduml block found in reasoning")
                else:
                    plantuml = m.group(1)
                    print("[WARN] plantuml_code was empty, extracted from reasoning code block")
        if plantuml:
            print(f"Reasoning: {reasoning[:50]}...")
            cleaned = plantuml

            # 0. Remove XML/HTML tag artifacts only (tags that look like <tagname> or <tagname attr=...>)
            # Must NOT remove content like "<60", "<0", "-->", etc.
            # Must NOT remove PlantUML stereotypes like <<choice>>, <<fork>>
            # So only match tags with alphabetic tag name, not <<...>>
            cleaned = re.sub(r'(?<!<)<(?!<)/?[a-zA-Z][a-zA-Z0-9]*\b[^<>]*?>', '', cleaned)
            cleaned = re.sub(r'```\w*\s*\n?', '', cleaned)

            # 0.5 Strip everything before @startuml and after @enduml
            cleaned = re.sub(r'^.*?@startuml', '@startuml', cleaned, flags=re.DOTALL)
            cleaned = re.sub(r'@enduml.*$', '@enduml', cleaned, flags=re.DOTALL)

            # 0.6 Remove bare English/Chinese monologue lines (lines that are clearly not PlantUML)
            # A valid PlantUML line starts with: @startuml, @enduml, state, note, skinparam, ', ->, -->, [*], hide, show, activate, deactivate, }
            valid_starters = r'^\s*(@startuml|@enduml|state\s|note\s|skinparam|activate|deactivate|\'|->|-->|\[\*\]|hide\s|show\s|title\s|legend|end\s*legend|footer|header|end\s*note|\}|\!|\$)'
            lines = cleaned.split('\n')
            filtered_lines = []
            in_multiline_note = False
            # Prose indicators: lines that look like natural language, not PlantUML
            prose_starters = r'^\s*(The|This|Let|I|We|A\s|An\s|But|So|And|Or|If|When|Each|All|Some|No\s|Not|It\s|That|These|Those|Here|There|Be|To\s|In\s|On\s|At\s|By\s|For|With|From|As\s|Is\s|Was|Are|Were|Has|Have|Had|Can|Could|Will|Would|May|Might|Must|Shall|Should|Note\s|Important|Make|Ensure|Avoid|Keep|Use|Set|Check|Verify|Follow|Remember|Think|Need|Want|Going|About|Also|Actually|First|Second|Third|Finally|Rather|Instead|Because|Since|Therefore|However|Moreover|Furthermore|Regardless|Here|Wait|Okay|Alright|Now|Then|One\s|Two\s|Three\s|Looks|Seems|Maybe|Perhaps|Probably)\s'
            for line in lines:
                stripped = line.strip()
                if not stripped:
                    filtered_lines.append(line)
                    continue
                # Track if we're inside a multi-line note...end note block
                if stripped == 'end note' or stripped.startswith('end note'):
                    in_multiline_note = False
                    filtered_lines.append(line)
                    continue
                if in_multiline_note:
                    filtered_lines.append(line)
                    continue
                if re.match(r'^\s*note\s', line):
                    in_multiline_note = True
                    filtered_lines.append(line)
                    continue
                # Skip lines that look like English/Chinese prose
                if re.match(valid_starters, line):
                    filtered_lines.append(line)
                elif re.match(r'^\s*[A-Za-z\u4e00-\u9fff]', line) and len(stripped) > 3:
                    # Check if it looks like prose (has spaces and no arrow/newstate keywords)
                    if re.search(r'[\u4e00-\u9fff]', stripped) or re.match(prose_starters, stripped):
                        continue  # Skip prose lines
                    # Also skip lines that look like bullet points or numbered lists
                    if re.match(r'^\s*[-*\d]+[\.\)]\s', stripped):
                        continue
                    filtered_lines.append(line)
                elif re.match(r'^\s*[-*\d]+[\.\)]\s', stripped):
                    continue  # Skip bullet/numbered list items that didn't match [A-Za-z]
                else:
                    filtered_lines.append(line)
            cleaned = '\n'.join(filtered_lines)

            # 1. Strip text after @startuml
            cleaned = re.sub(r'@startuml\s+.+$', '@startuml', cleaned, flags=re.MULTILINE)

            # 2. Fix lowercase skinparam
            cleaned = re.sub(r'skinparam\s+stateFontSize', 'skinparam StateFontSize', cleaned)
            cleaned = re.sub(r'skinparam\s+defaultFontSize', 'skinparam DefaultFontSize', cleaned)

            # 3. Remove card lines, and convert rectangle lines to state to prevent type misidentification
            cleaned = re.sub(r'^\s*card\s+".*$', '', cleaned, flags=re.MULTILINE)
            cleaned = re.sub(
                r'^\s*rectangle\s+"([^"]+)"\s+as\s+(\w+)',
                r'state "\1" as \2',
                cleaned,
                flags=re.MULTILINE
            )

            # 4. Fix :: references in note attachments
            cleaned = re.sub(
                r'note\s+(right|left|top|bottom)\s+of\s+\w+::(\w+)',
                r'note \1 of \2', cleaned
            )

            # 5. Fix ^ character in transition labels (replace ^ StateName with -> StateName)
            cleaned = re.sub(r'(\])\s*\^\s*(\w+)', r'\1\n\2', cleaned)

            # 5.5 Detect and fix garbled transitions (multiple --> or chained targets on one line)
            lines = cleaned.split('\n')
            fixed_lines = []
            for line in lines:
                stripped = line.strip()
                # Count occurrences of --> in the line (not inside notes)
                if 'note' not in stripped.lower() and stripped.count('-->') > 1:
                    # Split at each --> and reconstruct as separate transitions
                    parts = re.split(r'\s*-->\s*', stripped)
                    if len(parts) >= 2:
                        print(f"[WARN] Fixed garbled transition: {stripped[:80]}...")
                        # Keep only the first two parts (source --> target : label)
                        source = parts[0].strip()
                        target_and_rest = parts[1].split(':', 1)
                        target = target_and_rest[0].strip()
                        label = target_and_rest[1].strip() if len(target_and_rest) > 1 else ''
                        if label:
                            fixed_lines.append(f"{source} --> {target} : {label}")
                        else:
                            fixed_lines.append(f"{source} --> {target}")
                        continue
                # 5.6 Fix label containing "StateName : rest" pattern (leaked next transition)
                m = re.match(r'^(\s*)(\w+)\s*-->\s*(\w+)\s*:\s*(.+)$', line)
                if m and 'note' not in line.lower():
                    indent, src, tgt, label = m.group(1), m.group(2), m.group(3), m.group(4)
                    # Check if label contains a pattern like "word word : more"
                    leaked = re.match(r'^(.+?)\s+(\w+)\s*:\s*(.+)$', label)
                    if leaked:
                        prefix, leaked_state, rest = leaked.group(1), leaked.group(2), leaked.group(3)
                        # Only fix if the leaked word looks like a state name (starts with uppercase or known prefix)
                        if re.match(r'^[A-Z][A-Za-z_]*$', leaked_state) and len(leaked_state) > 2:
                            print(f"[WARN] Fixed leaked state in label: {stripped[:80]}...")
                            fixed_lines.append(f"{indent}{src} --> {tgt} : {prefix.strip()}")
                            fixed_lines.append(f"{indent}{tgt} --> {leaked_state} : {rest.strip()}")
                            continue
                fixed_lines.append(line)
            cleaned = '\n'.join(fixed_lines)

            # 6. Remove duplicate consecutive transitions (keep first occurrence)
            lines = cleaned.split('\n')
            result_lines = []
            prev_stripped = None
            for line in lines:
                stripped = line.strip()
                # Check if this is a transition line (contains -->)
                if '-->' in stripped:
                    if stripped == prev_stripped:
                        continue  # skip duplicate
                    prev_stripped = stripped
                else:
                    prev_stripped = None
                result_lines.append(line)
            cleaned = '\n'.join(result_lines)

            # 7. Keep <<choice>> and other stereotypes (valid PlantUML syntax)

            # 8. Fix composite state notes that self-reference inside their own braces
            lines = cleaned.split('\n')
            state_stack = []
            for i, line in enumerate(lines):
                # Detect entry into a composite state
                m_start = re.search(r'state\s+(?:"[^"]+"\s+as\s+)?(\w+)\s*\{', line)
                if m_start:
                    state_stack.append(m_start.group(1))
                    continue
                
                # Detect exit from a composite state
                if '}' in line:
                    if state_stack:
                        state_stack.pop()
                    continue
                
                # Inside a composite state, shorten a note targeting the current state to note <dir>
                if state_stack:
                    current_composite = state_stack[-1]
                    lines[i] = re.sub(
                        r'note\s+(right|left|top|bottom)\s+of\s+' + re.escape(current_composite),
                        r'note \1',
                        lines[i]
                    )
            cleaned = '\n'.join(lines)

            # 9. Auto-close unclosed multi-line notes
            # If a "note right/left/top/bottom of X" opens a multi-line note (no : on same line)
            # but end note is missing before the next transition/state/}, insert it.
            lines = cleaned.split('\n')
            fixed_lines = []
            in_note = False
            note_start_idx = -1
            for i, line in enumerate(lines):
                stripped = line.strip()
                # Detect opening of multi-line note (note right/left/top/bottom of X, no colon on same line)
                if re.match(r'^\s*note\s+(right|left|top|bottom)\s+of\s+\w+', stripped) and ':' not in stripped:
                    in_note = True
                    note_start_idx = i
                    fixed_lines.append(line)
                    continue
                if in_note and stripped == 'end note':
                    in_note = False
                    fixed_lines.append(line)
                    continue
                # Check if we're in an unclosed note and a clear transition/state is coming
                # Only trigger on unambiguous transition or state lines, NOT on lines starting with [ or (
                if in_note and (re.match(r'^\s*\w+\s*-->', stripped) or re.match(r'^\s*state\s+', stripped)):
                    # Look ahead up to 5 lines to see if end note already exists
                    has_end_note = False
                    for j in range(i+1, min(i+6, len(lines))):
                        if lines[j].strip() == 'end note':
                            has_end_note = True
                            break
                        if re.match(r'^\s*\w+\s*-->', lines[j].strip()) or re.match(r'^\s*state\s+', lines[j].strip()):
                            break
                    if not has_end_note:
                        fixed_lines.append('end note')
                        in_note = False
                        print("[WARN] Auto-closed unclosed note before line: " + stripped[:60])
                fixed_lines.append(line)
            # If note is still open at end of file
            if in_note:
                fixed_lines.append('end note')
                print("[WARN] Auto-closed unclosed note at end of file")
            cleaned = '\n'.join(fixed_lines)

            # 10. Final structural validation: detect and fix garbled transitions inside notes
            lines = cleaned.split('\n')
            validated_lines = []
            in_note = False
            note_indent = 0
            for i, line in enumerate(lines):
                stripped = line.strip()
                # Track note blocks
                if re.match(r'^\s*note\s+(right|left|top|bottom)', stripped):
                    in_note = True
                    note_indent = len(line) - len(line.lstrip())
                    validated_lines.append(line)
                    continue
                if in_note and stripped == 'end note':
                    in_note = False
                    validated_lines.append(line)
                    continue
                # If inside a note, check for garbled content that looks like a transition
                if in_note and '-->' in stripped:
                    print(f"[WARN] Garbled transition inside note at line {i+1}: {stripped[:80]}")
                    # Try to salvage: extract only the note-like portion before any transition syntax
                    salvaged = re.sub(r'\s*-->.*$', '', stripped)
                    if salvaged.strip():
                        validated_lines.append(' ' * note_indent + '    ' + salvaged.strip())
                    else:
                        validated_lines.append(' ' * note_indent + '    [content cleaned]')
                    continue
                # Check for transitions with invalid source (starts with digit, or source==target)
                m = re.match(r'^(\s*)(\S+)\s*-->\s*(\S+)\s*(:.*)?$', line)
                if m and 'note' not in stripped.lower():
                    indent, src, tgt, label = m.group(1), m.group(2), m.group(3), m.group(4) or ''
                    # Source starts with digit (e.g., "5 MotorExecute") — garbled
                    if re.match(r'^\d', src):
                        print(f"[WARN] Invalid transition source (starts with digit) at line {i+1}: {stripped[:80]}")
                        continue  # Drop this garbled line
                    # Self-transition (likely a parsing artifact)
                    if src == tgt and src not in ('[*]',):
                        print(f"[WARN] Self-transition detected at line {i+1}: {stripped[:80]}")
                        continue
                validated_lines.append(line)
            cleaned = '\n'.join(validated_lines)

            # 11. Escape < and > in transition labels (PlantUML may interpret as HTML/creole)
            lines = cleaned.split('\n')
            for i, line in enumerate(lines):
                stripped = line.strip()
                # Only process transition lines with labels
                if '-->' in stripped and ':' in stripped and not re.match(r'^\s*note\b', stripped, re.IGNORECASE):
                    # Split into arrow part and label part
                    parts = stripped.split(':', 1)
                    if len(parts) == 2:
                        arrow_part = parts[0]
                        label_part = parts[1]
                        # Escape < and > in label but not in arrow (-->)
                        label_part = label_part.replace('<', '&lt;').replace('>', '&gt;')
                        # Preserve indentation
                        indent = line[:len(line) - len(line.lstrip())]
                        lines[i] = indent + arrow_part + ':' + label_part
            cleaned = '\n'.join(lines)

            # 12. Validate state block balance
            state_depth = 0
            for i, line in enumerate(lines):
                stripped = line.strip()
                if re.match(r'^\s*state\s+\w+.*\{', stripped):
                    state_depth += 1
                if stripped == '}':
                    state_depth -= 1
            if state_depth != 0:
                print(f"[WARN] State block imbalance: depth={state_depth} (should be 0)")

            # 13. FORCE-MOVE inline notes OUTSIDE their parent state block (PlantUML crashes on inline notes)
            lines = cleaned.split('\n')
            state_stack = []  # (state_name, indent_level)
            i = 0
            while i < len(lines):
                line = lines[i]
                stripped = line.strip()
                m_state = re.search(r'state\s+(?:".+?"\s+as\s+)?(\w+)\s*\{', stripped)
                if m_state:
                    state_stack.append(m_state.group(1))
                    i += 1
                    continue
                if stripped == '}':
                    if state_stack:
                        state_stack.pop()
                    i += 1
                    continue
                # Detect note inside a state block (not "note right of X")
                if state_stack and re.match(r'^\s*note\s+(right|left|top|bottom)\s*$', stripped):
                    # This is a multi-line note inside a state block — must move it OUTSIDE
                    parent = state_stack[-1]
                    note_lines = [line]
                    i += 1
                    while i < len(lines):
                        note_lines.append(lines[i])
                        if lines[i].strip() == 'end note':
                            break
                        i += 1
                    # Remove this note block from output
                    for j in range(len(lines)):
                        if lines[j:j+len(note_lines)] == note_lines:
                            # Replace with empty lines (or just remove)
                            for k in range(j, j+len(note_lines)):
                                lines[k] = None  # mark for removal
                            # Find the matching '}' for this state
                            depth = 0
                            insert_pos = j
                            for k in range(j, len(lines)):
                                if re.match(r'^(\s*)state\s+\w+.*\{', lines[k].strip() if lines[k] else ''):
                                    depth += 1
                                if lines[k] and lines[k].strip() == '}':
                                    depth -= 1
                                    if depth == 0:
                                        insert_pos = k + 1
                                        break
                            # Convert note to external form
                            indent = ' ' * (len(lines[j]) - len(lines[j].lstrip()))
                            new_note = [f'{indent}note right of {parent}']
                            for nl in note_lines[1:-1]:
                                new_note.append(nl)
                            new_note.append(f'{indent}end note')
                            # Insert after the closing }
                            for item in reversed(new_note):
                                lines.insert(insert_pos, item)
                            print(f"[WARN] Moved inline note out of state '{parent}'")
                            break
                    continue
                i += 1
            lines = [l for l in lines if l is not None]
            cleaned = '\n'.join(lines)

            # Repair an omitted composite-state opener when the provider
            # emitted a transition body followed by ``}`` and an external note.
            # The note target gives us the authoritative state name.
            lines = cleaned.split('\n')
            depth = 0
            i = 0
            while i < len(lines):
                stripped = lines[i].strip()
                if re.match(r'^state\s+.+\{\s*$', stripped):
                    depth += 1
                elif stripped == '}':
                    if depth > 0:
                        depth -= 1
                    else:
                        note_target = None
                        for lookahead in range(i + 1, min(i + 4, len(lines))):
                            match = re.match(
                                r'^\s*note\s+(?:right|left|top|bottom)\s+of\s+(\w+)',
                                lines[lookahead],
                            )
                            if match:
                                note_target = match.group(1)
                                break
                        if note_target:
                            body_start = i
                            while body_start > 0:
                                previous = lines[body_start - 1].strip()
                                if not previous or previous in {'end note', '}'}:
                                    break
                                body_start -= 1
                            lines.insert(body_start, f'state {note_target} {{')
                            depth += 1
                            i += 1
                            print(
                                f"[WARN] Restored omitted composite-state opener "
                                f"for {note_target}"
                            )
                            depth -= 1
                i += 1

            # Always publish one well-formed PlantUML envelope. Providers
            # occasionally repeat @startuml as the final marker.
            lines = [
                line for line in lines
                if line.strip().lower() not in {'@startuml', '@enduml'}
            ]
            cleaned = '@startuml\n' + '\n'.join(lines).strip() + '\n@enduml'

            model = result.state_model.model_dump() if hasattr(result.state_model, "model_dump") else result.state_model.dict()
            return {"state_diagram": cleaned, "state_model": model}
        return {
            "state_diagram": "@startuml\n' State Modeler Error\n@enduml",
            "state_model": {},
            "error_context": "State Modeler failed to produce a state model.",
        }

state_diagram_agent = StateDiagramAgent()
