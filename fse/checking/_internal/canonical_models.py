"""Canonical, machine-verifiable design models shared by Agents 1-3."""

from __future__ import annotations

import json
import re
from typing import Any, Dict, List, Literal, Optional, Set

from pydantic import AliasChoices, BaseModel, ConfigDict, Field, field_validator, model_validator


class JsonObjectModel(BaseModel):
    """Accept provider tool output as either an object or a JSON object string."""

    @model_validator(mode="before")
    @classmethod
    def parse_json_object_string(cls, value: Any) -> Any:
        if not isinstance(value, str):
            return value
        try:
            parsed = json.loads(value)
        except json.JSONDecodeError:
            parsed = json.loads(value, strict=False)
        if not isinstance(parsed, dict):
            raise ValueError(f"{cls.__name__} JSON must decode to an object")
        return parsed


def _normalize_string_list(value: Any) -> Any:
    """Normalize provider variants to a canonical list of strings.

    Providers often return structured objects for fields that are intentionally
    modeled as concise string notes, for example:
    {"symbol": "motorPass", "rule": "set bit on pass"}.
    Treat those objects as useful text instead of failing schema validation.
    """
    if isinstance(value, str):
        return [value]
    if isinstance(value, dict):
        return [json.dumps(value, ensure_ascii=False, sort_keys=True)]
    if isinstance(value, list):
        normalized = []
        for item in value:
            if isinstance(item, str):
                normalized.append(item)
            elif isinstance(item, dict):
                normalized.append(json.dumps(item, ensure_ascii=False, sort_keys=True))
            elif item is not None:
                normalized.append(str(item))
        return normalized
    return value


def _normalize_optional_string(value: Any) -> Any:
    """Normalize provider variants to a scalar string when the field is textual."""
    if value is None or isinstance(value, str):
        return value
    if isinstance(value, (dict, list)):
        return json.dumps(value, ensure_ascii=False, sort_keys=True)
    return str(value)


class StateVariable(BaseModel):
    name: str
    type_name: str
    initial_value: Optional[str] = None
    writers: List[str] = Field(default_factory=list)
    allowed_aliases: List[str] = Field(
        default_factory=list,
        description="Equivalent implementation-local names/expressions; leave empty when exact symbol is required.",
    )
    externally_visible: bool = Field(
        default=False,
        description="True when tests, logs, host adapters, or platform code observe this state directly.",
    )
    invariants: List[str] = Field(
        default_factory=list,
        description="Required value ranges, sentinel meanings, or consistency constraints.",
    )
    source_requirements: List[str] = Field(default_factory=list)

    _normalize_allowed_aliases = field_validator(
        "allowed_aliases", mode="before"
    )(_normalize_string_list)
    _normalize_invariants = field_validator(
        "invariants", mode="before"
    )(_normalize_string_list)
    _normalize_initial_value = field_validator(
        "initial_value", mode="before"
    )(_normalize_optional_string)
    _normalize_source_requirements = field_validator(
        "source_requirements", mode="before"
    )(_normalize_string_list)


class StateTransition(BaseModel):
    model_config = ConfigDict(populate_by_name=True)

    source: str
    target: str = Field(
        alias="destination",
        validation_alias=AliasChoices("destination", "target", "to"),
        serialization_alias="target",
        description="Destination state name",
    )
    event: str
    guard: str = ""
    action_functions: List[str] = Field(default_factory=list)
    required_state_updates: List[str] = Field(
        default_factory=list,
        description="Concrete flag/timer/state/output updates performed by this transition.",
    )
    required_timers: List[str] = Field(
        default_factory=list,
        description="Timeout, tick, hysteresis, rate gate, or sentinel-tick rule controlling this transition.",
    )
    externally_visible_state_changes: List[str] = Field(
        default_factory=list,
        description="Observable state/log/host-symbol changes that must be visible after this transition.",
    )
    source_requirements: List[str] = Field(default_factory=list)

    _normalize_required_state_updates = field_validator(
        "required_state_updates", mode="before"
    )(_normalize_string_list)
    _normalize_required_timers = field_validator(
        "required_timers", mode="before"
    )(_normalize_string_list)
    _normalize_externally_visible_state_changes = field_validator(
        "externally_visible_state_changes", mode="before"
    )(_normalize_string_list)
    _normalize_source_requirements = field_validator(
        "source_requirements", mode="before"
    )(_normalize_string_list)


class StateMachine(BaseModel):
    name: str
    initial_state: str
    states: List[str]
    transitions: List[StateTransition] = Field(default_factory=list)


class StateModel(JsonObjectModel):
    variables: List[StateVariable] = Field(default_factory=list)
    machines: List[StateMachine] = Field(default_factory=list)


class DataField(BaseModel):
    name: str
    type_name: str
    description: str = ""


class TypeContract(BaseModel):
    name: str
    kind: Literal["enum", "struct", "alias"]
    fields: List[DataField] = Field(default_factory=list)
    enum_values: List[str] = Field(default_factory=list)
    source_requirements: List[str] = Field(default_factory=list)

    _normalize_source_requirements = field_validator(
        "source_requirements", mode="before"
    )(_normalize_string_list)


class FunctionParameter(BaseModel):
    name: str
    type_name: str
    unit: str = ""

    _normalize_unit = field_validator("unit", mode="before")(_normalize_optional_string)


class CanonicalObservable(BaseModel):
    model_config = ConfigDict(populate_by_name=True)

    canonical_name: str = Field(
        description=(
            "Canonical externally visible state/variable name required by REQ, "
            "platform integration, logs, or the frozen API."
        )
    )
    type_name: str = Field(default="", description="C type when known.")
    allowed_aliases: List[str] = Field(
        default_factory=list,
        description=(
            "Alternative implementation names or expressions that are semantically "
            "equivalent to the canonical observable, e.g. activeSetpoint for "
            "commanderSetpoint, or supervisorState == supervisorStateCrashed "
            "for isCrashed. Leave empty when the symbol must not be renamed."
        ),
    )
    access: Literal["read", "write", "read_write"] = "read_write"
    must_be_externally_observable: bool = Field(
        default=False,
        description=(
            "True when Implementation Generator must emit/link an externally visible symbol or "
            "adapter accessor for tests/logs/platform integration."
        ),
    )
    sync_semantics: List[str] = Field(
        default_factory=list,
        description="Rules for syncing the canonical observable with internal state.",
    )
    source_requirements: List[str] = Field(default_factory=list)

    _normalize_allowed_aliases = field_validator(
        "allowed_aliases", mode="before"
    )(_normalize_string_list)
    _normalize_sync_semantics = field_validator(
        "sync_semantics", mode="before"
    )(_normalize_string_list)
    _normalize_source_requirements = field_validator(
        "source_requirements", mode="before"
    )(_normalize_string_list)


class FunctionContract(BaseModel):
    model_config = ConfigDict(populate_by_name=True)

    name: str
    purpose: str = Field(
        default="",
        description="One-sentence behavioral purpose traceable to the requirements/API",
    )
    # The frozen-API alignment step fills public signatures deterministically.
    # A missing model-provided value should therefore remain diagnosable rather
    # than discard an otherwise useful semantic contract before that step.
    return_type: str = ""
    parameters: List[FunctionParameter] = Field(default_factory=list)
    visibility: Literal["public", "private"] = "public"
    preconditions: List[str] = Field(default_factory=list)
    postconditions: List[str] = Field(default_factory=list)
    reads_state: List[str] = Field(
        default_factory=list,
        validation_alias=AliasChoices("reads_state", "state_reads", "read_state"),
    )
    writes_state: List[str] = Field(
        default_factory=list,
        validation_alias=AliasChoices("writes_state", "state_writes", "write_state"),
    )
    error_cases: List[str] = Field(default_factory=list)
    req_anchors: List[str] = Field(
        default_factory=list,
        description=(
            "Precise anchors into the original requirements, ideally file:line-range "
            "plus a short quoted/paraphrased behavior snippet. More precise than broad "
            "test ranges."
        ),
    )
    semantic_requirements: List[str] = Field(
        default_factory=list,
        description=(
            "Implementation-level behavioral facts extracted from REQ/API: formulas, "
            "state-transition guards, timing rules, boundary cases, and required return semantics."
        ),
    )
    semantic_confidence: Literal["high", "medium", "low"] = Field(
        default="medium",
        description=(
            "Confidence that this contract fully captures the original requirement semantics. "
            "Use low when REQ/API/device evidence is ambiguous or when multiple designs are plausible."
        ),
    )
    open_questions: List[str] = Field(
        default_factory=list,
        description=(
            "Explicit uncertainties, ambiguities, missing source evidence, or design choices "
            "Implementation Generator must resolve by rereading req_anchors/API/device interface."
        ),
    )
    required_state_updates: List[str] = Field(
        default_factory=list,
        description=(
            "Concrete state/output updates that must occur on accepted/error/timeout branches."
        ),
    )
    required_timers: List[str] = Field(
        default_factory=list,
        description=(
            "Timer, tick, timeout, sentinel, hysteresis, or rate-gating variables/rules "
            "required by the function, including zero-sentinel behavior such as "
            "`latestLandingTick == 0 -> no timeout`."
        ),
    )
    canonical_observables: List[CanonicalObservable] = Field(
        default_factory=list,
        description=(
            "Canonical externally visible state variables/log symbols/test-observable "
            "symbols this function reads, writes, or must keep synchronized."
        ),
    )
    allowed_aliases: List[str] = Field(
        default_factory=list,
        description=(
            "Function-level alias rules in the form `canonical := alias1 | alias2`. "
            "Use only for implementation-local alternatives; public API names and "
            "must-be-observable symbols must not be renamed unless explicitly listed."
        ),
    )
    required_code_patterns: List[str] = Field(
        default_factory=list,
        description=(
            "Source-evidence patterns or explicit code-shape requirements that downstream "
            "tests or audits expect to see when they are required by REQ/API."
        ),
    )
    source_requirements: List[str] = Field(
        default_factory=list,
        validation_alias=AliasChoices(
            "source_requirements",
            "source_req_ids",
            "source_requirement_ids",
        ),
    )

    _normalize_source_requirements = field_validator(
        "source_requirements", mode="before"
    )(_normalize_string_list)
    _normalize_preconditions = field_validator(
        "preconditions", mode="before"
    )(_normalize_string_list)
    _normalize_postconditions = field_validator(
        "postconditions", mode="before"
    )(_normalize_string_list)
    _normalize_reads_state = field_validator(
        "reads_state", mode="before"
    )(_normalize_string_list)
    _normalize_writes_state = field_validator(
        "writes_state", mode="before"
    )(_normalize_string_list)
    _normalize_error_cases = field_validator(
        "error_cases", mode="before"
    )(_normalize_string_list)
    _normalize_req_anchors = field_validator(
        "req_anchors", mode="before"
    )(_normalize_string_list)
    _normalize_semantic_requirements = field_validator(
        "semantic_requirements", mode="before"
    )(_normalize_string_list)
    _normalize_open_questions = field_validator(
        "open_questions", mode="before"
    )(_normalize_string_list)
    _normalize_required_state_updates = field_validator(
        "required_state_updates", mode="before"
    )(_normalize_string_list)
    _normalize_required_timers = field_validator(
        "required_timers", mode="before"
    )(_normalize_string_list)
    _normalize_allowed_aliases = field_validator(
        "allowed_aliases", mode="before"
    )(_normalize_string_list)
    _normalize_required_code_patterns = field_validator(
        "required_code_patterns", mode="before"
    )(_normalize_string_list)

    @field_validator("canonical_observables", mode="before")
    @classmethod
    def _normalize_canonical_observables(cls, value: Any) -> Any:
        """Recover providers that wrap this list in repeated field objects."""
        while isinstance(value, dict) and "canonical_observables" in value:
            value = value["canonical_observables"]
        if isinstance(value, dict):
            return [value]
        return value
class InterfaceModel(JsonObjectModel):
    types: List[TypeContract] = Field(default_factory=list)
    functions: List[FunctionContract] = Field(default_factory=list)
    constants: Dict[str, str] = Field(default_factory=dict)

    @field_validator("constants", mode="before")
    @classmethod
    def _normalize_constant_values(cls, value: Any) -> Any:
        if not isinstance(value, dict):
            return value
        return {str(name): str(item) for name, item in value.items()}


class BehaviorStep(BaseModel):
    order: int
    function: str
    condition: str = ""
    step_conditions: List[str] = Field(
        default_factory=list,
        description="Branch/guard conditions that must hold for this step to execute.",
    )
    state_updates: List[str] = Field(default_factory=list)
    step_state_updates: List[str] = Field(
        default_factory=list,
        description="Concrete state/output updates caused by this step.",
    )
    step_req_anchors: List[str] = Field(
        default_factory=list,
        description="Precise original requirement anchors justifying this step.",
    )
    step_required_code_patterns: List[str] = Field(
        default_factory=list,
        description="Source-evidence/code-shape requirements that Implementation Generator must preserve for this step.",
    )

    _normalize_step_conditions = field_validator(
        "step_conditions", mode="before"
    )(_normalize_string_list)
    _normalize_state_updates = field_validator(
        "state_updates", mode="before"
    )(_normalize_string_list)
    _normalize_step_state_updates = field_validator(
        "step_state_updates", mode="before"
    )(_normalize_string_list)
    _normalize_step_req_anchors = field_validator(
        "step_req_anchors", mode="before"
    )(_normalize_string_list)
    _normalize_step_required_code_patterns = field_validator(
        "step_required_code_patterns", mode="before"
    )(_normalize_string_list)


class BehaviorScenario(BaseModel):
    name: str
    trigger: str
    mode: Literal["synchronous", "asynchronous", "periodic", "interrupt"]
    steps: List[BehaviorStep] = Field(default_factory=list)
    completion_function: str = ""
    abort_function: str = ""
    source_requirements: List[str] = Field(default_factory=list)

    _normalize_source_requirements = field_validator(
        "source_requirements", mode="before"
    )(_normalize_string_list)


class BehaviorModel(JsonObjectModel):
    scenarios: List[BehaviorScenario] = Field(default_factory=list)


class ModelIssue(BaseModel):
    severity: Literal["HIGH", "MEDIUM", "LOW"]
    model: str
    symbol: str
    message: str


class ModelValidationReport(BaseModel):
    valid: bool
    issues: List[ModelIssue] = Field(default_factory=list)


def _duplicates(values: List[str]) -> Set[str]:
    seen: Set[str] = set()
    duplicates: Set[str] = set()
    for value in values:
        if value in seen:
            duplicates.add(value)
        seen.add(value)
    return duplicates


_CALLABLE_REFERENCE = re.compile(r"^\s*([A-Za-z_][A-Za-z0-9_]*)\s*(?:\(|$)")
_NON_FUNCTION_MARKERS = {
    "void",
    "loop",
    "return",
    "returns",
    "complete",
    "completed",
    "none",
}


def _callable_name(reference: str) -> Optional[str]:
    """Extract a C symbol from an LLM sequence-step reference.

    Behavior models commonly contain call notation (``foo(arg)``) while the
    interface model stores only ``foo``. Completion fields may instead contain
    prose such as ``returns int16_t`` or control markers such as ``loop``; these
    are not function references and must not block code generation.
    """
    match = _CALLABLE_REFERENCE.match(reference or "")
    if not match:
        return None
    name = match.group(1)
    if name.lower() in _NON_FUNCTION_MARKERS:
        return None
    return name


def validate_models(
    state_model: StateModel,
    interface_model: InterfaceModel,
    behavior_model: BehaviorModel,
) -> ModelValidationReport:
    issues: List[ModelIssue] = []

    if not state_model.machines:
        issues.append(ModelIssue(severity="HIGH", model="state", symbol="", message="No state machines were produced."))
    if not interface_model.functions:
        issues.append(ModelIssue(severity="HIGH", model="interface", symbol="", message="No function contracts were produced."))
    if not behavior_model.scenarios:
        issues.append(ModelIssue(severity="HIGH", model="behavior", symbol="", message="No behavior scenarios were produced."))

    for machine in state_model.machines:
        state_set = set(machine.states)
        if machine.initial_state not in state_set:
            issues.append(ModelIssue(
                severity="HIGH", model="state", symbol=machine.name,
                message=f"Initial state {machine.initial_state!r} is not declared.",
            ))
        for duplicate in _duplicates(machine.states):
            issues.append(ModelIssue(severity="HIGH", model="state", symbol=duplicate, message="State is declared more than once."))
        for transition in machine.transitions:
            # PlantUML's initial/final pseudo-state is valid without a
            # matching entry in the declared domain-state list.
            if transition.source != "[*]" and transition.source not in state_set:
                issues.append(ModelIssue(severity="HIGH", model="state", symbol=transition.source, message="Transition source is not declared."))
            if transition.target != "[*]" and transition.target not in state_set:
                issues.append(ModelIssue(severity="HIGH", model="state", symbol=transition.target, message="Transition target is not declared."))

    function_names = [function.name for function in interface_model.functions]
    function_set = set(function_names)
    for duplicate in _duplicates(function_names):
        issues.append(ModelIssue(severity="HIGH", model="interface", symbol=duplicate, message="Function is declared more than once."))
    for duplicate in _duplicates([item.name for item in interface_model.types]):
        issues.append(ModelIssue(severity="HIGH", model="interface", symbol=duplicate, message="Type is declared more than once."))

    state_variables = {variable.name for variable in state_model.variables}
    for function in interface_model.functions:
        for variable in function.reads_state + function.writes_state:
            if variable not in state_variables:
                issues.append(ModelIssue(
                    severity="MEDIUM", model="interface", symbol=function.name,
                    message=f"References undeclared state variable {variable!r}.",
                ))

    for machine in state_model.machines:
        for transition in machine.transitions:
            for function_reference in transition.action_functions:
                function_name = _callable_name(function_reference)
                if function_name and function_name not in function_set:
                    issues.append(ModelIssue(
                        severity="MEDIUM", model="cross", symbol=function_name,
                        message=(
                            f"State transition in {machine.name} references an internal "
                            "action absent from the public interface model."
                        ),
                    ))

    for scenario in behavior_model.scenarios:
        orders = [step.order for step in scenario.steps]
        if len(orders) != len(set(orders)):
            issues.append(ModelIssue(severity="HIGH", model="behavior", symbol=scenario.name, message="Step order values are not unique."))

        for step in scenario.steps:
            function_name = _callable_name(step.function)
            if function_name and function_name not in function_set:
                issues.append(ModelIssue(
                    severity="MEDIUM", model="cross", symbol=function_name,
                    message=(
                        f"Behavior scenario {scenario.name} calls an internal or platform "
                        "function absent from the public interface model."
                    ),
                ))
        for special in [scenario.completion_function, scenario.abort_function]:
            function_name = _callable_name(special)
            if function_name and function_name not in function_set:
                issues.append(ModelIssue(
                    severity="MEDIUM", model="cross", symbol=function_name,
                    message=f"Behavior scenario {scenario.name} references an undeclared lifecycle function.",
                ))

    return ModelValidationReport(
        valid=not any(issue.severity == "HIGH" for issue in issues),
        issues=issues,
    )
