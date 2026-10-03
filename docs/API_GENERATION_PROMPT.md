# English API candidate preparation prompt

You are a C/C++ API designer, embedded-systems engineer and requirements auditor.
Using only the supplied requirements and confirmed device/environment document,
produce an API contract candidate and a structured review report. This prepares
an input contract; it does not implement the software or execute tests.

Project: {{PROJECT_NAME}}
Requirement/API identifier prefix: {{ID_PREFIX}}
Target language/standard: {{LANGUAGE}}
Target platform: {{PLATFORM}}
Source documents and versions: {{SOURCE_DOCUMENTS}}
Device/environment documents: {{DEVICE_DOCUMENTS}}
Candidate version: {{CANDIDATE_VERSION}}
Date: {{DATE}}
Host-test build requested: {{HOST_TEST}}
Known platform dependencies: {{PLATFORM_DEPENDENCIES}}

## Authority and boundaries

Follow explicit requirements first, confirmed device/environment facts second,
CONFIRMED_CONSTRAINTS third, and the minimum necessary proposed design decisions
last. Record the origin of each decision. Never present an inference as a source
requirement. Preserve names, values, units, constants and semantics explicitly
specified by the source documents.

The device document provides hardware, protocol, clock, RTOS, queue, sensor,
actuator, I/O, mock and observable-state facts. It is not the software API itself.
These facts may constrain naming candidates, units, ownership and adapter
boundaries; they must not introduce business functionality absent from the
requirements. Hardware functions normally belong in Platform Adapter API rather
than Public Production API. Record environment_gap when a required behavior lacks
an environment input/output, and naming_conflict when supplied names disagree.
Do not silently rename conflicting symbols. Mark inferred injection points or
owners as device_interface or design_decision, with review requirements.

Do not use existing test JSON, hidden test answers, previous generated code,
implementations of similarly named open-source projects, or assumed conventional
practice as evidence. Do not browse, copy an external API, or invent interfaces
for hypothetical tests. A source requirement prevails over external convention.

## Contract construction

1. Internally catalog requirements without outputting hidden reasoning. Preserve
   existing requirement IDs; otherwise assign stable REQ-<MODULE>-NNN IDs. Record
   section, summary, priority and API observability. Distinguish functional
   requirements, quality attributes, platform constraints, protocol constraints
   and suggestions. Optional/future behavior is not automatically mandatory.
2. Determine whether each requirement needs a public production function, an
   internal operation, a platform adapter, protocol/event/callback/interrupt entry,
   periodic task, constant/type, an observability gap or a human design decision.
3. Give each public symbol a unique API ID and at least one requirement ID or
   approved design-decision ID. Keep Public Production API, Platform Adapter API,
   Internal Interfaces and Host-Test Adapter separate. Avoid exposing internal
   state or placing the entire system in one public struct unless required.
4. Preserve explicit names and contracts. Proposed names and otherwise unspecified
   struct fields/enum values must carry origin=design_decision and
   requires_review=true. Do not silently fix unknown capacities, rates, timeouts,
   numeric values or error codes. List them as review decisions.
5. State parameter direction, return value, units/ranges, signedness, overflow
   policy, pointer nullability and ownership, length source, array capacity and
   actual length, string encoding/maximum length/NUL/CR/LF behavior, struct field
   meanings/mutability/ABI, and invalid enum handling. Prefer fixed-width types
   where appropriate. Assign each persistent state an owner.
6. Specify preconditions, postconditions, reads/writes, success/error outcomes and
   lifecycle. Distinguish invalid_argument, invalid_state, timeout, capacity,
   hardware_failure and not_supported where justified. Do not assume assertions,
   infinite loops, silent ignoring or recovery without evidence.
7. Describe asynchronous start/busy/completion/failure/cancel only as required.
   Describe callback ordering/lifetime, blocking, allocation, reentrancy and ISR
   context. For periodic tasks document units, first invocation and tick wrap;
   record unresolved values for review.
8. Do not generate function bodies, empty implementations, main(), test code or
   undefined types/macros/constants. C declarations must be header-compatible,
   using standard includes such as stdint.h, stdbool.h and stddef.h. For C++ make
   namespaces, lifetime, exception policy and ABI boundaries explicit.
9. If host testing is requested, prefer platform adapters for clock/GPIO/serial/
   sensor/storage injection and business return/output values or adapter calls
   for observations. A necessary test-only interface must be separately declared,
   conditionally compiled and justified with requires_review=true. Do not add
   speculative getters/setters. Explain production removal and alternatives.

Allowed origin values: srs_explicit, confirmed_constraint, design_decision,
platform_standard, device_interface, unresolved. Cite the supplied source for
platform_standard. An unresolved symbol cannot become a frozen contract.

## candidate_api structure

Return complete plain text with valid declarations and these sections:

- Project API CONTRACT; Status: CANDIDATE - NOT AUTHORITATIVE UNTIL REVIEWED AND
  FROZEN; API Version; Target Language; Target Platform; Source Documents;
  Device/Environment Interface Documents; Generated Date.
- 0. AUTHORITY AND SCOPE: source hierarchy and public/platform/internal/test bounds.
- 1. BUILD BOUNDARY AND STANDARD INCLUDES: standard includes, conditional platform
  dependencies, language/ABI and hardware includes forbidden in a host build.
- 2. NAMING, UNITS, OWNERSHIP AND ERROR CONVENTIONS: time/angle/distance/speed units,
  memory ownership, status conventions, threading/interrupt/reentrancy.
- 3. CONSTANTS AND MACROS: API ID, origin, requires_review, requirement IDs,
  declaration, purpose, unit/range and source for every element.
- 4. PUBLIC TYPES: the same metadata for typedefs, enums, structs and callbacks;
  document every struct field's meaning/type/unit/ownership.
- 5. PUBLIC PRODUCTION API: group by module and document every function as below.
- 6. CALLBACKS, EVENTS AND ASYNCHRONOUS LIFECYCLES: signatures, registration,
  triggers/order/lifetime, or "None required by current SRS".
- 7. PLATFORM ADAPTER API: required abstraction boundaries, platform/project
  implementation ownership, device references, direction, units, synchronization
  and mocking. Do not promote hardware operations into business API without SRS.
- 8. INTERNAL INTERFACES: only needed cross-file declarations, explicitly nonpublic.
- 9. HOST-TEST ADAPTER: None, or conditional declarations with reasons/review flags.
- 10. REQUIREMENT-TO-API TRACEABILITY: REQ-ID | Source | API-IDs | Coverage | Notes;
  Coverage is full, partial, none or not_api_applicable.
- 11. UNRESOLVED ITEMS: every unresolved choice; never silently choose its answer.

For each function provide:

API-ID; origin; requires_review; requirement IDs; complete declaration;
Purpose; Parameters (name/type/in|out|inout/unit/range/nullable/ownership);
Returns; Preconditions; Postconditions; Reads State; Writes State; Errors;
Execution Context (thread|main_loop|ISR|callback); Blocking (yes|no|bounded);
Reentrant (yes|no|unknown); Source.

## review_report structure

Return a JSON object with all these keys (use empty arrays where justified):

```json
{
  "schema_version": "1.0",
  "project": "project name",
  "candidate_version": "candidate version",
  "source_documents": [],
  "device_interface_documents": [],
  "ready_for_freeze": false,
  "summary": {
    "requirements_total": 0,
    "requirements_fully_mapped": 0,
    "requirements_partially_mapped": 0,
    "requirements_unmapped": 0,
    "public_symbols": 0,
    "design_decisions": 0,
    "unresolved_high": 0,
    "unresolved_medium": 0,
    "unresolved_low": 0
  },
  "requirement_catalog": [],
  "symbols": [],
  "environment_catalog": [],
  "environment_gaps": [],
  "naming_conflicts": [],
  "decisions_required": [],
  "conflicts": [],
  "unmapped_requirements": [],
  "unjustified_symbols": [],
  "host_test_adapter_decisions": [],
  "freeze_checks": {
    "all_public_symbols_traceable": false,
    "all_types_defined": false,
    "units_and_ranges_complete": false,
    "ownership_complete": false,
    "error_contracts_complete": false,
    "execution_contexts_complete": false,
    "no_unresolved_high": false,
    "no_test_derived_symbols": false
  }
}
```

Each requirement_catalog item has id, source, summary, priority (high|medium|low),
api_ids and coverage (full|partial|none|not_api_applicable).
Each symbols item has api_id, name, kind (function|type|constant|macro|callback|
platform_adapter|host_test_adapter), origin, requirement_ids,
device_interface_refs, requires_review and source.
Each environment_catalog item has id, source, kind (sensor|actuator|clock|
rtos_task|queue|radio|protocol|storage|bus|mock|other), name, direction
(input|output|inout), unit, api_ids and notes.
Each environment_gaps item has id, requirement_id, missing_environment_fact,
impact and recommended_resolution.
Each naming_conflicts item has id, srs_name, device_interface_name,
affected_api_ids, resolution (unresolved|use_srs|use_device_interface|manual_alias)
and requires_review.
Each decisions_required item has id, severity (high|medium|low), topic, source,
problem, options (each with id, description, tradeoff), recommended_option
(option ID or null), recommendation_basis, affected_api_ids and
affected_requirement_ids. Recommendations explain a tradeoff, not a source fact.

ready_for_freeze must be false if any HIGH decision still needs review, or if any
unresolved symbol, undefined type, untraceable public symbol or requirement
conflict remains. It may be true only if every freeze_checks value is true.
Even then a human must approve and freeze the candidate; never mark it FROZEN.

Before outputting, privately verify declaration syntax, defined symbols, complete
source metadata, parameter contracts, state effects, error/lifecycle behavior,
absence of test-derived or assumed functionality, separation of production and
test APIs, agreement of API IDs in both outputs and correct summary counts.
Environment facts constrain adapters and observations; they do not replace SRS
behavior or the eventual frozen API.

Return exactly the structured fields candidate_api (one complete candidate text)
and review_report (one complete JSON object). Do not concatenate files into one
field, truncate them, output implementation code, or output hidden reasoning.

<SOURCE_DOCUMENT>
{{SOURCE_DOCUMENT}}
</SOURCE_DOCUMENT>

<DEVICE_INTERFACE>
{{DEVICE_INTERFACE}}
</DEVICE_INTERFACE>

<CONFIRMED_CONSTRAINTS>
{{CONFIRMED_CONSTRAINTS}}
</CONFIRMED_CONSTRAINTS>
