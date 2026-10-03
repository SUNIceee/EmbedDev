"""Independent test designer: derives tests from requirements and design models, not generated code."""

from __future__ import annotations

import json
from typing import Any, Dict

from langchain_core.prompts import ChatPromptTemplate

from runtime import llm
from runtime import with_coercing_structured_output
from test_plan_models import TestDesignOutput


class TestDesignAgent:
    def __init__(self) -> None:
        self.prompt = ChatPromptTemplate.from_messages([
            ("system", """You are an independent verification architect for embedded C.
Create two artifacts from requirements and accepted design models, before any production code exists.

First, produce requirement coverage: every stated REQ rule must cite its exact REQ anchor and evidence from State Modeler (state), Interface Modeler (function contract), and/or Behavior Modeler (flow). Mark gaps honestly.

Second, produce a code-facing test plan. Tests must verify API conformance, behavior, state transitions, timeouts/safety, numerical boundaries, integration, and static completion as applicable. Every case needs REQ anchors and a concrete oracle. Do not inspect or assume generated production code.

Test access rules:
- Prefer frozen public API calls and device-interface mocks.
- For state that cannot be reached through public API, request a semantic fixture using a dotted canonical_state such as supervisor.armed or runtime.tick.
- Never emit `extern`, `static`, an implementation-local variable name, a guessed C symbol, or a new public function.
- A fixture is an adapter requirement; it does not change the frozen public API.
- Do not create tests that call unbounded firmware tasks. Require a bounded API/mocked scheduler or mark target/manual.
"""),
            ("human", """REQ:\n{req}\n\nFROZEN API:\n{api}\n\nDEVICE INTERFACE:\n{device}\n\nAGENT1 STATE MODEL:\n{state_model}\n\nAGENT2 INTERFACE MODEL:\n{interface_model}\n\nAGENT3 BEHAVIOR MODEL:\n{behavior_model}"""),
        ])
        self.chain = self.prompt | with_coercing_structured_output(
            llm, TestDesignOutput,
            nested_fields=("requirement_coverage", "code_test_plan", "fixture_requirements"),
        )

    def generate(self, inputs: Dict[str, Any]) -> TestDesignOutput:
        print("[TEST AGENT] Generating requirement coverage and code test plan...", flush=True)
        return self.chain.invoke({
            "req": inputs["req"], "api": inputs["api"], "device": inputs["device"],
            "state_model": json.dumps(inputs["state_model"], ensure_ascii=False, indent=2),
            "interface_model": json.dumps(inputs["interface_model"], ensure_ascii=False, indent=2),
            "behavior_model": json.dumps(inputs["behavior_model"], ensure_ascii=False, indent=2),
        })


test_design_agent = TestDesignAgent()
