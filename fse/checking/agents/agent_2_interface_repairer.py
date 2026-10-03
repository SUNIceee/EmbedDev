"""Targeted repair agent for Interface Modeler interface artifacts."""

from __future__ import annotations

import json
from typing import Any, Dict, List

from langchain_core.prompts import ChatPromptTemplate
from pydantic import BaseModel, Field

from design import InterfaceModel
from design import DesignIssue, summarize_issues
from context import retrieve_error_lessons
from runtime import llm


class InterfaceModelRepairOutput(BaseModel):
    function_skeleton: str = Field(
        description="Complete repaired C/C++ declaration-only header"
    )
    interface_model: InterfaceModel = Field(
        description="Complete repaired canonical interface model"
    )
    change_summary: List[str] = Field(default_factory=list)


class InterfaceModelRepairAgent:
    def __init__(self) -> None:
        self.chain = ChatPromptTemplate.from_messages(
            [
                (
                    "system",
                    "You repair Interface Modeler interface artifacts. The supplied API is frozen "
                    "and authoritative: never rename, add, remove, reorder, or change "
                    "a public declaration unless an error explicitly proves the "
                    "previous artifact differed from that API. Preserve correct "
                    "content and return complete replacement artifacts. Repair the "
                    "semantic contract too: every function must include purpose, "
                    "preconditions, postconditions, error_cases, source_requirements, "
                    "precise req_anchors, semantic_requirements, required_state_updates, "
                    "required_code_patterns where source evidence matters. "
                    "Populate reads_state/writes_state for persistent state access; leave "
                    "them empty only for truly stateless/read-only functions. Do not invent "
                    "public API elements or requirement facts. "
                    "Use the device/environment interface to repair adapter boundaries, "
                    "units, tick/queue/protocol facts, and observable/mock ownership; it "
                    "must not override frozen public declarations or requirement behavior.",
                ),
                ("human", "{input}"),
            ]
        ) | llm.with_structured_output(InterfaceModelRepairOutput)

    def repair(
        self,
        state: Dict[str, Any],
        issues: List[DesignIssue],
    ) -> Dict[str, Any]:
        error_text = summarize_issues(issues)
        lessons = retrieve_error_lessons(
            error_text,
            agent="agent_2_interface_modeler",
            top_k=int(state.get("error_notebook_top_k", 4)),
        )
        prompt = (
            "Repair the previous Interface Modeler output.\n\n"
            f"=== Original requirements ===\n{state['user_requirement']}\n\n"
            f"=== Device / Environment Interface ===\n{state.get('device_interface', '(not provided)')}\n\n"
            f"=== Frozen API ===\n{state['api_spec']}\n\n"
            f"=== Validation errors ===\n{error_text}\n\n"
            f"{lessons}\n\n"
            f"=== State Modeler state diagram ===\n{state['state_diagram']}\n\n"
            f"=== Previous function skeleton ===\n{state['function_skeleton']}\n\n"
            "=== Previous canonical interface model ===\n"
            f"{json.dumps(state['interface_model'], ensure_ascii=False, indent=2)}\n\n"
            "Fix only the reported Interface Modeler problems. Keep declarations aligned with "
            "the frozen API exactly and do not add public helper types. If validation "
            "reports missing semantic fields, repair those fields from the original "
            "requirements, State Modeler context, and frozen API; use `none` only when the "
            "contract explicitly has no precondition or error case. Source references must "
            "be precise enough for Implementation Generator to go back to the exact REQ/API sentence, not only "
            "a broad testcase range. Extract formula signs, timing constants, sentinel values, "
            "return semantics, and required state/output updates into semantic_requirements "
            "and required_state_updates."
        )
        result: InterfaceModelRepairOutput = self.chain.invoke({"input": prompt})
        return {
            "function_skeleton": result.function_skeleton,
            "interface_model": result.interface_model.model_dump(),
            "repair_summary": result.change_summary,
        }


agent_2_interface_modeler_repair_agent = InterfaceModelRepairAgent()
