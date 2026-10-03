"""Targeted repair agent for State Modeler state artifacts."""

from __future__ import annotations

import json
from typing import Any, Dict, List

from langchain_core.prompts import ChatPromptTemplate
from pydantic import BaseModel, Field

from design import StateModel
from design import DesignIssue, summarize_issues
from context import retrieve_error_lessons
from runtime import llm


class StateModelRepairOutput(BaseModel):
    state_diagram: str = Field(
        description="Complete repaired PlantUML state diagram"
    )
    state_model: StateModel = Field(
        description="Complete repaired canonical state model"
    )
    change_summary: List[str] = Field(default_factory=list)


class StateModelRepairAgent:
    def __init__(self) -> None:
        self.chain = ChatPromptTemplate.from_messages(
            [
                (
                    "system",
                    "You repair State Modeler state-design artifacts. Preserve all correct "
                    "content and make only changes required by the supplied errors. "
                    "Return complete replacement artifacts, not patches. The canonical "
                    "state model and PlantUML must remain semantically aligned. Do not "
                    "change the authoritative API. Use the device/environment interface "
                    "only for environment/platform state evidence; it must not override "
                    "requirements or the frozen API.",
                ),
                ("human", "{input}"),
            ]
        ) | llm.with_structured_output(StateModelRepairOutput)

    def repair(
        self,
        state: Dict[str, Any],
        issues: List[DesignIssue],
    ) -> Dict[str, Any]:
        error_text = summarize_issues(issues)
        lessons = retrieve_error_lessons(
            error_text,
            agent="agent_1_state_modeler",
            top_k=int(state.get("error_notebook_top_k", 4)),
        )
        prompt = (
            "Repair the previous State Modeler output.\n\n"
            f"=== Original requirements ===\n{state['user_requirement']}\n\n"
            f"=== Device / Environment Interface ===\n{state.get('device_interface', '(not provided)')}\n\n"
            f"=== Frozen API ===\n{state['api_spec']}\n\n"
            f"=== Validation errors ===\n{error_text}\n\n"
            f"{lessons}\n\n"
            f"=== Previous state diagram ===\n{state['state_diagram']}\n\n"
            "=== Previous canonical state model ===\n"
            f"{json.dumps(state['state_model'], ensure_ascii=False, indent=2)}\n\n"
            "Fix only the reported State Modeler problems. If an error is purely syntax, "
            "do not redesign the state architecture."
        )
        result: StateModelRepairOutput = self.chain.invoke({"input": prompt})
        return {
            "state_diagram": result.state_diagram,
            "state_model": result.state_model.model_dump(),
            "repair_summary": result.change_summary,
        }


agent_1_state_modeler_repair_agent = StateModelRepairAgent()
