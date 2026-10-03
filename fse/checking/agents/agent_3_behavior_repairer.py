"""Targeted repair agent for Behavior Modeler behavior artifacts."""

from __future__ import annotations

import json
from typing import Any, Dict, List

from langchain_core.prompts import ChatPromptTemplate
from pydantic import BaseModel, Field

from design import BehaviorModel
from design import DesignIssue, summarize_issues
from context import retrieve_error_lessons
from runtime import llm


class BehaviorModelRepairOutput(BaseModel):
    sequence_diagram: str = Field(
        description="Complete repaired PlantUML sequence diagram"
    )
    behavior_model: BehaviorModel = Field(
        description="Complete repaired canonical behavior model"
    )
    change_summary: List[str] = Field(default_factory=list)


class BehaviorModelRepairAgent:
    def __init__(self) -> None:
        self.chain = ChatPromptTemplate.from_messages(
            [
                (
                    "system",
                    "You repair Behavior Modeler behavior artifacts. Preserve correct scenarios "
                    "and call order. Make only changes required by supplied errors. "
                    "Return a complete sequence diagram and behavior model. Every "
                    "multiline note must use end note; every alt/opt/loop/group block "
                    "must have exactly one matching end. Use the device/environment "
                    "interface only to repair external actors, ticks, queues, protocols, "
                    "sensors, actuators, and mock/observable boundaries; do not change "
                    "frozen API calls.",
                ),
                ("human", "{input}"),
            ]
        ) | llm.with_structured_output(BehaviorModelRepairOutput)

    def repair(
        self,
        state: Dict[str, Any],
        issues: List[DesignIssue],
    ) -> Dict[str, Any]:
        error_text = summarize_issues(issues)
        lessons = retrieve_error_lessons(
            error_text,
            agent="agent_3_behavior_modeler",
            top_k=int(state.get("error_notebook_top_k", 4)),
        )
        prompt = (
            "Repair the previous Behavior Modeler output.\n\n"
            f"=== Original requirements ===\n{state['user_requirement']}\n\n"
            f"=== Device / Environment Interface ===\n{state.get('device_interface', '(not provided)')}\n\n"
            f"=== Frozen API ===\n{state['api_spec']}\n\n"
            f"=== Validation errors ===\n{error_text}\n\n"
            f"{lessons}\n\n"
            f"=== State Modeler state diagram ===\n{state['state_diagram']}\n\n"
            f"=== Interface Modeler function skeleton ===\n{state['function_skeleton']}\n\n"
            f"=== Previous sequence diagram ===\n{state['sequence_diagram']}\n\n"
            "=== Previous canonical behavior model ===\n"
            f"{json.dumps(state['behavior_model'], ensure_ascii=False, indent=2)}\n\n"
            "Fix only the reported Behavior Modeler problems. If they are PlantUML syntax "
            "errors, do not redesign behavior scenarios or call ordering."
        )
        result: BehaviorModelRepairOutput = self.chain.invoke({"input": prompt})
        return {
            "sequence_diagram": result.sequence_diagram,
            "behavior_model": result.behavior_model.model_dump(),
            "repair_summary": result.change_summary,
        }


agent_3_behavior_modeler_repair_agent = BehaviorModelRepairAgent()
