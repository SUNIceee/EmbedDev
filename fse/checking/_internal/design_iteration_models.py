"""Typed records used by the iterative three-stage design loop."""

from __future__ import annotations

from enum import Enum
import hashlib
import json
from typing import Any, Dict, List, Literal, Optional

from pydantic import BaseModel, Field


class IssueSeverity(str, Enum):
    CRITICAL = "critical"
    HIGH = "high"
    MEDIUM = "medium"
    LOW = "low"


class DesignIssue(BaseModel):
    code: str
    agent: Literal["agent_1_state_modeler", "agent_2_interface_modeler", "agent_3_behavior_modeler", "cross"]
    severity: IssueSeverity
    category: str
    artifact: str
    message: str
    details: Dict[str, Any] = Field(default_factory=dict)

    @property
    def fingerprint(self) -> str:
        payload = {
            "code": self.code,
            "agent": self.agent,
            "category": self.category,
            "artifact": self.artifact,
            "message": self.message,
            "details": self.details,
        }
        raw = json.dumps(payload, ensure_ascii=False, sort_keys=True)
        return hashlib.sha256(raw.encode("utf-8")).hexdigest()[:16]


class ValidationResult(BaseModel):
    valid: bool
    issues: List[DesignIssue] = Field(default_factory=list)
    checks: Dict[str, Any] = Field(default_factory=dict)

    def issue_signature(self) -> str:
        joined = "|".join(sorted(issue.fingerprint for issue in self.issues))
        return hashlib.sha256(joined.encode("utf-8")).hexdigest()[:16]


class IterationDecision(BaseModel):
    action: Literal["accept", "repair", "regenerate", "stop"]
    reason: str
    repair_agents: List[Literal["agent_1_state_modeler", "agent_2_interface_modeler", "agent_3_behavior_modeler"]] = Field(
        default_factory=list
    )
    regenerate_from: Optional[Literal["agent_1_state_modeler", "agent_2_interface_modeler", "agent_3_behavior_modeler"]] = None
    issue_signature: str = ""


class IterationHistoryEntry(BaseModel):
    iteration: int
    operation: Literal["generate", "repair"]
    repaired_agents: List[str] = Field(default_factory=list)
    decision: IterationDecision
    issue_count: int
    issue_signature: str
    artifact_hashes: Dict[str, str] = Field(default_factory=dict)


def summarize_issues(issues: List[DesignIssue]) -> str:
    if not issues:
        return "No issues."
    return "\n".join(
        f"- [{issue.severity.value.upper()}] {issue.code} "
        f"({issue.agent}/{issue.artifact}): {issue.message}"
        + ("\n  Details: " + json.dumps(issue.details, ensure_ascii=False, sort_keys=True)
           if issue.details else "")
        for issue in issues
    )
