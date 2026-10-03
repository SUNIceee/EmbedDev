"""Deterministic severity routing for iterative three-stage design generation."""

from __future__ import annotations

from dataclasses import dataclass
from typing import List

from _internal.design_iteration_models import (
    DesignIssue,
    IssueSeverity,
    IterationDecision,
    IterationHistoryEntry,
    ValidationResult,
)


@dataclass(frozen=True)
class DesignIterationPolicy:
    max_iterations: int = 8
    max_local_repair_attempts: int = 2
    regenerate_high_issue_count: int = 4


_FORCE_REGENERATE_CATEGORIES = {
    "architecture",
    "api_contract",
    "missing_artifact",
    "model_schema",
    "structured_output",
}


_AGENT_ORDER = {"agent_1_state_modeler": 1, "agent_2_interface_modeler": 2, "agent_3_behavior_modeler": 3, "cross": 1}


def _earliest_affected_agent(issues: List[DesignIssue]) -> str:
    owners = [issue.agent for issue in issues]
    if not owners:
        return "agent_1_state_modeler"
    owner = min(owners, key=lambda item: _AGENT_ORDER[item])
    return "agent_1_state_modeler" if owner == "cross" else owner


def _consecutive_same_repairs(
    history: List[IterationHistoryEntry],
    issue_signature: str,
) -> int:
    count = 0
    for entry in reversed(history):
        if (
            entry.operation == "repair"
            and entry.issue_signature == issue_signature
        ):
            count += 1
        else:
            break
    return count


def choose_iteration_action(
    validation: ValidationResult,
    history: List[IterationHistoryEntry],
    iteration: int,
    policy: DesignIterationPolicy,
) -> IterationDecision:
    signature = validation.issue_signature()
    if validation.valid:
        return IterationDecision(
            action="accept",
            reason="All deterministic design checks passed.",
            issue_signature=signature,
        )
    if iteration >= policy.max_iterations:
        return IterationDecision(
            action="stop",
            reason=f"Reached design iteration limit ({policy.max_iterations}).",
            issue_signature=signature,
        )

    issues = validation.issues
    if any(issue.category == "validator_failure" for issue in issues):
        return IterationDecision(
            action="stop",
            reason="The deterministic validator failed; regenerating content cannot repair the validation infrastructure.",
            issue_signature=signature,
        )
    repeated_generations = 0
    for entry in reversed(history):
        if entry.operation == "generate" and entry.issue_signature == signature:
            repeated_generations += 1
        else:
            break
    if repeated_generations >= policy.max_local_repair_attempts:
        return IterationDecision(
            action="stop",
            reason="Repeated regeneration left the same detailed issues; stop without consuming more calls.",
            issue_signature=signature,
        )
    if any(issue.severity == IssueSeverity.CRITICAL for issue in issues):
        critical = [
            issue for issue in issues
            if issue.severity == IssueSeverity.CRITICAL
        ]
        return IterationDecision(
            action="regenerate",
            reason=(
                "A critical missing/schema/generation failure requires regeneration "
                "from the earliest affected stage."
            ),
            regenerate_from=_earliest_affected_agent(issues),
            issue_signature=signature,
        )

    severe = [
        issue for issue in issues
        if issue.severity == IssueSeverity.HIGH
    ]
    if any(issue.category in _FORCE_REGENERATE_CATEGORIES for issue in severe):
        forced = [
            issue for issue in severe
            if issue.category in _FORCE_REGENERATE_CATEGORIES
        ]
        return IterationDecision(
            action="regenerate",
            reason=(
                "A high-severity architecture or frozen-contract issue requires "
                "regeneration from the earliest affected stage."
            ),
            regenerate_from=_earliest_affected_agent(issues),
            issue_signature=signature,
        )
    if len(severe) >= policy.regenerate_high_issue_count:
        return IterationDecision(
            action="regenerate",
            reason=(
                f"{len(severe)} high-severity issues exceed the full-regeneration "
                f"threshold ({policy.regenerate_high_issue_count})."
            ),
            regenerate_from=_earliest_affected_agent(issues),
            issue_signature=signature,
        )

    repeated = _consecutive_same_repairs(history, signature)
    if repeated + 1 >= policy.max_local_repair_attempts:
        return IterationDecision(
            action="regenerate",
            reason=(
                "Local repair made no progress for "
                f"{repeated + 1} consecutive attempts."
            ),
            regenerate_from=_earliest_affected_agent(issues),
            issue_signature=signature,
        )

    owners = []
    for issue in issues:
        if issue.agent == "cross":
            return IterationDecision(
                action="regenerate",
                reason=(
                    "A cross-agent consistency issue cannot be safely localized; "
                    "regenerate from State Modeler."
                ),
                regenerate_from="agent_1_state_modeler",
                issue_signature=signature,
            )
        if issue.agent not in owners:
            owners.append(issue.agent)
    owners.sort(key=_AGENT_ORDER.get)
    return IterationDecision(
        action="repair",
        reason="Issues are localized and below the full-regeneration threshold.",
        repair_agents=owners,
        issue_signature=signature,
    )
