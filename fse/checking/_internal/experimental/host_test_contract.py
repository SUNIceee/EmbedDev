"""Stable C adapter names for semantic host-test observables."""

from __future__ import annotations

import re
from typing import Any, Dict, List


_C_IDENTIFIER = re.compile(r"^[A-Za-z_]\w*$")
_NOT_C_IDENTIFIER = re.compile(r"[^A-Za-z0-9_]+")


def adapter_symbol_for_observable(canonical_name: str) -> str:
    """Return the exported C symbol for a canonical semantic observable.

    Existing contracts that already use a valid C identifier remain source
    compatible. Qualified names such as ``supervisor.isFlying`` are semantic
    paths, not C identifiers, so they receive a deterministic test adapter.
    """
    name = str(canonical_name or "").strip()
    if _C_IDENTIFIER.fullmatch(name):
        return name
    safe = _NOT_C_IDENTIFIER.sub("_", name).strip("_") or "observable"
    return f"cf_test_{safe}"


def render_host_observable_adapter_contract(interface_model: object) -> List[Dict[str, Any]]:
    """Render the Implementation Generator-facing mapping without mutating Interface Modeler artifacts."""
    if not isinstance(interface_model, dict):
        return []
    contract = interface_model.get("host_test_contract") or {}
    if not isinstance(contract, dict):
        return []
    symbols = contract.get("observable_symbols") or []
    if not isinstance(symbols, list):
        return []
    rendered: List[Dict[str, Any]] = []
    for item in symbols:
        if not isinstance(item, dict):
            continue
        canonical_name = str(item.get("name") or "").strip()
        if not canonical_name:
            continue
        rendered.append({
            "canonical_name": canonical_name,
            "adapter_symbol": adapter_symbol_for_observable(canonical_name),
            "type_name": str(item.get("type_name") or "").strip(),
            "access": str(item.get("access") or "read"),
            "maps_to": str(item.get("maps_to") or "").strip(),
            "sync_semantics": item.get("sync_semantics") or [],
            "source_requirements": item.get("source_requirements") or [],
        })
    return rendered
