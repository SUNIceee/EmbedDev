"""Helpers for LLM structured output that may stringify nested JSON fields.

Some providers (notably DeepSeek tool-calling) return nested Pydantic fields
such as ``state_model`` as a JSON *string* instead of an object. LangChain then
fails during ``Schema(**args)`` with ``model_type`` / ``input_type=str``.

This module coerces those fields before / around Pydantic validation.
"""

from __future__ import annotations

import json
import re
from typing import Any, Iterable, Optional, Sequence, Type, TypeVar

from langchain_core.runnables import Runnable, RunnableLambda
from pydantic import BaseModel

T = TypeVar("T", bound=BaseModel)


def extract_json_object(text: Any) -> Optional[dict]:
    """Extract the first JSON object from a provider's text response.

    Some OpenAI-compatible reasoning models return ``<think>...</think>``
    and/or Markdown fences even when structured output was requested.  The
    schema parser rejects that wrapper, but the enclosed JSON is still a valid
    answer.  Decode from each object boundary instead of relying on brittle
    fence stripping.
    """
    if not isinstance(text, str):
        return None
    cleaned = re.sub(r"<think>.*?</think>\s*", "", text, flags=re.DOTALL).strip()
    decoder = json.JSONDecoder()
    for match in re.finditer(r"\{", cleaned):
        try:
            value, _ = decoder.raw_decode(cleaned[match.start() :])
        except json.JSONDecodeError:
            continue
        if isinstance(value, dict):
            return value
    return None


def coerce_stringified_json(value: Any) -> Any:
    """If ``value`` is a JSON object/array string, parse it; recurse into containers."""
    if isinstance(value, str):
        text = value.strip()
        if len(text) >= 2 and (
            (text.startswith("{") and text.endswith("}"))
            or (text.startswith("[") and text.endswith("]"))
        ):
            try:
                return coerce_stringified_json(json.loads(text))
            except json.JSONDecodeError:
                return value
        return value
    if isinstance(value, dict):
        return {key: coerce_stringified_json(item) for key, item in value.items()}
    if isinstance(value, list):
        return [coerce_stringified_json(item) for item in value]
    return value


def coerce_nested_json_fields(
    data: Any,
    field_names: Optional[Iterable[str]] = None,
) -> Any:
    """Coerce selected top-level fields that were returned as JSON strings.

    If ``field_names`` is None, every top-level value is coerced when it looks
    like JSON. Nested content inside a coerced field is also walked.
    """
    if not isinstance(data, dict):
        return data

    names = set(field_names) if field_names is not None else None
    result = dict(data)
    for key, value in list(result.items()):
        if names is None or key in names:
            result[key] = coerce_stringified_json(value)
    return result


def extract_tool_call_args(message: Any) -> Optional[dict]:
    """Best-effort extraction of the first tool-call args dict from an AIMessage."""
    tool_calls = getattr(message, "tool_calls", None) or []
    for call in tool_calls:
        if isinstance(call, dict):
            args = call.get("args")
        else:
            args = getattr(call, "args", None)
        if isinstance(args, dict):
            return args

    additional = getattr(message, "additional_kwargs", None) or {}
    raw_calls = additional.get("tool_calls") or []
    for call in raw_calls:
        if not isinstance(call, dict):
            continue
        function = call.get("function") or {}
        arguments = function.get("arguments")
        if isinstance(arguments, dict):
            return arguments
        if isinstance(arguments, str) and arguments.strip():
            try:
                parsed = json.loads(arguments)
            except json.JSONDecodeError:
                continue
            if isinstance(parsed, dict):
                return parsed
    return None


def validate_with_coercion(
    schema: Type[T],
    data: Any,
    nested_fields: Sequence[str],
) -> T:
    """Run ``schema.model_validate`` after coercing known nested JSON fields."""
    coerced = coerce_nested_json_fields(data, nested_fields)
    return schema.model_validate(coerced)


def with_coercing_structured_output(
    llm: Any,
    schema: Type[T],
    nested_fields: Sequence[str],
) -> Runnable:
    """Like ``llm.with_structured_output(schema)``, but coerces stringified nests.

    Uses ``include_raw=True`` so a provider that returns nested objects as JSON
    strings can be recovered without failing the whole agent step.
    """
    inner = llm.with_structured_output(schema, include_raw=True)

    def _resolve(payload: dict) -> T:
        parsed = payload.get("parsed")
        if parsed is not None:
            # Parsed object may still need nothing; return as-is.
            return parsed

        raw = payload.get("raw")
        args = extract_tool_call_args(raw) if raw is not None else None
        if args is None and raw is not None:
            args = extract_json_object(getattr(raw, "content", None))
        if args is not None:
            try:
                return validate_with_coercion(schema, args, nested_fields)
            except Exception as coerce_error:
                original = payload.get("parsing_error")
                if original is not None:
                    raise original from coerce_error
                raise

        original = payload.get("parsing_error")
        if original is not None:
            raise original
        diagnostic = {
            "content": getattr(raw, "content", None),
            "tool_calls": getattr(raw, "tool_calls", None),
            "additional_kwargs": getattr(raw, "additional_kwargs", None),
            "response_metadata": getattr(raw, "response_metadata", None),
        }
        raise ValueError(
            f"Structured output for {schema.__name__} returned no parsed result "
            f"and no recoverable tool-call arguments. Raw response: {diagnostic!r}"
        )

    return inner | RunnableLambda(_resolve)
