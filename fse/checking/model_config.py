"""Resolve the shared FSE experiment model configuration without exposing keys."""

from __future__ import annotations

import json
import os
from dataclasses import dataclass
from pathlib import Path


_MODEL_FILE = Path(os.getenv("EMBEDDEV_MODELS_FILE") or (Path(__file__).resolve().parent.parent / "fse_models.json"))


@dataclass(frozen=True)
class FSEModel:
    name: str
    model: str
    base_url: str
    api_key: str
    max_tokens: int
    temperature: float


def model_names() -> list[str]:
    data = json.loads(_MODEL_FILE.read_text(encoding="utf-8"))
    return [item["name"] for item in data["models"] if item.get("enabled", True)]


def resolve_fse_model(name: str) -> FSEModel:
    data = json.loads(_MODEL_FILE.read_text(encoding="utf-8"))
    entry = next(
        (item for item in data["models"] if item["name"] == name and item.get("enabled", True)),
        None,
    )
    if entry is None:
        raise ValueError(f"Unknown or disabled FSE model: {name}")
    if entry.get("provider") != "openai_compatible":
        raise ValueError(f"Unsupported FSE provider for {name}: {entry.get('provider')}")

    base_url = entry.get("base_url") or os.getenv(entry.get("base_url_env", ""))
    api_key = os.getenv(entry["api_key_env"])
    if not base_url:
        raise ValueError(f"Missing endpoint: set environment variable {entry.get('base_url_env')}")
    if not api_key:
        raise ValueError(f"Missing API key: set environment variable {entry['api_key_env']}")

    model_id = os.getenv(entry.get("model_env", "")) or entry.get("model")
    if not model_id:
        raise ValueError(f"Missing model ID: set environment variable {entry.get('model_env')}")

    # ChatOpenAI expects the API root, while the direct-generation runner may
    # store a concrete Responses or Chat Completions endpoint in fse/.env.
    # Keep that direct-runner convention intact and normalize only here.
    base_url = base_url.rstrip("/")
    if base_url.endswith("/responses"):
        base_url = base_url[: -len("/responses")]
    elif base_url.endswith("/chat/completions"):
        base_url = base_url[: -len("/chat/completions")]

    return FSEModel(
        name=entry["name"],
        model=model_id,
        base_url=base_url,
        api_key=api_key,
        max_tokens=int(entry.get("max_tokens", 65536)),
        temperature=float(entry.get("temperature", 0.0)),
    )
