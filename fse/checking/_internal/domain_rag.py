# domain_rag.py
"""
Optional domain retrieval using lightweight keyword matching without embeddings

Retrieve matching domain knowledge from knowledge_base/domain_patterns/*.md
and supply it as context to the corresponding agent.

Usage:
    from domain_rag import retrieve_domain_context

    ctx = retrieve_domain_context(user_requirement, "agent_1_state_modeler")  # State-diagram patterns
    ctx = retrieve_domain_context(user_requirement, "agent_2_interface_modeler")  # API and skeleton patterns
    ctx = retrieve_domain_context(user_requirement, "agent_3_behavior_modeler")  # Sequence patterns
    ctx = retrieve_domain_context(user_requirement, "agent_4_code_generator")  # Code patterns
"""
import re
import os
from pathlib import Path
from typing import Optional

_PROJECT_DIR = Path(__file__).parent
_DOMAIN_DIR = Path(os.getenv("EMBEDDEV_DOMAIN_DIR") or (_PROJECT_DIR / "knowledge_base" / "domain_patterns"))

# Agent → Markdown section header mapping
_AGENT_SECTION_MAP = {
    "agent_1_state_modeler": "State Modeler_StatePatterns",
    "agent_2_interface_modeler": "Interface Modeler_APIPatterns",
    "agent_3_behavior_modeler": "Behavior Modeler_SequencePatterns",
    "agent_4_code_generator": "Implementation Generator_CodePatterns",
}

# General embedded-system keywords used for fallback weighting
_EMBEDDED_GENERAL_WORDS = {
    "init", "idle", "active", "error", "fault", "state", "isr",
    "timer", "gpio", "uart", "spi", "i2c", "adc", "dma",
    "struct", "typedef", "volatile", "static", "interrupt",
    "sensor", "actuator", "pwm", "encoder", "loop", "control",
}


def _tokenize(text: str) -> set:
    """Extract lowercase alphanumeric tokens of at least three characters"""
    return set(re.findall(r"[a-zA-Z_][a-zA-Z0-9_]{2,}", text.lower()))


def _extract_keywords(text: str) -> set:
    """Extract English keywords while filtering common stopwords"""
    tokens = _tokenize(text)
    stopwords = {
        "the", "and", "for", "that", "this", "with", "from",
        "are", "not", "has", "have", "will", "can", "its",
        "all", "each", "but", "also", "into", "over", "use",
        "when", "where", "which", "then", "than", "been",
        "should", "would", "could", "must", "shall", "may",
    }
    return {t for t in tokens if len(t) >= 3 and t not in stopwords}


def _score_domain(
    query_keywords: set,
    domain_content: str,
    domain_keywords_line: str,
) -> float:
    """
    Score a domain using keyword overlap with additional domain-keyword weight
    """
    # 1. Score exact matches in the Keywords metadata line
    kw_tokens = _tokenize(domain_keywords_line)
    if kw_tokens:
        kw_overlap = len(query_keywords & kw_tokens)
        kw_score = kw_overlap / max(len(kw_tokens), 1)
    else:
        kw_score = 0.0

    # 2. Add a lower-weight full-text overlap score
    content_tokens = _tokenize(domain_content)
    if content_tokens:
        content_overlap = len(query_keywords & content_tokens)
        content_score = content_overlap / max(len(query_keywords), 1)
    else:
        content_score = 0.0

    # Give metadata-keyword matches greater weight
    return kw_score * 0.7 + content_score * 0.3


def _parse_domain_file(md_path: Path) -> dict:
    """
    Parse one domain Markdown file and return:
    {
        "title": "Motor Control",
        "keywords": "motor, pid, pwm, encoder...",
        "sections": {
            "State Modeler_StatePatterns": "...",
            "Interface Modeler_APIPatterns": "...",
            ...
        }
    }
    """
    content = md_path.read_text(encoding="utf-8")
    title = ""
    keywords_line = ""
    sections: dict = {}
    current_section = "_preamble"
    current_lines: list = []

    for line in content.split("\n"):
        # Document title
        if line.startswith("# ") and not title:
            title = line.lstrip("# ").strip()
            continue

        # Keywords line
        if line.startswith("> Keywords:") or line.startswith("> Keywords："):
            keywords_line = line.lstrip("> Keywords:").lstrip("> Keywords：").strip()
            continue

        # Detect section headers
        if line.startswith("## "):
            # Save the preceding section
            if current_lines:
                sections[current_section] = "\n".join(current_lines).strip()
            current_section = line.lstrip("# ").strip()
            current_lines = []
            continue

        current_lines.append(line)

    # Save the final section
    if current_lines:
        sections[current_section] = "\n".join(current_lines).strip()

    return {
        "title": title,
        "keywords": keywords_line,
        "sections": sections,
    }


# Cache parsed domain files
_cache: Optional[list] = None


def _load_domains() -> list:
    global _cache
    if _cache is None:
        _cache = []
        for md_path in sorted(_DOMAIN_DIR.glob("*.md")):
            try:
                domain = _parse_domain_file(md_path)
                if domain["title"]:
                    _cache.append(domain)
            except Exception as e:
                print(f"[DomainRAG] Failed to parse {md_path.name}: {e}")
    return _cache


def retrieve_domain_context(
    user_requirement: str,
    agent_name: str,
    top_k: int = 1,
) -> str:
    """
    Match requirement keywords and return domain context for the requested agent.

    Args:
        user_requirement: Requirement text
        agent_name: "agent_1_state_modeler" | "agent_2_interface_modeler" | "agent_3_behavior_modeler" | "agent_4_code_generator"
        top_k: Maximum number of matching domains

    Returns:
        Formatted domain context, or an empty string when no match exists
    """
    if os.getenv("EMBEDDEV_ENABLE_CONTEXT", "0") != "1":
        return ""
    if not user_requirement or not user_requirement.strip():
        return ""

    domains = _load_domains()
    if not domains:
        return ""

    query_keywords = _extract_keywords(user_requirement)
    if not query_keywords:
        return ""

    target_section = _AGENT_SECTION_MAP.get(agent_name, "")

    # Score each domain
    scored = []
    for domain in domains:
        score = _score_domain(query_keywords, str(domain["sections"]), domain["keywords"])
        if score > 0.01:
            scored.append((score, domain))

    scored.sort(key=lambda x: x[0], reverse=True)

    if not scored:
        # Return no context if nothing matches
        return ""

    # Combine the target sections from the top matches
    lines = []
    for score, domain in scored[:top_k]:
        title = domain["title"]
        section_content = domain["sections"].get(target_section, "")

        if not section_content:
            # Use the first available section if the target section is absent
            for key in domain["sections"]:
                section_content = domain["sections"][key]
                break

        if section_content:
            lines.append(f"## Domain Reference: {title}")
            lines.append(f"### {target_section}")
            lines.append(section_content)
            lines.append("")

    context = "\n".join(lines)
    keywords_str = ", ".join(sorted(query_keywords)[:20])
    print(f"[DomainRAG] query=[{keywords_str}], agent={agent_name}, "
          f"matched={scored[0][1]['title']} (score={scored[0][0]:.3f}), "
          f"{len(context)} chars")
    return context


def build_vector_store() -> None:
    """Legacy compatibility hook; keyword retrieval needs no vector store"""
    domains = _load_domains()
    print(f"[DomainRAG] Loaded {len(domains)} domain files: "
          f"{[d['title'] for d in domains]}")


if __name__ == "__main__":
    import sys
    if "--build" in sys.argv:
        build_vector_store()
    elif "--test" in sys.argv:
        for req, agent in [
            ("motor PID control speed angle direction loops PWM output", "agent_1_state_modeler"),
            ("LCD driver SPI initialization display refresh", "agent_1_state_modeler"),
            ("G-code parsing CNC motion planning Bresenham interpolation stepper limit switch", "agent_1_state_modeler"),
            ("G-code parsing CNC motion planning Bresenham interpolation stepper limit switch", "agent_4_code_generator"),
            ("STM32 timer encoder speed measurement interrupt callback", "agent_2_interface_modeler"),
            ("battery management BMS charging discharging fuel gauge", "agent_1_state_modeler"),
        ]:
            ctx = retrieve_domain_context(req, agent)
            summary = ctx[:150].replace("\n", " ") if ctx else "(no match)"
            print(f"\nQ: {req[:60]} | agent={agent}")
            print(f"  → {summary}")
    else:
        print("Usage: python domain_rag.py --build | --test")
