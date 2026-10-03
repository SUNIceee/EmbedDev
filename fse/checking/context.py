"""Unified domain-context and error-notebook interface."""

from _internal.domain_rag import retrieve_domain_context
from _internal.error_notebook import record_lesson, retrieve_error_lessons

__all__ = ["record_lesson", "retrieve_domain_context", "retrieve_error_lessons"]
