# new_utils.py
import os
import sys
from dotenv import load_dotenv
from langchain_openai import ChatOpenAI
from langchain_google_genai import ChatGoogleGenerativeAI
from langchain_deepseek import ChatDeepSeek
import subprocess
import json
import re 
import time
from typing import Any, Optional

from langchain_core.runnables import RunnableConfig, RunnableLambda
from model_config import resolve_fse_model

# No credential files are searched or loaded during import or offline inspection.

# --- Optional Apalache bounded-model-checking helpers ---

# Optional Apalache paths
APALACHE_BIN = os.getenv("APALACHE_BIN", os.path.join(os.path.dirname(__file__), "apalache", "bin", "apalache-mc"))
_APALACHE_JAR = os.getenv("APALACHE_JAR", os.path.join(os.path.dirname(__file__), "apalache", "lib", "apalache.jar"))
# Apalache 0.57.x requires Java 17/21 rather than Java 26+
_JAVA_17_HOME = os.getenv("APALACHE_JAVA_HOME", "")
_SANY_CACHE = {}
DEFAULT_RUN_TIMEOUT_SECONDS = int(os.getenv("RUN_TIMEOUT_SECONDS", "600"))


def _make_env():
    env = os.environ.copy()
    if os.path.isdir(_JAVA_17_HOME):
        env["JAVA_HOME"] = _JAVA_17_HOME
    return env


def _run_sany_direct(tla_file_path: str) -> str:
    """Run the SANY parser directly to obtain diagnostics omitted by Apalache"""
    if tla_file_path in _SANY_CACHE:
        return _SANY_CACHE[tla_file_path]
    try:
        result = subprocess.run(
            ["java", "-cp", _APALACHE_JAR, "tla2sany.SANY", tla_file_path],
            text=True,
            encoding='utf-8',
            capture_output=True,
            timeout=DEFAULT_RUN_TIMEOUT_SECONDS,
            env=_make_env(),
        )
        detail = (result.stdout + "\n" + result.stderr).strip()
        _SANY_CACHE[tla_file_path] = detail
        return detail
    except Exception as e:
        return f"SANY direct run failed: {e}"


def run_apalache_verification(tla_file_path: str, cfg_path: str = None, length: int = 100) -> dict:
    """Invoke Apalache for bounded checking of a supplied TLA+ model"""
    print(f"--- Calling Apalache BMC on {tla_file_path} (length={length}) ---")

    if not os.path.exists(tla_file_path):
        return {"status": "error", "output": f"TLA file not found: {tla_file_path}"}
    if not os.path.exists(APALACHE_BIN):
        return {"status": "error", "output": f"Apalache not found: {APALACHE_BIN}"}

    try:
        command = [APALACHE_BIN, "check", f"--length={length}", "--features=no-rows"]
        if cfg_path and os.path.exists(cfg_path):
            command.append(f"--config={cfg_path}")
        command.append(tla_file_path)

        result = subprocess.run(
            command,
            text=True,
            encoding='utf-8',
            capture_output=True,
            timeout=DEFAULT_RUN_TIMEOUT_SECONDS,
            env=_make_env(),
        )

        raw_output = result.stdout
        raw_error = result.stderr

        results = {}
        if "EXITCODE: OK" in raw_output or "The outcome is: NoError" in raw_output:
            status = "success"
            results["overall"] = {"status": "true", "message": "BMC: All invariants hold up to bound."}
        else:
            status = "failure"
            results["overall"] = {"status": "false", "message": "BMC: Violation found or check failed."}

            error_parts = []

            # 1. Extract Apalache E@ error lines
            for line in raw_output.split("\n"):
                if "E@" in line:
                    error_parts.append(line.strip())

            # 2. Obtain detailed SANY diagnostics for parse errors
            if "SANY" in raw_output or "Parsing error" in raw_output or "Parsing error" in raw_error:
                sany_detail = _run_sany_direct(tla_file_path)
                if sany_detail:
                    # Extract the relevant SANY error lines
                    sany_errors = []
                    in_error = False
                    for line in sany_detail.split("\n"):
                        if "***Parse Error***" in line or "Fatal error" in line:
                            in_error = True
                        if in_error:
                            sany_errors.append(line.strip())
                        if "*** Errors:" in line:
                            break
                    if sany_errors:
                        error_parts.append("--- SANY Parse Errors ---")
                        error_parts.extend(sany_errors)
                    else:
                        # Fall back to stderr when no structured diagnostic exists
                        error_parts.append("--- SANY Details ---")
                        error_parts.append(sany_detail[:1500])

            # 3. Collect intermediate validation failures such as type errors
            for line in raw_output.split("\n"):
                if "type input error" in line.lower() or "type error" in line.lower():
                    error_parts.append(line.strip())

            # 4. Detect Apalache internal failures
            crash_keywords = ["InternalCheckerError", "PassNameError", "Unsupported", "NotImplementedError"]
            for line in raw_output.split("\n"):
                for kw in crash_keywords:
                    if kw in line:
                        error_parts.append(f"[Apalache Internal] {line.strip()}")

            # 5. Look for a counterexample if no prior diagnostic was found
            if "counterexample" in raw_output.lower() or "State 0:" in raw_output:
                results["counterexample"] = "A counterexample trace was found in raw output."
                if not error_parts:
                    error_parts.append("Invariant violation: counterexample found by BMC.")

            # 6. Finally fall back to stderr after filtering JVM-only warnings
            if not error_parts and raw_error.strip():
                # Filter harmless JVM warnings
                jvm_noise = ["sun.misc.Unsafe", "WARNING: A terminally deprecated",
                             "WARNING: A restricted method", "WARNING: Please consider",
                             "--enable-native-access"]
                meaningful_stderr = []
                for line in raw_error.split("\n"):
                    if not any(noise in line for noise in jvm_noise):
                        meaningful_stderr.append(line)
                clean_stderr = "\n".join(meaningful_stderr).strip()
                if clean_stderr:
                    error_parts.append(clean_stderr[:2000])
                else:
                    # Only JVM noise remains; inspect other evidence
                    # Inspect the end of stdout for useful diagnostics
                    stdout_tail = raw_output.strip()[-500:] if raw_output.strip() else "(empty stdout)"
                    error_parts.append(f"No parse/violation error captured. stdout tail: {stdout_tail}")

            results["error_details"] = "\n".join(error_parts) if error_parts else "No detailed error captured."

        return {
            "status": status,
            "results": results,
            "raw_output": raw_output,
            "raw_error": raw_error,
        }

    except subprocess.TimeoutExpired:
        return {
            "status": "error",
            "output": (
                "Apalache verification timed out "
                f"(>{DEFAULT_RUN_TIMEOUT_SECONDS}s)."
            ),
        }
    except Exception as e:
        return {"status": "error", "output": f"Unexpected error during Apalache call: {e}."}


# Backward-compatible alias
def run_tlc_verification(tla_file_path: str) -> dict:
    """[DEPRECATED] Use run_apalache_verification"""
    return run_apalache_verification(tla_file_path)

# --- Model initialization and configurable transient-error retries ---

_TRANSIENT_HTTP_STATUSES = {408, 409, 425, 429, 500, 502, 503, 504}
_TRANSIENT_EXCEPTION_NAMES = {
    "APIConnectionError",
    "APITimeoutError",
    "DeadlineExceeded",
    "InternalServerError",
    "RateLimitError",
    "ResourceExhausted",
    "ServerError",
    "ServiceUnavailable",
}


def _exception_status_code(exc: BaseException) -> Optional[int]:
    status = getattr(exc, "status_code", None)
    if isinstance(status, int):
        return status
    response = getattr(exc, "response", None)
    response_status = getattr(response, "status_code", None)
    return response_status if isinstance(response_status, int) else None


def _is_transient_llm_error(exc: BaseException) -> bool:
    status = _exception_status_code(exc)
    if status is not None:
        return status in _TRANSIENT_HTTP_STATUSES
    if isinstance(exc, (ConnectionError, TimeoutError)):
        return True
    return any(
        transient_name in type(exc).__name__
        for transient_name in _TRANSIENT_EXCEPTION_NAMES
    )


def _retry_after_seconds(exc: BaseException) -> Optional[float]:
    response = getattr(exc, "response", None)
    headers = getattr(response, "headers", None)
    if not headers:
        return None
    value = headers.get("retry-after") or headers.get("Retry-After")
    try:
        return max(0.0, float(value))
    except (TypeError, ValueError):
        return None


def invoke_llm_with_retry(
    runnable: Any,
    model_input: Any,
    *,
    provider: str,
    config: Optional[RunnableConfig] = None,
) -> Any:
    max_attempts = max(1, int(os.getenv("LLM_MAX_ATTEMPTS", "1")))
    initial_delay = max(0.0, float(os.getenv("LLM_RETRY_INITIAL_SECONDS", "2")))
    max_delay = max(initial_delay, float(os.getenv("LLM_RETRY_MAX_SECONDS", "30")))

    for attempt in range(1, max_attempts + 1):
        try:
            return runnable.invoke(model_input, config=config)
        except Exception as exc:
            if not _is_transient_llm_error(exc):
                raise
            status = _exception_status_code(exc)
            if attempt >= max_attempts:
                print(
                    f"[LLM ERROR] provider={provider} remained unavailable after "
                    f"{max_attempts} attempts (status={status or type(exc).__name__}). "
                    "Retry later or configure another MODEL_PROVIDER."
                )
                raise

            exponential_delay = min(max_delay, initial_delay * (2 ** (attempt - 1)))
            delay = _retry_after_seconds(exc)
            delay = exponential_delay if delay is None else min(max_delay, delay)
            print(
                f"[LLM RETRY] provider={provider} transient="
                f"{status or type(exc).__name__} attempt={attempt}/{max_attempts}; "
                f"waiting {delay:g}s"
            )
            time.sleep(delay)

    raise RuntimeError("Unreachable LLM retry state")


class ResilientStructuredLLM:
    """Add observable transient-error retries to every structured model call.

    Some OpenAI-compatible endpoints do not implement the modern JSON-Schema
    ``response_format`` selected by newer LangChain defaults.  Use their
    broadly-supported tool/function calling interface instead.
    """

    def __init__(
        self,
        model: Any,
        provider: str,
        structured_output_method: Optional[str] = None,
    ):
        self._model = model
        self._provider = provider
        self._structured_output_method = structured_output_method

    def with_structured_output(self, *args: Any, **kwargs: Any) -> RunnableLambda:
        if self._structured_output_method and "method" not in kwargs:
            kwargs = {**kwargs, "method": self._structured_output_method}
        structured_model = self._model.with_structured_output(*args, **kwargs)
        provider = self._provider

        def invoke(model_input: Any, config: Optional[RunnableConfig] = None) -> Any:
            return invoke_llm_with_retry(
                structured_model,
                model_input,
                provider=provider,
                config=config,
            )

        return RunnableLambda(invoke)


def initialize_llm():
    explicit_env_file = os.getenv("EMBEDDEV_ENV_FILE")
    if explicit_env_file:
        if not os.path.isfile(explicit_env_file):
            raise FileNotFoundError("EMBEDDEV_ENV_FILE is not an existing file")
        load_dotenv(explicit_env_file, override=False)
    fse_model_name = os.getenv("FSE_MODEL")
    provider = os.getenv("MODEL_PROVIDER", "openai").lower()
    llm_timeout = int(os.getenv("LLM_TIMEOUT", "900"))

    if fse_model_name:
        selected = resolve_fse_model(fse_model_name)
        print(
            f"--- Using FSE model: {selected.name} "
            f"(openai_compatible, Timeout: {llm_timeout}s) ---"
        )
        if selected.name == "deepseek-v4-pro":
            # Keep the same provider-specific client as the original checking
            # project. ChatDeepSeek serializes its structured tool calls in the
            # form expected by DeepSeek V4-Pro.
            model = ChatDeepSeek(
                temperature=selected.temperature,
                model=selected.model,
                api_key=selected.api_key,
                api_base=selected.base_url,
                timeout=llm_timeout,
                max_retries=0,
                max_tokens=selected.max_tokens,
                extra_body={"thinking": {"type": "disabled"}},
            )
            structured_output_method = None
        else:
            model = ChatOpenAI(
                temperature=selected.temperature,
                model_name=selected.model,
                api_key=selected.api_key,
                base_url=selected.base_url,
                timeout=llm_timeout,
                max_retries=0,
                max_tokens=selected.max_tokens,
            )
            # The configured MiniMax and GPT relays do not accept every
            # JSON-Schema generated by recent LangChain versions. Their
            # function-calling endpoints carry the same schema in a broadly
            # compatible tool definition.
            structured_output_method = (
                "function_calling"
                if selected.name in {"MiniMax-M3", "gpt-5.5", "gemini-3-flash-preview"}
                else None
            )
        return ResilientStructuredLLM(
            model,
            selected.name,
            structured_output_method=structured_output_method,
        )

    print(f"--- Using LLM Provider: {provider} (Timeout: {llm_timeout}s) ---")

    if provider == "openai":
        model_name = os.getenv("OPENAI_MODEL_NAME", "gpt-4o-mini")
        model = ChatOpenAI(
            temperature=0,
            model_name=model_name,
            timeout=llm_timeout,
            max_retries=0,
        )
        
    elif provider == "gemini":
        api_key = os.getenv("GOOGLE_API_KEY") or os.getenv("GEMINI_API_KEY")
        model_name = os.getenv("GEMINI_MODEL_NAME", "gemini-2.5-flash-lite")
        model = ChatGoogleGenerativeAI(
            temperature=0,
            model=model_name,
            api_key=api_key,
            request_timeout=llm_timeout,
            retries=0,
            convert_system_message_to_human=True,
        )
        
    elif provider == "deepseek":
        api_key = os.getenv("DEEPSEEK_API_KEY")
        api_base = os.getenv("DEEPSEEK_API_BASE", "https://api.deepseek.com")
        model_name = os.getenv("DEEPSEEK_MODEL_NAME", "deepseek-v4-pro")
        extra_body={"thinking": {"type": "disabled"}}
        model = ChatDeepSeek(
            temperature=0,
            model=model_name,
            api_key=api_key,
            api_base=api_base,
            timeout=llm_timeout,
            max_retries=0,
            max_tokens=380000,
            extra_body=extra_body,
        )
    else:
        raise ValueError("Unsupported MODEL_PROVIDER")

    return ResilientStructuredLLM(model, provider)


class LazyLLM:
    """Create a provider client only when a model operation is actually invoked."""

    def __init__(self):
        self._client = None

    def with_structured_output(self, *args: Any, **kwargs: Any) -> RunnableLambda:
        def invoke(model_input: Any, config: Optional[RunnableConfig] = None) -> Any:
            if self._client is None:
                self._client = initialize_llm()
            return self._client.with_structured_output(*args, **kwargs).invoke(model_input, config=config)
        return RunnableLambda(invoke)


llm = LazyLLM()

def create_plantuml_diagram(content: str, output_path: str):
    output_dir = os.path.dirname(output_path)
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
    with open(output_path, 'w', encoding='utf-8') as f:
        f.write(content)
    print(f"Diagram saved to {output_path}")

def get_user_input(input_source: str) -> str:
    if os.path.exists(input_source):
        with open(input_source, 'r', encoding='utf-8') as f:
            return f.read()
    else:
        raise ValueError(f"Input file not found: {input_source}")
