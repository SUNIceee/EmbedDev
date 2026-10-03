"""Local official PlantUML syntax checking; no renderer, server or LLM calls."""

from __future__ import annotations

from functools import lru_cache
import hashlib
import os
from pathlib import Path
import re
import shutil
import subprocess
import tempfile
import zipfile


PLANTUML_VERSION = "1.2026.8"
PLANTUML_SHA256 = "3629c9cd017c7f73e6450396eea0040216c7e1eef8473ce33cc1aad469dab2f9"
JAR_RELATIVE = Path(".tools/plantuml-1.2026.8/plantuml-mit-1.2026.8.jar")
TIMEOUT_SECONDS = 30
MAX_SOURCE_BYTES = 2 * 1024 * 1024
VALIDATOR_REVISION = "official-plantuml-syntax-v1"
PACKAGE_ROOT = Path(__file__).resolve().parents[3]


def _java_binary() -> str:
    explicit = os.environ.get("PLANTUML_JAVA")
    if explicit:
        found = shutil.which(explicit)
        if not found:
            raise FileNotFoundError("PLANTUML_JAVA does not identify an executable Java runtime.")
        return found
    java_home = os.environ.get("JAVA_HOME")
    if java_home:
        candidate = Path(java_home) / "bin" / ("java.exe" if os.name == "nt" else "java")
        if candidate.is_file():
            return str(candidate)
    found = shutil.which("java")
    if found:
        return found
    # The repository already uses this portable runtime; do not change PATH.
    candidate = PACKAGE_ROOT / ".tools/jdk-17.0.19+10-jre/bin/java.exe"
    if candidate.is_file():
        return str(candidate)
    raise FileNotFoundError("Java is unavailable. Set PLANTUML_JAVA or JAVA_HOME.")


def _jar_path() -> Path:
    explicit = os.environ.get("PLANTUML_JAR_PATH")
    if explicit:
        candidate = Path(explicit).expanduser().resolve()
        if candidate.is_file():
            return candidate
        raise FileNotFoundError("PLANTUML_JAR_PATH does not identify a JAR file.")
    candidate = PACKAGE_ROOT / JAR_RELATIVE
    if candidate.is_file():
        return candidate
    raise FileNotFoundError("PlantUML JAR is unavailable. Provide the pinned release using PLANTUML_JAR_PATH; see docs/CODE_CHANGES.md.")


@lru_cache(maxsize=8)
def _jar_metadata(path: str, size: int, mtime_ns: int) -> dict:
    jar = Path(path)
    digest = hashlib.sha256(jar.read_bytes()).hexdigest()
    # Pin both installed and explicitly selected copies to the tested release.
    if digest != PLANTUML_SHA256:
        raise ValueError("PlantUML JAR SHA256 differs from the pinned official release.")
    with zipfile.ZipFile(jar) as archive:
        manifest = archive.read("META-INF/MANIFEST.MF").decode("utf-8")
    match = re.search(r"(?m)^Implementation-Version:\s*([^\r\n]+)", manifest)
    if not match or match.group(1).strip() != PLANTUML_VERSION:
        raise ValueError("PlantUML JAR version differs from the pinned release.")
    return {"version": PLANTUML_VERSION, "jar_sha256": digest}


def validator_identity() -> dict:
    """Small stable cache key, also invalidating old regex-only reports."""
    return {"revision": VALIDATOR_REVISION, "engine": "PlantUML",
            "version": PLANTUML_VERSION, "jar_sha256": PLANTUML_SHA256}


def _child_environment() -> dict:
    # Do not expose provider keys or Java option injection to diagram preprocessing.
    allowed = {"SYSTEMROOT", "WINDIR", "PATH", "TEMP", "TMP", "TMPDIR",
               "COMSPEC", "LANG", "LC_ALL", "LC_CTYPE"}
    environment = {key: value for key, value in os.environ.items() if key.upper() in allowed}
    environment["PLANTUML_SECURITY_PROFILE"] = "SANDBOX"
    return environment


def check_plantuml(text: str, expected_kind: str) -> dict:
    """Return JSON-serializable diagnostics without changing the supplied source.

    Official ``-syntax`` parses stdin and reports type or ERROR + zero-based
    source offset. It performs no rendering and requires no Graphviz. The extra
    envelope/type checks enforce this pipeline's single-artifact contract.
    """
    result = {**validator_identity(), "status": "validator_failure",
              "expected_kind": expected_kind, "returncode": None,
              "line": None, "diagnostic": "", "security_profile": "SANDBOX"}
    if expected_kind not in {"STATE", "SEQUENCE"}:
        result["diagnostic"] = "Unsupported expected diagram kind."
        return result
    if not isinstance(text, str):
        result.update(status="artifact_error", diagnostic="PlantUML source must be text.")
        return result
    if len(text.encode("utf-8")) > MAX_SOURCE_BYTES:
        result.update(status="artifact_error", diagnostic="PlantUML source exceeds the 2 MiB input limit.")
        return result
    lines = text.splitlines()
    starts = [index for index, line in enumerate(lines)
              if re.match(r"^\s*\ufeff?@startuml(?:\s|$)", line)]
    ends = [index for index, line in enumerate(lines)
            if re.match(r"^\s*@enduml\s*$", line)]
    if len(starts) != 1 or len(ends) != 1 or ends[0] <= starts[0]:
        result.update(status="artifact_error", diagnostic=(
            "Exactly one complete @startuml ... @enduml diagram is required."))
        return result
    try:
        java = _java_binary()
        jar = _jar_path()
        stat = jar.stat()
        result.update(_jar_metadata(str(jar), stat.st_size, stat.st_mtime_ns))
        command = [java, "-Xmx512m", "-Dfile.encoding=UTF-8", "-Djava.awt.headless=true",
                   "-DPLANTUML_SECURITY_PROFILE=SANDBOX", "-jar", str(jar),
                   "-charset", "UTF-8", "-syntax"]
        with tempfile.TemporaryDirectory(prefix="embeddev-plantuml-") as directory:
            # Binary pipes preserve CRLF verbatim on Windows. Text mode would
            # turn existing CRLF into CRCRLF and shift official error positions.
            process = subprocess.run(command, input=text.encode("utf-8"), capture_output=True,
                                     timeout=TIMEOUT_SECONDS,
                                     cwd=directory, env=_child_environment(),
                                     creationflags=(subprocess.CREATE_NO_WINDOW if os.name == "nt" else 0))
        result["returncode"] = process.returncode
        stdout = process.stdout.decode("utf-8", errors="replace")
        stderr = process.stderr.decode("utf-8", errors="replace")
        result["stdout"] = stdout.strip()[:8000]
        result["stderr"] = stderr.strip()[:2000]
        output = stdout.strip().splitlines()
        if output and output[0] == "ERROR" and process.returncode == 200:
            if len(output) < 3 or not output[1].isdigit():
                result["diagnostic"] = "PlantUML returned a malformed error report."
                return result
            # -syntax strips leading material before @startuml; restore its offset.
            result.update(status="syntax_error", line=starts[0] + int(output[1]) + 1,
                          diagnostic="\n".join(output[2:])[:4000])
        elif process.returncode == 0 and len(output) == 2 and output[1].startswith("("):
            result["diagram_kind"] = output[0]
            if output[0] == expected_kind:
                result.update(status="valid", diagnostic="Official PlantUML syntax check passed.")
            else:
                result.update(status="artifact_error", diagnostic=(
                    f"Expected a {expected_kind} diagram; PlantUML parsed {output[0]}."))
        else:
            result["diagnostic"] = (
                "PlantUML did not return a recognized syntax report: "
                + (result["stderr"] or result["stdout"] or f"exit {process.returncode}")
            )[:4000]
    except subprocess.TimeoutExpired:
        result["diagnostic"] = f"PlantUML exceeded the {TIMEOUT_SECONDS}s timeout."
    except (OSError, ValueError, zipfile.BadZipFile, KeyError) as exc:
        result["diagnostic"] = f"PlantUML infrastructure error: {exc}"
    return result
