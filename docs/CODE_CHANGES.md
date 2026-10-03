# English implementation release and provenance

This directory is a runnable English development release of **EmbedDev**. It is
derived from the repository's current `fse/checking` modules, not reconstructed
from the September historical experiment archive. `code_release_provenance.json`
records the original and distributed SHA-256 for every copied Python module.
The original files were not modified. Historical source, inputs, model replies,
candidate bytes and reported experiment scores are not rewritten by this release.

English schema descriptions and prompt translations are a new treatment. Online
runs using this release must not be represented as byte-identical replays of the
historical prompt language. The API candidate prompt is an English adaptation of
the maintained preparation prompt; its substantive contract boundaries and
review obligations are retained, but the text and structured-output packaging
are new. The release has not been used here to call a model.

## Retained implementation

- State, interface and behavior schema definitions, generators and repairers.
- Deterministic model/API/PlantUML validation and severity-based repair routing.
- Production-code generation, syntax compilation and semantic fidelity audit.
- Candidate snapshots, explicit verification/publication status and preservation
  of prior trusted outputs when a later repair does not qualify for publication.
- Optional development helpers, including dataflow analysis and board code
  profiles, where present in the current source. Their inclusion is not evidence
  that every helper was enabled in the historical evaluation.

The current development verifier audits canonical design artifacts as well as
source inputs. Historical matched-experiment workers used a common requirements,
API and environment audit across conditions and their own candidate-selection
rules. Those historical workers are not silently replaced by this development
entry point. The offline result verifier and provenance identify saved evidence
separately from fresh generation.

## Release changes

1. Chinese comments, docstrings, diagnostic wording and schema/prompt descriptions
   were translated to English. Literal Unicode ranges in the state-diagram text
   recognizer were replaced by equivalent `\u4e00-\u9fff` escapes.
2. API preparation now loads the independent English
   `docs/API_GENERATION_PROMPT.md`. The required `{{FIELD_NAME}}` set is validated
   before one-pass substitution. Supplied source text containing such strings is
   preserved. An unknown or missing template field is an error. Prepared output
   remains a candidate; the tool neither freezes an API nor starts design.
3. Model clients initialize lazily at the first actual invocation. Importing
   agents, showing help, inspecting inputs and preparing API requests do not
   create clients or read credential files. Automatic parent-directory `.env`
   discovery was removed. To use a file deliberately, set `EMBEDDEV_ENV_FILE`;
   otherwise provide environment variables directly.
4. `fse/fse_models.json` contains fresh examples using public official endpoint
   roots and environment-variable names. It contains no original model-config
   contents, keys or private relay addresses. Set `OPENAI_MODEL`, `DEEPSEEK_MODEL`
   or `GEMINI_MODEL` explicitly to the model ID available to your account. The
   CLI labels select client/configuration behavior and do not certify model
   availability. Override the config path with `EMBEDDEV_MODELS_FILE` if needed.
5. Release defaults are four design iterations, three code repairs after initial
   generation and 900 seconds per model call. The development source previously
   defaulted to eight design iterations and four code repairs. Use the project
   configuration and `run --code-repairs` to specify limits. Provider-library
   `max_retries` remains zero, and the wrapper defaults to one transport attempt
   (`LLM_MAX_ATTEMPTS=1`, previously six). These controls are not a monetary or
   aggregate-token cap; this CLI is not the historical accounting scheduler.
6. Optional context retrieval and persistent error-notebook updates are explicitly
   disabled by default. Set `EMBEDDEV_ENABLE_CONTEXT=1` only for a separately
   identified development experiment. No author notebook or domain corpus was
   copied. Optional domain files can be supplied through `EMBEDDEV_DOMAIN_DIR`;
   the built-in PlantUML examples are English. Enabled notebook writes are local
   to this new installation, never the original experiment evidence.
7. The CLI defaults to `examples/crazyflie`; supplied `--project-config` paths are
   forwarded to child stages. Compiler resolution uses the configured executable
   or PATH rather than an author's machine-specific LLVM path. Optional Apalache
   locations use `APALACHE_BIN`, `APALACHE_JAR` and `APALACHE_JAVA_HOME` instead of
   a hard-coded system Java directory. These legacy helpers do not establish a
   formal-verification result for the main workflow.
8. PlantUML lookup is confined to this release or an explicit configured path;
   it no longer walks outside the package to borrow an author-workspace tool.
   The official parser version and hash validation are unchanged.
9. The legacy stand-alone API helper uses the configured project name instead of
   hard-coded Crazyflie wording. The maintained entry point is `prepare-api`.
   The optional documentation generator was translated and explicitly instructed
   not to invent a formal-verification result when none was supplied.

## Runtime dependencies

Python **3.11+** is required (`typing.NotRequired` is used). `requirements.txt`
pins versions present in the offline validation environment. No dependency was
installed, no compiler was downloaded and no provider request was issued while
preparing this release. Fresh installation and online service compatibility are
separate checks.

The design phase requires a Java runtime and **PlantUML 1.2026.8**. Supply the
official `plantuml-mit-1.2026.8.jar` via `PLANTUML_JAR_PATH`, or place it at
`.tools/plantuml-1.2026.8/plantuml-mit-1.2026.8.jar` under this release. Its required
SHA-256 is `3629c9cd017c7f73e6450396eea0040216c7e1eef8473ce33cc1aad469dab2f9`.
Use Java on PATH, `JAVA_HOME`, or `PLANTUML_JAVA`. Graphviz is not needed for the
syntax check. Missing or mismatched tools fail explicitly; validators are not
bypassed. The code syntax check also requires the compiler named in project.json
(normally GCC/Clang for C). Host evaluation and board-level execution are
separate from these local checks.

## Offline commands

Run these from the extracted release directory after installing requirements:

```console
python fse/checking/pipeline.py --help
python fse/checking/pipeline.py inspect --input-dir examples/crazyflie
python fse/checking/pipeline.py run --input-dir examples/crazyflie --model openai --dry-run
python fse/checking/pipeline.py prepare-api --input-dir examples/crazyflie --output-dir outputs/api_candidate
python fse/checking/scripts/offline_smoke.py
```

`prepare-api` requires a new output directory and does not call a model unless
`--generate --model <label>` is supplied. `run --dry-run` prints intended stage
commands and performs no generation. The full generation command is the same
`run` command with `--dry-run` removed, after explicit model/key/tool setup and
your own resource policy. Agent scripts do not accept or need saved author
credentials.

The smoke check parses Python, constructs all three model schemas, imports eight
active agents while model initialization is forbidden, checks design defaults,
checks English API-template substitution and no-overwrite behavior, and exercises
the four offline CLI modes from an unrelated working directory. It also checks
that retrieval and notebook updates are disabled by default. It does not invoke
a model, compiler, Host test, hardware toolchain or diagram parser. Therefore a
passing smoke check demonstrates package wiring, not online generation quality.
