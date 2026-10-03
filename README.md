# EmbedDev: Replication Package

This package provides an English-language implementation of EmbedDev and offline reconstruction of the saved RQ1--RQ4 results. This release synchronizes RQ1 and the six-task RQ3 view with the included manuscript sections. EmbedDev constructs linked state, interface, and behavior models before generating and repairing embedded C software.

## Start here: reproduce the reported numbers

Python 3.11 or later is required. The following commands use the standard library, saved observations, and local files only. They do not read credentials, contact a model, or execute generated C code.

```console
python -B scripts/verify_package.py
python -B scripts/reproduce_tables.py
python -B scripts/reproduce_rq3.py
```

The table script writes to `results/`; the RQ3 script writes to `outputs/rq3/`. For another RQ3 run, choose a fresh directory with `--output`. RQ1 has 135 selected positions and 15 method--model rows; RQ2 has 54 DeepSeek positions and six conditions; RQ3 displays six tasks, five methods, and 90 positions; RQ4 classifies 162 positions. These scopes overlap and must not be added together as independent experiments. Historical seven-task records remain available for provenance and are not included in the current RQ3 display.

For the six-task line chart, follow the optional plotting command in [docs/RQ3.md](docs/RQ3.md). The paper's RQ3 figure uses static support, not whole-Host success. EmbedDev is red; no uncertainty intervals are drawn.

## Additional GPIO evidence

The [GPIO output evidence bundle](evidence/gpio_output_deepseek_20261003/README.md) contains one documented EmbedDev/DeepSeek-V4-Pro run: unchanged generated C/header files, the original requirements and Host suite, the compiler diagnostic, and all eight static-review judgments. Static requirement support is **8/8 (100.00%)** under the recorded build-obstacle policy. Independent compilation **failed** because `NULL` was undeclared; **0 of 10 Host scenarios executed**. The static percentage is not an executable pass rate, and human review of this new assessment has not been attested.

This bundle is additional evidence. It does not silently replace the author-reported historical GPIO position, change the saved RQ1--RQ4 statistics, or establish the earlier Host-success claim. See [publication notes](docs/GPIO_EVIDENCE_PUBLICATION.md).

The repository preserves file bytes through `.gitattributes`. The package verifier ignores only root Git metadata, while continuing to check all distributed file hashes. Use `python -B` and place local generated results under `results/` or `outputs/`.

## What is included

| Directory | Contents |
| --- | --- |
| `fse/checking/` | State, interface, behavior, consistency-validation, code-generation, and repair implementation |
| `examples/` | Seven task input bundles with requirements, device/environment contracts, supplied APIs, and project configurations |
| `docs/API_GENERATION_PROMPT.md` | English prompt for the separate API-preparation branch |
| `data/rq1/` | Per-position observations, static reviews, requirement-leaf coverage records, and expected repeat-block statistics for RQ1 |
| `data/rq2/` | DeepSeek component-study observations, including failed and incomplete positions |
| `data/rq3/` | Six-task static-support display and per-point provenance, with historical seven-task evidence retained |
| `data/rq4/` | Mutually exclusive failure endpoints and saved mechanism observations |
| `evidence/` | Saved candidate sources and Host-report copies indexed by original and distributed hashes |
| `scripts/` | Offline verification and result reconstruction |
| `baselines/` | Baseline adaptation source and pinned upstream versions for inspection |
| `third_party/` | Available upstream license notices |
| `paper/` | Current RQ1/RQ3 LaTeX section fragments, matching figure, and a Data Availability template |

The implementation, documentation, task inputs, historical requirement text, review explanations, and candidate comments are provided in English. The 127 files containing remaining Chinese text in the preceding archive have English renderings here; original verdicts, scores, IDs, and executable code are preserved. See [docs/ENGLISH_RELEASE.md](docs/ENGLISH_RELEASE.md). Original working directories, credentials, provider sessions, local environment files, compiler binaries, and private account configuration are excluded.

## Two different reproduction tasks

**Result reconstruction** recomputes tables and the RQ3 chart from saved per-position observations. The offline commands above check the arithmetic and saved-file bindings. They do not independently validate an LLM judgment or demonstrate a new model run.

**New generation** runs the English release implementation on a task using the reader's own configured model and toolchain. The historical study used multiple source revisions and some non-English inputs. English translations and portability changes are recorded in the provenance files; this release is not a byte-identical replay of the historical prompts. Model-service changes and nondeterminism also prevent a promise of identical regenerated programs.

## Configure and inspect a task

Install the generation dependencies in a new virtual environment:

```console
python -m venv .venv
```

Activate the environment with `.venv\Scripts\Activate.ps1` on Windows PowerShell or `source .venv/bin/activate` on Linux/macOS, then install dependencies:

```console
python -m pip install -r requirements.txt
```

The pipeline entry point is `fse/checking/pipeline.py`. Consult its `--help` and [docs/CODE_CHANGES.md](docs/CODE_CHANGES.md) for the checked inspection, API-preparation, and dry-run commands. Task files use relative paths. The examples explicitly set the study's four design rounds; the release exposes a three-repair code budget.

```console
python fse/checking/pipeline.py inspect --input-dir examples/gpio_output
python fse/checking/pipeline.py run --input-dir examples/gpio_output --model deepseek-v4-pro --dry-run
python fse/checking/pipeline.py prepare-api --input-dir examples/gpio_output --output-dir outputs/api_candidate
python fse/checking/scripts/offline_smoke.py
```

These commands inspect inputs or prepare requests without calling a model. Use a fresh output directory for API preparation. See the model configuration and tool prerequisites below before removing `--dry-run` from a generation command.

Actual generation requires a configured model, an API key supplied through environment variables, a C compiler, and the documented PlantUML/Java requirements. No secret is included in the model example. Read the dry-run output and model-service costs before executing a new run. Offline result reconstruction requires none of these services.

The public endpoint examples are in [fse/fse_models.json](fse/fse_models.json):

| CLI configuration label | Model-ID environment variable | API-key environment variable |
| --- | --- | --- |
| `openai` | `OPENAI_MODEL` | `OPENAI_API_KEY` |
| `deepseek-v4-pro` | `DEEPSEEK_MODEL` | `DEEPSEEK_API_KEY` |
| `gemini-3-flash-preview` | `GEMINI_MODEL` | `GEMINI_API_KEY` |

The CLI label selects a configuration; the model-ID environment variable determines the model actually requested. Set it to an ID available to your account. A label is not evidence that a service currently provides a particular model. The [runtime dependencies](docs/CODE_CHANGES.md#runtime-dependencies) specify the fixed PlantUML version/hash, Java options, and compiler requirements. The release does not automatically load an author's environment file.

The API-preparation branch emits a **candidate** contract for review. It does not silently replace a frozen supplied API. Requirements and device/environment facts are generation inputs. Reference implementations and final evaluation cases are separate evaluation materials and must not be supplied as model feedback in a replication of the recorded protocol.

## Experimental scope and metrics

- **RQ1:** Crazyflie, OnStep, and DiscoBot; five methods, three models, and three repeat positions per project. Direct-LLM has no supplied API. The other main-comparison methods receive the shared API. Host, gated static support, assisted static support, and requirement coverage remain separate measures. Requirement coverage credits both fully and partially covered leaves, over the fixed static denominators 141/89/88. It measures breadth of at least partial implementation, not complete requirement satisfaction.
- **RQ2:** DeepSeek only; six workflow/component conditions with nine positions each. Direct generation with repair is distinct from RQ1's Direct-LLM. Missing code, compile failures, integration failures, and executed assertion failures remain separate.
- **RQ3:** Two GPIO tasks, one example-derived balance-control module, and the three application tasks. This is six evaluation tasks, not six independent upstream repositories or an equally spaced complexity scale. The first three tasks use 8/9/14 requirement items; the last three use 124/90/100 static testcases. Motor tracking is retained only as historical evidence. No common pooled denominator is constructed.
- **RQ4:** Six conditions, 27 positions each, classified into no code, candidate build failure, harness/link failure, executed assertion failure, or Host-all-pass. Separate mechanism observations are not additional samples in that table.

RQ1 uses **population SD** over three repeat-block means, each equally averaging the three projects. RQ3's supplementary numerical outputs use **population SD** over three positions within each task--method cell; the current line chart shows means only. These are descriptive dispersions, not confidence intervals.

For static support, uncertain items count in the denominator and inapplicable items do not. A position with no code contributes zero to the planned-position average; it does not generate fictitious per-requirement judgments. Compilation failure alone does not imply zero static support. These saved static observations are not executable test results. The author confirmed manual review of the current ChatDev judgments; `data/human_review_attestation.json` records that statement separately from the initial review fields. Offline reconstruction does not independently observe or repeat a human review.

The ChatDev/DeepSeek row reconstructs to Host **88.89 +/- 15.71**, gated support **70.39 +/- 8.28**, assisted static support **75.50 +/- 1.26**, and requirement coverage **91.35 +/- 0.31**. The current Crazyflie/ChatDev RQ3 point is **57.28%**, with an EmbedDev advantage of **10.93 percentage points** calculated from unrounded values. The selected ChatDev code still fails Host compilation. See [docs/STATIC_REVIEW.md](docs/STATIC_REVIEW.md) for the evidence and scoring boundaries.

## Provenance and limits

`data/source_provenance.json`, `code_release_provenance.json`, `input_translation_provenance.json`, and `rq3_export_provenance.json` bind the release to its source materials. `evidence/source_index.json` maps archived source paths to package files. Source paths in these records describe the historical workspace; use the corresponding `path` entry to access the distributed copy.

Most archived candidate source files are byte-identical. Chinese comments in the remaining candidate file have been translated with unchanged noncomment C text and source line counts. Review explanations, requirement checklists, and source requirement documents are also translated. Original source digests remain separate from the English distributed digests. These English JSON copies retain verdicts and anonymized paths, but are not byte-identical original reports. File and text-level translation bindings are in `evidence_translation_provenance.json`.

The RQ3 GPIO/EmbedDev third 100% value is an author-reported rerun. Its original rerun code and review artifact were not supplied to this package, and the data explicitly marks that binding as unavailable. The displayed point can be reconstructed; the missing underlying rerun cannot be independently re-executed from this archive. The other recorded values are not silently replaced.

The package does not include every historical request/response trace or the original non-English prompt corpus. Earlier local records remain unchanged. Baseline adapters and their upstream revisions are provided for inspection; they are not advertised as a tested one-command installation of every upstream framework. Saved Host reports do not amount to physical-board or real-time validation.

## Availability and attribution

The manuscript statement in `paper/data_availability.tex` is a template for a verified anonymous artifact URL. All documentation links within this package are relative. `RELEASE_VALIDATION.json` records the earlier English translation validation; the additional GPIO evidence and Git-distribution checks are described in [publication notes](docs/GPIO_EVIDENCE_PUBLICATION.md).

See [docs/THIRD_PARTY.md](docs/THIRD_PARTY.md) for upstream notices and license boundaries. See [docs/RELEASE_CHANGES.md](docs/RELEASE_CHANGES.md) for differences from the earlier September review package.
