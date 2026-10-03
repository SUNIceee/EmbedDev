# RQ3: Offline reproduction of the seven-task comparison

This subset reproduces the final RQ3 grouped-bar figure and its numeric table from fixed position scores. It does not run a model, compile or execute candidate code, rerun Host tests, or make new static judgments. It requires no credentials or network access.

## Run

Use Python 3.9 or later, from the release root:

```console
python scripts/reproduce_rq3.py --output outputs/rq3_table
```

The table-only command uses the Python standard library. It checks the bundled data hashes, reconstructs the 35 cells from 105 position records, verifies all locally bound P/F/U/N/A counts, and writes CSV, Markdown, JSON and a validation record.

To also produce the PDF and SVG grouped-bar figure, install the optional plotting dependency once in your environment and run:

```console
python -m pip install reportlab
python scripts/reproduce_rq3.py --plot --output outputs/rq3_figure
```

Installing ReportLab may require network access; reproduction itself is offline. The original figure and this export use ReportLab. The figure has seven task groups, five methods per group, 35 bars, 18 nonzero standard-deviation intervals, and two explicitly labelled zero-height bars. Outputs are written only to the selected directory. Choose a new output directory for another run; existing results are not overwritten. PDF binary hashes may vary with library versions or document metadata; the numeric table and plot geometry are the reproducibility targets.

## What is measured

The generation model is DeepSeek-V4-Pro. Methods are Direct-LLM, MetaGPT, ChatDev, StructGen, and EmbedDev. Each task/method cell retains three repeat positions. The position score is `100 * P / (P + F + U)`, excluding N/A. The plotted mean is the equal-weight mean of the three scores. Error bars are **sample standard deviations**, with divisor `n - 1 = 2`, not confidence intervals. This differs from the current RQ1 table's population SD over repeat blocks.

| Task | Review units | Number of units per candidate |
|---|---|---:|
| GPIO output | Whole original numbered requirements | 8 |
| Button debounce | Whole original numbered requirements | 9 |
| Balance control | Whole original numbered requirements | 14 |
| Motor tracking | Whole original numbered requirements | 27 |
| DiscoBot | Original project testcase reviews | 124 |
| OnStep | Original project testcase reviews | 90 |
| Crazyflie | Original project testcase reviews | 100 |

The first four tasks contain 58 distinct whole numbered requirements. The original three tasks contain 314 distinct testcase items. These are different review granularities: do not pool their denominators, compute a single overall score, or treat the horizontal task order as an equally spaced complexity scale. Compare methods within each task. Static support is not executable testcase success, ABI compatibility, a Host pass rate, or evidence of correct operation on physical hardware. Direct-LLM did not receive the supplied application API; the other methods did.

## The GPIO replacement score

The final displayed EmbedDev/GPIO cell is `[100, 100, 100]`, giving a mean of 100 and sample SD of 0. The first two scores have locally bound code and static-review records. The third score was supplied by the author after a reported rerun. The statement was: "No annotation is needed. I reran it myself; it is 100 now." This is an English translation of the original statement.

The third rerun's code and review were not supplied with the local evidence. Therefore its position record explicitly uses `author_reported_rerun_score`, with null candidate/review bindings and `local_verification_completed: false`. The original local no-code record and its zero contribution remain preserved as historical evidence. This plotting export reproduces the author-requested display; it does not invent item-level judgments, a Host result, or independent verification for that replacement.

The 105 plotted positions comprise 95 locally code-bound reviews, nine retained no-code positions, and this one author-reported replacement score. No-code positions contribute zero at aggregation time without fabricated per-item failures. Host failure does not automatically set a static score to zero. The separately recovered Motor artifacts retain their original generation status and internal-acceptance values in the position provenance.

## Files and traceability

- `data/rq3/display_cells.json`: the final 35 cells, including all three position values, fixed means and sample SDs.
- `data/rq3/position_scores.json`: the 105 position records, count-based scores, code/review hashes, historical generation states, and the explicit author-reported replacement.
- `data/rq3/protocol.json`: task order, review-unit counts, aggregation, and interpretation limits.
- `data/rq3/candidate_sources/`: all 59 available fixed candidates from the four new tasks, comprising 118 unmodified C/header files. `index.json` records packet provenance, exact original/exported hashes, retained generation states and the two saved-output recoveries. No candidate file was available for the locally recorded GPIO/EmbedDev repeat 3; its author-reported replacement does not create a source file here.
- `data/rq3/requirements/`: the four frozen whole-requirement checklists, containing the exact 58 numbered requirement texts in their original order, with source file hashes and line/byte references.
- `data/rq3/new_task_review_counts.json`: the 59 already-selected reviews' P/F/U/N/A counts and 862 retained item verdicts, plus the one explicit local no-code position. Numeric errata remain bound to the selected report and its preceding report; this export makes no new semantic judgment.
- `rq3_export_provenance.json`: SHA-256 bindings to the original display, the four-task result view, the three-task result view, and the source plotting code; hashes of the portable data files.
- `scripts/reproduce_rq3.py`: standard-library table recomputation and optional ReportLab plotting.

Original repository-relative evidence paths in the provenance are identifiers, not runtime dependencies. The four new tasks' candidate sources and compact item verdicts are included; this plotting subset does not include every original model response, full review rationale or Host report. Sources for the original three projects are indexed by the broader release evidence inventory. The subset is sufficient to reproduce the published numeric display, not to repeat the original generation or substitute a fresh independent code assessment. No original evidence file is changed during export or reproduction.

Task inputs are provided elsewhere in the release under `examples/<task>/`: `RE_req.txt`, `RE_device_interface.txt`, `RE_api.txt`, and `project.json`. The four new tasks' English input files are byte-preserved from the experimental inputs. Where the three original tasks needed English translations, `input_translation_provenance.json` records original and translated hashes. Those translated copies are suitable for a new run, but are not byte-identical replays of the original prompts. Input files are not required to redraw the stored results.
