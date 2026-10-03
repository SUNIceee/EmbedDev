# RQ3: Offline reproduction of the six-task comparison

This release reproduces the current RQ3 line chart and numeric table from fixed position scores. It performs no model requests, source compilation, Host tests, or new static judgments. No credentials or network access are required.

## Run

Use Python 3.11 or later from the release root:

```console
python scripts/reproduce_rq3.py --output outputs/rq3_table
```

The table-only command uses the Python standard library. It verifies bundled-file hashes, reconstructs 30 means from 90 positions, checks saved P/F/U/N/A counts, and cross-checks the 45 application-task positions against RQ1. Choose a fresh output directory for each run; existing result files are not overwritten.

For the PDF and SVG line chart, install the optional plotting dependency in your environment and run:

```console
python -m pip install -r requirements-plot.txt
python scripts/reproduce_rq3.py --plot --output outputs/rq3_figure
```

Dependency installation may require network access; reconstruction itself remains offline. The chart contains six tasks, five methods, 30 mean points, and 25 connecting segments. EmbedDev is red. There are no uncertainty bounds, error bars, or bottom explanatory footer. PDF bytes can depend on library versions; the numeric values and plot geometry are the primary reproduction targets.

## Scope and statistic

| Task | Review units | Units per candidate |
|---|---|---:|
| GPIO output | Whole original numbered requirements | 8 |
| Button debounce | Whole original numbered requirements | 9 |
| Balance control | Whole original numbered requirements | 14 |
| DiscoBot | Original project testcase reviews | 124 |
| OnStep | Original project testcase reviews | 90 |
| Crazyflie | Original project testcase reviews | 100 |

The generation model is DeepSeek-V4-Pro. Methods are Direct-LLM, MetaGPT, ChatDev, StructGen, and EmbedDev. Direct-LLM did not receive the supplied application API; the other methods did.

Each position score is `100 * P / (P + F + U)`, excluding N/A. Each plotted point equally averages three planned positions. The exported numeric table additionally reports **population SD**, with divisor 3, as descriptive dispersion; the chart does not display it. RQ1 also uses population SD, but its three blocks each average the three application projects.

The smaller tasks contain 31 distinct whole requirements, while the original applications contain 314 testcase items. These review granularities differ. Compare methods within a task; do not pool their item denominators or treat task ordering as an equally spaced complexity scale. Static support does not establish executable testcase success or hardware correctness.

## Selected replacements and provenance

Crazyflie / ChatDev / DeepSeek repeat 2 now uses the selected rerun artifact. Its twelve source files were extracted losslessly from the response without source repair. Its fixed static review contains 46 supported, 47 failed, and 7 uncertain cases, giving 46.00%. The original Host result remains compilation-failing. ChatDev's three Crazyflie scores are 66.6666666667%, 46.00%, and 59.1836734694%, giving 57.2834467120%. The corresponding RQ1 application aggregate is 75.4983054154%, with population SD 1.2551110700 percentage points. Earlier failed attempts remain historical evidence.

The author has attested that the replacement judgments were manually reviewed. This statement is recorded in `data/human_review_attestation.json`; the offline scripts do not independently witness that review. Historical initial-review fields remain preserved and are distinguished from the later attestation.

The third EmbedDev / GPIO output score remains the author's reported 100% rerun. Its candidate and review were not supplied with the local evidence. That position therefore retains null bindings and `local_verification_completed: false`. Reconstructing the point does not supply the missing artifact or fabricate item-level judgments.

The current 90 positions comprise 81 code-bound static reviews, eight no-code positions, and this one author-reported GPIO score. No-code positions contribute zero only when aggregating the planned positions.

## Additional code-bound GPIO observation

A [separate GPIO evidence bundle](../evidence/gpio_output_deepseek_20261003/README.md) now provides complete candidate code, an eight-item static review, and the independent compiler diagnostic for one new EmbedDev/DeepSeek-V4-Pro attempt. Its static support is 8/8 (100.00%); its independent Host compilation failed and none of the ten scenarios executed. This does not establish the historical author-reported Host result. The saved 90-position data and current chart remain unchanged; this additional observation is not silently pooled into them or treated as a seventh task. The new static assessment has no human-review attestation.

## Files

- `data/rq3/display_cells.json`: 30 cells, their three position scores, means, and population SD.
- `data/rq3/position_scores.json`: 90 positions with review/source bindings and the explicit replacements.
- `data/rq3/protocol.json`: task order, review-unit counts, aggregation, and plotting definitions.
- `data/rq3/candidate_sources/index.json`: the 44 available smaller-task candidates and 88 original C/header files. The original Motor tracking source files remain archived but are outside this current index and figure.
- `data/rq3/requirements/`: the three active frozen checklists contain 31 items. The retained Motor tracking checklist is historical and excluded from current reconstruction.
- `data/rq3/new_task_review_counts.json`: 457 retained item verdicts for 44 smaller-task candidates, plus one explicit local no-code position.
- `data/rq1/chatdev_crazyflie_r02/static_review.json` and `evidence/current_chatdev_crazyflie_r02/raw/`: the selected replacement's 100-case review and twelve unchanged source files.
- `rq3_export_provenance.json`: hashes of the portable data, source files, review, attestation, and reconstruction scripts.
- `history/rq3_seven_task_20261002/`: preserved earlier seven-task grouped-bar export. It is historical and is not the current manuscript result.

Original repository-relative paths in provenance are identifiers, not runtime dependencies. The distributed `path` entries resolve within this package. Static-review rationale is evidence for a recorded judgment, not an independent validation performed by these reconstruction scripts.
