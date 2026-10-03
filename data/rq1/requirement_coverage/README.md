# Requirement-coverage evidence

This directory contains 135 selected-position review records and the three frozen requirement checklists. There are 108 delivered candidates with 12,148 retained leaf judgments and 27 no-code positions. A no-code position contributes zero to the planned-position mean; its review does not fabricate per-leaf failures.

The current manuscript defines requirement coverage as `(fully_covered + partially_covered) / static_denominator`. Full and partial judgments remain separate. Partial items count as implementation evidence, not as proof that every requirement condition is satisfied. The preclassified static denominators are Crazyflie 141, OnStep 89, and DiscoBot 88. Hardware/execution and unresolved-contract leaves remain `unassessed` and are excluded consistently.

The three-project repeat-block means and population standard deviations are reconstructed by `scripts/reproduce_tables.py` from the item verdicts. The script checks unique item coverage, fixed classifications and denominators, candidate-file hashes, 135 identity matches, and all 15 method/model aggregates against `data/rq1/requirement_coverage.json`.

## Original evidence and later confirmation

`reviews/` preserves the original per-leaf verdicts, rationales, and initial-review metadata. Historical fields such as `rate`, `rate_percent`, or `coverage_percent_for_89_static_leaves` may describe the earlier **full-only** metric. They are retained as historical record fields and are not used as the current full-plus-partial score. Current arithmetic appears in `data/rq1/requirement_coverage.json` and is recomputed from item verdicts.

The selected ChatDev/DeepSeek/Crazyflie repeat-2 replacement is bound to the new 12-file candidate. Its later author confirmation is recorded separately in `data/human_review_attestation.json`. The preserved initial review flags are not rewritten, and this export does not create new human-review claims for any of the other 134 positions. The confirmation does not constitute a new repaired-code execution.

Original Chinese checklists, requirement text, and review rationales are retained in their original language to avoid silently changing judgments through translation. Workspace paths are anonymized in exported JSON. `source_requirements/` contains the original requirement texts; `checklists/` binds each split leaf to those texts. Original source and distributed-copy hashes are recorded separately.

Each review's `release_export.candidate_bindings` maps its original candidate to the actual packaged source. Existing distributed copies are reused when already indexed; otherwise exact source bytes are under `evidence/coverage_candidates/`. `source_bindings.json` records additional source-index bindings for the release builder. Initial review line numbers refer to original candidate files; any historical comment translation is explicitly identified by the original/distributed hashes and transformation field.

These saved records support offline reconstruction and inspection. They do not represent a new static assessment, model invocation, compiler run, or hardware experiment.
