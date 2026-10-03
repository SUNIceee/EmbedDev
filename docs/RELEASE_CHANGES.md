# Release changes

## Complete English edition

The remaining 127 Chinese-bearing evidence and comment files now have English renderings. All judgments, numerical results, IDs, original hashes, and source line counts remain unchanged. Distributed hashes are refreshed. See [ENGLISH_RELEASE.md](ENGLISH_RELEASE.md).

## Current RQ1/RQ3 synchronization

This version adds the selected ChatDev/Crazyflie candidate and its saved static/Host evidence, the separate requirement-coverage metric with per-leaf evidence, and the six-task RQ3 line chart. It updates the manuscript sections and documents the author-confirmed manual review. All current standard deviations use the population definition; the figure displays means only. The preceding sealed archive and all source experiments remain unchanged. See [CURRENT_RELEASE.md](CURRENT_RELEASE.md) for scope and validation limits.

## Previous release: changes from the September review package

The September 28 package described a historical 243-position collection and retained Chinese source text. Its tables were not the current paper tables. That archive remains unchanged outside this release.

The preceding release made the following distribution changes while retaining saved experimental outcomes:

1. Names the method EmbedDev and uses Direct-LLM as the displayed direct-generation label. Original method identifiers remain in numeric data for traceability.
2. Uses the current RQ1 135-position view, including the later Gemini replacement lineage, and the population-SD definition used by the author manuscript.
3. Selects the 54-position DeepSeek RQ2 presentation scope. Missing static reviews are left missing; they are not filled with old scores or zeros.
4. Reconstructs the seven-task, five-method RQ3 static-support bar chart. It preserves the author-reported GPIO rerun provenance and different review-item granularities.
5. Uses the current 162-position RQ4 endpoint classification and keeps mechanism cases separate.
6. Ships English prompts, input documents, comments, diagnostics, and new English documentation. Source-to-release transformations and both hashes are recorded.
7. Provides a portable current EmbedDev implementation for new runs, separately from offline reconstruction of the historical reported numbers. The release does not assert that historical experiments used the translated prompt bytes or the current implementation revision.

Original records, failed attempts, incomplete outputs, usage ledgers, and frozen inputs were not modified. Preparing this package made no external model calls and ran no new generated-program/Host experiment.
