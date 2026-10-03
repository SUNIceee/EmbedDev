# Complete English release

All user-facing text in this archive is provided in English, including the implementation, prompts, seven task input bundles, historical requirements, coverage checklists, saved review explanations, and candidate comments. Identifiers, mathematical notation, protocol fields, and original source hashes are retained.

The preceding synchronized archive already provided English executable inputs and prompts. This release translates its remaining 127 text files. Each distinct review explanation is translated with its candidate-specific evidence and failure conditions, rather than replaced by a generic requirement description. `evidence_translation_provenance.json` records prior distributed and English digests, plus per-string hash bindings for JSON translations.

## What did not change

Item IDs, verdicts, scores, counts, numerical values, booleans, and JSON structure are unchanged. Requirement source lines retain their order and line count. The one candidate C file with remaining Chinese headings changes comments only; its noncomment text and line count are unchanged. Runtime Python code, experimental inputs, model calls, and test execution are not changed by this translation release. No experiments or judgments were rerun.

The three source requirement translations reuse the already verified English task documents where they derive from exactly the same historical source. Original English lines remain unchanged. Historical checklist hashes inside review records identify the original review inputs, while distributed-file hashes identify the translated copies.

## Original records and reproduction

The preceding sealed archive and original working records are preserved outside this English archive. The translated copies are readable renderings, not evidence that the historical model or reviewer saw the English wording. Translation does not turn static support into executed testcase results or create a new human review.

Use the normal offline verification and table/figure reconstruction commands in the README. Translation changes file bytes, so the manifest and distributed bindings have been regenerated; original source digests remain intact. The English archive remains a local deliverable until it is uploaded to a verified repository.
