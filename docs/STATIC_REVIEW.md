# Interpreting the saved static observations

The result reconstruction scripts consume saved judgments. They do not rejudge code using keyword matches, compile results, or newly invoked language models.

For a reviewed artifact, support is P / (P + F + U), where P means supported, F means failed, and U means uncertain. Not-applicable items are excluded from the artifact's denominator. The exact count/fraction fields are retained wherever they occur in the selected source view.

An executable Host failure and a static support judgment answer different questions. Code can contain substantial functionality while failing integration. Conversely, passing the available executable scenarios does not prove support for every static requirement.

RQ1 uses its original project-specific testcase collections; RQ3's three displayed smaller tasks use whole requirement items. Do not concatenate those collections into a single uniform benchmark or infer a pooled static pass rate. The separate 135-position requirement-leaf coverage study is included as its own fourth RQ1 metric and is not substituted for testcase support.

Some historical reviews were made by an LLM assistant through complete code reading. The author subsequently confirmed manual review of the current ChatDev testcase and requirement judgments. That confirmation is recorded in `data/human_review_attestation.json`; original per-item `pending` or `false` human-review fields remain as initial-review metadata, not the current attested status. No per-case revised verdicts were supplied, so the original 46 supported / 47 failed / 7 uncertain testcase judgments remain unchanged. An offline hash/arithmetic check is not a second semantic review or independent observation of human verification.

## Requirement coverage

The frozen checklists contain 145/102/95 leaves for Crazyflie/OnStep/DiscoBot. Their statically reviewable denominators are 141/89/88. A fully or partially covered leaf receives one unit of credit; not-covered and uncertain leaves receive none while remaining in the denominator. The preclassified non-static leaves are excluded. This metric describes breadth of at least partial implementation, not fully satisfied or dynamically verified requirements.

`data/rq1/requirement_coverage.json` binds all 135 positions to their saved counts and distributed evidence. The detailed records and checklists are in `data/rq1/requirement_coverage/`. Initial review records can contain an older full-only summary; the current metric is recomputed from their leaf verdicts, without changing those verdicts. For the current ChatDev/Crazyflie code, 90 fully and 46 partially covered leaves give 136/141; five leaves are not covered and four are excluded. The three-project row becomes 91.35 +/- 0.31 using the same population-SD repeat blocks as the testcase metrics.

Original review text retains its language. Path anonymization and subsequent author attestations are explicit distribution annotations; they do not convert a static judgment into an executed testcase result. The current ChatDev source files are unchanged and its saved Host report still records compilation failure.
