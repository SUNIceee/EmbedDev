# GPIO output evidence supplement

This supplement records **one new logical attempt** of EmbedDev with `deepseek-v4-pro` on the original GPIO output task. It contains the unchanged generated candidate, original contracts and Host suite, independent compilation evidence, and a complete eight-item static review.

| Outcome | Recorded result |
|---|---|
| Complete C/header delivered | Yes |
| Internal method acceptance | Yes |
| Independent compilation | Failed: `NULL` is undeclared at `6_generated_code.c:62` |
| Original Host scenarios scheduled | 10 |
| Host scenarios actually executed | 0 |
| Host scenarios passed | 0; no behavioral failures were observed because execution never began |
| Static support of original whole requirements | 8/8 (100%) |
| Human review of this static assessment | Not performed or attested; `human_reviewed=false` |

The candidate is intentionally preserved with its compilation error. No header was added and no functionality was repaired. Static support is a code-reading result under the original protocol, which permits assessing unambiguous intended semantics despite a pure build obstacle. It is **not** executable testcase success, human repair, or evidence that the ten Host scenarios passed.

The first part of this attempt stopped after three completed model requests because a local inventory check incorrectly treated the compiler's normal runtime mutex as a new frozen source file. A separate corrected package excluded only that exact lock path, replayed the three completed responses locally, and continued the same attempt. Its method, inputs, final tests, and prior design context were unchanged. Four additional model requests completed the attempt: **7 physical provider requests and 135,348 reported tokens in total**, not seven experiments. No completed provider request was sent again. Raw provider messages, credentials, local authorization records, and the full runner tree are not included here.

This is new evidence, not a reconstruction of the missing historical artifact. It does not substantiate an earlier author-reported 100% Host result and does not automatically replace any manuscript score.

## Files

- `candidate/`: exact UTF-8 C/header bytes from the selected candidate packet, including original line endings.
- `inputs/`: the original eight numbered requirements, API, and device contract.
- `tests/`: the original ten Host scenarios, scenario inventory, and fixture header. The Host adapter used only an include alias to bind this suite to `6_generated_code.h`; it did not repair the candidate.
- `evidence/`: candidate packet, independent Host report and binding, and the complete recorded compiler result.
- `review/`: original eight-item checklist with public path bindings, and the full static assessment with per-condition code excerpts and line numbers. The review discloses prior exposure to the Host harness during infrastructure preparation; it does not claim complete evaluator blindness.
- `summary.json`: result and resource summary for this logical attempt.
- `provenance.json`: original and distributed SHA-256 mappings. A `source-evidence:` identifier refers to a retained origin artifact, not a file required for the offline checks.
- `manifest.sha256.json`: exact hashes of the distributed files.

## Offline verification

Python 3.10 or later is sufficient; no external Python packages, model access, credentials, or network connection are required.

```console
python -B scripts/verify_evidence.py
```

This verifies the file inventory, candidate bytes, original contract and Host hashes, eight distinct review judgments, quoted code lines, distributed provenance bindings, and the separation between static support and the compilation failure. It performs no compilation or testcase execution.

## Reproduce the compilation observation

The recorded independent evaluator used `gcc (Debian 12.2.0-14) 12.2.0`. Its complete command and diagnostics are in `evidence/compile_result.json`. The original network-disabled container image digest is also recorded there. This supplement does not download or install that image or a compiler.

With a locally installed GCC, first inspect the command:

```console
python -B scripts/reproduce_compile.py
```

To compile a separate byte-identical copy of the unchanged candidate:

```console
python -B scripts/reproduce_compile.py --run --compiler gcc
```

This optional command uses the recorded compilation flags and writes a new result under the repository's `outputs/gpio_evidence_compile/` directory (or `local_outputs/` when the supplement is distributed alone). It does not modify `candidate/`, link the Host harness, execute the candidate, or run the ten scenarios. An exit status of zero means that the recorded **compilation failure** was reproduced; it does not mean that the candidate compiled successfully. A different compiler environment may produce different wording or behavior, which is saved rather than substituted for the original observation.

`.gitattributes` disables newline conversion in this supplement so that the published hashes remain reproducible. Preserve those bytes when downloading or copying the files.
