# Evaluated tasks and English inputs

This release retains seven input examples from the historical evaluation. The current RQ3 display uses six of them, excluding Motor Tracking. The paper's broader collection contains nine real-world projects; these input bundles are not an inventory or redistribution of all nine upstream repositories.

| Task | Input directory | Evaluation scope | Frozen input source |
| --- | --- | --- | --- |
| Crazyflie | `examples/crazyflie/` | Original three-task scope; RQ1-RQ4 | `main_v4_800k_20260908/protocol/crazyflie/` |
| DiscoBot | `examples/discobot/` | Original three-task scope; RQ1-RQ4 | `main_v4_800k_20260908/protocol/discobot/` |
| OnStep | `examples/onstep/` | Original three-task scope; RQ1-RQ4 | `main_v4_800k_20260908/protocol/onstep/` |
| GPIO Output | `examples/gpio_output/` | Smaller-task extension; current RQ3 | `rq3_task_extension_20261002/preparation/gpio_output/` |
| GPIO Button Debounce | `examples/gpio_button_debounce/` | Smaller-task extension; current RQ3 | `rq3_task_extension_20261002/preparation/gpio_button_debounce/` |
| Balance Control | `examples/balance_control/` | Smaller-task extension; current RQ3 | `rq3_task_extension_20261002/preparation/balance_control/inputs/` |
| Motor Tracking | `examples/motor_track/` | Historical example; excluded from current RQ3 | `rq3_task_extension_20261002/preparation/motor_track/inputs/` |

The first three source directories are under `fse/artifacts/matched_experiments/`; the four extension sources are under `fse/`. These are provenance paths in the author's working repository, not paths required to use this release. Exact repository-relative source paths and hashes for all 21 input files are recorded in [input_translation_provenance.json](../input_translation_provenance.json).

The two GPIO tasks use self-authored contracts. Balance Control and Motor Tracking use the actual frozen contracts prepared for the extension evaluation. These task contracts do not imply that seven complete, independently licensed upstream repositories are redistributed here. See [THIRD_PARTY.md](THIRD_PARTY.md) for attribution and release boundaries.

## Files in each task

- `RE_req.txt`: functional and nonfunctional requirements, including the original requirement identifiers and acceptance rules.
- `RE_api.txt`: the frozen software interface, including types, constants, function signatures, and any interface-specific constraints.
- `RE_device_interface.txt`: device and host-adapter boundaries, including observable effects, timing, and failure handling.
- `project.json`: relative input locations and the C11 host build profile consumed by `fse/checking/config.py`.

All three contract files must remain together. Public symbol spelling, parameter order, types, constants, and requirement identifiers are part of the contract. The input files describe task requirements; they do not constitute evidence that a generated implementation meets them.

## Translation and replay boundary

The original three tasks contained Chinese prose and comments. This release translates that material into English while preserving every line, all previously English lines, requirement and testcase identifiers, and C declarations. The Crazyflie device contract and all twelve extension inputs were already English and are copied byte for byte. In total, eight files are translated and thirteen files retain their original bytes.

These English inputs are intended for new runs. Translation changes prompt bytes and may change model behavior, so a fresh run using a translated file is not a byte-for-byte replay of the historical generation request. The source inputs remain unchanged in the author's working repository. The provenance file separately records original and released SHA-256 hashes and the `translation` flag; it does not claim that omitted historical source files are available inside this archive.

No requirements were intentionally corrected, shortened, or strengthened during translation. Structural checks confirm matching line counts, matching requirement/testcase identifier sequences, matching identifiers containing underscores, unchanged API declaration lines, and zero remaining CJK ideographs or escaped CJK text. These checks support traceability; they are not a new semantic evaluation of the requirements or generated code.

## Project configuration and run limits

Each `project.json` uses filenames relative to its own task directory. It selects C11, `gcc`, the host platform, and `-Wall -Wextra`; generated outputs and archives use distinct relative directories. It explicitly sets `pipeline.design_iteration.max_iterations` to `4`, overriding the development configuration's default of eight design iterations.

Code repair is configured by the release CLI, rather than by an unsupported field in `project.json`. The main entry point, `pipeline.py run`, uses `--code-repairs 3`; the internal `run_agent_4_code_generator.py` entry point uses `--max-retries 3`. Both default to at most three repair retries after the initial code generation, or four code-generation attempts in total. This is separate from the four-iteration design limit and from the design engine's local-repair policy.

From the release root, an offline input inspection can use any listed task directory:

```console
python fse/checking/pipeline.py inspect --input-dir examples/gpio_output
```

The commands and provider configuration for an explicitly requested new generation run are documented in the [README](../README.md). The release preparation did not execute new model generations or Host experiments. Saved RQ results and their denominators are documented separately; see [RQ3.md](RQ3.md) for the six-task comparison and its provenance limits.
