# Baseline adaptation source

These five files are unchanged copies of the local framework adapters and extraction/transport helpers. Their hashes and source paths are in `provenance.json`. All files are in English.

`native_metagpt.py`, `native_chatdev.py`, and `native_structgen.py` show how the upstream roles and workflows were adapted to the shared C/API tasks. `c_project.py` handles generated multi-file C projects. `runtime_bridge.py` implements the adapters' file-based transport interface.

The pinned upstream repositories are listed in `upstream_versions.json`. They are deliberately not replaced by new toy implementations of the baseline methods. These modules require the original framework checkouts and an external transport worker; this release does not bundle or claim to validate that full historical Windows scheduling environment. Do not run them as standalone generation commands and expect an included service to answer the transport requests.

Use `scripts/reproduce_tables.py` and `scripts/reproduce_rq3.py` to reconstruct the saved baseline comparisons offline. Use the current EmbedDev CLI for the separately documented new-generation path. RQ1 Direct-LLM receives requirements and device/environment facts but no supplied software API or external repair; RQ2's direct-repair condition also receives the shared API and bounded repair feedback. They must not be treated as the same baseline.
