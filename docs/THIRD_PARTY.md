# Third-party materials and redistribution

The comparison uses project-level application tasks derived from Crazyflie, OnStep, and DiscoBot, together with four additional peripheral/control tasks. Task derivation is not a claim to redistribute complete upstream firmware repositories.

The following framework revisions are recorded in `baselines/upstream_versions.json`:

| Framework | Repository | Commit |
| --- | --- | --- |
| MetaGPT | https://github.com/FoundationAgents/MetaGPT | `335d972f4601b2c37614aa8129dd890ef6ad3dbe` |
| ChatDev | https://github.com/OpenBMB/ChatDev | `31fd994416a251ecdeb1f0a73c329271743bfb56` |
| StructGen | https://github.com/SEDevSys/StructGen | `0237cd22242fcb01ad26266179a2b240b42e30e6` |

Available MetaGPT and ChatDev license notices are included under `third_party/`. No StructGen license notice was present in the earlier review package; no license for that upstream code is inferred. Upstream checkouts are not redistributed here. Obtain third-party software from its original source and comply with its terms.

PlantUML and Java binaries, compilers, model SDK installations, and model weights are not bundled. Their respective upstream terms apply. Saved generated code is distributed as experimental material, with original source hashes and any comment translations documented; this package does not establish exclusive ownership of generated material.

No license file for the authors' own implementation was found in the source checkout. This preparation does not select or invent a distribution license. The authors should add their chosen license before public release. Third-party notices must remain intact.
