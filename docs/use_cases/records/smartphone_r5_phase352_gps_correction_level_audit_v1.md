# Phase352: GPS L1 correction level versus within-stream variation

Extended the existing raw correction-support diagnostic. For each GPS L1
stream with >=300 finite phone-time queries (threshold fixed before run),
compute its temporal median, absolute median and median absolute deviation
about its own median. Report medians across streams only; no per-satellite
correction table or receiver estimate is saved or reused for inference.

Fresh standalone build and one raw H base/nav/phone pass exited 0. Phase346
support/rate statistics repeated exactly. Ten streams, 25259 supported rows;
median absolute stream center 1.4325974756901332 m; median within-stream
MAD 0.20886414606111747 m. These statistics indicate a relatively persistent
component in these sampled corrections, not proof of any particular physical
bias. Different satellites have different arcs; no across-satellite receiver
clock removal was done, and the raw query set is not final FGO admission.

The result motivates examining whether persistent correction components
transfer between base and phone before changing temporal smoothing. It
does not authorize centering away corrections: that would also remove
legitimate common satellite/code errors. Any centered-value ablation must
remain raw-derived, explicitly diagnostic, support-preserving and frozen
before evaluation. GPS L5 is not covered by this level summary. No truth,
MAT, saved position input, solver run, new score or promotion here.

Executable: `/dev/shm/gnss-test-build-recovery.BJvQt9/correction_level_audit`;
same raw paths and native library build approach as Phase346.
