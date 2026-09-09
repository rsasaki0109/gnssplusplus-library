# Phase458 — frozen multiple-code contamination matrix

Extended the actual native raw-P fixture with five fixed cases, at the same
receiver/time and sixteen synthetic GPS broadcast orbits as Phase457:
clean; first row +300 m; first two rows +300 m; first four rows +75 m;
first two rows +150/-150 m. Both RAIM and iterative outlier detection are ON.
Huber OFF/ON uses unchanged built-in thresholds. No post-result case tuning.

Two quality profiles were specified in the test before execution. The
controlled profile admits all elevations, uses equal weights and a 100-step
budget. The native-quality profile uses zero-degree elevation mask, native
variance/elevation weighting, max GDOP 50 and ten-step budget. Atmosphere
remains disabled in both because it is absent from the synthetic generator.
This is GPS-only, single-epoch and known-start geometry, not a full recreation
of smartphone raw-clock bootstrap, C7 or the downstream FGO.

| Case | Controlled position error m, Huber OFF / ON | Native-quality position error m, OFF / ON |
|---|---|---|
| clean | 0 / 0 | 0 / 0 |
| one +300 | < 1e-7 / < 1e-7 | 491.525 / 491.525 |
| two +300 | 22.2072 / < 1e-7 | 491.525 / 491.525 |
| four +75 | 11.5962 / 26.5591 | 122.882 / 122.882 |
| mixed +150/-150 | 66.6594 / < 1e-7 | 245.763 / 245.763 |

All twenty solves were accepted. Native-quality masking leaves only four
used GPS rows in every case. Position plus one clock has four unknowns, so
these exactly determined configurations cannot expose a code outlier through
redundant residuals. Huber offers no protection in this fixture. Identical
errors for some cases are not evidence of equivalence across all raw rows:
some injected rows are removed by the elevation mask before solving.

The controlled profile shows both improvement and regression. Four +75 m
errors worsen with Huber and retain sixteen rows versus fifteen without it.
This rejects a blanket claim that turning on robust seed weights improves
initialization. Do not promote this switch or search its thresholds on H.

Verification: standalone C++ build succeeded; the twenty-case matrix passed,
then all 28 RawPSeedTest tests passed; `git diff --check` passed. Matrix tests
assert finite accepted results and clean-solve accuracy, not universal Huber
superiority. Printed errors are synthetic position errors, not Kaggle scores.
No production code/default changes, MAT, real truth access, raw replay or
submission. Full CTest/PPC remains outstanding.

Next discriminating evidence is actual raw-P seed redundancy/conditioning
and row admission around sparse LAX epochs, not another blanket Huber trial.
Use existing same-run seed reports if exposed, otherwise a raw-only seed
audit emitting counts/rank/status (no persisted coordinate input). If actual
seeds lack redundancy, investigate temporal raw constraints or quality-aware
initialization, not just scalar robust weights. This synthetic result does
not establish that LAX has the same failure mechanism.

The original performance objective remains active and unmet.
