# Phase83 native ClockFactor_CCDD gap audit v1

Status: truth-free source audit, 2026-09-02.  Scope is one topic only:
the pinned taroz `ClockFactor_CCDD` contract versus the Phase80 native
IMU/GTSAM clock path.  No solver was run and no code or parameters were
changed by this record.  `ground_truth.csv`, MATLAB data, Phase82 candidate
coordinates, Kaggle, and token services were not read.

## Decision

Select one future, default-off **source-exact CCDD active-clock-row** candidate
for a separately frozen structural test.  It is plausible, but unproven, that
linking the existing per-epoch receiver clock to the existing Doppler drift
could improve horizontal error without route tuning: it changes how P/D
clock-versus-position error is allocated, rather than selecting a route or
using truth.  The Phase46 raw-clock audit found a common-mode clock signal, so
this is a mechanism hypothesis, not evidence of a horizontal gain.

Do not promote it or make it a Phase80 reclassification.  Do not call a
C0/D-only port full taroz equivalence.  A full seven-component C-state
migration is explicitly **not selected for Phase83**; it needs its own state,
factor, signal-index, and parity review.  The selected test may preserve the
current static ISB representation while proving exact CCDD behavior for the
active receiver-clock component and drift.

## Official contract

The source cache is the pinned taroz/gsdc2023 specification at commit
`29923f9f370f09ebc00f96d8cca375007a18e7d5`.

| Evidence (exact file:line) | sha256 | Contract |
|---|---|---|
| `output/reproducibility-cache/gsdc2023/fgo_gnss_imu.m:102-114,169-186,189-218` | `c70090ccb8b27fc8ac7fd2929e2f995a14cfe7f089bb1fef067370e22051c3e3` | Each epoch has a seven-element `c` vector and one-element `d`; P and D factors consume them. Clock noises are `[0.1; zeros(6,1)]` and `[Inf; zeros(6,1)]` through the published parameter. |
| `output/reproducibility-cache/gsdc2023/fgo_gnss_imu.m:252-277` | `c70090ccb8b27fc8ac7fd2929e2f995a14cfe7f089bb1fef067370e22051c3e3` | `dtgps` is raw epoch spacing; CCDD is added only when `dtgps < prm.time_diff_th` and the phone is not in the three-device exclusion list. A jump selects `noise_clkjump`. |
| `output/reproducibility-cache/gsdc2023/parameters.m:46-47,130-133` | `518925e9c75c7a14fceb5cd99432fe883311e0d64c72b94396744ac765120f52` | `time_diff_th = 1.5` s and `sigma_motion_clk = 0.1` (the other six clock rows are zero sigma in `fgo_gnss_imu.m`). |
| `output/reproducibility-cache/gtsam_gnss/src/ClockFactor_CCDD.h:16-20,32-61` | `7c174217218aa0059155e1e5a7182c69d56a3cd389d0ebd95c242d6d8c65b9fc` | Exact residual is `e = c2 - c1`, then `e[0] -= (d1[0]+d2[0])*dt/2`; `Hc1=-I`, `Hc2=I`, and both drift Jacobians have only row 0 equal to `-dt/2`. |
| `output/reproducibility-cache/gtsam_gnss/src/PseudorangeFactor_XC.h:48-64` and `functions/sysfreq2sigtype.m:6-17` | `7f467698f2239818724eba4485b830fbc543586bbee06a21b8b2c03ae5b56415`; `5ec1d299e04b45f604921cbae3b1fac8f78f3f76a0152496d86011a32f1d0079` | Pseudorange indexes the seven `c` columns by source signal/constellation mapping; this is not merely a scalar receiver-clock state. |

Thus the official row semantics are: component 0 is receiver-clock
continuity minus midpoint drift times the measured interval; components 1--6
are exact clock-vector continuity and have no drift term.  A detected jump
loosens only component 0 (`Inf`); the remaining rows stay exact by the source
noise declaration.

## Phase80 native implementation

Phase80’s no-base entrypoint explicitly selects GTSAM/Pose3/IMU and the direct
observable-quality option only changes P/D quality settings; it does not
select a CCDD factor or alter clock state semantics
(`apps/native/gnss_fgo_imu_no_base.cpp:3698-3719,3803-3824`, sha256
`f75eaf3bb9b1b138396fbb550ef04925cfd2b68cd306f8e01a43147354028cfe`).

The current GTSAM graph has the following contract:

- `clockKey(epoch)` is one scalar base clock in seconds.  Non-GPS systems use
  one global/static ISB scalar each (`src/algorithms/fgo_gtsam_backend.cpp:442-475`,
  sha256 `c48c1452a4f31058965bf1e22d9ed41ee54e2f52c5ea5cf22cd14f1b2d8d4dfb`).
  The key-space description confirms scalar per-epoch `c`, global ISBs, and a
  separate `d` key space (`src/algorithms/fgo_gtsam_internal.hpp:361-388`,
  sha256 `b0ee9f5659d701a658e8f88992e46aebd5ef9e5ca084c136cae4a6029659084b`).
- Receiver-only D rows initialize an independent per-epoch
  `dopplerClockDriftKey(epoch)` (`src/algorithms/fgo_gtsam_backend.cpp:206-261`,
  sha256 `c48c1452a4f31058965bf1e22d9ed41ee54e2f52c5ea5cf22cd14f1b2d8d4dfb`)
  and use it with `velocityKey` (`src/algorithms/fgo_gtsam_backend.cpp:673-700`,
  same backend sha256).  The D state therefore exists in Phase80 but is not
  coupled to the clock continuity row.
- The clock factor itself is only
  `BetweenFactor<double>(c[i-1], c[i], 0.0)`, over every adjacent graph epoch.
  It has no `dt`, no `d1/d2`, no seven-vector, and no `<1.5 s` edge gate
  (`src/algorithms/fgo_gtsam_backend.cpp:914-933`, same backend sha256).
  Regular sigma is the public `clock_motion_sigma_m = 300` m and a detected
  jump uses `1e6` m (`include/libgnss++/algorithms/fgo_config.hpp:286-300`,
  sha256 `5bf6a54be423a283e42256df2adbd50f4b30083c317bfc6b46cda6a54efd0423`).
  Both are converted to seconds in the `BetweenFactor`.
- Ordinary TDCP also uses the scalar base clock at both endpoints
  (`src/algorithms/fgo_gtsam_backend.cpp:728-764`, same backend sha256); it
  does not supply the missing seven-component CCDD state.

The exact implemented gap is therefore not just a sigma mismatch.  It is a
missing CCDD factor contract: current c continuity is scalar, weak and
clock-only; current d is separate; actual interval and source edge gating are
discarded; and source C[1..6] signal-indexed continuity is not representable
in the current factor.  This is a source gap, not a claim that a run has
already shown an accuracy deficit.

## Prior overlap and evaluation status

| Prior record | What it covered | Relation to this audit |
|---|---|---|
| `docs/use_cases/records/smartphone_r5_phase46_pixel5_raw_clock_timing_result_v1.json:2-5,88-95,220-232`, sha256 `d183bc8630a06545c4cb46dd4d061bb4d819fc48414b9557c5dfcad097e3e5c1` | Raw clock timing was classified common-mode-only; no clock correction was promoted. | Diagnostic input to plausibility only; it did not test CCDD c--d coupling. |
| `docs/use_cases/records/smartphone_r5_gsdc2023_native_gnss_pdc_state_backend_gap_v1.json:61-64,78-111,123-129,150-158`, sha256 `dcbbaf989918d349964ae9a511b49474c620d6b42b38a3215771b685986d46e1` | Historical custom-Eigen PDC gap audit selected an actual-`dt` PDC temporal candidate and separately documented five-clock versus seven-clock gaps. | Different PDC/Eigen path; it did not evaluate the Phase80 IMU/GTSAM clock factor. |
| `docs/use_cases/smartphone_native_only.md:175-186`, sha256 `74024a5dc0632e18da973748b2632fd2ea6a3d3e9d1cf18ca6acf296aaf5369e` | Explicitly says the prior actual-`dt` port was PDC-only and not ENU/seven-clock/GTSAM equivalence. | Confirms non-overlap. |
| `docs/use_cases/records/smartphone_r5_phase10_native_pdc_imu_tdcp_clock_gate_recovery_v1.json:2-29,43-66`, sha256 `4908e8ecbc04adbee63dbb8f20625c610a604e8ad90c1981b770e95ed3c8d45b` | Added/recovered a TDCP clock-discontinuity rejection gate; no CCDD dynamics. | Not this candidate. |
| Phase80/81/82 records: `...phase80_source_exact_direct_p_quality_freeze_v1.json:1-7,33-61`; `...phase81_phase80_direct_observable_quality_reclassification_result_v1.json:1-6,96-125`; `...phase82_phase80_direct_quality_accuracy_result_v1.json:1-20,32-66` | Phase80 froze direct P/D quality, Phase81 reclassified its immutable structure, and Phase82 evaluated that direct-quality candidate. | None defines or evaluates a CCDD clock candidate; Phase82’s no-go must not be attributed to this gap. Hashes: `9ebdde21fe7fd955029243d3bdbba3a9f9d3832388d67480760394177c8f685e`, `f5809f173c3e346aec775ca6dd152de5436eb68ea348d3fe90dffc7f82153b14`, `39c5a38b7e3e61fda0306582fffb961dad3e6fb0bd49fcfe70e66674e2c90873`. |

Conclusion on evaluation: the clock gap has been source-audited and
structurally discussed, but **has not been evaluated as a Phase80 candidate**.
No horizontal-score conclusion is supported by the existing records.

## Frozen next protocol

No implementation is authorized by this document.  If the selected opt-in
candidate is implemented later, freeze the following before any truth read:

1. Keep Phase80’s direct/no-PDC source-quality composition, raw Android GNSS
   + broadcast navigation + IMU inputs, route order, and two repeats per the
   sealed four-route Pixel5 cohort. Keep all route settings and solver limits
   fixed; no route tuning, parameter search, PDC bridge, base compensation,
   MATLAB input, validation/holdout data, or candidate-coordinate reuse.
2. Port and test the CCDD equation/Jacobians and noise mapping exactly:
   active C0/D midpoint with measured positive `dt`; exact C1--C6 continuity;
   jump loosens only C0; omit nonpositive or `dt >= 1.5 s` edges; preserve the
   source phone gate. If the existing static ISB model is retained, label the
   result C0/D active-row parity, not full seven-clock parity.
3. Before route execution, require a synthetic factor fixture covering
   non-unit `dt`, a jump, all seven residual rows/Jacobians, zero-sigma exact
   rows, and edge inclusion/exclusion. On the routes, record CCDD edge counts,
   C/D key connectivity, factor counts, finite/converged output, earth-valid
   keyed rows, zero over-70-m/s transitions, and byte-identical repeats.
   Structural failure is fail-closed and blocks accuracy evaluation.
4. Only after structural GO, freeze a separate accuracy authorization. Reuse
   the exact Phase82 metric and gates from its frozen manifest/result (including
   exact-key matching, Haversine `(P50+P95)/2`, full prediction-domain
   coverage, continuity, finiteness, and all-route gates); do not relax them.
   Permit one explicitly authorized development-train read per route, compare
   against immutable declared controls, and prohibit validation/holdout/Kaggle,
   solver reruns after truth, post-truth tuning, or route selection. Promotion
   remains off unless every predeclared gate passes.

This protocol tests the causal clock-state hypothesis while keeping the
Phase80 experiment and its accuracy record immutable.
