# Phase113 remaining-routes structural audit

Status: sealed read-only audit of the remaining MTV-H and MTV-U routes.  This
audit reads only official/native source text, sealed Phase95 telemetry, and
sealed route/provenance metadata.  It does not open or hash raw phone GNSS,
IMU, broadcast-navigation, or base-RINEX payloads; it does not read truth or
MAT data, invoke a solver/evaluator, inspect solution coordinates, access
Kaggle/token resources, or change the inference implementation.

## Decision

The priority candidate—remove the nonempty undifferenced-Doppler admission
guard for MTV-H when the strict Phase101 selectors and raw-base selectors are
present—is **not source-safe as a guard-only change**.  MTV-H has 1,247 finite
raw-D initializer values and 1,041 eligible adjacent C0D pairs, but its
GNSS-first problem has zero undifferenced Doppler factors.  In the native
backend that condition suppresses the Point3/velocity path, C0D graph-factor
insertion, and the exported velocity/D state sequence.  Removing only the
exception would therefore not produce the exact GNSS-first C7/D/position/
velocity handoff required by Phase101.  Supplying velocity/D states from a
position gradient or another source would be a new state-initialization and
graph-path change, not the requested guard-only admission, and has no sealed
factor/residual evidence supporting it.

MTV-U already reaches the C0D path, but its sealed main result has only 564 of
720 positions within the finite Earth-valid norm and accepts zero main LM
steps.  Raw base correction is a pseudorange-value correction applied before
the shared GNSS-first/main problem; it does not change the C0D/D/velocity
topology or prove that 156 out-of-Earth main positions become valid.  The
sealed provenance establishes that a raw base member exists for U, not that
base correction repairs this output.  No raw-base execution is authorized by
this audit.

Exactly one candidate is frozen in the companion JSON: a default-off,
read-only structural diagnostic for H/U that records the already-defined
admission predicates and factor/state incidence without changing graph
construction, values, equations, units, sigma, filtering, LM behavior,
initialization, or fallback.  It is not a guard-removal or velocity-handling
implementation candidate and it does not authorize a raw run.

## Pinned evidence

| Item | Path or commit | SHA-256 / role |
| --- | --- | --- |
| Phase95 sealed structural result | `docs/use_cases/records/smartphone_r5_phase95_raw_input_path_corrected_structural_result_v1.json` | `beba25d4b0d910064d76bea7aa6b7a884937ca01c3b00cf0f7f9f4b6267ea281`; H/U telemetry |
| Phase95 raw-path freeze | `docs/use_cases/records/smartphone_r5_phase95_raw_input_path_availability_freeze_v1.json` | `983761ee5ddeeab1ff9274d0a5aa062d3c5ed59aa55075f558a0af29e3d3f044`; sealed raw route path lineage |
| Phase107 base source-parity audit | `docs/use_cases/records/smartphone_r5_phase107_raw_base_source_parity_audit_v1.md` | `79d1c79510138498c05d4096ed7c3f97b34b1ee16690f3fc7a7b57af3b0192ab`; sealed base provenance only |
| Phase107 base freeze | `docs/use_cases/records/smartphone_r5_phase107_raw_base_source_parity_freeze_v1.json` | `8c05c89dda9cb3356b4c9bdf1c0ec1f2f41145bb37a7b5aee7ff12978aba9d76`; four-route member metadata |
| Phase112 structural result | `docs/use_cases/records/smartphone_r5_phase112_main_output_offset_structural_result_v1.json` | `087b1bf51b4e584b86a7b558560df4ceb78e19b01e572f2d60aeef4c3d759ba0`; successful A/LAX recipe, no H/U execution |
| Phase112 source/freeze | `docs/use_cases/records/smartphone_r5_phase112_main_fgo_source_parity_freeze_v1.json` | `8917b194d46f92943a3fbde7681d961594488fcd3bed0e44ec08a3ba92ae16aa`; current recipe boundary |
| Official GNSS source | `output/reproducibility-cache/gsdc2023/fgo_gnss.m` | `5368e4056f70f448b728792c5e4c124b7f2afc1e00653492b807083bd3ccf0c3` |
| Official GNSS/IMU source | `output/reproducibility-cache/gsdc2023/fgo_gnss_imu.m` | `c70090ccb8b27fc8ac7fd2929e2f995a14cfe7f089bb1fef067370e22051c3e3` |
| Native app | `apps/native/gnss_fgo_imu_no_base.cpp` | `268bd030496bb677a42d3400e20ec37bc8e36589827fb28fbf57e3e398a1b1be` |
| Native GTSAM backend | `src/algorithms/fgo_gtsam_backend.cpp` | `781d65e34826ec08e04d1dfdac0ab10b2ed9409726078465f3020b57afac4baa` |
| Native factor definitions | `src/algorithms/fgo_gtsam_internal.hpp` | `cc6327e36800afa8217e5a834260adc81c0a74a5aefbfc956bb4679c8d86105f` |
| Native problem construction | `src/algorithms/fgo_problems.cpp` | `e7607ce8f0fe27fd382a01b5b6f147b027b7bf51453e920eae458c86ddac0c17` |
| Native base model | `src/algorithms/base_pseudorange_compensation.cpp` | `f3c3c859793c4bed9c7ae93825d32565f20eab0badb09412a4fab2a5930c9817` |

The current audit head before adding this document was `0151bd0` with a clean
working tree.  The source pins above are current working-tree hashes and are
not new executions.

## Sealed Phase95 route facts

The following values are copied from the sealed Phase95 result.  “Eligible
C0D” is the preflight adjacent-pair count; “C0D inserted” is the active
optimizer factor count.  They must not be conflated.

| Route | Retained epochs | Raw-D initializer | Eligible C0D pairs | Undifferenced Doppler factors | GNSS-first | Main result | Main earth-valid |
| --- | ---: | ---: | ---: | ---: | --- | --- | ---: |
| MTV-H (`2021-08-24-20-32-us-ca-mtv-h/pixel5`) | 1,247 | 1,247/1,247 finite | 1,041 | 0 | guard rejected; 0 iterations, no cost | not reached | 0/0 reported |
| MTV-U (`2023-03-08-21-34-us-ca-mtv-u/pixel5`) | 720 | 720/720 finite | 671 | 4,845 | 1,000 accepted; `23,108,268,994.98668 -> 82,026,870.12520209` | 0 accepted; cost unchanged, lambda `1e-5 -> 100,000` | 564/720; 156 out of Earth |

### MTV-H exact failure

Phase95 preflight records these facts for H:

* `backend_configuration_allowed=true`, `c0d_factor_enabled=true`,
  `gnss_first_handoff_enabled=true`, `raw_d_initializer_enabled=true`, and
  every one of the 1,247 retained D initializers is finite.
* `retained_undifferenced_doppler_factor_count=0` and
  `source_clock_c0d_problem_path=false`.
* The recorded failed predicates are
  `source_clock_c0d_problem_path_unavailable` and
  `gnss_first_problem.undifferenced_doppler_factors.empty()`.
* `eligible_c0d_pair_count=eligible_c0d_factor_count=1041` is only a
  preflight edge count.  The GNSS-first telemetry records
  `c0d_factor_count=0`, `c0d_active_solve_attempted=false`, zero iterations,
  no initial/final cost, and terminal branch
  `source-c0d-backend-admission-guard`.

Thus the earliest evidence is before graph optimization and before any main
handoff.  There is no H main C0D result to reinterpret as a numerical rank
failure.

### MTV-U exact failure

Phase95 preflight and stage telemetry record a valid path for U:

* 720 retained epochs and finite raw D initialization, 4,845
  undifferenced-Doppler factors, 671 eligible/inserted C0D factors, and exact
  C/D handoff coverage.
* GNSS-first makes finite strict progress through the sealed 1,000-iteration
  bound.  The main graph is attempted with 720 positions, velocities, D
  states, and receiver clocks, all finite and exact-key aligned.
* Main telemetry reports `accepted_outer_iterations=0`, initial and final
  cost both `15,619,537,525,702.566`, `inner_lambda_attempts=10`, final
  lambda `100000`, `position_solution_size=720`,
  `position_out_of_earth_count=156`, `earth_valid_position_count=564`, and
  terminal branch `no_progress_unclassified`.

This is a main validation/earth-domain failure after a successful GNSS-first
handoff, not a missing-D or missing-C0D admission failure.

## Static factor/state incidence

### Official source topology

The official `fgo_gnss.m` source initializes an `x`, `v`, seven-component `c`,
and scalar `d` value at every optimization epoch and adds an infinite prior
factor for each (`fgo_gnss.m:89-121`).  Pseudorange and Doppler factors are
then independently conditional on finite residuals
(`fgo_gnss.m:123-152`).  Adjacent epochs add `MotionFactor_XXVV` and
`ClockFactor_CCDD` when the time-gap rule passes, and TDCP is separately
conditional (`fgo_gnss.m:154-201`).  The official GNSS/IMU entry point loads
the GNSS-first `posest`, `velest`, `clkest`, and `dclkest` together
(`fgo_gnss_imu.m:40-48`), then constructs Pose3/velocity/bias and IMU factors
(`fgo_gnss_imu.m:169-217,280-299`).  Therefore the official graph can contain
velocity/D state nodes even when a particular Doppler row is absent; those
states are not created by a Doppler-count gate in the MATLAB source.

### Native H path if only the exception were removed

The native GNSS-first configuration sets Point3, no IMU, and
`use_velocity_states=true` (`gnss_fgo_imu_no_base.cpp:7078-7132`).  The
backend, however, defines the actual state path as:

| Native expression | H value | Consequence |
| --- | --- | --- |
| `use_gnss_velocity_states = !use_imu && !use_pose3_state && use_velocity_states && !problem.undifferenced_doppler_factors.empty()` (`fgo_gtsam_backend.cpp:43-47`) | false | no `v_i` state family is inserted and no `epoch_velocities_ecef_mps` export is produced |
| `use_native_source_clock_c0d_gnss_first = handoff && use_gnss_velocity_states` (`:47-49`) | false | C0D is not active in the staging graph |
| `source_clock_c0d_problem_path = use_imu || use_native_source_clock_c0d_gnss_first` (`:50-52`) | false | the current top-level C0D exception is the correct fail-closed result |
| `use_native_source_clock_c0d_factor = config.c0d && (use_imu || use_native_source_clock_c0d_gnss_first)` (`:305-307`) | false | no active C0D insertion |
| Doppler insertion condition (`:1027-1057`) | false/empty | zero `v_i`/`D_i` observation factors |
| velocity/D initialization and broad priors (`:445-540,1565-1587`) | not entered | no velocity/D Values or priors |
| velocity export (`:2200-2218`) | `use_gnss_velocity_states=false` | result cannot satisfy the exact velocity handoff helper |

Code factors can still be constructed from retained pseudorange rows, and
position/legacy scalar-clock motion rows can be considered in a hypothetical
Point3 graph.  That does not make the requested Phase101 handoff valid:
there is no native velocity state sequence, no C0D factor family, no epoch
local C7 state path, and no optimized D vector to pass to the main graph.
The 1,041 eligible C0D edges cannot change those booleans.  A finite broad
velocity prior is also conditional on the velocity state family, so it cannot
be counted as a hidden anchor in H.

The official topology could be approached by changing native state creation
to retain `v_i`/`d_i` without a Doppler row and choosing a source-defined
initialization (the official first-run velocity is `posbl.gradient(obs.dt)`).
That is velocity handling and graph/state construction, not a guard-only
admission.  No sealed residual or rank artifact proves that such a change is
safe for H, so it is not frozen.

### Native U path

U satisfies the native C0D staging predicates, so the existing graph does
contain the relevant families:

* raw code factors connect each retained epoch position to its clock/C7 state;
* 4,845 receiver-only Doppler rows connect velocity and D states;
* 671 active C0D rows connect adjacent C0 and D states;
* Pose3/velocity/bias priors, CombinedImuFactor or dropout continuity,
  position motion, clock motion, and the main C7/D handoff are present under
  the Phase101/Phase112 selectors (`fgo_gtsam_backend.cpp:672-704,
  850-1018,1027-1057,1302-1410,1486-1587`).

The sealed U telemetry confirms the corresponding state sizes and finite
handoff, but the main accepted-step and earth-valid predicates still fail.
This incidence is enough to classify the failure stage; it is not evidence
that a base code correction will repair the 156 positions.

## Raw-base provenance boundary

Phase107/Phase65 sealed metadata records an exact base member for each of the
four frozen train routes.  The values below are carried from that seal; this
audit did not stat, hash, open, or copy any member.

| Route | Sealed base member | Bytes / SHA-256 | Observed interval / smoothing |
| --- | --- | ---: | --- |
| MTV-A | `output/smartphone-r5/phase63-settings-integrity-recovery-v1/routes/2021-03-16-18-59-us-ca-mtv-a__pixel5/base.obs` | 10,708,536 / `380b8ff9091344fb756697e27f0983d9a0ba2cf0c201b96849bfc1ecc1af0e52` | 1 s / 151 |
| MTV-H | `output/smartphone-r5/phase63-settings-integrity-recovery-v1/routes/2021-08-24-20-32-us-ca-mtv-h__pixel5/base.obs` | 11,854,125 / `4d3e37cbe0347fa56216db54ede9e0f30731885f337f1653ab5a86afb2bb2150` | 1 s / 151 |
| LAX-T | `output/smartphone-r5/phase63-settings-integrity-recovery-v1/routes/2022-04-01-18-22-us-ca-lax-t__pixel5/base.obs` | 719,969 / `d731e0e8a7ba4396d62340c85b6238e66c50c6a349621f9dcea0ea6885fd4cfe` | 15 s / 11 |
| MTV-U | `output/smartphone-r5/phase63-settings-integrity-recovery-v1/routes/2023-03-08-21-34-us-ca-mtv-u__pixel5/base.obs` | 5,950,275 / `aedb7a39e7b6612ea97964b363c74cd2c10318255b7f2287f4720d18e71803e6` | 1 s / 151 |

The existing native base path builds a same-process correction model from the
raw base RINEX and broadcast navigation, then subtracts finite corrections
from the problem's pseudorange factors before the GNSS-first copy and main
handoff (`gnss_fgo_imu_no_base.cpp:6438-6490,6857-7000`; `base_pseudorange_compensation.cpp:215-339`).  It does not add a
velocity, C0D, or earth-validity factor.  The Phase107 audit also records that
the runtime station reference is restricted to raw RINEX header approximate
XYZ; phone/result/WLS/PDC/precomputed coordinates are forbidden.  Base-member
availability and correction provenance therefore support a future raw-base
structural lane, but not a conclusion about U's final earth-valid coverage.

## Candidate comparison

| Candidate | Disposition | Evidence |
| --- | --- | --- |
| H: remove only the nonempty-Doppler C0D admission guard under strict Phase101 + raw-base flags | **Rejected** | H would still have `use_gnss_velocity_states=false`, no native C0D insertion, and no velocity/D export; it cannot reach exact handoff. |
| H: add velocity/D state handling from a position-gradient or other non-Doppler source | Not frozen | It changes native state construction/initialization and needs new factor/residual evidence; it is not guard-only. |
| U: admit raw base on the assumption it makes all 720 main positions Earth-valid | **Rejected** | Base changes code values only; sealed U has 564/720 Earth-valid and zero accepted main steps, with no causal base-to-validity evidence. |
| H/U: read-only structural admission/incidence diagnostic with no graph or value changes | **Selected; exactly one frozen candidate** | It can distinguish preflight eligible edges from active factors and record U's post-handoff validation without speculative solver changes. |

## Single candidate boundary

Candidate ID:
`phase113-remaining-routes-structural-admission-incidence-diagnostic-v1`.

This candidate is default-off and diagnostic-only.  If separately implemented
and authorized later, it may record for H and U:

* retained epoch count, finite raw-D initializer count, eligible C0D pair
  count, active undifferenced-Doppler count, active C0D factor count, and the
  exact failed admission predicate;
* whether velocity/D Values and exports exist, C7/C0D/D exact-key alignment,
  active GNSS-first progress, and main position/velocity/clock state sizes;
* main finite/earth-valid counts, accepted-step/cost/lambda/terminal fields,
  and raw-base correction application/miss counts if a future raw run is
  separately admitted.

It must not remove the H guard, invent/hold/interpolate a velocity or D
sequence, change factor equations or topology, alter C7/D/CCDD units or
sigma, alter filtering/LM/ordering/initialization, add fallback, expose
solution coordinates, or read truth/MAT/Kaggle/precomputed phone
coordinates.  A future raw run would require a new manifest and independent
one-shot authorization; this audit authorizes zero raw/solver executions.

## Read accounting

| Resource/action in Phase113 | Count |
| --- | ---: |
| Official/native source reads | nonzero, source text only |
| Sealed Phase95/107/112 JSON/MD and summary metadata reads | nonzero, aggregate/provenance only |
| Raw phone GNSS/IMU/navigation payload reads | 0 |
| Raw base RINEX payload/header/stat/hash reads | 0 |
| Truth payload or MAT reads | 0 |
| Solution coordinate-field reads | 0 |
| Native solver invocations | 0 |
| Truth/accuracy evaluator invocations | 0 |
| Kaggle/token access | 0 |
| Raw reruns, fallback, or publication | 0 |

The audit is complete without changing the implementation.  The companion
freeze JSON is committed separately and records the one diagnostic candidate
and the fail-closed execution boundary.
