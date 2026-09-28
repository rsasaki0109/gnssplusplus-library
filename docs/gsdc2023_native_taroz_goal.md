# GSDC 2023 native parity and improvement goal

Status: active. The user authorized this goal after PR #505 was merged.
Starting revision: `832dd7991cb8d042977d1e0775fcf27deb9c422c`.
Branch: `feat/gsdc2023-native-taroz-parity`.

## Current verified checkpoint

All 40 test drives now pass the frozen same-executable native/official-key audit.
The local submission contains 71,936 official rows across 14 phones, with zero
missing, interpolated or held output coordinates and no sample coordinates.
The fixed recipe artifact is `4267205e47200562...`; a separately audited candidate
changes only A205u initialization (1,699 rows), keeping the other 39 drives' values
identical, and has SHA-256 `152ead0ed97cca96...`. The user authorized that candidate
and completed official CLI OAuth authentication. Submission **56479759** is
COMPLETE with official **Public 1.698 m / Private 1.333 m**. The original fixed
recipe artifact remains unsubmitted. Private score improved by 2.011 m versus
the authenticated historical 3.344 m, but remains 0.405 m above the 0.928 m goal;
Public remains 0.909 m above the 0.789 m goal. The goal is not achieved.
The current server payload was downloaded and its SHA-256 matches the reviewed
local artifact exactly. Historical payloads were recovered too: submission
56309958 changes 16,330 rows on eight drives and retains 55,606 v5 rows across
32 drives, including six record-identified interpolated rows. The initial HTTP
403 resulted from using the wrong request field; the official historical SDK's
`submissionId` contract resolved it. Original per-run logs remain unavailable.
See [official submission evidence](use_cases/records/gsdc2023_all40_official_submission_v1.json).
The user also confirmed no alternative MATLAB environment exists. Pinned MATLAB
runtime parity remains unproven; native development and source inspection continue.
[Candidate artifact review](gsdc2023_all40_a205u_source_init_submission_v1_review.md).

The same submitted executable is being evaluated on 40 existing train settings,
grouped into 30 routes to keep same-trip phones together. Seventeen drives now
pass native/key audits and local scoring; one failed initialization, two are
running and 20 are not started at this checkpoint. Full-set means remain null.
The completed February 24 LAX-o Pixel5 has P50 1.234332 m, P95
2.345171 m and mean 1.789751 m across 2,438 truth keys; all 2,439 raw native
keys are present, with one extra native key excluded only from scoring.
The separately recovered January 4 MI8 scores 0.588876 m but is not counted
as a success of the frozen baseline that failed temporal bracketing. These
are development results, not heldout or official scores.
[Train evaluation](use_cases/records/gsdc2023_train40_fixed_recipe_evaluation.json).
The two January 4 Pixel5 source-initialization comparisons completed with
changes of 0 m and -0.000019798 m; neither supports promotion.
The TDCP sigma-only pair is mixed (January 4 improves, March 10 regresses),
and its two-route mean worsens by 0.080104 m. No uniform promotion is made.

The source IMU observation-stage opt-in has compiled successfully. Its first
integrated run passed 22/23 focused tests; the synthetic residual-center fixture
was corrected to calibrate from pre-residual rows (Earth-rotation correction had
already filtered its artificial observations). Production code was unchanged by
that fixture correction. The retry passed all 23 focused tests, eight legacy
regressions and nine CLI checks. Matched MI8/A205u replays with frozen executable
`d0cabb6c2f6b3630...` completed and passed native/base-once audits. Both controls
reproduce the prior executable byte-for-byte and GNSS stages remain identical.
MI8 changes from 1.1750051842 to 1.1763141171 m (+0.0013089329 m), while A205u
changes from 1.9616781852 to 1.9582353596 m (-0.0034428256 m). These small mixed
development effects are not promoted. The cold GNSS initial-mask opt-in has since completed development comparisons:
MI8 changes by +0.00000693 m; A205u requires the Samsung clock-drift
preprocessing to admit Doppler and then changes by -0.00037664 m. Neither
small final-stage effect is promoted; paired MATLAB runtime parity is unproven.
[Observation-stage experiment](use_cases/records/gsdc2023_source_imu_observation_stages_experiment.json).

The opt-in source IMU stop-phase experiment passed its forced rebuild, thirteen
focused tests, eight legacy regressions and eight CLI rejection cases. Frozen
binary `ffba0bde697dfd01...` completed matched MI8/A205U pairs. It omits only
the initial zero-velocity priors, retains
initial pose stop factors, and gates pose factors in both passes on raw UTC
intervals strictly below 1,500 ms. Observation masks are unchanged in this
experiment; their phase-specific rebuild is the separately tested opt-in above. The exposed
MI8/A205U pairs pass native/base-correction audits, preserve GNSS-stage bytes,
and reproduce prior control outputs. Mean-score changes are +0.0000031 m and
-0.0000264 m respectively, with no material improvement or promotion. A205U
exercises one initial pose-gap rejection on real raw UTC keys.
[Stop-phase experiment](use_cases/records/gsdc2023_source_imu_stop_phases_experiment.json).

The next observation-rebuild audit found a residual-center population difference:
the pinned GSDC function centers cached code residuals, whereas the legacy native builder
centers only surviving code factors. An arithmetic example demonstrates
different row admission with the same threshold. The inspected MatRTKLIB dependency
was separately pinned for this audit; its historical competition revision is not
proven, and no MATLAB execution or dataset accuracy effect is claimed. Preserve
separate center/admission populations in the next instrumented comparison instead
of changing thresholds alone.
[Residual-center audit](use_cases/records/gsdc2023_residual_center_population_audit.json).
The next initial-IMU refresh now has a separate GNSS-state handoff helper with
three passing standalone tests for exact epoch identity, clock units, velocity
frames and attitude validity. It is now wired into the opt-in observation-stage
implementation; integrated accuracy is not yet established.
[Initial handoff](use_cases/records/gsdc2023_initial_imu_handoff.json).

A second MI8 route (`2022-01-26-20-02-us-ca-mtv-pe1`) completed its
matched refinement on/off pair with the validated v2 executable. All 1,699
native and truth keys pass, with byte-identical initial GNSS/IMU stages.
The local mean worsens from 2.0769 m to 2.0985 m: P50 improves slightly but
P95 worsens. Together with August's regression, this does not support enabling
the added pass across MI8. No candidate is promoted.
Selection used the existing archive's settings (five MI8 train routes versus
one A205U train route), before truth was read for this experiment. The route's
Pixel5 belongs to the same evaluation group even though only MI8 is run here.
Prior project use is unknown; this is development validation, not held out.
[Second MI8 route](use_cases/records/gsdc2023_second_mi8_route_experiment.json).

The goal is **not achieved**. The records linked here are the current evidence;
subsequent chronological entries retain earlier states that may be superseded.
[Requirement evidence snapshot](use_cases/records/gsdc2023_goal_evidence_checkpoint.json)
keeps the original submitted-CSV provenance, all-drive native output, paired
reference runtime and official-score requirements separate from development gains.
The historical v5 record chain has now been checked after LF normalization:
it reports zero sample-coordinate fallback rows but seven seam interpolation
replacements and native-FGO/WLS source lanes. The later 3.344 m submission payload has now been recovered and compared
key by key: eight drives changed, 32 retained v5 values, and six identified
interpolated rows remained. Original per-run provenance is still partial,
so not every historical coordinate is proven to be an optimized graph state.
[Historical provenance audit](use_cases/records/gsdc2023_historical_submission_provenance_audit.json).

- The opt-in final-graph Doppler path now admits MI8 and A205U with the existing
  corrected-row and IMU checks. Ten focused backend tests pass, including
  actual stage-to-main synthetic solves for both phones. The A205U same-binary
  on/off pair has identical GNSS-stage CSVs and passes native coverage, but
  local P50/P95 mean worsens from 2.0409 m to 2.0497 m with 12,041 final
  Doppler factors. This condition is not promoted. A matched MI8 pair improves
  the local mean from 1.1591 m to 1.1483 m (P50 improves, P95 slightly worsens)
  with identical GNSS-stage output. Neither result supports phone-wide adoption.
  [Main Doppler experiment](use_cases/records/gsdc2023_a205u_main_doppler_experiment.json).
  [MI8 Doppler experiment](use_cases/records/gsdc2023_mi8_main_doppler_experiment.json).
- Stage scoring keeps exact truth keys and drops only extra native keys.
  A205U GNSS-first scores 2.8829 m before phone offset, versus 2.0409 m for
  the final IMU-plus-offset control; this is not an isolated IMU effect.
  MI8 GNSS-first scores 1.2651 m over all 1,400 truth keys; the native result
  retains all 1,417 raw keys, with only 17 extra keys dropped for scoring.
  [Stage scores](use_cases/records/gsdc2023_native_stage_pair_scores.json).
- Source cold start runs GNSS, initial IMU, then a second IMU pass with refreshed
  geometry/residual selection and previous attitude. The default native path
  still has one main IMU solve after GNSS. An opt-in second pass is now wired
  to rebuild raw geometry/weights/masks and apply the existing base model once
  to its new P rows. The full build, five helper tests, twelve focused integration
  tests, eight legacy builder regressions and six CLI rejection cases pass.
  The rebuilt v2 MI8 pair passes native provenance and has byte-identical
  initial GNSS/IMU stages. Refinement worsens the local mean from 1.1483 m to
  1.1750 m. A205U improves from 2.0497 m to 1.9617 m, with both P50 and P95
  improving and both initial stages byte-identical. Mixed results preclude
  a global or phone-wide promotion. The v1 MI8 pair
  remains preserved as an incomplete diagnostic due to a stale app object;
  v2 explicitly rebuilt both affected translation units. No general accuracy improvement
  or exact source cold-start reproduction is claimed. Cached external positions
  and reference-height files are not inputs. Both initial stages are exported
  for bytewise comparison before final phone offset.
  MI8's Highway/L5 preset worsens the local final mean from 1.1591 m to 1.1792 m
  and is not promoted.
  [Source-stage audit](use_cases/records/gsdc2023_cold_start_stage_gap.json).
  [MI8 refinement v2](use_cases/records/gsdc2023_mi8_refinement_experiment_v2.json).
  [A205U refinement v2](use_cases/records/gsdc2023_a205u_refinement_experiment_v2.json).
  Chronological-quarter diagnostics show that the A205U improvement is not
  uniform: its first quarter worsens by 0.1738 m while the other three improve.
  MI8 improves in its second quarter but worsens in the other three. These are
  already exposed truth subsets; their percentile scores are not additive,
  and they do not justify segment-specific settings or held-out claims.
  [Segment diagnostic](use_cases/records/gsdc2023_refinement_segment_diagnostic.json).

- An earlier mixed-variant aggregate verified **36/40 official-key drives** and
  **40/40 declared raw-key contracts**. The completed same-executable all40
  audit and official submission above supersede that coverage checkpoint.
  [Coverage evidence](use_cases/records/gsdc2023_combined_replay_coverage_checkpoint.json).
- The fixed 40-drive test recipe completed with executable SHA256 `c779bfbb4a4e4832...`.
  All 40 drives passed the official-key/native audit. The separately reviewed
  A205u source-initialization variant is the officially scored artifact above.
  Its explicit phone/gap rules are frozen; development
  noise/tolerance candidates are not included.
  [Fixed recipe](use_cases/records/gsdc2023_fixed_all40_recipe.json).
- A205U now passes all 1,699 official keys, including two leading IMU states.
  They are numerical initial guesses subsequently optimized by the graph, not
  accepted standalone SPP fixes. Output interpolation and holds are zero.
  [Leading-state evidence](use_cases/records/gsdc2023_samsung_leading_state_feasibility.json).
- On the previously exposed Pixel5 development route, source IMU noise changes
  local Haversine P50/P95 mean from 0.5764 m to 0.5525 m. Source stopping
  tolerances reduce iterations but worsen either noise setting's accuracy.
  Pixel4's matched pair worsens from 0.8454 m to 0.8978 m; no phone-wide
  promotion is made.
  [Four-condition comparison](use_cases/records/gsdc2023_pixel5_noise_tolerance_matrix.json).
- Existing train checks give local P50/P95 means of 1.2131 m (MI8),
  2.2232 m (August A325F), and 2.0147 m (October A325F). The A205U
  clock-boundary retry passes native coverage but scores 53.9019 m. Source
  IMU initialization reduces this to 2.1514 m with an identical GNSS-first
  summary (P50 1.3780 m / P95 2.9249 m). This is one exposed route, so
  the fixed all-drive recipe is unchanged. These are development
  results, not official scores or taroz parity.
  [Train evidence](use_cases/records/gsdc2023_existing_train_phone_groups.json).
  The same initialization change also preserves all 1,699 official keys on
  the existing A205U test drive; no test accuracy claim is made.
  [A205U test check](use_cases/records/gsdc2023_a205u_test_source_initialization.json).
- A205U source position offset is now admitted for the explicit raw-SPP
  Phase171 IMU path. A matched executable pair improves the existing train
  score from 2.1514 m to 2.0409 m. All recorded solver costs and iterations
  are identical; correction is applied once after optimization.
  [Offset evidence](use_cases/records/gsdc2023_a205u_position_offset_experiment.json).
- Archived reference outputs are provisionally decoded for four existing
  development phones. Common-key comparisons still favor the archived final
  results on all four; their generating commit/settings are unverified, so
  these do not establish pinned runtime parity or official score equivalence.
  October also has reference-height MAT input that source may consume and
  native does not; final-score gaps are not algorithm-only comparisons.
  [Archived comparison](use_cases/records/gsdc2023_archived_phone_group_comparison.json).
- A325F source initialization pairs on both existing route groups show no
  accuracy gain (score changes below 0.0002 m). MI8 position offset changes
  local score from 1.2131 m to 1.1591 m: P95 improves, P50 worsens.
  It remains an exposed-route candidate, not a global promotion. An opt-in native GNSS-stage ECEF sidecar has been built;
  its real-data validation passed on 1,213 exact original keys and preserves
  the final CSV byte-for-byte. Common-key A205U scores are 2.8526 m at
  native GNSS stage and 2.0400 m after IMU and position offset. The source
  Highway/L5=0 preset worsens the full-key final score from 2.0409 m to
  2.2208 m, so it is not promoted. A matched Samsung GPS L1 clock-drift
  initializer comparison reduces GNSS iterations 284 to 268 but changes
  final score by only 0.0033 mm; no accuracy gain is established.
  [Stage export](use_cases/records/gsdc2023_native_gnss_stage_export.json).
- Local submission assembly is implemented and its CTest suites pass (15 Python
  cases). It refuses incomplete drives and has produced no final submission.
  [Assembly evidence](use_cases/records/gsdc2023_native_submission_assembly.json).
- The pinned taroz runtime comparison is still unreproduced: the installed
  MATLAB lacks a working license. Native implementation and source inspection
  continue. No new external evaluation dataset has been added and no external
  submission has been made.

## Target and evidence boundary

Build native C++ GNSS/IMU inference that matches or exceeds a pinned
taroz/gsdc2023 run under the same inputs, output keys and scoring rules.
Published taroz scores (Public 0.789 m / Private 0.928 m) are reference
milestones, not substitutes for paired evaluation. Imported precomputed
results and sample coordinates do not count as native results.

Use existing GSDC data; do not add a new external evaluation dataset.
Group phones from the same drive together for evaluation. Record previous
development exposure rather than claiming those drives are unseen.
Prepare and validate submission artifacts locally; a new external submission
requires an explicit submission instruction. No deployment or merge is part
of this implementation instruction.

## Ordered work

1. Inventory all 40 historical test runs: device, input hashes, executable,
   settings, output origin, returned epochs and first failure reason.
2. Reproduce a non-Pixel5 initialization failure. Check raw timing, clock
   resets, initial position/velocity availability and stage epoch identity.
   Generalize device-specific code only after validating its assumptions.
3. Establish stable native output for all runs, with no crashes, complete
   submission keys and an explicit native fallback count.
4. Compare preprocessing, GNSS initialization, IMU attitude initialization
   and final optimization against upstream stage by stage. Keep upstream
   reference execution separate from candidate inference.
5. Freeze settings, compare official score plus per-drive/device P50/P95,
   missing outputs, fallback rate and runtime. Improve one cause at a time.
   Do not close the goal merely because a partial milestone is reached.

## Initial findings

The repository's latest submission narrative reports Public 3.460 m / Private
3.344 m, applying base-surveyed FGO to 8 Pixel5 runs and retaining v5 for
32 runs. It also describes sample-coordinate fallback; per-key provenance
must be audited before making an all-native accuracy claim.

[Initial 40-run inventory](use_cases/records/gsdc2023_native_taroz_kickoff_inventory.json)
is derived from the historical allowlist and submission narrative. It is not
a new replay or an independently verified submission composition.

The current Windows workspace contains historical logs but the searched
workspace/data locations have not yielded the raw GNSS CSVs or dataset ZIP.
The user has been asked for the data/execution location. Source analysis
continues while that information is pending.

The source has explicit Pixel5 timing/model guards. A separate raw seed
adapter emits `raw-p-result-input-epoch-count-mismatch` before checking
whether the upstream seed run failed. Thus that message alone does not
prove a timestamp alignment bug: inspect the original seed failure first.
Do not remove either the size check or device guards simply to obtain output.

### Initialization source audit

The Phase165 path calls `raw_p_seed::solve` with the default fail-fast policy,
then passes its result to `adaptSameRunNoDopplerSeeds`. The raw solver builds
its epoch vector during timestamp validation. Invalid configuration or an
early nonfinite, duplicate, nonmonotonic or over-limit-gap timestamp can
therefore leave fewer result epochs than input epochs. The adapter reports
the size mismatch before the original raw rejection. This is a possible
secondary symptom, not evidence that GNSS and IMU row counts differ.

No new native diagnostic is necessary for this branch: the existing
`makePhase165RawPNoDopplerGraphJson` exports `raw_failure_reason`,
`raw_failure_status`, `raw_input_epoch_count`, `raw_evaluated_epoch_count`,
`raw_failed_epoch_index` and `raw_failed_epoch_reason`. Obtain these fields
for a failing non-Pixel5 run before selecting a timing/segmentation fix.
Inspect `raw_unassessed_epoch_count` too; an early return is not a full-run
assessment. Existing `RawPSeedTest` cases cover duplicate, nonmonotonic,
over-gap and nonfinite time rejection.

The data search was repeated with gitignore filtering disabled, including
the main workspace's ignored `data` and `output` trees, `E:/datasets` and
`E:/kaggle_ws_data`. No GSDC raw CSV/archive was found there. The local
smartphone output tree contains 30 summary/log candidates from older work,
not the latest 40-run submission replay. This does not establish absence on
other disks or a remote server. The original execution-log location remains pending.

### Existing archive restoration and local build

The historical archive URL is accessible. The same dataset is being restored
from `https://www.taroz.net/data/dataset_2023.zip` into
`E:/rtklib_v2_ws_data/gsdc2023/cache/`. Its expected SHA-256 is
`bda30ab456e6fd6f83550c246e8dbd287306d5385f1f1069c99c16298e647408`.
Restoration completed: 2,761,355,999 bytes and exact expected SHA-256 match,
recorded in `cache/restore_manifest.json`. Download session `50783` exited 0.
This restores existing data; it does not introduce another evaluation dataset.
The archive has 1,048 entries and 81 raw GNSS/IMU file pairs.

Extracted ten raw/nav/base/settings files for existing development routes
`2021-07-27-19-49-us-ca-mtv-b/pixel4` and
`2021-08-24-20-32-us-ca-mtv-h/pixel5` into `inputs/dataset_2023/` beneath
the restored data root. `inputs/initial_extraction.json` records each hash.
No truth or MAT payload was read. Navigation/base files are at route level,
while raw GNSS/IMU CSVs are at route/phone level.

Candidate build: `E:/gnsspp-build-gsdc-native`, VS 2022 x64 / MSVC 14.44,
GTSAM `E:/gtsam/install`, Eigen/GTest from `C:/vcpkg/installed/x64-windows`.
Configuration succeeded. The `gnss_fgo_imu_no_base` target is building in
session `76054`; log: `E:/rtklib_v2_ws_tmp/gsdc_native_build.log`.
The only native edit so far selects `_getpid`/`process.h` on Windows in the
atomic output helper. Solver behavior is unchanged; build validation pending.
Poll the existing handles before restarting either operation.

### Pinned upstream inspection

Cloned upstream into `E:/rtklib_v2_ws_data/gsdc2023/upstream-source` and detached
at `29923f9f370f09ebc00f96d8cca375007a18e7d5`, matching the old source pin.
No precomputed output has been read. In its `fgo_gnss.m`, Motion/Clock edges
are conditional on UTC delta below `time_diff_th` (1.5 s in `parameters.m`),
whereas this native raw seed stage rejects an entire run at a gap above 2 s.
This is a candidate failure mechanism to verify with the restored input,
not a confirmed cause for any of the 40 runs. TDCP is outside that upstream
Motion/Clock gap condition; do not assume all temporal factors share it.

Upstream also varies clock and TDCP factors by device: some Samsung devices
omit ClockFactor_CCDD; some use drift-based TDCP with an offset; others omit
TDCP. `gnsslog2obs.m` has duplicate-block handling for sm-a205u/sm-a600t.
Capture these distinctions in paired preprocessing/factor comparisons before
generalizing the Pixel5-only path. A single shared flag relaxation would not
establish upstream parity.

### Completed raw UTC audit

`scripts/analysis/inventory_gsdc_raw_timing.py` hashed the restored archive
and read only test `device_gnss.csv` payloads, using clock columns only.
The archive has 40 raw test members. A set comparison found one historical
identity mismatch: `2022-04-25-22-36-us-ca-ebf-z` is listed as `mi8` in the
old allowlist but has `pixel5` raw data in the hash-matching archive. The
kickoff inventory now uses the archive identity and preserves the historical
identity explicitly. Official submission keys still need verification.
There are 72,050
raw UTC groups, no reverse/reappearing UTC groups, no invalid integer time
rows and no zero-TimeNanos rows. Only LAX-m/Pixel5 has UTC gaps above 2 s:
10, 45, 10 and 30 seconds. Consequently raw UTC gaps cannot explain a broad
non-Pixel5 failure; check native row filtering, reconstructed GPST and later
stage conditions before changing gap handling.

The two extracted development routes (Pixel4 MTV-b and Pixel5 MTV-h) have
1,678 and 3,140 raw UTC groups respectively, with no reversals or gaps above
2 s. Their unique-group counts were independently checked, and a synthetic
case verified duplicate row grouping, reappearing groups, reverse transitions
and gap detection. These counts are not native accepted epoch counts or the
official submission key count. No accuracy scores were computed.

Full output: `E:/rtklib_v2_ws_data/gsdc2023/test_raw_timing_inventory.json`;
per-run timing results are also retained in the kickoff inventory. Session
`14124` completed successfully. Build session `76054` remains the next native
execution dependency.

### Frozen first native replay

[Three initialization cases](use_cases/records/gsdc2023_initialization_replay_plan.json)
record argv and raw/nav/base hashes before execution: Pixel5 MTV-h control,
Pixel4 MTV-b with the exact control flags (expected device-guard rejection),
and Pixel4 MTV-b with the four Pixel5-specific selectors omitted, as described
by the earlier submission narrative. The latter is a diagnostic reduced
recipe, not a claim that model differences can safely be removed. Base station
and surveyed coordinates are selected by course/device/year from the restored
settings and base-position tables. No route-specific accuracy tuning was used.
These executions have not started; wait for build session `76054` to complete.
Prepared runner: `E:/rtklib_v2_ws_tmp/run_gsdc_initialization.py` (Python syntax
check passed). It verifies all input hashes and a fixed binary hash, runs the
two Pixel4 cases followed by the Pixel5 control serially, and writes argv,
PID/state, exit code, wall time and output hashes under
`E:/rtklib_v2_ws_output/gsdc_native/initialization_v1/`. It refuses existing
run directories. Do not launch before the build succeeds or reuse stale
executables from other branches. At the latest check, build handle `76054`
was live and compiler PID 47004 was accumulating CPU in solver code generation.

Build session `76054` subsequently completed with exit 0. The canonical target
linked with the repository's existing GTSAM `/FORCE:MULTIPLE` policy (duplicate
inline STL warnings). Frozen source/binary/build-log hashes are saved in
`E:/rtklib_v2_ws_output/gsdc_native/initial_build_manifest.json`.
Replay session `97405` is now running; log:
`E:/rtklib_v2_ws_tmp/gsdc_initialization_replay.log`.
Pixel4 exact-guard returned 2 as expected, naming the Pixel5-only Phase197
timing preset. Pixel4 reduced passed that guard and is executing; native PID
at launch is recorded in its `run.json`. Pixel5 control follows serially.
Keep the executable unchanged until all three cases are terminal.

### Reference runtime and scoring checks

MATLAB R2024a is installed, but a minimal `-batch` availability check exited
1 with License Manager Error -1 (license file not found). Evidence:
`E:/rtklib_v2_ws_tmp/gsdc_matlab_availability.log`. No upstream MATLAB
inference has run. The user has been asked for a usable reference environment;
this does not block the currently running native replay.

The existing `gnss_smartphone_kaggle.py` scorer explicitly distinguishes the
published per-phone P50/P95 aggregation from unspecified Earth-model and
percentile-interpolation details. Preserve its named WGS84/Vincenty and
spherical/Haversine local variants in paired analysis. Do not relabel local
diagnostics as an official Kaggle server score. Comparison against taroz on
the same local scorer remains useful, but external-score claims require the
actual submission result.

### Reproduced Samsung loader failure

A separate raw-P-only invocation on existing test
`2022-10-06-20-46-us-ca-sjc-r/sm-a205u` returned 1 before initialization:
`duplicate supported satellite/signal row in one raw epoch`. Evidence is in
`E:/rtklib_v2_ws_output/gsdc_native/samsung_raw_seed_v1/` (input and binary
hashes, exact argv, stderr, return code). No summary was generated because
conversion failed before the raw seed stage; this is not a solver failure.

The app constructs `AndroidRawGnssConfig` without setting `device_model`.
The loader already implements SM-A205U/SM-A600T duplicate handling, plus
device-specific ADR and GLONASS behavior, but this entry point never activates
them. Added assignment from the explicit dataset device identity. This source
fix is **not yet built or verified**. The original executable remains unchanged
while replay `97405` runs. Once it finishes, rebuild and rerun the identical
Samsung input, then check Pixel4/Pixel5 regression. Do not claim all Samsung
or upstream parity from resolving this one loader failure.

### First native results and device fix validation

Original replay `97405` completed: Pixel4 exact guard returned 2; Pixel4
reduced returned 0 in 573.391 s; Pixel5 control returned 3221225725
(`0xC00000FD`, Windows stack overflow) after 10.875 s. Do not score the
crashed control as a positioning result. Pixel4 accuracy has not been scored.

The device-routing fix was built separately using the unchanged canonical
libraries through `E:/rtklib_v2_ws_tmp/gsdc-device-app/CMakeLists.txt`;
build session `14103` completed successfully. The same SM-A205U raw/nav
inputs now return 0 and accept all 1,697 raw-P epochs, versus the baseline
loader duplicate failure. Evidence: `samsung_raw_seed_device_v2` under the
gsdc output root. This is a successful initialization regression, not full
GNSS/IMU or accuracy validation. The original executable was never replaced.

For the Pixel5 crash, copied the original executable to `stack_experiment`
and changed only the PE stack-reserve setting to 8 MiB using MSVC editbin.
Byte comparison verifies only stack-reserve/checksum fields changed; no
device fix or solver code is mixed into this experiment. The same Pixel5
argv is running in session `59726`, outputs `pixel5_stack8m_v1`. This is
an experiment, not yet a justified permanent build-setting change. Inspect
its terminal result before deciding on the fix.

### Verified partial results (2026-09-23)

Stack experiment `59726` returned 0 after 203.031 s. Changing only the stack
reserve resolves this Pixel5 failure. Its 3,139 output keys exactly match
3,139 truth keys; local Haversine/linear score is 0.576433 m (WGS84/linear
0.577578 m). Pixel4 reduced has 1,677/1,677 exact keys and local scores
1.034102 m / 1.032917 m respectively. Both were scored only after native
execution ended and candidate hashes were checked. These are development
results, not an official submission or taroz paired-run result.

Device-routing raw-P regression on Pixel4 and Pixel5 returned 0 for baseline
and fixed apps; each pair's entire summary JSON is byte-identical. Evidence:
`device_routing_regression/comparison.json`. These two phones are unaffected
by the loader's device-specific ADR/GLONASS rules.

Added an MSVC-only 8 MiB stack reserve to the canonical smartphone target.
Canonical combined device-routing/stack build is running in session `98384`,
log `E:/rtklib_v2_ws_tmp/gsdc_device_stack_build.log`. No inference is currently
running. Original baseline/device-only executables were preserved under
`E:/rtklib_v2_ws_output/gsdc_native/binaries/` before rebuilding.
[Portable first results](use_cases/records/gsdc2023_initialization_first_results.json)
retain run records, scores, initialization regression and archived binary hashes.
Next: verify the canonical build, then execute Samsung through the GNSS/IMU
stages and expand the per-device initialization audit. Full-goal success is
still unproven.

## Samsung graph diagnosis after canonical rebuild

The canonical device-routing / 8 MiB stack build completed successfully.
SM-A205U full inference accepted all 1,697 raw-P seeds, but retained only
one graph epoch and returned 1. Removing base correction produced the
same result. Diagnostic-only counters identified 33,230 adjacent P-D
rejections, zero missing seeds, and two centered P residual rejections.

The direct observable-quality configuration cleared the phone identity.
Upstream exobs_residuals.m explicitly skips the P-D screen for sm-a205u
and sm-a505u; the library already implements this rule. The direct
configuration now passes phoneFromDatasetId to that existing rule.
The legacy Phase13 configuration remains unchanged.

The rebuilt app progressed beyond the prior admission behavior (epoch 678
retained seven P observations) but exited with Windows code 3221226505
(0xC0000409), without a solution or summary. The exact crash site is not
yet known; this is not a successful Samsung inference. No test truth was
read. Next: locate the abnormal termination, verify the repaired graph
counts and complete Samsung inference, then run device regression.
Evidence: [Samsung run records](use_cases/records/gsdc2023_samsung_graph_diagnosis.json).
Pre-mask-routing diagnostic executable is archived in the local binaries
directory. Build log: E:/rtklib_v2_ws_tmp/gsdc_mask_routing_build.log.

## Samsung optimizer exception localized

The previous turn made progress: same-input builder diagnostics proved the
phone identity bug and the repaired run reached a new terminal condition.
No inference job remains running.

LLDB located a C++ exception in optimizeProblemWithGtsam, called from the
GNSS-first solve. The ordinary app path did not catch it. Added a fail-closed
std::exception handler that reports the message and writes the available
raw-P graph diagnostics, without returning seed positions as a solution.

Canonical rebuild succeeded. Same-input run samsung_exception_report_v1
returned 1 with the explicit Phase171 complete-recipe guard message.
All 1,697 epochs now remain, with 31,681 P rows and 13,830 staging D rows.
P-D rejections are zero; TDCP rows are still zero. The recipe guard explicitly
rejects nativeSourceClockC0DPhoneExcluded phones, including sm-a205u.

This guard cannot simply be removed to claim upstream parity. At pinned
upstream fgo_gnss.m lines 171-196, sm-a205u omits CCDD clock edges and uses
TDCPFactor_XXDD with Loffset, rather than XXCC. Native staging also currently
requires every adjacent CCDD edge to be eligible (backend line 339).
Next implementation must represent the phone-specific clock/TDCP recipe,
including raw carrier admission and appropriate gauge constraints, in both
GNSS-first and IMU-main stages. Then run Samsung inference and Pixel
regression. No Samsung truth, external submission, or new dataset was used.
Goal remains unachieved.

## Source drift TDCP factor and admission evidence

Added phone temporal-recipe mapping and Point3/Pose3 affine drift TDCP
factors. They implement LOS displacement from fixed anchors plus the
trapezoidal integral of two m/s drift states. These factors are **not yet
connected to the production graph**; Samsung inference still fails closed.

The pinned upstream gtsam_gnss revision e679b72b620fc6800723578e4077dd157587d129
TDCPFactor_XXDD header was compiled directly into a local comparison test.
All three tests passed, including residual and all Jacobian comparisons
over four durations, numerical Pose3 derivatives with nonzero antenna arm
and navigation transform, and invalid state rejection. The test is also
registered in the canonical test source list; the local isolated GTSAM
test target ran it with the upstream comparison macro enabled.

Same-input Samsung diagnostic run samsung_tdcp_admission_v1 completed with
return 1. All 10,759 TDCP candidate pairs were rejected by the legacy
code-phase-jump gate; none by gap, clock discontinuity or loss of lock.
The source exobs_residuals.m uses the Doppler-carrier difference and phone
offset. Integration must use that source admission for drift-family phones,
not blindly weaken all-phone gates. It must also omit source-excluded CCDD
edges and handle unobserved per-epoch clock components without fake data.

[Factor validation record](use_cases/records/gsdc2023_tdcp_drift_factor_validation.json).
No inference job is running. No new evaluation data or truth was used.
Next: integrate these verified factors and the corresponding phone-specific
admission/clock model, then rerun Samsung and Pixel development controls.

## Phone graph integration and all-40 initialization audit

Integrated the verified drift TDCP factors into native ECEF-D staging and
Phase171 Pose3 main, using raw UTC dt and the published 1.117 m offset.
Drift-family carrier admission keeps the upstream Doppler-carrier screen
but does not require code-clock continuity. Source-excluded phones omit
CCDD edges. Added weak gauges for locally unobserved C7 components and
the main-stage drift null mode; these are numerical priors, not observations.

First integrated Samsung run stopped at a 2-second gap before optimization.
The next build permits the source-excluded clock gap and omits the staging
motion row at UTC dt >= 1.5 s. The second run inserted 10,759 drift TDCP
rows. Session 52876 is now terminal, return 1 after 146.065 s:
E:/rtklib_v2_ws_output/gsdc_native/samsung_phone_graph_gap_v2.
GNSS-first returned 1,697 solution epochs after 359 iterations, but the
app handoff still requires native_source_clock_c0d_factor_count > 0
(line 13226), incompatible with this source-excluded phone. The log also
reports 356 lambda-floor failed trials; finite cost progress must be checked,
not inferred merely from the presence of output states. No final solution
was published. Main-stage checks at lines 13780 and 13864 also assume a
positive CCDD count and need the same phone-aware review. Canonical binary
is no longer held by a running Samsung process.

Pixel regression session 29131 uses an archived pre-gap-adjustment binary.
Pixel5 completed successfully and its solution is byte-identical to the
previous full native run. Pixel4 is still running (PID 63372).
The later gap adjustment is limited to the drift-phone branch.

Extracted raw GNSS and navigation only for all 40 existing test drives.
The fixed Phase149/157 raw-P audit completed all 40 (session 42804):
28 accepted, 7 corrected-geometry failures, 2 too-few-satellite failures,
2 time-gap failures, and 1 raw timing loader failure. This is initialization
coverage, not full-pipeline coverage. The MI8 loader failure includes a
negative GLONASS SV time at a day boundary (row 680); inspect time wrapping
and state validation rather than treating the whole drive as unusable.

[All-40 raw-P evidence](use_cases/records/gsdc2023_test_raw_seed_audit.json).
Next: poll Pixel4 in session 29131, then update the source-excluded-phone
handoff/main validation while retaining finite-cost, accepted-step and
identity checks. Re-run Samsung and address the 12 initialization failures. Full-goal success
is still not established. No new evaluation dataset or truth was used.

## First Samsung full native completion and input parser fixes

Pixel4 and Pixel5 full regression both completed successfully with
byte-identical solution CSVs (session 29131 terminal). Added phone-aware
CCDD-count checks to the Phase171 staging handoff and main output gate;
finite-cost, accepted-step, state coverage and identity checks remain.
Four temporal model/handoff tests pass.

The negative SV-time parser fix lets invalid signed transmit times reach
the existing upstream <1e10 ns row screen. MI8 MTV-m now accepts all 1,395
raw-P epochs. This raises verified initialization successes to 29/40
across the fixed audit plus this same-input follow-up. It is not a full
40-drive rerun on one final binary.

Samsung next exposed the shared IMU CSV splitter dropping final empty
fields. Preserving the trailing empty field fixes the 58-vs-57-column
error. A direct mapping probe succeeds on Samsung (1,699 mapping anchors).
New parser tests pass. Broad local tests have four raw-GNSS and two
UTC/GPS numerical-precision failures, with exactly identical failure
messages on the unmodified baseline. Do not describe all tests as green.
The large-nanosecond floating arithmetic needs a Windows precision fix.

Samsung session 81413 completed return 0 in 195.143 s, output directory
samsung_empty_csv_v4. It optimized 1,697 epochs and wrote 1,696 unique,
finite coordinate rows, matching all accepted raw keys after the first
warmup key; zero interpolation, edge hold and unresolved epochs.
GNSS initial/final costs are 2.73287e10 / 3.77816e6 (359 iterations).
Both stages inserted 10,759 drift TDCP factors. No truth was opened and
no official submission was made. This proves native execution, not taroz
accuracy parity. No job remains running.

IMPORTANT NEXT DIAGNOSTIC FIX: backend and app postfit TDCP reporting
still reconstruct the old clock-bias-difference residual. The reported
516.7 m RMS is therefore not the residual of the new drift factors.
Export/evaluate the actual inserted factor residuals, retain row identity,
and use them for both reports before interpreting residual statistics.
This does not change which factors were optimized.

[First completion evidence](use_cases/records/gsdc2023_samsung_first_complete.json).
The completed binary is archived as binaries/phone_temporal_csv_fixed.exe.
Next: correct residual diagnostics, address the remaining 11 raw-P
initialization failures and Windows time precision, expand full native
coverage and upstream comparison. Goal remains active and unachieved.

## Actual drift-factor residual diagnostics verified

Added identity-keyed initial/final residual export from the actual inserted
GTSAM drift factors. Both backend RMS and app runtime/per-signal reporting
consume this export. Cardinality, satellite, signal, epoch identity and
finiteness checks reject a mismatched handoff. This is diagnostic-only and
does not feed a later solver invocation. The summary labels the evaluation
as inserted-drift-factor.

Canonical full rebuild session 15166 completed successfully. Samsung
session 53312 completed return 0 in 189.393 s at samsung_factor_residual_v5.
Solution CSV and stdout are byte-identical to samsung_empty_csv_v4.
All 10,759 inserted factor residuals are finite; backend/app RMS both
equal 280.3276329557209 m. The old 516.7249164461647 m reconstruction used
the wrong clock model and is invalid for this phone. The actual residual
is still large (max 25,297.6 m; 827 Huber-tail rows). Do not mistake the
diagnostic repair or a completed trajectory for accuracy parity. Investigate
these observation outliers and source preprocessing alongside remaining
initialization coverage.

[Residual validation](use_cases/records/gsdc2023_drift_residual_validation.json).
The complete binary is archived as binaries/phone_drift_residual_fixed.exe.
No jobs remain running. No truth or additional evaluation data was used.
Next: Windows raw-time precision (integer-nanosecond differences and
relative-time fits; also correct synthetic fixtures that themselves create
large floating timestamps without relaxing tolerances), the 11 remaining
raw-P failures, and source-consistent outlier admission/full-drive expansion.
Goal is still active and unachieved.

## Integer Android time arithmetic checkpoint (2026-09-23)

MSVC long double is double precision. Subtracting floating ~1e18 ns
hardware clocks lost real timing information. The raw loader now subtracts
checked int64 values and splits integer GPS week/day remainders before
floating conversion. GNSS/UTC fitting uses relative GPS times, and fallback
IMU conversion starts from a split GPS reference instead of an absolute
floating nanosecond timestamp. Synthetic tests now construct exact integer
inputs and independent expected ranges; existing tolerances were not relaxed.

All 23 raw-loader and 23 IMU CSV tests pass, including new overflow and
1 ns GPS/GLONASS/BeiDou range cases. Native Release build succeeds with the
previously documented linker warnings. Binary SHA256:
`5ef26fa03a121632c59f2079d68216bb84f8f717a7e23296bafcf783d8780aaf`.

The full 40-drive raw-seed rerun is complete: 28 accepted, 7 corrected-geometry
failures, 2 insufficient-satellite failures, and 3 time-gap failures. MI8's
negative-SV-time case now passes. Samsung regresses at a strict 2 s gate:
epoch 1618 has GPS dt 2.0000000639702193 s but UTC dt 2000 ms. No gap
threshold or tolerance has been changed yet. This is a concrete remaining
boundary-contract bug, not grounds to claim successful full coverage.

Pixel5 completes natively in 256.969 s with all 3,139 truth keys matched.
Haversine/linear diagnostic changes from 0.576432758 m to 0.576232064 m;
WGS84/linear is 0.577181414 m. These are development diagnostics, not an
official score or paired taroz result. Pixel4 replay PID 28420 is still running
at this checkpoint; its parent runner session is 1607. Do not restart it.
Samsung integer-clock full replay failed before entering the graph, as above.

[Integer clock validation](use_cases/records/gsdc2023_integer_clock_validation.json).
Next: resolve the 2 s boundary consistently with the existing time-equality
contract and test both sides, then repeat Samsung; collect Pixel4 and finish
local scoring. Remaining geometry failures and source outliers still prevent
40-drive native completion. Goal remains active and unachieved.

## Gap equality and completed replays (2026-09-23)

Resolved the previous Samsung regression by applying the raw-P stage's existing
1 microsecond time-equality resolution at its max-gap boundary. The configured
2 s maximum and observed timestamps remain unchanged. The new boundary test
accepts -0.5 us, 0, +64 ns and +0.5 us offsets, and rejects +2 us and +1 ms.
All 35 raw-P seed/adapter tests pass. Canonical Release build succeeds.

Archived binary `binaries/integer_clock_gap_fixed.exe` SHA256:
`0dc022156675b50f72933363026194ed2114b6f1b2dafe554de1657e1bec0833`.
The complete 40-drive raw-seed audit on that exact binary accepts 29 drives;
7 fail corrected geometry, 3 fail satellite count and 1 fails the time-gap
contract. The April 2023 MTV gap now clears the boundary but still fails
satellite count, so it is not counted as newly successful.

Samsung full GNSS/IMU run completes in 202.214 s. Its 1,696 CSV rows match
accepted raw UTC keys after warmup exactly; finite coordinates and phone IDs
were independently checked. Output interpolation, edge hold and unresolved
counts are all zero, and device WLS coordinates are not used. No test truth
was opened. Drift TDCP factors now number 10,756 after exact-time preprocessing.
This proves execution and output coverage, not positioning accuracy.

Pixel4's prior live replay also completes: 628.320 s, 1,677/1,677 truth keys,
Haversine/linear 1.033916924 m (previous 1.034101799 m), WGS84/linear
1.032819294 m. Pixel5 is 0.576232064 m Haversine/linear, 3,139/3,139 keys.
These development replays use the integer-clock binary before the separate
gap-boundary change; their exact binary hashes are recorded. Neither is an
official score or same-input upstream comparison.

[Validation](use_cases/records/gsdc2023_gap_equality_validation.json).
All replay/audit sessions (1607, 18050, 99206, 52119) are terminal with runner
exit code zero; per-drive failures remain in records. No inference job remains.

### Remaining raw-seed failures

Ran the existing all-epoch diagnostic mode on the 11 remaining failed drives.
This independently cold-starts every epoch and disables velocity derivation;
its failures must not be confused with the sequential run's exact failure
mechanism. `scripts/analysis/summarize_gsdc_seed_failures.py` verifies complete
ordered diagnostic rows and records consecutive failure spans without creating
any seed or output interpolation.

[Failure spans](use_cases/records/gsdc2023_seed_failure_spans.json) show that
10 drives have only 1-3 consecutive failed epochs, bracketed by accepted
estimates 2-4 s apart. The February LAX-m Pixel5 drive has 37 failed epochs
in 18 spans and one accepted-endpoint separation of 57 s. Missing raw epochs
can make this separation much larger than the failed row count.

Next implementation should explicitly support same-run initialization through
short underdetermined spans and retain the real observations for subsequent
GNSS/IMU optimization. Do not manufacture satellite rows or silently mark an
interpolated initial guess as an independently solved SPP seed. Record seed
provenance, check endpoint clocks/identities and coverage, and distinguish
initial-guess completion from final output interpolation. The long LAX gap
needs separate temporal/segment treatment. Full native 40-drive processing,
source outlier/weight parity, executable upstream comparison and official
scoring remain incomplete; goal stays active.

## Explicit short-span numerical initialization (2026-09-23)

Added `raw_p_seed::initializeShortGaps` and the explicit native selector
`--native-temporal-seed-initialization` for Phase163 or Phase171. It consumes
complete independent raw-P diagnostics, preserves the original SPP statuses
and NaN failed coordinates, and produces a separate typed handoff. Interior
geometry/solver failures can receive a linear ECEF/clock numerical initial
guess from accepted GPS-reference SPP brackets at most 4 s apart in both GPS
and UTC. No endpoint extrapolation, time-gap failure completion, absent raw
clock drift, nonmonotonic/mismatched identity or unproven clock reference is
accepted. Velocities are gradients of the same-run initial positions. Original
observation rows are not changed by this helper, and D comes from the exact
original epoch. These are starting guesses, not new position measurements.

The typed seed marks `temporal_initial_guess` and original bracket source IDs.
The adapter reports their count separately from independent SPP successes.
C[0] availability for a completed epoch denotes a numerical guess, not an
independent clock estimate. A Phase171 invocation writes
`<summary>.initialization.json` before graph entry, including this provenance.
The old strict route remains the default. Full graph admission, source row
filtering, and final output coverage are still separately enforced.

All 36 raw-P seed/adapter tests pass. The new test covers two consecutive
failures, exact identities, preserved original failures, raw D provenance,
short-span interpolation, a smaller span limit, missing drift, unbracketed
failure, time-gap rejection and non-GPS endpoint rejection.

A separate native probe using the same source code tested the 11 remaining
failed drives: 10 accept initialization with 26 total numerical guesses; the
long LAX-m case still rejects. This is initialization evidence only, not 10
new complete FGO trajectories or a 39/40 full native result.
[Probe results](use_cases/records/gsdc2023_temporal_initialization_probe.json).

The canonical build is still live in session **59828**, compiler PID 36252
(last CPU 489.81 s). The public seed structure changed and FGO depends on its
header, so the full rebuild must finish before running the new app. Do not
launch the older executable as if it contains this feature, and do not restart
the live build. Log: `E:/rtklib_v2_ws_tmp/gsdc_temporal_seed_native_build.log`.
Probe session 14578 and test session 68629 are terminal.

Prepared the first complete S20 trial from the existing archive:
`2021-08-31-20-37-us-ca-mtv-e/sm-g988b`. P178 2021 ECEF is
(-2708471.0323, -4279023.3576, 3864681.6305). IMU and base hashes are in
`E:/rtklib_v2_ws_data/gsdc2023/s20_temporal_initialization_inputs.json`;
no test truth was opened. Frozen args are in `s20_temporal_replay_plan.json`
in that directory, using the Pixel4 reduced recipe plus explicit temporal
initialization and sparse-P staging.

After session 59828 succeeds, copy the canonical exe to
`E:/rtklib_v2_ws_output/gsdc_native/binaries/temporal_seed_fixed.exe`, then run
`python E:/rtklib_v2_ws_tmp/run_gsdc_s20_temporal.py` once. Its destination
`s20_temporal_seed_v1` has not been created/launched yet. Validate the one
completed initial guess, original 1140/1141 SPP successes, stage handoffs,
final key coverage, and actual optimized output. Fix any graph incompatibility
rather than treating the probe as completion. The long LAX gap, full40 native
runs, upstream comparison and official evaluation still remain. Goal active.

## S20 full success and full40 launch (2026-09-23)

Canonical build session 59828 completed successfully. Frozen binary
`binaries/temporal_seed_fixed.exe` SHA256 is
`a87d1775f3dc403e58c6237a5f00fca10d5059f5dba1a0baf76485d19f2c1b3b`.
S20 full replay session 17957 completed rc=0 in 162.916 s: independent SPP
1140/1141, one numerical initial guess at raw source 787 (brackets 786/788),
then 1140/1140 exact post-warmup output keys. Final coordinate interpolation,
edge hold and unresolved counts are zero. Input hashes, CSV phone/finite
coordinates and identity against the initialization sidecar were independently
checked. Main optimization accepted 26 iterations, cost 10663580.2003 to
16404.5926, with convergence-tolerance termination. This is a native trajectory
through a previously failing epoch, not a positioning accuracy score.
[S20 evidence](use_cases/records/gsdc2023_s20_temporal_full_replay.json).

Restored all40 IMU/base inputs from the same archive (1,701,037,579 additional
bytes); settings and year-specific ECEF bases match each drive. Local manifests:
`E:/rtklib_v2_ws_data/gsdc2023/test_full_native_inputs.json` and
`test_full_native_replay_plan.json`. Both are hashed in
[full40 kickoff](use_cases/records/gsdc2023_full_native_test_kickoff.json).

Full40 runner **session 49715** is live, two workers, output root
`E:/rtklib_v2_ws_output/gsdc_native/test_full_native_temporal_v1`.
First live children: 00 Pixel4XL PID 20384 and 01 Pixel5 PID 64436. Check
current run.json PIDs against processes before claiming completion or restart.
Every input hash is checked before launch. Ten diagnosed drives explicitly use
temporal initialization plus sparse-P staging; the other 30 use the strict
baseline. The S20 all40 recipe matches its validated pilot except output paths.
No official submission or test truth has been used. Do not claim full40
completion before collecting this audit and checking output keys/coverage.

Long LAX-m diagnosis: raw input itself has 10,45,10,30 s timestamp gaps;
source setting and raw inventory both have 2805 epochs. The 57 s bracket
contains a satellite-count failure plus two time-gap-gated rows. The separate `long_gap_probe` build session 73980 completed; run session
35521 terminates rc=1 as expected. At a diagnostic-only 60 s max gap,
2769/2805 epochs independently solve, leaving 36 failures. The 4 s initializer
rejects with `temporal-initialization-bracket-span-exceeds-limit`.
[Diagnostic](use_cases/records/gsdc2023_long_gap_diagnostic.json).
This does not alter the frozen production recipe and shows that increasing
a timestamp threshold alone cannot solve the drive.
Goal active; source parity, official evaluation and all40 completion unproven.

## Output audit and all-phone UTC interval correction (2026-09-23)

Added `scripts/analysis/audit_gsdc_native_outputs.py`: audits completed frozen
run hashes, exact ordered keys against original raw CSV UTC groups after the
explicit warmup exclusion, duplicate/missing keys, finite/in-range coordinates,
phone IDs, summary counts and temporal initialization provenance. It neither
scores test accuracy nor claims official key verification. Pending records are
not process-liveness evidence. Verified the prior S20 pilot and first three
completed all40 runs (00 Pixel4XL, 01 Pixel5, 02 S20) with this auditor.
All40 **session 49715 remains live**, now processing 03/04 (PIDs 33400/57776 at
last process check); frozen binary remains a87d1775... and must not be replaced.
Audit snapshot: `test_full_native_temporal_v1/output_audit.json`.

Pinned upstream `parameters.m:47` and `fgo_gnss.m:164-175` apply UTC dt and
strict dt<1.5 s motion/clock admission to every phone. Our native staging used
that motion interval only for drift-TDCP phones and rejected Gap edges for
ordinary phones. Changed source-phone staging/motion/CCDD timing to UTC and
allowed intentional Gap skips for all source phones. Nonpositive/nonfinite dt
and unsupported clock jumps remain rejected. Five temporal/factor tests pass,
including 1.499/1.5 s boundary, long gaps and invalid values.

Canonical build **session 7772** completed rc=0; log
`E:/rtklib_v2_ws_tmp/gsdc_all_phone_utc_native_build.log`. This is a separate
candidate from the frozen all40 run. After success, archive canonical exe as
`E:/rtklib_v2_ws_output/gsdc_native/binaries/all_phone_utc_fixed.exe` and run
`python E:/rtklib_v2_ws_tmp/run_gsdc_pixel5_source_utc.py` once.
Candidate `pixel5_gap_source_utc_v2` is now live in **session 37760**, PID 60008,
binary hash recorded in the checkpoint JSON. Do not restart it.
The archive copy and candidate launch described above are already done.

Control `pixel5_gap_control_v1` on the frozen temporal binary completed rc=1
(session 14388 terminal), reproducing the April 2023 MTV Pixel5 failure:
clock edge dt=3.00000022 s; raw 1357 epochs versus retained 1356; no GNSS-first
solutions. The dropped epoch is a separate remaining coverage issue to inspect
after the source interval fix. Do not count mere graph admission as full output
coverage. [Checkpoint](use_cases/records/gsdc2023_all_phone_utc_gap_checkpoint.json).
Latest frozen all40 audit: 4 exact-raw-key successes, 3 execution failures,
33 without completion records. Live all40 children at last check were 04
PID 57776 and 08 PID 58544; session 49715 remains live. MI8 cases 05/07
fail because raw receiver clock drift is missing. Pixel6pro case 06 hits the
same non-Samsung clock-gap rejection, with 2 raw epochs dropped. These need
separate fixes/verified reruns. Do not classify missing raw D as a position
failure or replace it with an unrecorded zero; investigate source/native
Doppler initialization and preserve provenance.
Goal remains active; full40 completion and taroz parity unproven.

## Sparse state retention and Mi8 source clock drift (2026-09-23)

UTC interval candidate `pixel5_gap_source_utc_v2` finished rc=0 (264.832 s),
but retained only 1356/1357 states and interpolated one final output. Fixed
`--native-sparse-p-staging` wiring: it now sets builder
`retain_sparse_epochs_for_imu` as well as backend admission. Candidate
`mi8_sparse_regression_v1/pixel5_sparse` finishes rc=0 in 352.032 s, keeps
1357 states including one empty-observation epoch, and emits all 1356 target
keys with zero output interpolation/hold/unresolved counts. Independent CSV
raw-key/hash/coordinate audit passes. The auditor now explicitly distinguishes
raw-key coverage from the fraction originating at exact native states.

Added `source_mi8_clock_drift.hpp`, porting pinned upstream preprocessing.m
Mi8/xiaomimi8 raw-clock gradient, >1000 magnitude mask, linear fill/extrap,
>50 adjacent-rate mask on both endpoints and subsequent fills. Unit sample
spacing is intentional (`gradient(obs.clk)`), not a new Doppler measurement or
position-derived rate. All-missing support fails. Four unit tests pass.
MATLAB endpoint-fill semantics checked against official fillmissing docs:
https://www.mathworks.com/help/matlab/ref/fillmissing.html .
This is applied only to these phones under the existing explicit native
source-direct-quality selector. Raw input files remain unchanged; initialization
sidecars label the derived rate and full summaries record repair counts.

Build 35273 completed; `binaries/mi8_sparse_fixed.exe` SHA256
`b71da06b2d381e8333d627d6fc23413659e35d70f35c0d9fc12cc9df549a5524`.
Regression **session 77932** remains live only for September Mi8 (PID 2832,
CPU 637.59 s at last verified check). November Mi8 ends rc=1 after successful
GNSS-first optimization: 99 accepted iterations, cost ~4.07012e8 to 15490.5.
Its prior missing-clock-drift blocker is fixed. Pixel5 sparse trial is terminal.
[Evidence](use_cases/records/gsdc2023_mi8_drift_sparse_checkpoint.json).

Added unconditional error reason + small failure JSON at the IMU initialization
failure boundary (previously only generic stderr was emitted). Build 24726
completed and archived `binaries/imu_failure_diagnostic.exe`; diagnostic replay
`mi8_imu_failure_v2`, session 23402, completed rc=1 in 100.237 s. Exact failure:
`leveling window is not stationary/low-dynamics under frozen gravity gate`.
Its GNSS-to-UTC mapping separately passes (1395 anchors, -0.0893752 ppm).
A raw first-250-acceleration preview has mean norm 7.0873 and std 3.3244;
this preview is not the gyro-synchronized gate calculation. No truth was used.

Next fix: native `buildImuInput` always enforces first-250 static leveling and
calls `fusion_initialization::alignStatic`, retaining its acceleration/gyro bias
estimates. Android then overwrites attitude with velocity-based vel2rpy anyway.
Pinned upstream `fgo_gnss_imu.m` initializes first-pass rpy with vel2rpy and
sets bias to zero (`imuBiasZero`, line 152); vel2rpy uses zero roll/pitch and
smoothed-velocity heading. Implement an explicitly scoped source initialization
path, preserving raw finite/time/heading checks and recording its provenance,
then re-evaluate this moving-start Mi8 and development-route accuracy. Do not
simply increase the static gravity thresholds or silently use a fallback.

Frozen full40 **session 49715** is still live. Latest audited snapshot: 7 raw-key
successes, 4 failures, 29 pending. Live children at last authoritative check:
11 LAX-p Pixel5 PID 6924 (797.34 CPU s), 12 LAX-i Pixel5 PID 13168 (675.02 s).
This frozen run does not include later UTC, sparse-retention or Mi8 repairs.
Do not restart live jobs. Goal active; full native coverage/parity unproven.

## References

- [Upstream](https://github.com/taroz/gsdc2023)
- [Upstream multipass entry point](https://github.com/taroz/gsdc2023/blob/main/run_fgo.m)
- [Historical submission](use_cases/records/smartphone_native_base_surveyed_test_submission_v1.md)
- [Pixel5 development results](use_cases/records/smartphone_base_surveyed_route_results_v1.md)

## Source velocity attitude and zero bias initialization (2026-09-23)

The November Mi8 control failed the native first-250-sample stationary
leveling gate after completing GNSS initialization. Pinned upstream
`fgo_gnss_imu.m` instead inserts velocity-derived attitudes and zero IMU
biases. The explicit Android Phase171 option
`--native-source-imu-initialization` now follows that initialization:
nearest heading filling, per-epoch attitude seeds, zero accel/gyro bias.
It retains finite-sample, timing, origin and heading-observability checks.
The old static initialization remains the default; stationary gyro bias
estimation cannot be combined with this option. Summary JSON records the
selected source initialization.

Release build and all 21 existing fusion initialization tests passed
(including source nearest-heading fill and invalid-input rejection).
A frozen candidate is replaying November Mi8
and the two existing Pixel4/Pixel5 development routes under
`source_imu_regression_v1`; completion and accuracy are still pending.
The separate original all40 run remains unchanged. Its current independent
raw-key audit has 8 verified, 5 execution failures and 27 without completion
records (including running and queued work). This is neither official
submission-key validation nor accuracy parity. See the source IMU checkpoint
record for the binary hash and validation state.

### Moving-start Mi8 replay result

November Mi8 completed with return code 0 in 363.949 s using frozen
`source_imu_fixed.exe` (SHA-256
`1d2665469ff5f7186c69e9f6a4f9a00869430b3f82196782424c69b2772ced78`).
All 1,395 raw epochs had independent SPP initial states; the final 1,394
post-warmup output keys match the original Raw UTC groups exactly, with
zero missing/extra/duplicate keys, interpolation, edge hold or unresolved
positions. No device WLS coordinates were used. This proves output coverage,
not position accuracy. The main solve accepted 15 iterations and reduced
cost from 20,298,575.7933 to 19,197.3008. The initialization report confirms
source velocity attitude/zero bias and 175 nearest-filled low-speed headings.
Pixel4/Pixel5 accuracy regressions remain running.

Pixel6Pro's November drive is also being replayed under
`pixel6pro_sparse_regression_v1` with the already frozen Mi8/sparse binary
and `--native-sparse-p-staging`, without the new source IMU option. The
old run rejected a 3-second clock edge after dropping two sparse epochs.
No binary or configuration of the original all40 batch was changed.

Source review also confirms a remaining Samsung preprocessing discrepancy:
`preprocessing.m:139-160` derives both receiver clock drift (GPS L1 Doppler
residual median) and clock bias (GPS L1 code residual median) at the source
initial trajectory. Native presently uses raw clock drift for these phones.
An aligned repair must use the own native initial trajectory, retain its
provenance, and compare the sign, units, masks and fill rules before changing
factor screening. Imported device WLS positions cannot count as native.

### September Mi8 completed; another sparse Pixel5 failure classified

The September Mi8 clock-gradient/sparse-state candidate completed in
1,687.331 s (return code 0). Its 2,478 post-warmup output keys exactly match
2,479 original Raw epochs minus the declared warmup. All coordinates are
finite and every output comes from a native state, with zero interpolation,
edge hold, unresolved, missing, extra or duplicate keys. This frozen binary
predates the new source IMU initialization option. Its 28-minute runtime is
recorded rather than hidden; no position-accuracy result is claimed.
Evidence: `mi8_sparse_regression_v1/mi8_sep/output_audit.json` and the Mi8
clock/sparse checkpoint record.

The original all40 case 15 (2022-03-22-18-44-us-ca-mtv-pe1/pixel5) rejected
a 2-second clock edge after retaining 2,111 of 2,112 raw epochs. This is the
same sparse-state/UTC-gap failure class as the corrected April Pixel5 case.
Its retry uses explicit sparse staging with the fixed UTC handling;
the failed original record remains in the all40 audit.

### Pixel4 source-initialization development score

The candidate completed in 724.002 s. All 1,677 truth keys match, with full
Raw-key coverage and no output interpolation. On the already-exposed
Pixel4 development route, Haversine/linear (P50+P95)/2 improves from
1.0339169239 m to 0.7079091813 m; P50 changes from 0.6889283873 to
0.6312931638 m and P95 from 1.3789054605 to 0.7845251988 m. WGS84/linear
score changes from 1.0328192942 to 0.7075712028 m. These are local metric
variants, not an official Kaggle score or a comparison with a taroz replay.

Because the historical control predates other timing fixes, causality has
not yet been isolated. `source_imu_control_v1` is now running Pixel4 and
Pixel5 with the exact candidate binary and the new source initialization
flag absent. Retain both controls and report the matched-binary result.
The Pixel5 source-initialization candidate is still running.

### Pixel5 source-initialization development score

Pixel5 completed in 429.341 s with all 3,139 expected keys and zero output
interpolation/hold. Haversine/linear (P50+P95)/2 changes from 0.5762320637 m
to 0.5764189113 m (0.0001868476 m worse). Candidate P50 is 0.3671594634 m
and P95 0.7856783592 m. WGS84/linear changes from 0.5771814145 m to
0.5775529825 m. This route does not show an improvement over the historical
control. Both same-binary controls remain running to isolate initialization
from other intervening fixes. Source initialization is still opt-in; these
local development scores do not establish taroz parity.

### Samsung clock-drift diagnostic using native initial trajectory

For sm-a205u (2022-10-06-20-46-us-ca-sjc-r), a separate native diagnostic
completed in 8.896 s and estimated a GPS L1 residual median at all 1,697
epochs using same-invocation raw-P SPP position-gradient velocities.
The supplied raw drift has median 18,522.727373 m/s; the GPS L1 residual
median sequence has median 206.592165 m/s. Their absolute difference has
median 18,315.800659 m/s. No adjacent residual-median jump exceeds 50 m/s.
This strongly motivates the source phone-specific drift replacement, but
does not by itself prove an upstream-equivalent implementation.

The probe did not change the estimator or access truth/device WLS
coordinates. It uses the existing native transmit-time/Sagnac convention,
GPS L1 raw rates and source-like P/SNR eligibility. Upstream's trajectory
and satellite-state conventions still need paired verification; source
jump-mask/fill rules must be preserved in an actual candidate. Evidence,
input/binary/source hashes and limits are in
`gsdc2023_samsung_clock_drift_diagnostic.json`. The next change is an explicit
Samsung drift-preprocessing option using the native initial trajectory,
with per-epoch provenance and a same-input replay. Keep the raw-drift
control and do not present the diagnostic as a corrected solution.

### Explicit Samsung drift preprocessing candidate

`--native-samsung-clock-drift` is now implemented for source-listed Samsung
phones in the Android Phase171 raw-P graph. It computes GPS L1 rate residuals
at the same-invocation native initial trajectory, takes the finite median,
masks both sides of adjacent jumps strictly above 50 m/s, and applies the
source linear/nearest fill ordering. It changes the selected epoch drift
and typed graph initial drift, preserving raw measurement rows. The default
remains unchanged. A per-epoch `.samsung-clock.json` records UTC identity,
raw drift, eligible row count, residual median and selected value.

Release build and five targeted tests passed (phone routing, nearest fill,
linear/extrapolated jump fill, strict threshold/no-anchor rejection, and
receiver/satellite motion sign and units). Frozen binary SHA-256 is
`3e6a92978c3042554126a7535c2e900ac1799bc667993ee99730dff0bd2ee474`.
`samsung_drift_regression_v1`
is running control and candidate with identical binary and input hashes;
the only inference argument difference is the new flag. All 1,697 preprocessed
medians exactly match the independent diagnostic, with no masks/fills on
this drive. Final output coverage, residual change and runtime are pending.

The output auditor now requires complete, finite and hash-verified Samsung
clock provenance when this flag is present. This is drift-only: source
clock-bias median replacement and paired upstream satellite-state/trajectory
equivalence remain incomplete. The raw adapter still requires a finite
incoming drift before this stage. See
`gsdc2023_samsung_clock_drift_candidate.json` for the precise checkpoint.

### Samsung paired drift result and key-scope correction

Both same-binary Samsung runs completed. Control/candidate TDCP RMS is
8.9433409118 / 8.9428147286 m, with maximum residual still about 883 m.
The coordinate difference between their solutions is only 0.0000445 m
median and 0.0002572 m maximum; these are differences, not position errors.
Thus the large incoming raw drift discrepancy is largely removed by the
existing GNSS optimization, and this preprocessing change does not establish
a material final-solution improvement on this route. Candidate/control
runtimes were 393.802 / 367.048 s. Retain the source-alignment candidate
as opt-in, and do not claim an accuracy gain.

Independent audit exposed a scope distinction: the original Samsung CSV has
1,699 Raw UTC groups; the loader accepts 1,697 and outputs 1,696 after warmup.
The first two original groups (1665089219996, 1665089221001) have no valid
ReceivedSvTimeNanos and BiasUncertaintyNanos=2,000,021.75; upstream
preprocessing also removes rows above 10,000. Source settings declare
Nepoch=1,697. Matching accepted keys is therefore not proof of matching all
original CSV keys or official submission keys. The independent full-raw-key
auditor correctly reports coverage-failed. The earlier gap validation record
now explicitly names loader-selected keys and marks original-CSV coverage
false. Samsung preprocessing provenance is validated against its declared
selected epoch set; it cannot turn the separate full-raw-key failure into a pass.

The March Pixel5 sparse retry completed in 1,158.775 s with 2,112 graph
epochs and 2,111 exact post-warmup original Raw keys; zero interpolation,
hold, missing/extra or duplicate keys. Accuracy remains unevaluated.

### Source-setting audit: replace route-name heuristic next

Current Phase80 robust thresholds use a route-name heuristic outside four
fixed development IDs. Comparing all40 manifest source settings with the
actual current code identifies 29 P/D robust-threshold mismatches, six
elevation-mask mismatches (source non-L5 10 degrees versus native 5),
and 16 velocity-motion sigma mismatches. The detailed per-route record is
`gsdc2023_source_setting_differences.json`. The source BDS field is metadata
in the inspected code; do not infer an active exclusion from its name alone.

Next implementation should consume explicit source Type/L5 metadata with
provenance, rather than guessing Type from route names, and apply the
source Pixel4 Doppler exception, motion sigma and phone IMU noise deliberately.
Keep source initflag-specific P/D residual screens and the multi-pass IMU
attitude schedule as separate unresolved parity requirements. No running
binary or original all40 plan was changed by this audit.

### Explicit source Type/L5 candidate and matched Pixel4 initialization result

The same-binary Pixel4 initialization control completed and reproduces
Haversine/linear 1.0339169239 m exactly, versus 0.7079091813 m with source
velocity-attitude/zero-bias initialization. The previously reported improvement
is therefore confirmed in the matched-binary comparison on this already
exposed development route. Pixel5's matched comparison remains a tiny
regression, not an improvement. Neither result establishes taroz parity.

New explicit `--native-source-route-type Highway|Street|Mix` and
`--native-source-l5 0|1` options select source P/D Huber thresholds, elevation
mask and velocity-motion sigma, with Pixel4 D and Mi8 motion exceptions.
They require paired valid metadata, Android Phase171 and direct source
quality. The historical no-option behavior is unchanged. Summary JSON
records the selected Type/L5 and numeric settings. Existing explicit TDCP
Huber selectors also consume the supplied Type, but are not enabled by the
new options alone. Phone-specific IMU noise and source initflag masks remain
separate missing parity work.

The Release build passed after avoiding MSVC's nesting limit in the legacy
CLI else-if chain: new options use independent early branches. Four route
settings tests and three invalid/incomplete CLI cases pass. Frozen
`source_settings_fixed.exe` is replaying Pixel4 (actual source Highway/L5=1)
and Pixel5 (Street/L5=1) with source IMU initialization, followed by matched
binary controls under `source_settings_regression_v1`. Source metadata
hashes and scope are in `gsdc2023_source_settings_candidate.json`.

### Pixel6Pro sparse-state replay completed

The November Pixel6Pro retry completed in 2,125.226 s. Independent raw-key
audit confirms all 1,445 post-warmup outputs for 1,446 raw epochs, with
zero missing, extra or duplicate keys, interpolation, hold or unresolved
positions. Every output is from an exact native state. This uses the
frozen `mi8_sparse_fixed.exe` and explicit sparse staging, not the new
source Type/L5 candidate. Position accuracy and official test-key equality
remain unverified. See `gsdc2023_pixel6pro_sparse_replay.json`.

### Combined retry coverage audit

The new `scripts/analysis/summarize_gsdc_native_replays.py` aggregates all40
original attempts and explicitly supplied retry roots without selecting
coordinates or blending outputs. It rejects changed benchmark input paths,
excludes train/development drives from the test count, preserves every
attempt, and counts unique drives with independently audited native Raw-key
coverage. The current result is 17/40 across several frozen variants:
12 original-binary, four Mi8/sparse-binary, and one source-IMU-binary drives.
This is not a single-recipe all40 result or official test-key/accuracy proof.

The underlying auditor now requires converged Phase171 IMU inference,
raw-clock-only input, no truth/device-WLS seed and no output interpolation
for its native-success classification. Full key coverage remains a separate
property; a matching CSV with an imported seed or fallback cannot become a
native success. Three targeted tests cover that distinction and mismatched
replay arguments. They pass, and the stricter aggregate still verifies 17.

The newly completed failures for March xiaomimi8 and April mi8 lacked raw
clock drift, while April 27 EBF-zz Pixel5 dropped one sparse epoch. These
three are replaying in `remaining_known_failures_v1` using the previously
verified source-IMU binary, sparse staging, and source attitude initialization
for Mi8 only. Source Type/L5 is not mixed into these recovery comparisons.
The original frozen batch and the separate Type/L5 development comparisons
continue unchanged. See `gsdc2023_combined_replay_coverage_checkpoint.json`.

### Source Type/L5 candidate scores and long-gap sensor evidence

Both Type/L5 development candidates completed with native Raw-key coverage.
Pixel5 Haversine/linear score is 0.5763756947 m (499.356 s), versus the prior
source-IMU candidate's 0.5764189113 m. Pixel4 is 0.8453631253 m (577.501 s),
worse than its prior 0.7079091813 m. Actual summary settings confirm
Highway P/D Huber=0.2/0.2 for Pixel4 and Street motion sigma=0.05 for Pixel5.
Matched-binary controls are still running. Keep this source-alignment
candidate explicit; do not claim a general accuracy gain. A remaining known
difference is TDCP Huber=4.0 versus source Type-dependent 0.2/0.5; the existing
Phase184 selector can test this using the new explicit metadata. Cold-start
P/D residual masks and multi-stage IMU attitude remain distinct differences.

For the previously blocked LAX-m drive, all four GNSS gaps (10,45,10,30 s)
have continuous raw IMU coverage: maximum sample spacing is 24 ms for
acceleration and 20 ms for gyro. The 57-second native-SPP bracketing span
contains 3,483 accelerometer and 2,902 gyro timestamps. A separate diagnostic
copy of the mapping code changed only its 5-second anchor-gap ceiling to
60 seconds; it then accepts all 2,805 anchors, constant hardware clock,
0.009117874 ppm drift, and maximum affine-fit residual 0.009075349 ms.
Production mapping code remains unchanged.

This supports investigating an explicit IMU-supported long-gap path rather
than attributing the failure to unavailable sensors. It does not yet solve
the 4-second initialization limit or establish observability of sparse
long-gap states in the GNSS-first graph (which has no IMU factors). Those
constraints must be addressed and validated before accepting a full replay.
Evidence and diagnostic hashes are in `gsdc2023_long_gap_sensor_coverage.json`.

### Matched Type/L5 result, Xiaomi completion, and bounded-gap diagnostic

Matched Type/L5 controls completed: Pixel4 changes 0.7079091813 ->
0.8453631253 m and Pixel5 0.5764189113 -> 0.5763756947 m (Haversine/linear).
Inputs, binary hashes and all arguments except the two metadata options
match. This confirms the Pixel4 regression. No default promotion is made.
An existing explicit Phase184 TDCP-Huber selector is now being tested on
the same source-settings binary, with k=0.5 (Highway Pixel4) or 0.2 (Street
Pixel5) versus the control's legacy k=4.0. This single-flag comparison uses
one worker and is recorded in `gsdc2023_source_tdcp_k_candidate.json`.

March xiaomimi8 completed in 837.250 s with all 1,678 post-warmup Raw keys
from 1,679 original epochs, native contract verified, no interpolation/hold,
and no missing/extra/duplicate keys. Accuracy is unevaluated. Evidence is
`gsdc2023_xiaomimi8_march_replay.json`.

A separately compiled full-app diagnostic for LAX-m changes only the copied
SPP time-gap, temporal-initialization bracket and mapping anchor-gap bounds
to 60 s and enables the existing temporal/sparse options. Production source
is unchanged. It accepts 2,769 independent SPP seeds plus 36 explicitly
marked numerical initial guesses and advances into GNSS optimization. The
36 remain rejected SPP epochs in the original diagnostic; they are not new
measurements or successful standalone SPP solutions. Main IMU completion,
rank/observability and full-key output are unproven. Any promoted policy must
be opt-in and bounded with actual sensor-coverage checks, not a global guard
relaxation. The live run is `long_gap_native_diagnostic_v1`; hashes and
requirements are in `gsdc2023_long_gap_native_diagnostic.json`.

### Coverage checkpoint: 22 distinct test drives

The aggregate raw-key audit now verifies native output coverage for 22/40 distinct
test drives across explicitly retained binary/configuration variants. This is
not a homogeneous all-drive recipe, official submission-key verification, or
position-accuracy comparison with taroz. See
`use_cases/records/gsdc2023_combined_replay_coverage_checkpoint.json`.

The April 27, 2022 EBF Pixel5 sparse retry completed in 632.705 s. It preserves
all 1,314 output keys after the declared first warmup epoch, with no output
interpolation, holds, or unresolved epochs. The 1,315 raw epochs contain 1,313
independently accepted SPP seeds and two explicitly recorded temporal initial
guesses. The audit verifies the native IMU estimation contract; no ground truth
was evaluated. See `use_cases/records/gsdc2023_pixel5_april_sparse_replay.json`.

The long-gap LAX diagnostic and source TDCP robust-loss comparison remain
running at this checkpoint; neither is promoted or included as a new success.

### Additional Mi8 failure replay

The 2022-04-27 EBF Mi8 route is now replaying with the frozen source-IMU
initialization binary, source MI8 clock-gradient preprocessing, and sparse-state
retention already tested on other routes. All four benchmark input hashes were
verified before execution. The former missing-clock-drift rejection has been
passed; final convergence and output coverage remain unproven. This pending
run does not increase the 22/40 audited coverage count. See
`use_cases/records/gsdc2023_mi8_april27_replay.json`.

### April 25 Mi8 completion and mapped-IMU gap witness

The April 25 EBF Mi8 retry completed in 1789.171 s with all 1,923 expected
output keys after the first warmup epoch, 1,924 independently accepted SPP
seeds, and no output interpolation, holds, or unresolved epochs. Native IMU
estimation provenance and frozen artifact hashes pass the output audit.
No position truth was evaluated. See `use_cases/records/gsdc2023_mi8_april25_replay.json`.

A separate diagnostic using the actual Android IMU adapter and the bounded
60-second UTC/GPS mapping diagnostic confirms 147,430 paired samples with
zero omissions and strictly increasing mapped timestamps. The 57-second LAX
initialization bracket contains 2,902 samples, has both endpoints covered,
and has a maximum adjacent mapped-sample gap of 0.0200000002 s (whole route:
0.0210000002 s). This strengthens the raw-sensor witness without claiming
final solver success. The current native integration loop can hold samples
across gaps; any production long-gap option therefore needs an explicit
paired/mapped-sample continuity check before accepting this relaxation.
See `use_cases/records/gsdc2023_long_gap_sensor_coverage.json`.

The refreshed aggregate verifies native raw-key coverage on **23/40** distinct
test drives across variants. The official-key and paired-taroz accuracy gates
remain unproven. Pending runs are excluded from this count.

### Official key authority recovered from existing checkout

The pinned upstream checkout contains `results/sample_submission.csv`. Its
CRLF-normalized-to-LF bytes exactly match the archived authenticated Kaggle
artifact: SHA-256 `b0c4853076f715d6bdca46e5c8c99e575f7982d8ee4fc9c0fe417507badcb780`,
5,286,788 bytes, 71,936 unique ordered keys across 40 drives. The raw Windows
checkout differs only by CRLF line endings. No network download or new dataset
was needed. Only trip IDs and UTC keys were extracted for output comparison;
sample coordinates are not used for estimation. See
`use_cases/records/gsdc2023_official_key_recovery.json`.

This corrects the earlier assumption that official key authority was unavailable.
Of the completed attempts checked at this snapshot, **10 distinct drives**
have every official key plus a verified native estimation contract and no output
interpolation/hold. The existing 23/40 figure describes raw-key coverage after
the declared first warmup exclusion, not submission completeness. Many routes
require that first timestamp in the official sample. Some also have extra raw
tail keys that must be removed when assembling an ordered submission. The
Samsung A205U requires the two previously identified unobservable raw epochs
and the loader's first epoch, so first-epoch inclusion alone cannot fix it.

The existing `--android-include-first-native-epoch` option is now being tested
on the November Mi8 using the same frozen source-IMU binary and inputs as the
earlier successful replay. It changes output selection only. Final native
first-state availability is not assumed until the replay and key audit pass.

### Official-key audit integration and long-gap diagnostic completion

Both the native-output auditor and replay aggregator now accept an official
sample together with its independently recorded SHA-256. The loader normalizes
CRLF to LF, verifies the digest, and reads only trip IDs and UTC keys. Per-drive
reports distinguish required keys, extra nonofficial tail keys, ordered required
coverage, exact submission order, and native provenance without interpolation
or holds. A raw warmup-excluding success cannot silently count as official
completeness. Six auditor tests pass, including missing first key, extra tail,
fallback rejection, normalized authority hash, and duplicate official-key cases.

The LAX long-gap diagnostic completed in 1172.092 s. Its main graph converged
in 24 iterations, reducing cost from 42,941,053.36 to 56,970.88. All 2,804 raw
output keys after warmup are native states, with no interpolation, holds, or
unresolved epochs; 36 temporal initial guesses retain their separate provenance.
The official set has 2,805 keys and still requires UTC 1645655744438. This
diagnostic remains outside the accepted aggregate while the production option,
runtime mapped-IMU continuity check, and first-state output are implemented.

The Pixel4 paired source-TDCP Huber comparison finished on the same binary and
inputs. Haversine linear P50/P95 mean increased from 0.8453631253 m to
0.9209424046 m. This is previously exposed development data, not a taroz paired
or official Kaggle score. The candidate is not promoted; Pixel5 remains running.

### First-state output verified; bounded long-gap candidate implemented

November Mi8 with `--android-include-first-native-epoch` completed in 266.477 s
and matches all 1,395 official keys with exact native states, no interpolation,
holds, or unresolved epochs. Same binary and normalized arguments match the
previous run; the previous 1,394 CSV rows equal the new rows after the first
exactly. A batch of 13 other native-success routes missing one official key
is replaying with this output flag, preserving each run's frozen binary and
other arguments. All four input hashes are checked before each run.

The production-source long-gap candidate is explicit and disabled by default:
`--native-imu-supported-long-gaps` requires Phase171, temporal initialization
and sparse staging. It bounds SPP time gaps, temporal initialization brackets
and the UTC/GPS anchor gaps at 60 s. Before accepting IMU input, the paired
mapped samples must bracket the entire graph interval, be finite and strictly
increasing, and have no overlapping sample interval over 50 ms (1 ns numeric
tolerance). This check also covers temporal brackets spanning many short GNSS
intervals. Failures prevent native output. The two-argument mapping API retains
its 5 s bound; the explicit overload rejects bounds beyond 60 s.

The IMU suite passes 24 tests and the output auditor passes seven, including
runtime long-gap coverage proof requirements. The full app build and bounded
long-gap real-data replay remain pending at this checkpoint; no new default
or long-gap coverage success is claimed from implementation alone.

### Samsung disabled-TDCP output gate corrected

The June 28 Samsung A32 full replay stopped after optimization because the
output gate required every constructed TDCP candidate to have been inserted
(10,635 built, zero inserted). The backend intentionally disables TDCP for
`samsunga32` and `sm-a325f` under the source phone recipe. The app's runtime
report now identifies this selection, preserves candidate counts, skips
hypothetical post-fit residual evaluation, and requires zero inserted factors.
Other phones retain the positive equal-count gate and finite-residual check.
This changes validation/reporting, not the graph model. The app is building;
real-data verification on both source-disabled phones is pending. See
`use_cases/records/gsdc2023_disabled_tdcp_phone_contract.json`.

### First-epoch batch comparison automated

`scripts/analysis/compare_gsdc_first_epoch_replays.py` verifies the same frozen
binary and arguments except output paths and first-epoch inclusion, hashes
the prior frozen solution, runs the native/official-key audit, and compares
every previous CSV row with the new output after its first row. The August
31 Samsung S20 replay passes: all 1,141 official keys are native, and the
previous 1,140 rows are identical. Pending runs are explicitly left unverified.
The batch preserves per-run variants and is not a homogeneous all-drive
recipe or an accuracy claim.

The mapped-IMU probe also passes over the complete official LAX interval:
147,383 samples lie inside it, both endpoints are bracketed, and the maximum
sample interval is 0.0210000002 s. There are no omitted paired rows. This
checks the entire interval required by the new runtime gate, rather than only
the 57-second initialization bracket. The production app replay is still pending.

### Optional source LM stopping-tolerance comparison

The source GNSS and IMU scripts change maximum iterations but leave stopping
tolerances at GTSAM defaults. The installed native GTSAM header defines both
relative and absolute tolerances as 1e-5; the native backend instead uses
1e-8 relative and 1e-10 absolute when no explicit config values are supplied.
`--native-source-lm-tolerances` now explicitly selects 1e-5 for both GNSS-first
and IMU-main in the Phase171 lane. Defaults and iteration limits remain unchanged.
This is a source/library-default comparison, not proof of a licensed MATLAB
runtime's effective parameters. A matched-binary four-run development comparison
is prepared but not yet started; no runtime or accuracy benefit is claimed.

### Bounded-gap/phone-contract build complete

The Release app build passed and was frozen as
`bounded_gap_phone_contract_fixed.exe` (d835bab7f987b4fb33f42035aa6fc7afc85a10958868ab2031faf0333a2e3517). Both new switches reject
invalid prerequisite combinations before file processing. The LAX long-gap
route and Samsung A32 are replaying; Samsung A325F is queued in the same
two-worker run. All include the first native state; only LAX opts into the
bounded long-gap policy. The matched development LM-tolerance comparison
also started with one worker using the same frozen binary.

The source TDCP Huber experiment is complete on both development routes.
Pixel5 Haversine linear P50/P95 mean increased from 0.5763756947 m to
0.6107707309 m; Pixel4 increased from 0.8453631253 m to 0.9209424046 m.
Neither route supports promoting that candidate. These results remain local
previously exposed development comparisons, not official or paired taroz scores.

### A205U leading-state feasibility

The two missing initial raw epochs have positive transmit-time fields but none
reach the loader/source's 1e10 ns timing threshold; their first GPS rows have
state 35 rather than the later decoded state 47. They must not be admitted as
valid pseudorange observations. Their receiver-clock uncertainty is also above
the source quality bound. The mapped paired IMU nevertheless covers both
endpoints through the first accepted raw epoch: 201 samples, maximum gap
0.0110000045 s, zero omitted paired rows. Existing whole-route clock mapping
passes with maximum residual 0.86805 ms and 2 s anchor gap. This supports
investigating explicit leading graph states; it does not authorize output
interpolation or prove those missing positions have been estimated. See
`use_cases/records/gsdc2023_samsung_leading_state_feasibility.json`.

The LM-tolerance Pixel4 candidate completed in 290.941 s, with seven main
iterations and Haversine linear score 0.8483875346 m. The matched-binary
control is still pending, so no speed/accuracy benefit or promotion is claimed.

### Leading numerical initializer unit-tested

`raw_p_seed::initializeWithLeadingGuesses` is an explicit library entrypoint
for leading numerical guesses from two later accepted same-run SPP anchors.
The leading interval and later-anchor separation are each bounded by four
seconds, independently of any larger interior-bracket policy. Original failed
SPP rows retain rejected status and nonfinite measured positions; the typed
initial guess has separate provenance and forward anchor source IDs. Trailing
gaps and a single available SPP anchor remain rejected. The existing
`initializeShortGaps` entrypoint retains its bracket-only behavior. All 38
raw-seed tests pass, including the four-second boundary and rejection beyond
it even with a 60-second interior bound.

This API is not yet connected to the native application. Remaining work is
to retain time-only leading states without accepting invalid GNSS rows, keep
the accepted-observation clock reference stable, enforce mapped IMU coverage,
and verify graph observability/clock gauges and converged final native states.
No missing official position is counted as recovered by this unit test.

### Leading-state loader and app integration

The opt-in loader retains receiver-clock-only leading states for at most four
seconds and one hardware clock domain. Invalid P/D/carrier observations remain
excluded. The first actually accepted observation epoch supplies the clock
reference, preserving all later accepted timestamps and pseudoranges. The
default loader remains unchanged. Leading positions are explicitly initialized
to zero at this input boundary and then receive only typed numerical SPP-based
guesses before graph construction. Clock-only states are counted separately
from diagnostic selected observation epochs.

`--native-leading-imu-states` connects this loader and the explicit leading
initializer to Phase171, requiring sparse staging, temporal provenance, raw
clock-only input, and first-epoch output. The complete mapped-IMU coverage gate
is mandatory. The auditor verifies runtime coverage and the rejected-SPP
provenance of the leading numerical guesses. All 25 loader, 38 raw-seed, and
eight auditor tests pass.

The real A205U initialization probe passes with 1,699 retained epochs, 1,697
independently accepted SPP epochs, and two explicit numerical guesses. This
does not prove graph observability or recovered final coordinates. The normal
app build is running; a serialized final incremental build (needed after the
explicit zero-initialization fix) and the full A205U replay are queued. No
new binary is frozen or replayed unless that final build succeeds.

### Bounded LAX replay passes official-key/native contract

The regular app's opt-in bounded-gap LAX replay completed in 1446.278 s and
passes all 2,805 official keys with exact native states, no output interpolation,
holds, or unresolved epochs. The runtime mapped-IMU gate verifies the complete
graph interval at maximum sample gap 0.0210000002 s. Main optimization converged
in 24 iterations, with the same initial/final costs as the diagnostic replay;
all prior 2,804 diagnostic CSV rows exactly match the new output after its
first row. The 36 numerical initial guesses remain separately recorded from
2,769 independently accepted SPP epochs. No truth accuracy was evaluated.
See `use_cases/records/gsdc2023_bounded_gap_native_replay.json`.

The matched-binary Pixel4 stopping-tolerance pair is now complete: Haversine
linear score changes from 0.8453631253 m to 0.8483875346 m (+0.0030244093 m).
Observed wall time is 697.813 s for the control and 290.941 s for the candidate;
concurrent workload differs, so this alone does not establish a controlled
speedup. Pixel5's pair remains pending and the candidate is not promoted.

Refreshed aggregate: 17/40 distinct drives have complete official-key native
coverage across retained variants; 26/40 pass their declared raw-key contract.
Neither count establishes homogeneous all-drive settings or taroz accuracy parity.

### Both stopping-tolerance pairs complete

The matched-binary Pixel5 pair is also complete: Haversine linear score
changes from 0.5763756947 m to 0.5855683249 m (+0.0091926302 m).
Both previously exposed development routes worsen with the source stopping
tolerances; the candidate remains opt-in and is not promoted. These are local
development results, not official scores or a taroz runtime comparison.
The first-epoch batch now has four verified completed pairs: the previous
rows are unchanged, and the added first native state supplies the missing key.

### Samsung A32 output contract verified

The source-disabled TDCP phone correction passes the A32 real-data replay
(2014.123 s): 1,838 native output epochs cover the official keys. The summary
explicitly reports omitted_by_phone_model=true, 10,635 candidate factors,
and zero inserted TDCP factors. No hypothetical TDCP residuals are presented
as optimized residuals. A325F remains in progress.
The latest combined audit verifies 18/40 official-key drives and 28/40 declared
raw-key contracts across different variants. Pixel7 Pro case 30 has now
completed its raw-key replay and is queued separately for first-key verification
in official_first_epoch_batch_v2; previously queued cases are excluded.

### Source fallback IMU noise paired experiment

The pinned source uses coefficient 1.0 when observation elapsed time is absent,
whereas the current native Pixel control uses coefficient 0.5 for measurement
noise. The existing Phase194 selector changes white-noise sigmas from
0.025/0.0005 to 0.05/0.001 without changing bias random walk or integration
noise. A Pixel5 candidate now runs against the already frozen, same-binary
Pixel5 control from source_lm_tolerance_regression_v1. Inputs and all other
arguments are checked by the paired comparator. The executable restricts this
selector to Pixel5, so no Pixel4 candidate has been launched. No accuracy
result or adoption is claimed while execution remains pending.

### A325F complete; leading-state executable built

The A325F replay completed successfully in 1323.422 s and passes all 1,782
official keys exactly, with native states, no output interpolation, no holds,
and no missing keys. Both source-disabled TDCP phone replays now pass.
The leading-state executable built successfully, including the final rebuild
for explicit zero initialization of empty leading observations. Frozen SHA256:
`4d139e334d69f9d96131f24aea28e6783992bb42599b6a686f2e900722be3f26`.
The A205U full-graph replay is running; initialization-only success is not
counted as final native output. The eight Python output-audit tests and
`git diff --check` pass after the build.

### Pixel5 source IMU noise improves the paired local diagnostic

The completed, same-binary/input Pixel5 comparison changes the Haversine
linear score from 0.5763756947 m to 0.5525424972 m (-0.0238331975 m).
The only argument change selects source fallback white-noise densities.
The native key contract passes; this is one previously exposed development
route, not official or taroz-comparative accuracy.
A separate default-off --native-source-phone-imu-noise option now follows
the pinned source phone branches, including the literal sm-g325f spelling.
It rejects unsupported phones and simultaneous Phase194 override. The two
C++ tests cover 16 presets under both clock branches and unknown-phone
rejection; four legacy Phase194 source-contract tests still pass.
Canonical compilation and Pixel4 paired evaluation remain pending.

The first leading-state full-graph run returned success in 336.192 s, but
the auditor rejected its summary: empty leading epoch count was computed
after observation vectors had been moved, reporting 1699 instead of 2.
The count now captures the loader output before the move. The corrected
build passed; leading_imu_state_regression_v2 is running. The rejected v1
remains preserved as evidence and is not counted in native coverage.
The same corrected binary also runs a candidate/control Pixel4 pair for
the new source-phone noise option. No production default is changed.

### A205U leading-state audit passes; fixed all-drive replay starts

The corrected A205U replay passes all 1,699 official keys in 318.701 s.
It separately reports 1,697 accepted native SPP epochs and two leading
numerical guesses; mapped IMU coverage has maximum gap 0.034000014 s.
All final rows are native graph states, without output interpolation or holds.
Solution CSV bytes exactly match the previous graph run, confirming the
metadata correction did not change positions. Accuracy remains unevaluated.
The all-40 fixed coverage recipe now runs with binary c779bfbb4a4e4832...
and the complete frozen plan recorded in gsdc2023_fixed_all40_recipe.json.
Source-phone noise and source LM tolerance candidates are not promoted into
this coverage recipe. Mi8-family initialization, A205U leading states and
the proven bounded LAX gap policy are explicit fixed exceptions. This run
is intended to establish one executable and fixed rules across all drives;
its completion and accuracy are not yet claimed.

### Local submission assembly and next paired measurements

The local assembler scripts/analysis/assemble_gsdc_native_submission.py
requires the pinned plan/sample hashes, exactly 40 official drives, one
verified executable, four input hashes per drive and the existing native
output audit. It selects native coordinates in official key order, discards
only extra keys, and writes a provenance manifest without submitting. Seven
tests pass, including a synthetic full-40 export and rejection of incomplete
runs, imported WLS provenance, changed inputs and mixed executable records.
The real current plan correctly rejects all 40 pending runs and creates no
submission directory.
Pixel4 source-phone noise candidate completed with local Haversine linear
score 0.897767959 m; its matched control remains running. No matched delta
or promotion is claimed. A Pixel5 candidate now tests source LM tolerances
on top of the completed source-noise baseline, changing only that selector.
Historical separate-IMU and final-Doppler experiments already exist in
Phase211/215 records; both worsened their older H baseline slightly. Their
results do not establish same-input/runtime parity for the current pipeline,
and no extra repeat of those experiments has been launched here.

### Pixel5 noise/tolerance combinations compared

Four completed same-binary/input combinations now have verified output hashes
and identical nonexperimental arguments. Baseline / source noise / source
tolerances / both score 0.576375695 / 0.552542497 / 0.585568325 / 0.564864918 m
on the reused Pixel5 route. Main iterations are 40 / 46 / 10 / 10, and GNSS
initial iterations 69 / 69 / 14 / 14. Noise alone improves both P50 and P95;
source tolerances reduce iterations but worsen the accuracy of either noise
setting. Observed wall times are recorded separately with the concurrent-load
caveat. None of these options changes the frozen all-40 coverage recipe.
See gsdc2023_pixel5_noise_tolerance_matrix.json for P50/P95 and exact outputs.
The canonical CTest audit/submission suites both pass (15 Python cases total).
Latest mixed-variant coverage is 27/40 official-key drives and 33/40 declared
raw-key contracts; homogeneous all-drive completion remains unproven.

### Pixel4 noise comparison complete

The matched c779bfbb4a4e4832... pair verifies unchanged inputs and all other
arguments. Pixel4 score changes from 0.8453631253 m to 0.8977679586 m
(+0.0524048333 m). Both outputs pass the native key contract. This contradicts
a blanket promotion based on the favorable Pixel5 result; source-phone noise
remains opt-in and the fixed all-40 coverage recipe stays unchanged.

### Existing-archive non-Pixel development checks

To check accuracy beyond the two Pixel development cases, four phone runs
from two existing train route groups were selected before truth access:
2022-08-04-20-07-us-ca-sjc-q (mi8, sm-a325f) and
2022-10-06-21-51-us-ca-mtv-n (sm-a205u, sm-a325f). Their prior project usage
is unknown; these are development checks, not an unused-data or heldout claim.
The original cached archive SHA256 was reverified; 14 raw/navigation/base/IMU
files were extracted or matched. No external dataset/download was added.
The c779bfbb4a4e4832... executable and same phone rules as the frozen test
recipe are used; source route Type/L5 metadata is recorded but not newly
applied. Two workers run four cases. Truth remains unread until each native
output is complete, frozen and audited. The evaluator will verify archived
truth hashes, require every truth key, drop only extra native keys, and report
phone metrics grouped by route. No overall score is emitted until all four
checks succeed. These training cases never contribute to test coverage.
The first-key preservation batch is also complete: all 13 comparisons pass.

### Long LAX-p replay completes without restart

The original 2022-02-24-15-10-us-ca-lax-p/pixel5 replay finished successfully
after 13672.181 s, with 147 main iterations and cost decreasing from
301926507.767 to 3597736.749. All 4,514 official keys exactly match native
output states. Five numerical initial guesses remain distinguished from
4,510 independent SPP epochs; output interpolation/holds are zero.
CPU liveness checks correctly prevented a duplicate restart during this
long computation. This establishes coverage, not position accuracy or a
controlled runtime benchmark. The fixed-recipe replay is still required.

### Samsung development score and leading-clock boundary correction

The August A325F native run scores 2.223245985 m locally (P50 1.425058169 m,
P95 3.021433800 m). Its native output was frozen and audited before truth
was extracted from the same verified archive. This exposes a non-Pixel
accuracy gap; no all-phone parity claim is made. A source-phone noise-only
comparison is frozen for both A325F route groups before accessing October
truth. Both pairs use the original c779bfbb4a4e4832... binary.
The October A205U failed raw loading before truth access: the raw GPS span
is 4.000002688 s while UTC spans 3993 ms. The leading eligibility rule now
keeps strict UTC <=4000 ms, with GPS boundary tolerance of one UTC-key tick
(1 ms) plus the existing 1 us subtraction tolerance. Timestamps themselves
are unchanged, and continuous mapped IMU remains mandatory. Shared loader/
seed policy passes 26 loader and 39 seed tests, including acceptance of a
3 us overrun and rejection of 2 ms GPS excess or >4000 ms UTC. The new
bf7eaaa18732928d... binary passes real-data loader/seed admission and is now
running the full graph. It does not replace the active test-40 executable.

### Initial IMU observation stages implementation

The opt-in `--native-source-imu-observation-stages` connects the tested GNSS-only handoff to a fresh raw observation rebuild before the initial IMU solve, followed by another rebuild before the final solve. Source initialization/final residual limits and the cached pre-admission code-residual population are selected together. Each new P row receives the existing exact-stream base correction once. Build, regressions and matched MI8/A205u replays passed. The GNSS stages and disabled-selector controls reproduced the previous outputs byte-for-byte. Local final-score changes were +0.001309 m for MI8 and -0.003443 m for A205u; the recipe remains unpromoted. This composite experiment does not establish isolated causal attribution or pinned MATLAB runtime parity. See `gsdc2023_source_imu_observation_stages_experiment.json`.

A separate all40 candidate recipe reuses the audited A205u test source-initialization output with the identical c779 executable, four raw inputs, and otherwise identical arguments. Its all40 audit and local assembly passed: exactly 1,699 A205u rows changed, while the other 70,237 rows across 39 drives retained identical values. Submission 56479759 scored Public 1.698 m / Private 1.333 m after user-authorized submission. See `gsdc2023_all40_a205u_source_init_recipe.json` and `gsdc2023_all40_official_submission_v1.json`.

### Stage-conditional audit

Pinned source inspection confirms that initialization changes the P/D residual thresholds, state/attitude handoff and final-only velocity/height constraints. The source SNR, robust and IMU noise parameters do not otherwise branch on `initflag`; phone/course differences remain separate. The hash-verified existing archive has no `RPYReset=1` train rows, and only `2022-02-24-15-10-us-ca-lax-p/pixel5` requests it among test rows. The current MI8/A205u comparisons correctly retain the previous attitude for their settings, but the native reset option remains a parity gap for that test drive. This is source inspection, not MATLAB runtime evidence. See `gsdc2023_imu_stage_conditionals_audit.json`.

The source `RPYReset` handoff has three passing tests for attitude-only changes, unsmoothed state preservation, nearest-filled low-speed gaps and invalid-state rejection. It is now connected through `--native-refinement-attitude-reset`, requiring the existing complete refinement recipe. The application build and ten CLI checks passed; a disabled-selector MI8 replay reproduced the preceding executable's final and intermediate output bytes exactly. The source-enabled Pixel5 LAX-p matched pair completed with frozen executable `61a713c180acebe09...`: both runs cover all 4,515 raw epochs and all 4,514 required official keys with native states. GNSS-first and initial-IMU stage bytes match between arms. Independent reconstruction matches all 4,515 reset rotations (1,441 low-speed nearest fills), with maximum matrix-element error 1.28e-15. Test accuracy is unavailable and the submitted all40 CSV is unchanged. See `gsdc2023_refinement_attitude_reset_handoff.json` and `gsdc2023_refinement_attitude_reset_experiment.json`.

The completed IMU-only observation-stage experiment leaves the GNSS-first stage unchanged by design. Pinned cold-start source calls the GNSS-only stage with initial residual thresholds too; native Legacy masks still differ there. `gsdc2023_cold_gnss_observation_phase_gap.json` records the source call chain and the separately pinned MatRTKLIB position-gradient method. The new GNSS-initial opt-in below derives positions, clocks and velocity from its own raw-data seeds, with no archived position input.

### Cold GNSS observation initialization implementation

The opt-in `--native-source-gnss-initial-observations` rebuilds GNSS P/D/TDCP rows from the existing same-run native raw-P seeds, validates exact position/clock/UTC provenance, and computes the componentwise position gradient with the source-style nominal interval. It uses initial residual limits and cached code-residual centers, applies base corrections once to fresh P rows, and retains all native keys and the existing C7/D policies. It requires the already tested IMU observation-stage recipe so the following IMU passes also refresh their observations. Three isolated helper tests and the integrated build passed, followed by 26 focused cases, eight legacy cases and ten CLI cases. With frozen executable `36fb14b834260918...`, both disabled-selector controls reproduced preceding d0cabb outputs exactly. MI8 GNSS score improved from 1.265087 to 1.247762 m, but final score changed from 1.176314117 to 1.176321050 m: no final improvement. A205u failed closed with zero retained Doppler rows before scoring. Its raw drift was approximately 18,520 m/s. A separate matched A205u experiment enables the existing native Samsung GPS-L1 median drift preprocessing in both arms before testing cold GNSS masks. This has not changed the frozen all40 submission artifacts or established improved accuracy. See `gsdc2023_cold_gnss_initial_experiment.json` and `gsdc2023_cold_gnss_samsung_drift_experiment.json`.

The matched A205u drift-preprocessed experiment completed and passed native/base-once audits. Its GNSS score changed from 2.882814 to 2.555940 m, initial IMU from 2.114807 to 2.111648 m, and final output from 1.958233 to 1.957856 m. The final gain is only 0.000377 m on one exposed route; the opt-in remains unpromoted.

### Submitted recipe train40 evaluation

The next evaluation freezes the submitted c779 executable and common recipe over all 40 source-settings train drives (13 phones, 30 route groups), including the existing MI8/A205u initialization selectors. No new dataset or per-train-route adjustment is used. Historical project usage is unknown; this is development evaluation, not heldout validation. The known A205u leading-span boundary in this older executable may fail and will be counted as a failure. Full-set aggregate scores remain unavailable unless all 40 pass native and truth-key coverage audits. The test-only LAX-m long-gap exception is not generalized automatically. See `gsdc2023_train40_fixed_recipe_evaluation.json`.

## Intermediate source position offsets (2026-09-23)

Pinned `fgo_gnss.m` corrects phone position before saving `result_gnss.mat`;
`fgo_gnss_imu.m` loads that corrected position for its initial pass and likewise
saves a corrected position before the final pass. Current native stage handoffs
retain uncorrected optimized positions, with a correction only on final output.
The new isolated `native_stage_position_offset.hpp` supplies separate validated
GNSS and IMU handoffs: source velocity heading for GNSS, optimized RPY for IMU,
and only receiver positions changed in a copied handoff. Raw measurements,
velocity, clock states, and next-stage attitude seeds remain unchanged.
The opt-in `--native-source-stage-position-offsets` is now connected to both
observation rebuilds, requires the full source observation-stage recipe and
final output offset, and exports per-stage provenance. The app build and
12 CLI rejection checks passed. Frozen executable `d490772096f5e917...` is
running MI8/A205u matched development pairs, first requiring selector-off
outputs to match previous stage and solution bytes. Submitted outputs and
default settings are unchanged; matched accuracy comparisons remain pending.

The isolated helper passed three tests covering GNSS velocity-heading versus
IMU optimized-attitude offsets, preservation of non-position states, repeated
call isolation, and invalid inputs. Independent scalar arithmetic on 4,515
existing native Pixel5 epochs gives horizontal offsets 0.129–0.316 m; these
are position changes, not measured accuracy gains. See
`gsdc2023_intermediate_position_offset_audit.json`.

The new stage-offset executable passed its MI8 selector-off native replay:
final solution, GNSS-first, initial-IMU, and initialization metadata bytes
match the previous frozen executable. The control retains local score
1.176314117 m. Its stage-offset candidate is running; both stage scores and
independent per-epoch offset reconstruction are required before comparison.

The MI8 stage-offset pair completed all 1,417 native keys, with all 1,400
truth keys matched and 17 extra native keys excluded only from scoring.
Final mean changes from 1.176314117 to 1.174892336 m (-0.001421781 m).
GNSS-stage output is identical; initial-IMU mean changes by -0.000009157 m.
Independent reconstruction matches both handoffs (maximum ENU error below
5e-16 m, identical rounded ECEF coordinates). This small effect on one exposed
route is not promoted. The A205u pair remains pending. See
`gsdc2023_stage_position_offset_mi8_comparison.json`.

## Direct pinned C++ factor comparison (2026-09-23)

The standalone `tests/reference_gsdc_factors` suite compiles the original
`ClockFactor_CCDD`, `MotionFactor_XXVV`, and `DopplerFactor_VD` headers from
pinned taroz/gtsam_gnss commit `e679b72b620fc6800723578e4077dd157587d129`.
All three tests pass over 42 deterministic state cases, comparing unwhitened
residuals and analytic Jacobians against native factor classes. The Doppler
origin-to-absolute-measurement convention is explicitly converted. The build
rejects a different source commit or modified reference headers.
This is executed C++ factor evidence, not MATLAB or full-trajectory parity.
Whitening, graph selection, preprocessing, P/TDCP/IMU/height factors, and
optimizer convergence are outside this suite. No accuracy improvement is
claimed. See `gsdc2023_pinned_reference_factor_comparison.json`.

The v2 direct-factor suite passes six tests: 66 matching states across clock,
motion, Doppler, optional affine P and optional affine TDCP; plus a separate
test of the expected nonlinear-P difference. The nonlinear P factor matches
the source linearization anchor, but a synthetic 1,000 m transverse displacement
produces a 0.014711357 m residual difference. Optional affine-class equality
does not establish that the submitted recipe selects them. No accuracy effect
is inferred. See `gsdc2023_pinned_reference_factor_comparison_v2.json`.

The A205u stage-offset pair also completed: all 1,213 native keys pass, both
controls reproduce prior outputs, and both offset handoffs match independent
reconstruction. Final mean worsens from 1.958235360 to 1.959737934 m
(+0.001502575 m), despite an initial-IMU change of -0.000472455 m. Together
with MI8's small improvement, this does not support promoting intermediate
position offsets. The application option stays disabled by default.

Next development comparison: the already supported TDCP-only affine selector
on existing January 4 MTV Pixel5 train input, with submitted executable c779
and the completed 0.594509466 m baseline reused unchanged. Only geometry
changes; no stage offsets or source IMU initialization are added. This route
is already exposed. Both GNSS-first and main insertion counters, full native
coverage, input hashes, and exact truth keys must pass before interpreting
accuracy. See `gsdc2023_train_pixel5_january04_affine_tdcp.json`.

## Source TDCP weights queued separately (2026-09-23)

The completed January 4 MTV Pixel5 baseline summary confirms fixed TDCP sigma
0.03 m and disabled source metre-sigma weighting. Pinned `parameters.m` and
`obserrmodel.m` use a 1/400 m carrier coefficient multiplied by per-band
85th-percentile SNR scaling and signal-type factors. The existing default-off
`--native-source-tdcp-meter-sigma` selector is now frozen for a separate
same-c779, same-input comparison after the active affine-geometry candidate.
Only the weighting selector changes; it is not combined with affine geometry,
source IMU initialization, or intermediate position offsets. The baseline
route was already scored and this is development validation, not heldout.
The queued supervisor checks the predecessor process and terminal record
before launching; no extra native process is started while the slot is busy.
See `gsdc2023_train_pixel5_january04_tdcp_sigma.json`. No gain is claimed.

## Submitted route-setting audit (2026-09-23)

All 40 actual submitted-recipe summaries were checked against source settings
read directly from the pinned archive: 30 route labels differ. Counts are
Highway->Street 19, Street->Highway 6, Mix->Street 3, Mix->Highway 2,
Street->Street 5 and Highway->Highway 5. All 40 use fixed TDCP sigma 0.03 m.
These are configuration differences, not proof of accuracy loss; label
inequality does not imply every processing parameter differs. Source L5/BDS
metadata was recorded but effective native admission was not audited here.
See `gsdc2023_submitted_route_settings_audit.json`.

For the active January 4 MTV Pixel5 comparison, the source row is Highway
(L5=1, BDS=1), while native direct quality is Street. Its source TDCP Huber-k
is therefore 0.5, not the Street/Mix value 0.2; native current k is 4.0.
The active geometry-only and queued sigma-only conditions remain unchanged.
A default Phase184 toggle would follow the native environment, so a later
source-preset comparison must explicitly bind the source route type and
separate that change from TDCP weighting. No candidate is promoted from
this audit. See `gsdc2023_active_tdcp_noise_gap.json`.

## January 4 Pixel5 affine TDCP result and explicit route comparison (2026-09-23)

The same-input, same-c779 affine-only comparison passed provenance and native
coverage checks for all 1,855 epochs. The score changed from 0.594509466 m
to 0.595507114 m (+0.000997648 m, worse). Both GNSS-first and main stages
inserted 40,347 affine TDCP factors; initial seed metadata stayed identical.
This exposed development route does not support promoting the selector.
See `gsdc2023_train_pixel5_january04_affine_tdcp.json` for P50/P95, hashes
and observed runtime.

The sigma-only candidate is running separately. After its verified terminal
comparison, the queued route candidate uses only
`--native-source-route-type Highway --native-source-l5 1`, matching the
archived settings row. TDCP remains fixed at sigma 0.03 m and Huber k=4;
source IMU initialization, affine geometry and stage offsets are disabled.
This compares a route preset bundle, not one numerical parameter and not
the complete source TDCP noise model. Plan SHA-256:
`555177e2b97214540eebaf37f4fa0a4489994bc69c1ca4ab4367db8a76951ec9`.
See `gsdc2023_train_pixel5_january04_source_route.json`. No new submission
or accuracy gain is implied by the queue.

## Incremental existing-train results (2026-09-23, 13:08 JST)

The fixed submitted-recipe train audit now scores 12/40 drives, with one
initialization failure, two live native jobs and 25 not yet started at this
checkpoint. January 26 MI8 passed native/key provenance checks: P50
0.460721950 m, P95 0.952115331 m, mean 0.706418640 m. The complete-40
aggregate remains unavailable; the separately recovered January 4 MI8
run is not counted as the frozen baseline.

The first of two source IMU initialization comparisons (January 4 highway
Pixel5) passed matched-input/executable, native coverage, seed and truth
hash checks. Both arms score 0.674903313 m (P50 0.445788297 m, P95
0.904018329 m). Whole solution CSV hashes differ, so equal score must not
be described as byte-identical output. The second route is still running.
The comparator now supports partial audited results without marking the
two-route experiment complete. See
`gsdc2023_train_pixel5_source_initialization.json`. No promotion or official
submission follows from this partial result.

## Source TDCP noise bundle queued (2026-09-23)

A comparison against the explicit Highway/L5=1 control is frozen before
its result: add only source metre sigma and Phase184 Highway Huber k=0.5.
It follows the queued route comparison sequentially, reusing c779. Both
noise controls change together; this tests the source-prescribed noise
bundle, not individual attribution or full MATLAB preprocessing parity.
The supervisor verifies predecessor liveness and successful audited
completion. Plan SHA-256:
`3e6e0a540d608c3f340b248d090d66353cd8d05c3d5f5bb310e6bf1927e13597`.
See `gsdc2023_train_pixel5_january04_source_tdcp_noise.json`.

A static named-field audit clarifies that archived L5=1 is not evidence
of a simple band enable/disable switch: pinned parameters.m uses it for
P/D/L elevation thresholds, and exobs.m iterates both bands when present.
No direct read of prm.BDS was found in the pinned MATLAB .m files. This
does not prove dynamic/external toolbox behavior or complete admission
parity. See `gsdc2023_source_l5_bds_static_audit.json`.

## TDCP sigma improvement and separate-route replication (2026-09-23)

January 4 MTV Pixel5 sigma-only comparison completed with matched executable,
inputs, initial seed metadata, truth keys and all 1,855 native epochs verified.
P50 improved 0.492372006 -> 0.336097218 m; P95 0.696646926 -> 0.457355862 m;
their mean improved 0.594509466 -> 0.396726540 m (-0.197782927 m).
Both arms insert 40,347 TDCP factors. Candidate reported representative
sigma 0.002798401 m instead of fixed 0.03 m; Huber and geometry are unchanged.
This is one exposed development route, not official accuracy or a promoted
all-phone recipe. See `gsdc2023_train_pixel5_january04_tdcp_sigma.json`.

The same sigma-only selector is frozen for a separate March 10 Street
Pixel5 route, using its already completed same-c779 baseline. It will run
after the two-route source-initialization comparison releases its slot.
No parameters are fitted; this checks transfer to another route group.
Plan SHA-256 `498fd81d57bd5a43543a71f0417ce5636e48640ca1cf4c7d155faaf379b26392`.
See `gsdc2023_train_pixel5_march10_tdcp_sigma.json`. The independent
Highway/L5 route comparison has started, followed by its frozen source
TDCP noise-bundle comparison.

The reusable `scripts/analysis/summarize_gsdc_train_validation.py` now
summarizes the frozen 40-drive validation snapshot by phone and route group.
It preserves failed/pending denominators, distinguishes mean per-drive
P50/P95 from pooled percentiles, reports output coverage and initial-guess
counts separately, and leaves full-scope means null until complete. The
validator regenerates this summary after each audit. At this checkpoint
13/40 drives and 9/30 route groups are complete; all 21,586 scored raw
epochs have zero missing keys and zero output interpolation/hold. These
rates do not cover failed or pending drives. No full-set accuracy is claimed.

## Pixel5 initialization comparison complete (2026-09-23)

Both matched January 4 routes passed native output, exact-key, same-input,
same-executable, seed-metadata and truth-hash checks. Highway score remains
0.674903313 m; MTV score changes 0.594509466 -> 0.594489668 m
(-0.000019798 m). This does not establish a useful transferable improvement.
The option is not promoted. See
`gsdc2023_train_pixel5_source_initialization.json`.

The source-initialization supervisor completed successfully and the frozen
March 10 sigma-only replication started in the released slot (native PID
28396 at this checkpoint). The current source metre-sigma CLI admits only
Pixel5. The underlying formula uses signal type and SNR rather than phone
identity, but source consistency of that formula alone does not validate
other phones' clock models, admitted observations, or accuracy. Cross-phone
rollout still requires explicit admission changes and matched native runs;
no other phone has been claimed improved by the January 4 result.

## Source sigma admission for Mi8/Pixel4 comparison (2026-09-23)

The default-off source metre-sigma CLI now also admits Mi8/xiaomimi8 and
Pixel4/Pixel4XL, which use the same source bias-difference TDCP family.
Only admission changed; the signal/SNR formula, factor construction and
default flag selection did not. Drift-clock and disabled-TDCP phones remain
excluded. The application rebuilt successfully; frozen executable SHA-256:
`76010f6c5e9a9296b06f8f389554f786c8d5ba8727543c7008f8e90f716ca08d`.
Twelve CLI checks pass, including accepted-phone probes terminating at an
intentionally absent raw input and rejected incompatible configurations.
These tests do not establish native accuracy or default-output regression.
See `gsdc2023_source_sigma_phone_admission.json`.

January 5 Mi8 and July 14 Pixel4 controls/candidates are frozen to this same
executable and queued after the March 10 Pixel5 replication. New controls
must match historical frozen baseline solution bytes. Candidate changes
only `--native-source-tdcp-meter-sigma`; all native/key/input/seed/truth
audits and paired metrics remain required. Both are exposed development
route groups. No cross-phone gain or promotion is claimed before results.
See `gsdc2023_train_mi8_pixel4_tdcp_sigma_control.json` and
`gsdc2023_train_mi8_pixel4_tdcp_sigma_candidate.json`.

## Explicit Highway preset result and sigma epoch diagnostic (2026-09-23)

January 4 MTV Pixel5's explicit Highway/L5=1 preset alone worsened the
score from 0.594509466 to 0.651683176 m (+0.057173709 m). P50 is
0.546540783 m and P95 0.756825568 m. Inputs/executable/seed metadata and
native coverage passed the matched audit. Actual reported elevation and
velocity-motion sigma remained 5 degrees and 0.01 m; P/D Huber thresholds
changed 0.1/0.4 -> 0.2/0.8, while TDCP sigma/k stayed 0.03/4.
The preset alone is not promoted. See
`gsdc2023_train_pixel5_january04_source_route.json`. Its previously frozen
source TDCP sigma-plus-Huber comparison has started; results remain pending.

A posthoc epoch diagnostic reproduced the sigma-only pair's P50/P95 from
the frozen outputs and pinned truth. There are 1,855 native raw epochs but
1,854 scored truth epochs: both arms omit the same one extra native key
from scoring. Of the scored epochs, 1,465 improve and 389 worsen; median
epoch error change is -0.132420635 m and mean change -0.118342250 m.
This is diagnostic evidence from the same exposed route, not independent
validation. See `gsdc2023_january04_tdcp_sigma_epoch_diagnostic.json`.

## Sigma stage-attribution replays queued (2026-09-23)

The historical c779 executable does not expose GNSS/IMU stage CSV options;
the original sigma pair therefore cannot establish stagewise position
accuracy. A same-input pair is now frozen with executable 76010f6c and
`--native-export-gnss-stage --native-export-imu-stage` on both arms. Only
the candidate enables source metre sigma. Each final output must reproduce
its own historical c779 solution byte-for-byte, and initial seed metadata
must match, before its intermediate coordinates are interpreted.

The new comparator requires every stage's ordered keys to equal all native
raw keys, finite valid ECEF positions, verified executable/input/output hashes,
and the same pinned 1,854 truth keys. It scores GNSS-first and IMU-initial
exports before any final output phone offset. This measures native sigma
attribution, not MATLAB runtime parity. The queue follows the running source
noise-bundle comparison, maintaining one native process in that experiment
slot. See `gsdc2023_train_pixel5_january04_sigma_stages_control.json` and
`gsdc2023_train_pixel5_january04_sigma_stages_candidate.json`. Results and
default-output equivalence remain pending.

## Sigma-only replication regresses: no uniform promotion (2026-09-23)

March 10 Street Pixel5 completed with the same c779 executable, inputs,
seed metadata, truth keys and native output provenance verified. P50/P95
change 0.547142943/0.808678316 -> 0.871529037/1.200272834 m. The score
regresses 0.677910629 -> 1.035900935 m (+0.357990306 m), despite January
4's improvement. Both March arms insert 24,947 TDCP factors. Observed
shared-load wall times are 407.7/1,147.6 s, not a controlled benchmark.
See `gsdc2023_train_pixel5_march10_tdcp_sigma.json`. The two exposed-route
mean worsens; source sigma alone is not promoted or selected per known route.
See `gsdc2023_pixel5_tdcp_sigma_two_route_comparison.json`.

A conditional comparison is frozen to add only source Street Huber k=0.2
to the sigma-only March candidate. Native and source Type both say Street;
no explicit route preset or motion-sigma change is added. It is queued after
the now-running Mi8/Pixel4 pair. Its comparator also reports difference
from the original 0.677910629 m recipe, since beating the regressed sigma
control alone would be insufficient. See
`gsdc2023_train_pixel5_march10_tdcp_huber.json`; plan SHA-256
`14bb4b2bef52a349d1439a7d8c192266da3134e083706b1e7f3bd8e32c05f89f`.

## Mi8 source-sigma admission default regression passed (2026-09-23)

The new 76010f6c executable's January 5 Mi8 control completed successfully
with all 1,302 native raw keys. Its final solution and initialization
metadata exactly match the old c779 submitted-recipe control bytes; inputs
and normalized argv match. GNSS/main accepted iterations remain 42/22.
This establishes default-output regression only for this Mi8 route, not
new-sigma accuracy or all-phone equivalence. Pixel4 control has started;
both sigma candidates remain queued behind these controls. See
`gsdc2023_sigma_phone_mi8_default_regression.json`.

## Two-phone control regression and 14-drive checkpoint (2026-09-23)

The July 14 Pixel4 control joins Mi8 in reproducing the old c779 final
solution and initialization metadata byte-for-byte with executable 76010f6c.
Both passed native/key/truth audits: Mi8 1,302 raw keys at 0.766759593 m;
Pixel4 1,188 raw keys at 0.466482272 m. Candidate runs have now started.
This covers only these two control routes, not all admitted phone aliases
or candidate accuracy. See `gsdc2023_sigma_phone_control_regression.json`.

The fixed train snapshot now scores 14/40 drives. February 24 LAX-o Pixel5
completed in 2,196.2 s (shared load) and scores 1.789751416 m, with P50
1.234332292 m and P95 2.345170541 m. Its 2,439 raw outputs are native, and
all 2,438 truth keys are present. Full-scope aggregates remain unavailable;
24,025 completed raw epochs have zero output missing/interpolation/hold,
which does not describe failed or pending drives.

## LAX-o final Doppler omission isolated for comparison (2026-09-23)

The completed February 24 LAX-o Pixel5 baseline inserts 47,507 Doppler
factors in GNSS-first but zero Phase213 Doppler factors in the final IMU
graph. Pinned fgo_gnss_imu.m:214-217 inserts DopplerFactor_VD in that stage.
An exposed-route comparison is frozen to add only
`--native-phase213-main-doppler` with the same c779 executable and inputs.
Native Highway mapping remains unchanged even though the source row is
Street; TDCP remains fixed sigma 0.03 m / k=4. This separates Doppler
from the pending noise and route-setting hypotheses.

The comparator requires measured main-Doppler insertion, equal inputs and
seed metadata, raw/key/native audits, and the baseline quality settings.
It runs after the March 10 Huber comparison, without increasing native
concurrency. Plan SHA-256
`0e616596bf23b38e4e5fd08118c4d0b4334aabf6cf4ae4970e899947bcda0e5a`.
See `gsdc2023_train_pixel5_lax_o_main_doppler.json`. No gain is claimed.

## August 24 fixed-recipe score and historical recipe distinction (2026-09-23)

August 24 MTV-h Pixel5 completed in 4,180.8 s (shared load), with all
3,140 raw keys native and all 3,139 truth keys covered. P50 0.613411259 m,
P95 1.024314017 m, mean 0.818862638 m. The fixed train audit now scores
15/40 drives; one initialization failure remains and 24 drives are pending.
No full-set score is available.

The old 0.576375695 m development control used the same four input files
(hash equality reverified), but a different executable and multiple options:
source UTC IMU offset, main Pose3 motion, main Doppler, metre TDCP sigma,
source initialization and explicit Street/L5 settings. The frozen submitted
recipe instead includes first-native-epoch, temporal seed and sparse-P
recovery selectors. Do not attribute the score difference to one change
or call it a matched executable regression. See
`gsdc2023_august24_recipe_difference_audit.json`.

## Sixteenth fixed-recipe train result (2026-09-23)

April 1 LAX-t Pixel5 passed the frozen native/key audit with 1,466 raw
epochs, 1,465 truth epochs and one extra native epoch dropped only from
scoring. P50 0.606412372 m, P95 0.948238291 m and mean 0.777325331 m;
observed shared-load runtime 477.8 s. Fixed-recipe status is now 16 scored,
one initialization failure, two running and 21 not started. Full-scope
means remain null.

The Mi8/Pixel4 sigma candidate validator/comparator now accept incremental
audited reporting, while their default full mode still requires both
routes. Partial reports retain paired_validation_complete=false; incomplete
process records are never treated as liveness proof or scored outputs.
The native jobs and their frozen inference plans are unchanged.

## TDCP sigma/Huber interaction in the actual scalar noise model (2026-09-23)

Native makeNoise wraps Isotropic::Sigma with GTSAM Huber. Inspected GTSAM
loss/whitening code therefore yields scalar loss rho_k(r/sigma), transition
at |r|=k*sigma, quadratic curvature 1/sigma^2 and linear-tail gradient k/sigma.
At the January 4 and March 10 sigma-only summaries' representative values,
keeping k=4 makes the tail gradient 10.720x and 17.828x the fixed-0.03-m
legacy value. Substituting source k=0.5/0.2 at the same representative sigma
gives 1.340x/0.891x. These latter rows are analytical examples, not measured
results from the pending joint runs.

The summary sigma is the first finite-residual factor's value, not a median
or a universal factor sigma. This interaction motivates the already frozen
joint tests, but does not prove the cause of trajectory regression: actual
residual distributions, graph coupling and initialization also matter. See
`gsdc2023_tdcp_sigma_huber_interaction_audit.json` for code hashes and scope.

## First fixed-recipe Pixel6 Pro result (2026-09-23)

May 13 Pixel6 Pro completed in 825.5 s with 2,180 native raw epochs and
all 2,162 available truth keys covered. The 18 native epochs without truth
are excluded only from accuracy scoring. P50 0.771107554 m, P95
1.119794350 m and mean 0.945450952 m. The fixed snapshot now scores
17/40 drives; no full-scope score is available. The grouped summary now
reports truth-epoch counts and unscored native counts separately from
output-missing rates, preventing incomplete truth coverage from being
confused with missing estimator outputs.


## A325G error localization and 18-drive checkpoint (2026-09-23)

The July 26 SJC Samsung A325G baseline completed all 1,512 native raw
epochs in 436.3 s under shared load. All 1,487 available truth keys are
covered; 25 native epochs have no truth and are not scored. The local
haversine/linear-percentile score is 11.929479401 m (P50 2.903674679 m,
P95 20.955284122 m), so native completeness does not establish accuracy.
The fixed baseline validation now contains 18 scored drives; full-scope
accuracy remains unavailable and the taroz goal remains unmet.

Posthoc time localization, recorded in
`use_cases/records/gsdc2023_a325g_baseline_error_localization.json`,
finds 351/1,487 errors above 10 m. The first 120 seconds have a median
20.895 m error; further large errors occur around 600 seconds and after
1,080 seconds, with a maximum 53.145 m near the end. Thus an initial
transient alone does not explain the full trajectory. Truth-speed
stratification is diagnostic only and is never an estimator input.

GNSS-first and main stages both report convergence (251 and 114
iterations), but convergence is not evidence of positional accuracy.
Source IMU initialization and Samsung clock-drift preprocessing are
both disabled in this frozen baseline. Their causal contribution is
unproven. Next comparisons should isolate these source-backed options
with identical inputs and keys, without adding concurrent native jobs
beyond the existing four-process budget or changing the baseline recipe.


### A325G isolated comparisons queued

Two matched candidates now wait sequentially after the LAX-o main-Doppler
comparison, preserving at most four native processes. Both use the frozen
c779 binary and the exact baseline-19 inputs and raw keys. The first adds
only `--native-source-imu-initialization`; the second independently adds
only `--native-samsung-clock-drift` to the baseline (not to the first
candidate). Neither is promoted or externally submitted.

Plans are pinned by SHA-256 in `gsdc2023_train_a325g_source_init.json`
and `gsdc2023_train_a325g_samsung_drift.json` under `use_cases/records`.
Post-run validation checks native provenance, full available truth-key
coverage, matching input hashes and normalized arguments, converged stages,
and the selected option. The IMU-only comparison additionally requires
identical seed metadata. Clock-drift preprocessing is allowed to change
its seed metadata, as that is the intervention being measured. The source
Samsung phone branch explicitly includes A325G in preprocessing.m:139/150;
this does not prove complete satellite-state or MATLAB runtime parity.


## Mi8 source-TDCP-sigma partial result (2026-09-23)

January 5 Mi8 completes with 1,302 native epochs and unchanged 25,444
TDCP factors. The matched 76010 control reproduces historical c779 output
exactly. Sigma-only changes the local score from 0.766759593 to
0.839478741 m (+0.072719148 m): P50 0.398461414 to 0.526624556 m,
P95 1.135057772 to 1.152332926 m. The two-phone comparison remains
incomplete while the Pixel4 candidate runs. No default change is justified.

The native solver diagnostic records GNSS iterations 42 to 78 and main
iterations 22 to 125, with convergence reported in both arms. Reconstructed
TDCP residual RMS changes from 0.026546391 to 0.027469179 m, while
normalized RMS changes from 0.884879712 to 11.849671205 and Huber-tail
counts from 71 to 6,151. These are differently weighted objectives and
do not identify a causal error mechanism or establish source equivalence.
The corresponding records are `gsdc2023_train_mi8_pixel4_tdcp_sigma_candidate.json`
and `gsdc2023_mi8_sigma_solver_diagnostic.json` under `use_cases/records`.


## August 4 Mi8 and 19-drive checkpoint (2026-09-23)

The fixed submitted-recipe replay for August 4 SJC-q Mi8 completed in
827.0 s under shared load. The native audit verifies all 1,417 raw keys,
zero output interpolation/hold, and no missing output. All 1,400 available
truth keys are covered; 17 additional native keys are unscored. Local
haversine/linear-percentile P50 is 0.661367040 m, P95 1.203182237 m,
and their mean is 0.932274638 m. This is an exposed development result,
not an official competition score. The fixed evaluation now has 19
scored drives, one failed drive, two running and 18 not started. Full
40-drive accuracy and achievement of the taroz target remain unproven.


## August 4 Pixel5 and 20-drive checkpoint (2026-09-23)

August 4 SJC-q Pixel5 completed in 441.9 s under shared load.
All 1,450 raw keys are native, with no missing output or
output interpolation/hold. All 1,431 available truth keys
are covered; 19 extra native keys are unscored.
Local P50 is 0.382793218 m and P95 0.987825970 m,
with mean 0.685309594 m. This phone shares its evaluation
route group with the August 4 Mi8, rather than constituting an independent
route. Fixed baseline evaluation has reached 20 scored drives of 40,
with one failed, two running and 17 not started. These development scores
do not establish official target achievement or full-scope accuracy.


## Source TDCP sigma: four-route decision (2026-09-23)

The Pixel4 candidate completed and passed paired native/output/input audit.
Its local score worsens from 0.466482272 to 0.518897993 m (+0.052415721 m),
with P50 0.254032567 to 0.329647893 m and P95 0.678931976 to 0.708148093 m.
All 1,188 native raw keys are retained; TDCP factor count remains 21,090.
Together with Mi8 and the two Pixel5 routes, sigma-only improves one of
four exposed route groups and worsens three. Their descriptive mean
changes from 0.626415490 to 0.697751052 m.
This is neither a full-scope score nor a heldout result. Uniform promotion
is rejected, and selecting the one known favorable route is not justified.
The March 10 conditional-Huber experiment has now started automatically
after the two-phone comparison completed. See
`use_cases/records/gsdc2023_source_sigma_four_route_comparison.json`.


## August 4 A325F and fixed A205u failure (2026-09-23)

August 4 A325F completed 1,452 native raw keys in 650.1 s, with all
1,434 truth keys covered and 18 native keys without truth. Local P50
is 1.418157712 m, P95 3.084413211 m, mean 2.251285461 m. The three
phones of this route are now complete as one evaluation group, whose
mean phone score is 1.289623231 m; no coordinate fusion was performed.

The next October A205u run failed before solving at the known leading
clock four-second boundary in the frozen c779 binary. Exact raw input
hashes and executable hash match the earlier failing run. Existing
UTC-bounded correction evidence is linked by
`use_cases/records/gsdc2023_train40_a205u_boundary_failure.json`; older
corrected outputs do not replace this fixed-recipe failure or prove
full-recipe equivalence. The frozen evaluation now contains 21 scored,
two failed, two running and 15 not started. Full-scope accuracy is null
and the taroz goal remains unmet.


## A217M coverage failure and A205u historical-recipe difference

October A217M returned rc=0 but fails the independent raw-key audit:
1,220 native outputs omit two of 1,222 raw epochs. Both omissions precede
the first output (UTC 1665093113990 and 1665093115004; first output
1665093115304). The internal summary counts only the admitted 1,220 keys,
so successful execution and its self-reported completeness are insufficient.
No accuracy score is accepted and truth was not read by this localization.
Leading IMU states are disabled in this baseline; a separate recovery
should test bounded leading-state admission with continuous same-stream
IMU and preserve these exact UTC keys. See
`use_cases/records/gsdc2023_a217m_missing_keys.json`. The fixed evaluation
remains 21 scored, now three failed, two running and 14 not started.

The historical A205u source-initialization result also cannot substitute
for the failed fixed baseline: its base ECEF coordinate differs by
0.538272153 m and the executable differs. Raw inputs and other arguments
match after ignoring output paths and the order of two boolean flags.
This is documented in `use_cases/records/gsdc2023_a205u_historical_recipe_difference.json`;
no score change is attributed to the base-coordinate difference.


### Queue recovery and A217M leading-state experiment

The waiting LAX-o supervisor terminated on WinError 5 during atomic
replacement of its state JSON, cascading to the two waiting A325G
supervisors. Their terminal process handles and absence of native output
directories were verified before recovery. Failed state records remain
archived with `.failed_<timestamp>.json` names. Bounded PermissionError
retry was added to atomic state replacement; only these waiting
supervisors were restarted, preserving the four active native jobs.

A217M now has a separate recovery plan adding only
`--native-leading-imu-states` to the same c779 binary and fixed baseline
inputs. It waits after both A325G comparisons. Post-run checks require
all 1,222 original raw keys, the two specifically missing leading UTC
keys, exactly two leading states, zero output interpolation/hold, and
native provenance before scoring available truth. The original failed
baseline remains unchanged and has no accepted score. Plan SHA-256:
`212974275f17c78d2b66cc5870abefa603d30d3fc8b3d8474082fa44a55ecdd8`.
See `use_cases/records/gsdc2023_train_a217m_leading_states.json`.


## January 4 Pixel5 source-noise bundle completed

Under the same explicit source Highway/L5=1 settings, enabling source
TDCP meter sigma together with source Huber k=0.5 improves local score
from 0.651683176 to 0.541417591 m (-0.110265585 m). Candidate P50 is
0.450390739 m and P95 0.632444442 m. Both arms retain 40,347 TDCP
factors and 1,855 native raw keys, with the same seed metadata and input
hashes. Candidate convergence is reported after 178 GNSS and 196 main
iterations; wall time is 5,170.2 s versus 778.7 s under shared load.

Compared with the original frozen recipe (0.594509466 m), the candidate
is better by 0.053091876 m, but this changes
multiple controls including source route P/D weights. It is not an
isolated Huber effect or evidence for uniform promotion. The conditional
March10 Huber run remains active, and the January4 stage-export control
has started after this completed audit. See
`use_cases/records/gsdc2023_january04_source_tdcp_noise_decision.json`.


### Fixed-input A205u recovery queued

A new recovery retains all four current raw inputs, the current base ECEF
coordinate and all baseline option values, including leading IMU states
and source IMU initialization. Only the executable and output paths
change: the frozen 76010 binary includes the leading-clock boundary
repair. Its other default-off changes mean this is not a pure single-code
change causal experiment. Existing historical output with a different
base coordinate is not reused. The plan waits after A217M and requires
1,213 exact native raw keys, four leading states, complete truth coverage,
and no output interpolation/hold before accepting a local score. The
original fixed baseline remains failed. Plan SHA-256:
`caa25bfd7bc4a56f24638dbc316d7a329103f4aa7ede16bee6f50191146e511a`.
See `use_cases/records/gsdc2023_train_a205u_fixed_recipe_recovery.json`.


## October A325F and 22-drive checkpoint (2026-09-23)

October 6 A325F passed the independent audit for all 1,230 raw UTC keys
and all 1,230 available truth keys. Missing output, interpolation and
hold counts are zero. The run used one temporal initial guess and 1,229
independent SPP initializations; these are solver seeds, not substituted
output coordinates. Local P50 is 1.285457719 m, P95 2.725284586 m,
mean 2.005371152 m, with 791.6 s wall time under shared load. Both
A325F train routes are now scored, but this October route group remains
incomplete due to the A205u and A217M failures. The fixed baseline
contains 22 scored, three failed, two running and 13 not started;
full-scope mean remains unavailable and the taroz goal remains unmet.


## January4 stage-export control verified

The 76010 stage-export control reproduces the historical c779 final
solution and seed metadata byte-for-byte. All 1,855 raw stage keys are
ordered identically and finite; 1,854 available truth keys are scored.
The native GNSS-first stage local score is 0.777884109 m (P50
0.624645664, P95 0.931122554), initial IMU stage 0.594510467 m
(P50 0.492371414, P95 0.696649520), final rounded output 0.594509466 m.
This establishes control export equivalence for this route and gives
a native-stage baseline, not MATLAB runtime equivalence. Candidate
stage comparison remains pending; its run has started. The comparator
now supports an available-only diagnostic that explicitly records
`paired_validation_complete=false` and cannot report a paired delta
until both audited arms exist. See
`use_cases/records/gsdc2023_january04_sigma_stage_partial.json`.


## First Pixel7 Pro fixed-recipe result: 23-drive checkpoint

November 15 MTV-a Pixel7 Pro completed in 432.8 s under shared load.
All 1,231 raw UTC keys pass the independent native audit, with no
missing output, interpolation or hold. All 1,193 available truth keys
are covered; 38 native keys without truth are unscored. Two temporal
initial guesses are recorded separately from the final native positions.
Local P50 is 0.332680350 m, P95 0.684374132 m, mean 0.508527241 m.
This is one of five planned Pixel7 Pro routes and does not establish
phone-wide accuracy. The fixed baseline now has 23 scored drives, three
failed, two running and 12 not started; full-scope accuracy remains null.


## March 8 Pixel5 and 24-drive checkpoint

The 2023 March 8 MTV-u Pixel5 baseline passes audit on all 1,102 raw
and truth UTC keys, with no missing output, interpolation or hold.
Local P50 is 0.458724635 m, P95 0.640732132 m and mean 0.549728384 m.
Shared-load wall time is 251.7 s. One temporal initial guess and 1,101
independent SPP seeds are recorded separately from the optimized outputs.
The fixed evaluation now contains 24 scored drives, three failed, two
running and 11 not started. No full-scope accuracy or goal completion
is claimed; all are existing development data.


## March 8 Pixel6 Pro and 25-drive checkpoint

March 8 MTV-u Pixel6 Pro passes all 1,102 raw/truth keys, with zero
missing output, interpolation or hold. Local P50 is 0.802946452 m,
P95 1.124946051 m and mean 0.963946252 m. Wall time is 225.4 s
under shared load; one temporal seed is recorded separately from the
optimized output. It remains in the same evaluation group as the
March 8 Pixel5 and other planned phones from that route. The fixed
baseline now has 25 scored drives, three failed, two running and ten
not started. Full-scope accuracy remains unavailable and goal unmet.


## January4 matched native-stage comparison complete

Both 76010 export arms reproduce their historical c779 final output
bytes exactly. All stage raw keys match and share the same 1,854 scored
truth keys. Sigma-only changes GNSS-first local score from 0.777884109
to 0.730633641 m (-0.047250468), and initial IMU from 0.594510467
to 0.396727370 m (-0.197783097). Candidate final rounded-output score
is 0.396726540 m. The larger post-IMU difference does not isolate an
IMU-only cause: both stages change weighting and the GNSS solution
is passed into the IMU stage. This remains one exposed native comparison,
not taroz MATLAB runtime parity, and the sigma-only uniform policy
remains unpromoted given regressions on three other routes. See
`use_cases/records/gsdc2023_january04_sigma_stage_comparison.json`.

The completed stage pair released one native slot. The unstarted A325G
source-initialization experiment was moved from waiting on LAX-o to
this released lane without changing its plan, binary or inputs. Only
the positively identified waiting supervisor was stopped; its prior
state is retained with a `.rerouted_<timestamp>.json` name. The new
native run is active, downstream drift/coverage-recovery supervisors
remain live, and the four-native-process concurrency ceiling is kept.


## A325G source initialization fixes the large local failure

The matched same-c779, same-input A325G experiment adds only source
IMU initialization and improves local score from 11.929479401 to
1.317772064 m. P50 changes from 2.903674679 to 0.974115480 m,
P95 from 20.955284122 to 1.661428648 m. All 1,512 raw native
keys and all 1,487 available truth keys are retained. GNSS-first summary
and raw seed metadata match exactly; unexported GNSS state arrays are
not claimed byte-identical. TDCP factor count remains 19,656.

The optimized maximum acceleration bias falls from 19.624158182 to
0.274142665 m/s^2, with main iterations 114 to 107. This supports
initialization as an actionable contributor to the poor solution, but
the source option changes attitude and bias initialization together,
so their individual causal effects remain unresolved. Posthoc error
localization now counts 0 errors above 10 m
and maximum 2.717253 m. Candidate shared-load runtime
is 339.1 s. The separate baseline-plus-Samsung-drift run has started.

See `use_cases/records/gsdc2023_train_a325g_source_init.json`,
`gsdc2023_a325g_source_init_stage_diagnostic.json` and
`gsdc2023_a325g_source_init_error_localization.json` in that directory.
This remains one exposed training route, not official test validation.
It is a strong candidate for same-phone native test replay; no new
external submission or global-default promotion is authorized here.


### A325G same-phone test replay queued

Both existing A325G test routes (May12 MTV-pe1 and June22 LAX-hh)
are selected by phone name for a source-IMU-initialization-only replay.
The c779 binary, four raw input hashes per route, base coordinates and
all other flags match the reviewed submitted recipe. Each old run record
is hash-pinned. Input hashes were checked before queueing; max workers
is one and the replay waits after LAX-o main-Doppler, preserving the
four-native-process ceiling. Plan SHA-256:
`70f6b23745a979cb7f325f65031d0e56dd4cab59d5768ddd8105a766362c5a15`.

The post-run audit requires native raw-key completeness, exact coverage
of official sample keys (coordinates are not used), convergence, identical
GNSS-first summaries and seed metadata versus the baseline, and only
the intended option difference. Test accuracy remains unknown: these
runs do not use test truth and no external submission is queued. See
`use_cases/records/gsdc2023_test_a325g_source_init.json`.


## A325G clock-drift-only comparison complete

Adding only source Samsung clock-drift preprocessing to the poor frozen
baseline leaves the local P50/P95 score exactly unchanged at
11.929479401 m; all 1,512 native raw keys are retained. The output CSV
is not byte-identical: 197 coordinate rows differ, with maximum
horizontal change 1.42092277029e-05 m. No accuracy benefit is
established from this intervention on this route. The separate source
IMU initialization result (1.317772064 m) is therefore the supported
improvement candidate; these experiments do not claim an interaction
test between initialization and drift. Shared-load runtime is 365.2 s.
The A217M leading-state recovery has now started in the released slot.
See `use_cases/records/gsdc2023_train_a325g_samsung_drift.json` and
`use_cases/records/gsdc2023_a325g_clock_drift_output_delta.json`.


## A217M recovery reaches GNSS solve but fails IMU bracket check

Leading-state admission recovered the two input epochs, but the candidate
returned rc=1 after GNSS-first optimization: mapped IMU does not bracket
the complete GNSS interval. No solution or score is accepted. Raw IMU
starts before the missing leading epochs, while its gyro ends 31 ms
before the last GNSS UTC key (accel ends 21 ms before). The successful
1,220-key baseline mapping likewise ends approximately 30.775 ms early.
This points to the full-graph endpoint check activated by the leading
option, rather than absent leading IMU. Exact failed-run mapped times
are not exported. Investigate coverage-check scope and normal tail
integration semantics before changing any acceptance rule; no fabricated
samples or output interpolation is introduced. See
`use_cases/records/gsdc2023_a217m_imu_bracket_failure.json`.

The dependent A205u waiting supervisor was terminally failed by the
A217M failure even though the experiments are independent. Its failure
record was preserved; after verifying no A205u native run had started,
its dependency was moved to the completed A325G drift comparison. A205u
now uses the released slot without restarting any live native run.


### A205u fixed-input recovery and A217M tail audit

A205u recovery completed with all 1,213 raw keys represented by native states, four leading states, and strict IMU coverage. Local development P50/P95 are 1.318105211 / 2.515007521 m, score 1.916556366 m, wall time 227.665 s. The frozen baseline remains failed; this recovery is separate and is not an isolated one-code-change comparison. See `use_cases/records/gsdc2023_train_a205u_fixed_recipe_recovery.json`.

A217M inspection established that default native integration reuses the final IMU measurement to reach each GNSS endpoint, whereas pinned source uses inclusive sample selection with forward sample durations. Neither proves equivalent runtime behavior. Leading-state recovery currently requires full-graph bracketing, and the independent audit requires that same proof. Therefore a scope change cannot silently reuse the existing full-graph coverage field. Exact failed-run mapped bounds still need instrumentation before a new policy experiment. Source hashes and locations: `use_cases/records/gsdc2023_a217m_imu_tail_semantics.json`. No coverage policy or inference output was changed.


### Exact A217M coverage failure and March10 Pixel5 Huber result

Diagnostic-only C++ instrumentation now exports mapped sample bounds on coverage failure. Build succeeded; the same A217M input/options rerun (binary `979f7882dcbe6a8601dcf0ca148da8e08f8d2694ffccf079b06c0cb1f4adc944`) reproduced the expected failure. Relative to its first GNSS epoch, first IMU = -2.563251953921281 s, last IMU = 1219.9692237828276 s, required end = 1219.9987840430113 s; exact trailing shortfall = 0.02956026018364355 s. This replaces the approximate 30.77 ms inference from the older baseline mapping. No coverage acceptance policy changed. Record: `use_cases/records/gsdc2023_a217m_coverage_diagnostic.json`.

March10 Pixel5 sigma-plus-source-Huber-k=0.2 completed with 1,465 native raw keys. Local score = 0.5443656080013418 m (P50 = 0.5007200555912674, P95 = 0.5880111604114162), versus sigma-only 1.0359009354593491 and original fixed recipe 0.6779106294218623 m. Delta from original = -0.1335450214205205 m. This is one exposed development route, not heldout evidence or a global promotion. Wall time = 4159.587830543518 s under shared load; no controlled runtime claim. Matched inputs/seed metadata and audit are recorded in `use_cases/records/gsdc2023_train_pixel5_march10_tdcp_huber.json`. The dependent LAX-o main-Doppler evaluation has started.


### Follow-up validation after March10 Huber result

The two A325G test replays were moved into the freed fourth native lane after the A217M diagnostic. The waiting supervisor was verified idle and stopped, its prior state archived, and its dependency moved to the completed A205u recovery. Inference plans, binary and inputs are unchanged; neither replay reads test truth or submits externally.

The same two previously exposed Mi8/Pixel4 routes that regressed under source TDCP sigma alone are now queued for a matched Huber follow-up after the A325G test replays. Frozen executable remains `76010f6c5e9a9296b06f8f389554f786c8d5ba8727543c7008f8e90f716ca08d`; only `--native-phase184-source-tdcp-huber-k` is added to the sigma-only candidate options. Plan SHA256 `60b293ac7598b0ca6fdf42036336a7d1e31de5004ecda3cc5b0259d714a76fe9`. The comparator checks both sigma-only and original fixed-setting scores, native keys, input hashes, seed metadata, factor counts, sigma and actual Huber threshold. These are preselected development comparisons, not heldout tests. No global promotion follows merely from improvement over the regressed sigma-only controls. Record: `use_cases/records/gsdc2023_train_mi8_pixel4_tdcp_huber.json`.


### LAX-o Pixel5 main-Doppler comparison complete

Adding only `--native-phase213-main-doppler` to the fixed LAX-o Pixel5 recipe improved local score from 1.7897514161349024 to 1.7459558079350193 m (delta -0.04379560819988315 m). Candidate P50/P95 = 1.1563500141759702 / 2.3355616016940686 m. Its 209.7489516735077 s wall time versus baseline 2196.177908182144 s was recorded under shared load, not a controlled speed benchmark. The matched audit completed, but one exposed route does not establish global benefit. See `use_cases/records/gsdc2023_train_pixel5_lax_o_main_doppler.json`.

Frozen baseline 2023-05-09-21-32-us-ca-mtv-pe1/pixel5 completed with local score {"distance_variant": "haversine_sphere", "p50_m": 0.6416612246605231, "p95_m": 1.290429910550626, "percentile_variant": "linear_n_minus_1", "phone_score_m": 0.9660455676055746}. The authoritative run identifier is May9 Pixel5 (the earlier progress description misidentified pending slot 29 as March8 Pixel7 Pro). Use its actual course as the evaluation group. Incremental total is now 26 scored, 3 audit failures, 2 pending and 9 not started. Full-set success remains unproven.


### Bounded terminal IMU support helper

Added a separate bounded-tail coverage helper; the strict check remains unchanged and no inference caller uses the new helper yet. The proposal bounds the existing integrator terminal measurement hold by the existing 50 ms continuity ceiling, requires real coverage through the recovered leading interval, validates the whole timestamp stream, and rejects interior gaps. It reports full real-sample bracketing separately from terminal measurement reuse. All 27 IMU CSV/coverage tests passed, including three new policy cases. No accuracy or source-equivalence claim follows from these tests. CLI opt-in, independent audit integration and A217M runtime evaluation remain outstanding. Evidence: `use_cases/records/gsdc2023_bounded_imu_tail_policy_tests.json`.


### Explicit bounded-tail CLI and audit integration

`--native-leading-imu-bounded-tail` now opts into the tested terminal measurement-hold policy only alongside leading-state recovery. Extended-gap and source-inclusive-forward schedules are rejected; actual leading states are required at runtime. The report distinguishes full real-sample coverage from bounded terminal measurement reuse and exports actual relative sample bounds, required real interval and hold duration. The independent output audit validates these fields and rejects excessive holds, conflicting full-coverage claims, invalid leading coverage, and unsupported schedules. Default strict behavior remains unchanged. Build succeeded; 27 C++ IMU tests, 11 Python audit tests and three CLI rejection cases passed. The initial pytest plugin/console failure was bypassed with unittest discovery.

A217M matched runs use frozen binary `d122fbeecd8d7295c5f37b3660f699aa3cc37993e586a7a55e49182503bdc43d`, identical inputs/options except the new flag, and separate output directories. The strict arm is expected to fail the original full-graph bracket check; only a complete, converged native candidate with all 1,222 raw keys and two proven leading states can be scored. Runtime and accuracy results are still pending. Pipeline: `E:/rtklib_v2_ws_output/gsdc_native/train_a217m_bounded_tail_pipeline.json`.

A325G test source-initialization replays passed the matched audit: May12 has 1,435 raw native rows / 1,423 official keys, June22 has 1,862 raw native rows / 1,861 official keys. Both preserve GNSS-first summary and seed metadata, use the same raw inputs/executable/other options, and cover every official key with native states. No test truth was read; accuracy remains unknown and no submission was performed.


### Coincident May16 evaluation groups corrected

Raw observation times prove that May16 xe1 Pixel5 and Pixel7 Pro overlap for 2,320.447 s (99.9331% of the shorter interval), with matching date/route identifier despite 19:54 versus 19:55 folder names. They are conservatively treated as one evaluation group. Frozen inference plans and per-drive scores remain unchanged; the aggregation override is pinned to the plan and evidence hashes, cannot split an existing group, and preserves incomplete groups. Three focused tests passed. Corrected planned evaluation groups: 29 (previous course-name grouping was 30). Future train40 validation uses this grouping audit.

May16 Pixel5 local score: {"distance_variant": "haversine_sphere", "p50_m": 0.96309193084128, "p95_m": 1.513573876464443, "percentile_variant": "linear_n_minus_1", "phone_score_m": 1.2383329036528616}. Train status: 27 scored, 3 audit failures, 2 pending, 8 not started. No full-scope score or completion claim. Evidence: `use_cases/records/gsdc2023_coincident_route_group_audit.json`.


### A217M bounded-tail recovery verified

The matched strict arm failed the original bracket gate as expected. With only explicit bounded-tail acceptance enabled, A217M produced all 1,222 raw keys, including two leading states with recorded non-SPP temporal initialization. Candidate local score is 2.019059794920038 m (P50 1.4209790871168606 / P95 2.617140502723216), wall time 208.27556443214417 s. The audit records a terminal IMU measurement hold of 0.02956026018364355 s and maximum paired-sample gap 0.03899998334236443 s; full real-sample bracketing remains false. Output-coordinate interpolation/hold remains zero. Strict and candidate binaries, raw inputs and initial seed metadata match. This recovery is separate from the failed frozen baseline and is not a precision improvement claim against an incomplete baseline.

A follow-up adds only `--native-source-imu-initialization` to this complete recovered A217M configuration, using the same `d122fbeecd8d7295c5f37b3660f699aa3cc37993e586a7a55e49182503bdc43d` executable. Plan SHA256 `062c81f39c6607e435898e9b81f88afde4189865092bf1c44a9b065d91db5b65`. It will compare local score, native keys, leading provenance, bounded measurement hold and seed metadata. This exposed development route is not independent of the Oct6 A205u/A325F recordings. Pipeline: `train_a217m_source_init_pipeline.json`. No external submission or default promotion.


### Audited all40 A325G submission candidate prepared

A local candidate now replaces only the two A325G test runs in the last official all40 payload; 3,284 rows change, 68,652 rows across 38 other drives remain identical. All 71,936 official keys pass native provenance checks. SHA256 `119279101c2cf3bf50b8b64f2b195cb1ba8e7939b903cc5a8477cc41799e9125`. It has not been submitted and its score is unknown. Review: `gsdc2023_all40_a325g_candidate_review.md`. Explicit user instruction is required before this new external submission; other local experiments continue independently.

A217M source-initialization follow-up completed: local score 2.019059794920038 -> 2.018911155622045 m (delta -0.0001486392979930251 m). Matched native output/seed provenance audit passed. This is the already exposed Oct6 route group; no global promotion or independent validation claim. Record: `use_cases/records/gsdc2023_train_a217m_source_init.json`.


### sm-g988b long-running control follow-up

The May13 sm-g988b fixed-recipe native process remains live; it was not stopped or restarted. Before its final score became available, a separate matched source-IMU-initialization candidate was selected to investigate the long runtime and initializer dependence. Same frozen c779 executable, raw input hashes, base coordinates and options; only `--native-source-imu-initialization` is added. Plan SHA256 `77ee0b394e17d1ce95649f33c06b87163a5fa394b661c8d596a294631a4f0078`. The candidate will be audited/scored separately, then its supervisor waits on the actual control process before a paired comparison; a failed control cannot supply a fabricated score. No controlled runtime speed claim will be made under shared load. This is the same May13 group as Pixel6 Pro, not an independent route.

The prepared A325G all40 submission candidate remains unsubmitted pending explicit user instruction. Local experiments continue independently; the lack of a submission response is not a blocker for these evaluations.


### Train checkpoint: May16 Pixel7 Pro and May23 A505G

The frozen-recipe evaluation now has 29 scored, 3 audit failures, 2 pending and 6 not started. May16 Pixel7 Pro: local score 0.8567100739528557 m, P50 0.6021609895988409 / P95 1.1112591583068705 m. Both May16 phones now complete the same corrected evaluation group; equal-phone group score = 1.0475214888028588 m. May23 sm-a505g: local score 1.7925908486090332 m, P50 1.2912038756457003 / P95 2.293977821572366 m. There are 29 planned evaluation groups after the pinned coincidence correction. Full-scope scores remain unavailable because failed and pending runs remain. The original sm-g988b process and source-init candidate remain live; no timeout-based restart was performed.


### A325G initializer attribution and train30 checkpoint

Code/runtime inspection confirms that the default Android path replaces the static-alignment attitude with the GNSS first velocity heading, then propagates per-epoch orientations using IMU delta rotations. Source initialization instead inserts GNSS-derived orientations at all 1,512 A325G epochs, changes 244 interior low-speed heading fills from linear to nearest (280 nearest in total), and zeroes both initial biases. The previously reported 19.624 m/s2 is an optimized bias maximum, not evidence of a 2g initial bias. Existing heading-only CLI admission is Pixel5-only; no A325G isolation run was silently attempted. Component attribution remains unresolved. Evidence: `use_cases/records/gsdc2023_a325g_initializer_components.json`.

May24 Pixel7 Pro passed native coverage and local scoring: {"distance_variant": "haversine_sphere", "p50_m": 0.30260029453524095, "p95_m": 0.5569036862688171, "percentile_variant": "linear_n_minus_1", "phone_score_m": 0.42975199040202905}. Frozen train status is now 30 scored, 3 audit failures, 2 pending and 5 not started; full-scope metrics remain unavailable.


### sm-g988b source-initialization candidate scored; control pending

May13 source-initialization candidate completed and passed the full native audit: 2,182 raw keys, zero missing/output-interpolated/held coordinates, local score 1.2956856153497034 m (P50 0.8619685628371849 / P95 1.7294026678622219), shared-load wall time 1059.8011481761932 s. The fixed-recipe control is still live, so a paired accuracy improvement is unproven. Its supervisor is explicitly waiting on that control before comparison.

The other planned sm-g988b train route, `2021-07-14-20-50-us-ca-mtv-e/sm-g988b`, is now running with only the same source-initialization flag added. Plan SHA256 `e929585d697c216578ae4d493ab5a9b1f9039f944a4021ad1900c4c6ddf829d6`; raw inputs, frozen c779 binary, base coordinates and all other options match. This is a separate date/route group, but previously exposed development data. Candidate identity was read from the frozen plan after correcting an initial mistaken 2022 date lookup; no erroneous run was launched.


### Train checkpoint: S908B scored, new Pixel7 Pro initialization failure

May25 sm-s908b passed all 1,399 native raw keys with local score 1.0837207115506682 m (P50 0.9297729128480485 / P95 1.237668510253288). The next May25 Pixel7 Pro run failed before IMU/main graph entry with `temporal-initialization-invalid-identity-time-or-raw-drift`, despite all 1,259 raw-SPP epochs being accepted. Raw input drift-field inspection is recorded in `use_cases/records/gsdc2023_pixel7pro_may25_raw_drift_failure.json`; no admission policy was changed. Train totals: 31 scored, 4 audit failures, 2 pending, 3 not started.


### Pixel7 Pro failure traced to missing common clock preprocessing

The May25 he2 Pixel7 Pro input has exactly five leading missing DriftNanosPerSecond epochs; the first finite value is 15 ns/s = 4.49688687 m/s. No adjacent finite drift jump exceeds 50 m/s. Pinned `preprocessing.m:161-171` applies jump cleanup and remaining-NaN nearest fill to every phone, outside the Mi8 and Samsung-specific branches. Thus the source recipe supplies the first finite drift at these five leading keys. Native loading preserves missingness and the temporal adapter rejects it before the graph. An explicit provenance-preserving preprocessing opt-in is the next recovery action; do not mislabel filled drift as an original finite raw measurement or confuse drift initialization fill with trajectory/output interpolation. No MATLAB execution equivalence or code fix is claimed yet. Record: `use_cases/records/gsdc2023_pixel7pro_may25_raw_drift_failure.json`.


### Source raw clock cleanup implemented and matched replay started

Added default-off `--native-source-raw-clock-drift-cleanup`, initially admitted for Pixel7 Pro Phase171 raw-clock temporal initialization including the first epoch, excluding Samsung drift estimation. It reuses the source common jump-mask/fill arithmetic and adds donor-index/weight provenance without changing the arithmetic of the existing Samsung path. Original drift values, selected values and donors are written to a hashed sidecar; seed metadata explicitly labels preprocessed drift and exports the actual clock-rate initializer. The independent audit reconstructs expected cleanup directly from the original CSV and checks donor indices, values, counts and seed values. Six C++ cleanup tests, five raw-clock audit tests, eleven existing output-audit tests and two CLI rejection checks passed.

Frozen binary `c3abb65f98098f4be611426285e743d2ebe8e7a48b3e61718e3a0d4a24f4a9b3`; candidate plan SHA256 `3ef9e1e4f9e059d1b479c1a405f943a50a25f8d834ed44cead7f97b3b16e23aa`. The same-binary strict control reproduced the original adapter failure; candidate inference is running. No recovery or accuracy claim until all 1,259 keys pass native audit and local evaluation.

July14 2021 sm-g988b source initialization completed: 0.6259142594821209 -> 0.6261038489634385 m (delta +0.00018958948131764242 m). This slight regression does not support global promotion; May13 control remains pending.


### Clock-cleanup binary provenance export issue caught by audit

Early independent audit rejected the first Pixel7 Pro cleanup candidate: its sidecar correctly records five leading fills, but all 1,259 seed rows lack `selected_clock_rate_mps`. The original c3abb65 binary lacks that string, despite the late serializer edit existing in source; the edit occurred during compilation and a subsequent incremental build skipped recompilation. Its output is not accepted as audited, and the audit was not weakened. The existing native run is preserved.

Forced recompilation after touching the source produced frozen binary `d73f779ea848a7409a5e12d0c1e6e129cf0b8f004a7e9e2fcf7f9a44d6f231f5`, confirmed to contain the required export label. A fresh matched strict/candidate replay is queued after the original native run terminates; plan SHA256 `e1a946684fe8a25a915fd8c25b11737338d0b19937a2d46d8654027561be13b9`. A new regression test explicitly rejects metadata without numeric seed exports; six raw-clock audit tests pass. Actual new runtime exports and native recovery still require verification. Incident: `use_cases/records/gsdc2023_raw_clock_seed_export_build_incident.json`.


### Mi8 sigma-plus-Huber comparison and train32 checkpoint

Jan5 Mi8 sigma-plus-source-Huber completed and passed matched input/binary/option/native-output/seed-provenance checks. Original fixed score 0.7667595928133726 m, sigma-only 0.8394787410888614 m, sigma-plus-Huber 0.6800592724954455 m (P50 0.4203673360248381 / P95 0.939751208966053). Delta from original = -0.08670032031792707 m; from sigma-only = -0.15941946859341594 m. Pixel4 remains pending, so the two-phone comparator now supports explicitly partial reports and does not claim paired-comparison completion. Record: `use_cases/records/gsdc2023_train_mi8_pixel4_tdcp_huber.json`.

September5 Pixel5 local score: {"distance_variant": "haversine_sphere", "p50_m": 0.5164153440028334, "p95_m": 1.0003401312520697, "percentile_variant": "linear_n_minus_1", "phone_score_m": 0.7583777376274515}. Frozen train status is 32 scored, 4 failures, 2 pending, 2 not started. September5 Pixel5 and September6 Pixel6 Pro with route name routen have non-overlapping raw time intervals separated by 1,687,564 ms, so they are not merged merely because the route suffix matches; the corrected planned group count remains 29.


### Full raw-drift scope inventory and train33 checkpoint

Audited first-Raw-row drift fields for the frozen 40 train and 40 test inputs; all raw hashes match their frozen plans. Among the 29 train and 29 test runs where pinned source preprocessing uses the original raw drift, only May25 he2 Pixel7 Pro has a common-cleanup trigger: five leading missing epochs, no >50 m/s adjacent jump. No original-raw-drift test run triggers this cleanup. All ten Mi8/Xiaomi Mi8 runs have missing raw drift throughout, but source replaces drift with the receiver-clock gradient before common cleanup; listed Samsung phones also replace drift first. Thus raw-field missingness on these phones is not evidence of a source-effective cleanup failure. This inventory does not establish post-replacement Samsung/Mi8 equivalence or accuracy. The Pixel7 Pro recovery remains necessary for train coverage but this correction alone is not expected to alter current test inputs on the original-raw-drift path. Record: `use_cases/records/gsdc2023_raw_clock_cleanup_inventory.json`.

September6 routen Pixel6 Pro passed all 1,650 native keys with zero coordinate interpolation/hold. Local score: {"distance_variant": "haversine_sphere", "p50_m": 0.7158974623255253, "p95_m": 1.5300846707700977, "percentile_variant": "linear_n_minus_1", "phone_score_m": 1.1229910665478116}. Runtime 704.8995225429535 seconds. Frozen train status: 33 scored, 4 audit failures, 2 pending, 1 not started; full-scope score remains unavailable.


### Corrected Pixel7 Pro initializer export verified; January4 Huber ablation queued

The d73f779 corrected binary is now running. Independent reconstruction from the original raw CSV verified all 1,259 numeric seed clock-rate exports, five leading fills and zero jump masks. Actual runtime export is now proven; final optimized output and accuracy remain pending. The original c3abb65 candidate finished inference but correctly failed the missing-numeric-export audit. Record: `use_cases/records/gsdc2023_pixel7pro_clock_initializer_check.json`.

Queued January4 Pixel5 conditional Huber-only ablation after the Mi8/Pixel4 Huber comparison. Plan SHA256 b5040939ec05ea0a7e289157ac4a99706f463844dbfa088a30fd9a14443e1c2b, same c779 binary and sigma-only input recipe. Only source Huber flag is added; frozen native Street context gives k=0.2. Source settings say Highway, so this is explicitly a conditional native-context ablation, not full source-route parity. Original score 0.5945094662429546 m, sigma-only 0.3967265396421846 m. Existing Highway route-plus-noise bundle is separate. Record: `use_cases/records/gsdc2023_train_pixel5_january04_tdcp_huber.json`.


### Pixel7 Pro common clock-cleanup recovery fully audited

The corrected d73f779 matched replay completed and passed the independent raw-input/donor/numeric-seed/native-output audit. All 1,259 raw UTC keys are present as native estimated states, with zero coordinate interpolation or hold. The same-binary strict control still fails the original raw-drift gate. Five leading drift values are filled from source epoch index 5; no jump mask is needed. Local development score is 0.7998717050807369 m (P50 0.7145458230395427 / P95 0.8851975871219311), wall time 1505.2338078022003 s. This is a recovery from an execution failure; no valid strict-control accuracy score exists. Final solution CSV is byte-identical to the prior c3abb65 diagnostic-incomplete run: True. Only the corrected replay passes the required numeric seed provenance audit. No official submission or test accuracy improvement is claimed. Record: `use_cases/records/gsdc2023_train_pixel7pro_may25_clock_cleanup_verified.json`.


### Train34 checkpoint and January4 Huber replay started

September6 routebb1 Pixel7 Pro passed all 1,602 native raw keys; local score {"distance_variant": "haversine_sphere", "p50_m": 0.5446666362309458, "p95_m": 1.5656382161345792, "percentile_variant": "linear_n_minus_1", "phone_score_m": 1.0551524261827625}. Runtime 1948.6314897537231 s. Frozen train status: 34 scored, 4 audit failures, 2 running, none not started. The four original failures remain preserved separately from their recovery experiments; full-scope fixed-recipe score is still unavailable.

After Pixel7 Pro clock-cleanup verification freed a native slot, the January4 Huber-only replay was started with its existing immutable plan. Only the waiting supervisor was rescheduled (old PID 26536 archived; new PID 36512), after confirming no native output directory existed. No native inference was restarted. Candidate native PID 19592; the old Mi8/Pixel4 dependency was a capacity wait, not a data dependency.


### Consolidated recovery coverage, distinct from fixed-recipe success

Re-audited all four failed frozen train cases against their original identical inputs, recorded binary hashes, frozen output hashes and native-key contracts. Mi8 long-gap recovery: 1,856 keys, 0.5888759370502147 m. A205u boundary recovery: 1,213 keys, 1.916556366229106 m. A217M bounded-tail recovery: 1,222 keys, 2.019059794920038 m. Pixel7 Pro raw-drift recovery: 1,259 keys, 0.7998717050807369 m. Every recovered output has zero missing raw keys and zero coordinate interpolation/hold. This does not remove initializer/sensor handling: A217M explicitly holds the terminal IMU measurement for 29.560260 ms.

The May13 SM-G988B source-initialization candidate also independently passes all 2,182 native keys at 1.2956856153497034 m, while its original control remains running. Combined with the 34 fixed-recipe successes, individually audited native results exist for 39 distinct train drives. September7 Pixel5 remains pending. These are multiple executable/configuration recipes, not a unified 39/40 recipe result; no composite full-scope accuracy score is reported. All four original fixed-recipe failures remain unchanged. Consolidated evidence: `use_cases/records/gsdc2023_train_recovery_coverage_summary.json`.


### All40 train drives now have individual native-output evidence

September7 Pixel5 completed with all 1,172 raw UTC keys as native states and local score {"distance_variant": "haversine_sphere", "p50_m": 0.6875661885363057, "p95_m": 1.4164365286824563, "percentile_variant": "linear_n_minus_1", "phone_score_m": 1.052001358609381}. Runtime 877.1592252254486 seconds. Frozen baseline now has 35 scored runs, four preserved failures and the May13 SM-G988B control still running. Including the four independently audited failure recoveries and the already scored May13 source-initialization candidate, every one of the 40 planned train drives has an individually audited native output and local score. Updated `use_cases/records/gsdc2023_train_recovery_coverage_summary.json` records the distinct-drive coverage as 40.

This is coverage evidence across multiple frozen binary/configuration experiments, not a single corrected-recipe full-train score, source stage parity, or proof of taroz-level accuracy. No composite full-scope score is claimed and no new official submission has been made. Pixel4 and January4 Pixel5 sigma-plus-Huber comparisons remain running.


### A325F final-graph Doppler comparison started with matched executable controls

Existing records show the two-route A325F source-initialization-only experiment had negligible regressions (+0.0001314675 and +0.0000738188 m); no duplicate initializer replay was started. Source fgo_gnss_imu.m:214-217 adds Doppler to the final graph, whereas both frozen A325F baselines have zero main Doppler factors (13,150 and 12,235 GNSS-first Doppler factors). A two-route isolated main-Doppler replay was prepared, but c779 rejected both cases before inference because its Phase213 admission is Pixel5-only. Those rc2 attempts remain preserved, without scores.

The d73f779 executable supports the broader Phase213 admission present in current source. Fresh same-executable controls and candidates are running sequentially (one worker); only the main-Doppler flag differs. All original raw inputs, native Street context and other settings remain fixed; source labels Highway are recorded as a separate mismatch. Control plan SHA256 f3f4b2896f9a6d0b39173914585ac6fe7201466267c223762bd25653d37728c6; candidate plan SHA256 3a0be226b365e79256f86ac2da1549aea065b4717bacc9847364445446ad5ce3. No accuracy claim until both routes pass native audit and matched comparison. Records: `use_cases/records/gsdc2023_train_a325f_main_doppler.json` (legacy CLI rejection) and `use_cases/records/gsdc2023_train_a325f_main_doppler_matched.json` (current experiment).


### Mi8/Pixel4 sigma-plus-Huber matched comparison complete

Pixel4 July14 now passes the three-arm matched audit: original 0.466482271733667 m, sigma-only 0.5188979925650274 m, sigma-plus-Huber 0.44173254449840604 m (P50 0.2702025399595372 / P95 0.6132625490372748). Delta from original is -0.024749727235260977 m; from sigma-only -0.0771654480666214 m. All 21,090 TDCP factor counts and sigma-only/candidate metre sigmas match; both use the same raw inputs, frozen executable, seed metadata and evaluation truth. The Huber arm uses the frozen native-context k=0.2. Mi8 also improves from 0.7667595928133726 to 0.6800592724954455 m.

These two previously exposed development routes support the paired sigma/Huber combination over sigma alone, but do not establish uniform improvement or full source route parity. Pixel4 shared-load runtimes were 318.2170548439026 s original, 1204.7946166992188 s sigma-only and 3934.026197195053 s sigma-plus-Huber; this is not a controlled speed benchmark, but the cost must be considered before promotion. January4 Pixel5 remains running. Record: `use_cases/records/gsdc2023_train_mi8_pixel4_tdcp_huber.json`.


### Conditional main-Doppler comparison on improved March10 TDCP recipe

Started a same-c779-executable replay adding only Phase213 final-graph Doppler to the completed March10 Pixel5 source metre-sigma plus Huber recipe. Baseline local score 0.5443656080013418 m, wall time 4159.587830543518 s, TDCP sigma 0.0016827902832903905 m, k=0.2 and 24,947 factors are frozen. The comparator requires those same TDCP diagnostics, inputs, initialization provenance and truth hashes, while verifying main Doppler insertion only in the candidate. Plan SHA256 a1fb42c3b618ea524d045fadf7626dd09f44da84b2f8ce9c115d9116a49f31b0. This tests the missing source final Doppler component conditional on the improved weighting; shared-load runtime is recorded without a controlled speed claim. Record: `use_cases/records/gsdc2023_train_pixel5_march10_huber_main_doppler.json`.


### A325F August4 same-input new-binary control reproduced

The first d73f779 A325F control replay completed and passed independent native-output audit. After normalizing only executable and output paths, all options and raw input entries match the original c779 fixed baseline. Final solution CSV and initialization metadata are byte-identical to that baseline (solution SHA256 756080b65086bb9e989a3f92f57ca203fcc124548bfe3bfdd081549f8e6239df); main Doppler remains disabled with zero factors. This verifies unchanged control behavior for this one drive despite the required admission-capable executable change. October6 control is running; candidate Doppler effects are still unmeasured. Record: `use_cases/records/gsdc2023_a325f_august04_binary_control_check.json`.


### March10 source-weighted main-Doppler result audited

Adding final-graph Doppler to the frozen source metre-sigma plus Huber Pixel5 recipe improved local score 0.5443656080013418 -> 0.5313629338581825 m, delta -0.013002674143159365 m (candidate P50 0.4825233107981316 / P95 0.5802025569182332). All 1,465 native raw UTC keys pass audit with no output coordinate interpolation/hold. The same executable, raw inputs, seed metadata, TDCP sigma/k/count, observable-quality configuration and evaluation truth are fixed. Candidate inserts 26,186 final-graph Doppler factors; baseline inserts zero, while both retain 26,186 GNSS-first Doppler factors.

Recorded wall time is 4159.587830543518 -> 570.9635047912598 seconds (about 69.3 -> 9.5 minutes). Shared concurrent workloads prevent a controlled speed claim, but the observed reduction and accuracy gain justify checking the same missing final-graph source component on other development routes. No uniform promotion, official improvement or full source-stage equivalence is claimed. Record: `use_cases/records/gsdc2023_train_pixel5_march10_huber_main_doppler.json`.


### January4 conditional main-Doppler replication started

Started the second Pixel5 development route with only final-graph Doppler added to the running January4 source metre-sigma plus Huber recipe. Plan SHA256 1e8433097bfe0079082994f8e07cb94ef8679ee4250a2171bdbb38bc66b45237. The comparator will require matched inputs/binary/seed metadata, source TDCP metre sigma 0.002798401423210426, 40,347 TDCP factors and native-context Huber k=0.2. The frozen native Street/source Highway mismatch remains explicitly separate. Baseline Huber-only inference is still running; the supervisor waits for its completed audit before candidate evaluation/comparison. No pending baseline score is assumed. Record: `use_cases/records/gsdc2023_train_pixel5_january04_huber_main_doppler.json`.


### Second A325F binary control reproduced

October6 A325F d73f779 control passed native output audit. Its solution CSV and seed metadata are byte-identical to the c779 baseline; final-graph Doppler remains off with zero factors. Both A325F controls now reproduce the frozen baseline outputs under the newer admission-capable executable. This removes an observed default-output difference as a confound for these two drives, while candidate Doppler effects remain unmeasured. Record: `use_cases/records/gsdc2023_a325f_october06_binary_control_check.json`.


### A325F matched final-Doppler comparison complete

Both same-d73f779-executable controls and candidates passed the matched native-output, input, seed, quality-configuration and truth checks. August4 local development score improved 2.251285461298199 -> 1.866516644469123 m (P50 1.4181577118931417 -> 1.2495323426482385; P95 3.0844132107032567 -> 2.4835009462900075). October6 improved 2.005371152308499 -> 1.9217048379274535 m, but its P95 regressed 2.725284585709374 -> 2.7728578405368163 despite P50 improving 1.2854577189076237 -> 1.0705518353180907. Final-graph Doppler factor counts were 0 -> 13,150 and 0 -> 12,235; GNSS-first counts and initialization hashes stayed fixed.

Recorded shared-load wall times were 609.003 -> 201.559 s and 738.296 -> 68.308 s; these are not controlled runtime benchmarks. The two exposed development groups support further testing of source final-graph Doppler, but the October tail regression rules out claiming improvement in every metric. Native Street/source Highway context remains unresolved. No global promotion or official submission is implied. Record: `use_cases/records/gsdc2023_train_a325f_main_doppler_matched.json`. January4 Huber control and May13 SM-G988B baseline remain live; January4 Doppler candidate has finished inference and awaits the matched control audit.


### Main-Doppler evidence across existing matched recipes

Six completed matched comparisons across different frozen recipes show five phone-score improvements and one regression (A205U +0.008872203302244586 m). P95 improves in four and regresses in two (MI8 and October6 A325F). Source-record and plan hashes were verified; this is a synthesis of existing audits, not a new replay or a pooled single-recipe score. No uniform adoption is supported. Evidence: `use_cases/records/gsdc2023_main_doppler_cross_recipe_evidence.json`.


### User instruction: Kaggle authentication UI

Open Kaggle authentication only when the user explicitly asks to open it. Goal continuations, pending submissions and missing authentication do not authorize opening or reopening the authentication browser. Continue local work independently.


### A325F source route metadata comparison started

Frozen a two-route same-d73f779 comparison conditional on enabled final-graph Doppler. Candidate adds only the original settings Type=Highway/L5=0 to the completed native Street controls; input settings rows were checked against settings_train.csv. Plan SHA256 f4f296384cabaa7b5d856c72ac9e7d9ffdd66d251aaaa075c188813fc858fc3a. This can change observation admission, weights and initialization, so seed equality is measured rather than assumed. Native inference started under supervisor 29824; independent validation and comparison follow sequentially. No source noise flags or truth-derived settings were added. Record: `use_cases/records/gsdc2023_train_a325f_doppler_source_route.json`.

May13 SM-G988B frozen control has now exited successfully (rc0, wall 16976.30044579506 s); the existing supervisor is auditing/scoring it before the paired initialization comparison. No accuracy result is inferred from process completion.


### May13 SM-G988B initializer comparison and frozen train evaluation complete

The long-running frozen control completed with local score 11.513059749518698 m (P50 2.435778236565316 / P95 20.59034126247208). Adding only source IMU initialization produces 1.2956856153497034 m (P50 0.8619685628371849 / P95 1.7294026678622219), a -10.217374134168995 m score change. Both inputs, executable and native output audits were independently rechecked: all 2,182 raw UTC states, no missing keys or coordinate interpolation/hold. Initial GNSS seed metadata hashes match. Recorded shared-load wall time is 16976.30044579506 -> 1059.8011481761932 seconds; no controlled runtime claim. This establishes a substantial initializer-dependent failure on this exposed drive; July14 SM-G988B previously showed a tiny regression, so improvement is not universal.

The frozen train recipe is now terminal: 36 scored, four preserved failures, no pending inference. Re-audited recovery coverage remains all 40 distinct drives across multiple recipes, with the May13 candidate now explicitly counted as having a scored baseline rather than a pending baseline. Full unified-recipe accuracy and taroz parity remain unproven. Records: `use_cases/records/gsdc2023_train_g988b_may13_source_init.json` and `use_cases/records/gsdc2023_train_recovery_coverage_summary.json`.


### SM-G988B source initialization transfer to existing test inputs

After both exposed train comparisons completed (May13 large improvement, July14 tiny regression), started the same-c779-binary source-initialization-only replay for both existing test SM-G988B drives: 2021-08-31 MTV-e and 2022-03-17 SJC-q. Plan SHA256 b8749283603e218d69827e764230e9d6b738ba0edc38bf49dac530aa1e660e97. Baseline record hashes are pinned; validation requires native raw/official-key coverage, identical inputs/binary/other options and initial GNSS summary/seed provenance. This measures transfer coverage, not test accuracy. No submission or new authentication is performed. Record: `use_cases/records/gsdc2023_test_g988b_source_init.json`.

The completed frozen train grouped summary has 29 planned groups, 26 fully scored groups, 36 scored drives and four failures. Mean of the 36 scored drives is 1.513340663226937 m; full-scope metrics remain null. This is a completed-subset diagnostic, not a 40-drive score.


### A325F source Type/L5 comparison: mixed accuracy

Both source Type=Highway/L5=0 candidates completed the same-binary native/input/output audit against final-Doppler-enabled Street controls. August4 score worsened 1.866516644469123 -> 1.9484165438592431 m (+0.0818998993901201), with both P50/P95 worse. October6 improved 1.9217048379274535 -> 1.8915033174035858 m (-0.03020152052386771), with P50 worse but P95 better (2.7728578405368163 -> 2.6135713038848305). Initial GNSS seed metadata is byte-identical in both pairs. Two-route mean changes by +0.025849189433126195 m, so source route metadata alone does not provide a uniform accuracy fix. Other source weighting and initialization differences remain; no automatic promotion. Shared-load wall times: 201.559 -> 167.191 s and 68.308 -> 45.087 s. Record: `use_cases/records/gsdc2023_train_a325f_doppler_source_route.json`.


### A325F source TDCP noise admission built

Pinned source uses the same previous-endpoint metre sigma for A325F bias-difference TDCP as the already admitted Pixel/Mi8 lane. Extended only the default-off source-sigma CLI phone admission to sm-a325f; no solver arithmetic or default recipe changed. Build succeeded and frozen executable SHA256 04a853b3af950229b3fa4aadadbfc48edbf2340cc72bc8b61a7b6cae5039b6f1. Fourteen runtime CLI admission/rejection checks passed, including A325F acceptance to deliberately missing input and continued rejection of drift-clock/unsupported phones and conflicting modes. This is admission evidence only; same-new-binary controls and source sigma/Huber candidates still need native replay before any accuracy conclusion. Record: `use_cases/records/gsdc2023_a325f_source_sigma_admission.json`.


### Matched A325F source sigma/Huber replay started

Started serial same-04a853b3 executable controls followed by paired source metre-sigma plus Huber candidates on both A325F train routes. Source Highway/L5=0 and main Doppler stay enabled in both arms. Control plan SHA256 c416d6e6ec8d8dfc19ba14ac7c043653ef5795e693c0d8df7661ce87dcfc9ea9; candidate plan d665b498a25c219aaf7f43262ec42435080ad0ac214433c4c1f27f77264a95ba. Comparator requires the new controls to reproduce prior d73f779 solution and seed bytes, as well as independent input/output audits, same pair options except source noise flags, fixed native/source route context, matched factor counts and evaluation truth. Supervisor 60288, initial native control PID 34924. No source-noise accuracy outcome yet. Record: `use_cases/records/gsdc2023_train_a325f_source_tdcp_noise.json`.


### SM-G988B test transfer audited and combined all40 candidate assembled

Both SM-G988B test source-initialization replays passed same-input/binary/options and independent native raw/official-key audits. August31: 1,141 raw/official keys. March17: 1,172 raw states and 1,171 official keys; the single extra native state is excluded only at official-key assembly. No missing keys or coordinate interpolation/hold. Both initial GNSS summaries and seed metadata match controls. Unlike the May13 train gain, these test coordinate changes are tiny: median 0.0011178563 m / 0.0005959272 m, maxima 0.0022290468 m / 0.0028459595 m. These are coordinate changes, not accuracy measurements.

Assembled a separate all40 candidate combining both A325G and both SM-G988B source initializations with the existing A205U correction. CSV `E:/rtklib_v2_ws_output/gsdc_native/all40_a205u_a325g_g988b_source_init_submission_v1/submission.csv`, SHA256 c318140ed183290e07101ce877860de8807f3c3ac8b636dfd6d62cdb215837ef, 71,936 native rows, 5,862,276 bytes. The other 36 drives / 66,340 rows are byte-identical to submission 56479759 (Public 1.698 / Private 1.333). Four drives / 5,596 rows changed; no sample coordinates or coordinate interpolation/hold. Plan SHA256 7a238eecb96b6a48f1e187e66f7d1ee26962c3706794235479a4c7a1bfb15154. Local assembly/review only; official accuracy unknown, not submitted. Prior candidates remain preserved. Explicit user submission instruction is required for this new artifact; do not reopen authentication automatically. Review: `use_cases/records/gsdc2023_all40_a325g_g988b_candidate_review.json`.


### A325F source-noise new binary controls verified

Both 04a853b3 controls independently passed native-output/input/executable audit and reproduced the previous d73f779 Highway/L5=0/main-Doppler solution CSV and seed metadata byte-for-byte (1,452 and 1,230 native states). The only normalized control differences are executable/output paths. This supports unchanged baseline behavior for these two drives after expanding the default-off phone admission. Source metre-sigma/Huber candidate inference remains running, with no accuracy result yet. Evidence: `use_cases/records/gsdc2023_a325f_source_noise_binary_controls.json`.


### Correction: A325F source explicitly disables TDCP

The previous source-sigma admission rationale was incorrect: fgo_gnss_imu.m:301 (also fgo_gnss.m:179) wraps all TDCP insertion in an exclusion of sm-a325f/samsunga32. The earlier inspection missed this outer guard. Native smartphone_temporal_recipe.hpp already matches it with CarrierClock::Disabled. Both experimental candidates have zero inserted TDCP factors, with 7,290/5,747 built-but-omitted pairs, and final coordinates/seed metadata exactly equal controls. Scores are unchanged at 1.9484165438592431 / 1.8915033174035858 m. This is an ineffective weighting experiment, not evidence for source-sigma accuracy.

The comparator correctly failed its positive-factor assertion and has not been weakened. Reverted only the two-line A325F CLI admission expansion; original source-disabled behavior remains. The experimental binary/plans/results remain for provenance, and the canonical build is being restored. Earlier sections describing A325F as an ordinary source TDCP phone are superseded by this correction. Future factor experiments must inspect enclosing phone guards and actual inserted-factor counts before launch. Record: `use_cases/records/gsdc2023_train_a325f_source_tdcp_noise.json`.

Restored canonical build completed successfully; all 14 CLI checks pass, including renewed rejection of the source-disabled A325F noise lane. Restoration evidence is attached to the admission record.


### Unified executable corrected train40 replay started

Frozen and launched all 40 existing train drives on d73f779 with a single fixed recipe plan, SHA256 67efef7c87fc883ffe56fb28974692f96a8f77ee7ed51a47fe1ad0d61554a7f3. Two native workers plus the existing January4 ablation remain below the four-job limit. The plan retains original fixed observable/route/noise settings, adds phone-wide source initialization for A325G/SM-G988B (MI8/A205U already enabled), and carries four explicit, provenance-pinned coverage repairs: Mi8 IMU-supported long gaps, the updated executable A205U GPS boundary correction, A217M leading states with bounded terminal IMU hold, and Pixel7 Pro common raw-drift cleanup. No candidate output is chosen per-drive by score; every planned drive is rerun on the same executable.

The existing coincident May16 route evidence was transferred only after confirming identical raw paths/hashes; evaluation uses 29 groups. This is a corrected full-train replay, not full taroz source parity or a heldout result. No extra final Doppler/source-noise ablations are promoted into it. Validation, per-phone/group metrics, native coverage and shared-load runtimes follow the frozen plan after completion; failures stay visible. Record: `use_cases/records/gsdc2023_train40_corrected_recipe_evaluation.json`.


### January4 Pixel5 Huber and conditional Doppler comparisons complete

Source metre-sigma-only score 0.3967265396421846 m worsens to 0.5131883274333603 with native-context Huber k=0.2 (+0.11646178779117572). Candidate P50/P95 are 0.4124292314760013 / 0.6139474233907193 m. The same c779 binary, inputs, seeds, 40,347 TDCP factors and representative sigma 0.002798401423210426 m remain fixed. Native Street versus source Highway context is still an explicit limitation.

Adding only final-graph Doppler to that sigma/Huber recipe improves 0.5131883274333603 -> 0.4680536296011435 m (-0.0451346978322168), with P50/P95 0.36074462556437675 / 0.5753626336379103. Inserts 19,529 final Doppler factors; GNSS-first counts and seed metadata stay fixed. Both 1,855 native raw-key outputs passed audit. Shared-load wall time: Huber 4850.518833637238 s versus Doppler candidate 832.4418766498566 s, not a controlled benchmark. The Doppler candidate remains worse than sigma-only 0.3967265396421846; no per-route best-arm promotion is made.

Cross-recipe Doppler summary now contains seven comparisons: six score improvements, one A205U regression; five P95 improvements, two regressions. All are exposed development inputs with differing fixed recipes, not one pooled evaluation. The corrected train40 replay retains its already frozen settings. Records: `use_cases/records/gsdc2023_train_pixel5_january04_tdcp_huber.json`, `use_cases/records/gsdc2023_train_pixel5_january04_huber_main_doppler.json`, `use_cases/records/gsdc2023_main_doppler_cross_recipe_evidence.json`.


### Corrected train40 first score and TDCP phone-guard inventory

The first corrected-recipe drive, Jan4 highway Mi8, passed all 2,002 native/truth keys with no coordinate interpolation/hold. Score 0.8417227921467705 m (P50 0.42588022043278184 / P95 1.2575653638607591), shared-load wall 476.76578545570374 s. Solution bytes match the old fixed baseline: True. Snapshot: one scored, two pending, 37 not started; no full-scope metric yet.

Added an inventory deriving TDCP-disabled/integrated-drift phone sets from the enclosing pinned source guards and checking actual baseline summary hashes and inserted-factor diagnostics. All 37 existing final graph summaries match the source phone classification; the remaining three lack a final graph. One of those 37 is the A217M output with failed full-key coverage, explicitly retained as failed. Across planned drives: 34 bias-difference, four integrated-drift, two intentionally disabled. This checks family/admission only, not source observation values or weights. Future weighting experiments must require nonzero inserted factors, not merely built candidates. Record: `use_cases/records/gsdc2023_tdcp_phone_guard_inventory.json`.


### Corrected-versus-fixed evaluation snapshots

Added `E:/rtklib_v2_ws_tmp/compare_gsdc_train40_corrected_vs_fixed.py`, using pinned plan/validation snapshots, unchanged raw input entries and the verified 29-group mapping. It checks native-score/executable/metric contracts through the existing summarizer, verifies scored corrected solution hashes, and requires identical truth hashes and scored key counts for paired deltas. Per-drive/phone/group deltas remain null when either arm lacks a valid score; recovered failed baselines are classified separately. Current snapshot has one paired drive with zero score/P50/P95 change, 39 not yet scored, and no full-scope delta. This is a multi-change corrected-recipe comparison, not single-factor attribution. Record: `use_cases/records/gsdc2023_train40_corrected_vs_fixed_snapshot.json`.


### Corrected train40: first failed-baseline recovery reproduced

January4 MTV-a Mi8 completed under the unified d73f779 recipe and passed all 1,856 raw/truth keys with no coordinate interpolation/hold. Local score 0.5888759370502147 m reproduces the prior isolated long-gap recovery. The original frozen baseline failed initialization, so its paired accuracy delta remains null. Shared-load wall time 1130.8010025024414 s; temporal seed filling is explicitly retained in coverage metadata. Corrected snapshot: two scored, two pending, 36 not started. The matched snapshot now distinguishes one comparable baseline drive and one recovered failure. Full-scope metrics remain unavailable.


### Corrected train40 third score

January4 MTV-a Pixel5 passed 1,855 native raw keys and 1,854 truth keys (one extra native state excluded only from accuracy scoring). Score 0.5945094662429546 m reproduces the frozen baseline; solution bytes identical: True. No coordinate interpolation/hold or missing raw keys. Shared-load wall 574.1757469177246 s. Corrected snapshot: three scored, two pending, 35 not started; matched comparison includes two baseline-success pairs with zero score delta and the one recovered Mi8 failure. Full-scope metrics remain null.


### Corrected train40 fourth score

January5 MTV-d Mi8 passed all 1,302 native/truth keys with no missing keys or coordinate interpolation/hold. Local score 0.7667595928133726 m reproduces the original fixed baseline; solution bytes identical: True. Shared-load wall 233.89175391197205 s. Corrected snapshot: four scored, two pending, 34 not started. Three baseline-success pairs have zero score change, and one original failed drive is recovered. Full-scope metrics remain unavailable.


### Corrected train40 fifth score

March10 Pixel5 passed all 1,465 native raw keys and 1,464 truth keys, with one extra native state excluded only from scoring. Local score 0.6779106294218623 m; solution bytes identical to frozen baseline: True. Missing raw keys and output interpolation/hold remain zero. Shared-load wall 257.306941986084 s. Corrected snapshot: five scored, two pending, 33 not started. Four comparable baseline-success pairs have zero score change; one failed baseline drive is recovered. Full-scope metrics remain null.


### Corrected train40 sixth score

January4 highway Pixel5 completed successfully after 2753.2983021736145 s of shared-load wall time. All 2,002 raw keys are native optimized states, with no missing keys or output interpolation/hold; 2,001 truth keys are scored and the extra native key is excluded only from scoring. Score 0.674903312900923 m (P50 0.4457882970396253 / P95 0.9040183287622207) and solution bytes exactly reproduce the frozen baseline. The audited snapshot has six scored drives, two pending, and 32 not started. Five baseline-success pairs have zero score/P50/P95 delta; one original failed drive is recovered. Full-scope metrics remain null. Live successor workers were verified as PID 39260 (March16 Pixel5) and PID 32688 (March16 Pixel4XL); the completed January4 process was not restarted.


### Corrected train40 seventh score

March16 MTV-a Pixel5 passed all 2159 raw/truth keys with no missing keys or output coordinate interpolation/hold. Local metrics: {"distance_variant": "haversine_sphere", "p50_m": 0.4469443117089434, "p95_m": 0.8667763538645277, "percentile_variant": "linear_n_minus_1", "phone_score_m": 0.6568603327867355}. Solution bytes identical to the frozen baseline: True. Shared-load wall time 519.8334848880768 s. Audited snapshot: seven scored, two pending, 31 not started. Six comparable baseline-success pairs have zero score/P50/P95 delta, and one failed baseline drive is recovered. Full-scope accuracy remains unavailable.


### Corrected train40 eighth score

March16 MTV-b Pixel4XL passed 1476 native raw keys and 1461 truth keys; extra native keys are excluded only from accuracy scoring. No missing raw keys or output coordinate interpolation/hold. Metrics: {"distance_variant": "haversine_sphere", "p50_m": 0.3991552487567759, "p95_m": 0.6112428879487922, "percentile_variant": "linear_n_minus_1", "phone_score_m": 0.505199068352784}. Solution bytes identical to frozen baseline: True. Shared-load wall time 515.8428320884705 s. Snapshot: eight scored, two pending, 30 not started; seven baseline-success pairs have zero score/P50/P95 delta, plus one recovered failed baseline. Full-scope metrics remain null.


### Corrected train40 ninth score

2021-07-14-20-50-us-ca-mtv-e/pixel4 passed 1188 native raw keys and 1188 truth keys, with no missing raw keys or output coordinate interpolation/hold. Metrics: {"distance_variant": "haversine_sphere", "p50_m": 0.25403256719963185, "p95_m": 0.6789319762677022, "percentile_variant": "linear_n_minus_1", "phone_score_m": 0.466482271733667}. Solution bytes identical to frozen baseline: True. Shared-load wall time 212.92619442939758 s. Snapshot: nine scored, two pending, 29 not started. Eight baseline-success pairs have zero score/P50/P95 delta, plus one recovered failed baseline. Full-scope metrics remain null.


### Corrected train40 tenth score: G988B initialization policy

2021-07-14-20-50-us-ca-mtv-e/sm-g988b passed all 1165 raw/truth keys with no missing keys or output coordinate interpolation/hold. Metrics: {"distance_variant": "haversine_sphere", "p50_m": 0.4546566128864969, "p95_m": 0.7975510850403802, "percentile_variant": "linear_n_minus_1", "phone_score_m": 0.6261038489634385}. Baseline score 0.6259142594821209 m; delta 0.00018958948131764242 m (slight regression). This reproduces the previously observed source-initialization result; it is retained under the frozen phone-wide policy, without per-drive selection. Shared-load wall time 352.9058258533478 s. Snapshot: ten scored, two pending, 28 not started. Nine valid baseline pairs and one recovered baseline failure; full-scope metrics remain null.


### Corrected train40 eleventh score

2021-07-19-20-49-us-ca-mtv-a/pixel5 passed 1897 native raw keys and 1896 truth keys; the extra native key is excluded only from scoring. No missing raw keys or output coordinate interpolation/hold. Metrics: {"distance_variant": "haversine_sphere", "p50_m": 0.37486338498497396, "p95_m": 0.6253355530040343, "percentile_variant": "linear_n_minus_1", "phone_score_m": 0.5000994689945042}. Solution bytes identical to frozen baseline: True. Shared-load wall time 638.3059351444244 s. Snapshot: eleven scored, two pending, 27 not started. Ten valid baseline pairs and one recovered baseline failure; full-scope metrics remain null.


### Corrected train40 twelfth score

2021-07-27-19-49-us-ca-mtv-b/pixel4 passed 1678 native raw keys and 1677 truth keys; the extra native key is excluded only from scoring. No missing raw keys or output coordinate interpolation/hold. Metrics: {"distance_variant": "haversine_sphere", "p50_m": 0.6889283872923793, "p95_m": 1.37890546049602, "percentile_variant": "linear_n_minus_1", "phone_score_m": 1.0339169238941996}. Solution bytes identical to frozen baseline: True. Shared-load wall time 860.5094833374023 s. Snapshot: twelve scored, two pending, 26 not started. Eleven valid baseline pairs and one recovered baseline failure; full-scope metrics remain null.


### Corrected train40 thirteenth score

2022-01-26-20-02-us-ca-mtv-pe1/mi8 passed 1699 native raw keys and 1699 truth keys. No missing raw keys or output coordinate interpolation/hold. Metrics: {"distance_variant": "haversine_sphere", "p50_m": 0.4607219498826623, "p95_m": 0.9521153305916731, "percentile_variant": "linear_n_minus_1", "phone_score_m": 0.7064186402371677}. Solution bytes identical to frozen baseline: True. Shared-load wall time 518.4517805576324 s. Snapshot: thirteen scored, two pending, 25 not started. Twelve valid baseline pairs and one recovered baseline failure; full-scope metrics remain null.


### Authorized A325G/G988B candidate official submission 56491795

The user explicitly authorized the exact c318140ed183290e candidate. Saved credentials initially returned HTTP 401; a noninteractive refresh succeeded without opening a browser. Kaggle accepted submission 56491795 and reported COMPLETE, Public 1.396 m / Private 1.333 m. Compared with submission 56479759 (1.698 / 1.333), Public improved by 0.302 m and displayed Private was unchanged. This remains above the taroz reference targets 0.789 / 0.928; the goal is not achieved. The immutable candidate manifest describes pre-submission assembly; the separate receipt and updated review record the actual submission and scores. No resubmission is needed.


### Corrected train40 fourteenth score

2022-01-26-20-02-us-ca-mtv-pe1/pixel5 passed 1698 native raw keys and 1698 truth keys. No missing raw keys or output coordinate interpolation/hold. Metrics: {"distance_variant": "haversine_sphere", "p50_m": 0.2909813315305927, "p95_m": 0.5160591130004955, "percentile_variant": "linear_n_minus_1", "phone_score_m": 0.40352022226554407}. Solution bytes identical to frozen baseline: True. Shared-load wall time 668.657684803009 s. Snapshot: fourteen scored, two pending, 24 not started. Thirteen valid baseline pairs and one recovered baseline failure; full-scope metrics remain null.


### Corrected train40 fifteenth score

2022-02-24-18-29-us-ca-lax-o/pixel5 passed 2439 native raw keys and 2438 truth keys; the extra native key is excluded only from scoring. No missing raw keys or output coordinate interpolation/hold. Metrics: {"distance_variant": "haversine_sphere", "p50_m": 1.2343322915310424, "p95_m": 2.3451705407387626, "percentile_variant": "linear_n_minus_1", "phone_score_m": 1.7897514161349024}. Solution bytes identical to frozen baseline: True. Shared-load wall time 2367.5849022865295 s. Snapshot: fifteen scored, two pending, 23 not started. Fourteen valid baseline pairs and one recovered baseline failure; full-scope metrics remain null.


### Corrected train40 sixteenth score

2021-08-24-20-32-us-ca-mtv-h/pixel5 passed 3140 native raw keys and 3139 truth keys; the extra native key is excluded only from scoring. No missing raw keys or output coordinate interpolation/hold. Metrics: {"distance_variant": "haversine_sphere", "p50_m": 0.6134112587863061, "p95_m": 1.0243140169851732, "percentile_variant": "linear_n_minus_1", "phone_score_m": 0.8188626378857397}. Solution bytes identical to frozen baseline: True. Shared-load wall time 4317.110270738602 s. Snapshot: sixteen scored, two pending, 22 not started. Fifteen valid baseline pairs and one recovered baseline failure; full-scope metrics remain null.


### Corrected train40 seventeenth score

2022-04-01-18-22-us-ca-lax-t/pixel5 passed 1466 native raw keys and 1465 truth keys; the extra native key is excluded only from scoring. No missing raw keys or output coordinate interpolation/hold. Metrics: {"distance_variant": "haversine_sphere", "p50_m": 0.606412371614075, "p95_m": 0.9482382911808503, "percentile_variant": "linear_n_minus_1", "phone_score_m": 0.7773253313974626}. Solution bytes identical to frozen baseline: True. Shared-load wall time 476.80806374549866 s. Snapshot: seventeen scored, two pending, 21 not started. Sixteen valid baseline pairs and one recovered baseline failure; full-scope metrics remain null.


### Corrected train40 eighteenth score

2022-05-13-20-57-us-ca-mtv-pe1/pixel6pro passed 2180 native raw keys and 2162 truth keys; extra native keys are excluded only from scoring. No missing raw keys or output coordinate interpolation/hold. Metrics: {"distance_variant": "haversine_sphere", "p50_m": 0.7711075537254555, "p95_m": 1.1197943497239309, "percentile_variant": "linear_n_minus_1", "phone_score_m": 0.9454509517246932}. Solution bytes identical to frozen baseline: True. Shared-load wall time 814.3073453903198 s. Snapshot: eighteen scored, two pending, 20 not started. Seventeen valid baseline pairs and one recovered baseline failure; full-scope metrics remain null.


### Corrected train40: G988B and A325G large-error recoveries reproduced

Both phone-wide source-initialization corrections reproduce their isolated development results under the unified d73f779 executable and frozen train40 recipe. All raw keys pass native-state provenance with zero missing raw keys or output interpolation/hold. Extra native keys are excluded only from truth scoring. Detailed paired metrics and shared-load wall times:

[
  {
    "case": "2022-05-13-20-57-us-ca-mtv-pe1/sm-g988b",
    "raw": 2182,
    "truth": 2163,
    "score": {
      "distance_variant": "haversine_sphere",
      "p50_m": 0.8619685628371849,
      "p95_m": 1.7294026678622219,
      "percentile_variant": "linear_n_minus_1",
      "phone_score_m": 1.2956856153497034
    },
    "baseline": {
      "distance_variant": "haversine_sphere",
      "p50_m": 2.435778236565316,
      "p95_m": 20.59034126247208,
      "percentile_variant": "linear_n_minus_1",
      "phone_score_m": 11.513059749518698
    },
    "wall_s": 891.2216346263885
  },
  {
    "case": "2022-07-26-21-01-us-ca-sjc-s/samsunga325g",
    "raw": 1512,
    "truth": 1487,
    "score": {
      "distance_variant": "haversine_sphere",
      "p50_m": 0.9741154803332509,
      "p95_m": 1.6614286476795936,
      "percentile_variant": "linear_n_minus_1",
      "phone_score_m": 1.3177720640064223
    },
    "baseline": {
      "distance_variant": "haversine_sphere",
      "p50_m": 2.903674679428252,
      "p95_m": 20.9552841224887,
      "percentile_variant": "linear_n_minus_1",
      "phone_score_m": 11.929479400958476
    },
    "wall_s": 330.14089345932007
  }
]

Snapshot: twenty scored, two pending, eighteen not started. Nineteen valid baseline pairs and one recovered failed baseline; full-scope metrics remain null. These exposed development improvements do not establish official parity.

### Corrected train40: August 4 Pixel 5 reproduced

The audited snapshot now contains 21 scored drives, two pending and 17 not started. Slot 21 (`2022-08-04-20-07-us-ca-sjc-q/pixel5`) completed in 299.62489652633667 seconds: 1,450 native raw epochs, 1,431 truth keys, zero missing raw keys and zero output interpolation/hold. Its solution is byte-identical to the old fixed recipe. Local development score is 0.6853095941541939 m (P50 0.3827932180015632 m; P95 0.9878259703068246 m). The paired snapshot has 20 valid baseline pairs and one recovered baseline failure; full-scope scores remain null. Existing exposed training inputs are not heldout data. The unified replay continues under the frozen plan.

### Corrected train40: A205u boundary recovery reproduced

Slot 23 (`2022-10-06-21-51-us-ca-mtv-n/sm-a205u`) now completes under the unified d73 executable and frozen corrected plan. The native output audit verifies all 1,213 raw/truth keys with zero missing keys and zero output interpolation/hold; four temporal initial guesses remain explicitly counted. The local development score is 1.916556366229106 m (P50 1.3181052110818825 m; P95 2.51500752137633 m), reproducing the isolated boundary-repair result. Wall time is 232.59338760375977 seconds. The old fixed recipe failed on this input, so its accuracy delta remains null. The snapshot contains 23 scored drives, including two recovered baseline failures; all-40 metrics remain unavailable. Slot 20 Mi8 also reproduced its old solution byte-for-byte (0.9322746384381055 m, 1,417 raw keys, 1,400 truth keys, no missing/interpolated output).

### Corrected train40: A217M boundary coverage and A325F control

The audited snapshot reaches 25 scored drives, two pending and 13 not started. Slot 24 A217M recovers the two formerly missing leading keys under the unified binary: all 1,222 raw keys are native outputs, 1,221 truth keys are scored, no keys are missing and no coordinates are interpolated/held. Two temporal initial guesses are recorded. Local score 2.019059794920038 m (P50 1.4209790871168606 m, P95 2.617140502723216 m) reproduces the isolated bounded-tail repair; wall time 179.47722148895264 seconds. Its failed old baseline retains a null accuracy delta. Slot 22 August A325F reproduces its baseline solution byte-for-byte: score 2.251285461298199 m, P50 1.4181577118931417 m, P95 3.0844132107032567 m; 1,452 raw keys and 1,434 truth keys, zero missing/interpolated output, wall time 596.3474853038788 seconds. The paired snapshot now has 22 valid pairs and three recovered old failures. Full-scope evaluation remains incomplete.

### Corrected train40: 35-drive checkpoint and expanded sigma experiment

The latest audited snapshot contains 35 scored drives, two pending and three not started. All completed outputs have zero missing raw keys and zero coordinate interpolation/hold. Slots 25 through 34 each reproduce their old fixed-recipe solution byte-for-byte; the recovered old failures remain explicitly separate. The May 16 Pixel5/Pixel7Pro pair is evaluated as one coincident route group, mean local phone score 1.0475214888028588 m. Full-scope accuracy remains null until all forty finish. Exact P50/P95, coverage and runtimes are retained in `gsdc2023_train40_corrected_recipe_evaluation.json`, its referenced validation/summary, and `gsdc2023_train40_corrected_vs_fixed_snapshot.json`.

Static source/CLI scope audit (`gsdc2023_train40_source_sigma_admission_scope.json`) found 23 drives admitted to the source metre TDCP sigma experiment, 11 further bias-difference drives not yet admitted, four integrated-drift drives requiring separate validation, and two A325F drives intentionally disabling TDCP. The initial audit remains historical evidence of the pre-extension code hash.

The explicit sigma option now also admits sm-g988b, pixel6pro, pixel7pro and sm-s908b. Defaults and solver/noise formulas are unchanged; integrated-drift and TDCP-disabled phones remain rejected. Native build succeeded with the existing GTSAM duplicate-symbol linker warnings. Twenty-six repository CLI cases and twenty-two full-recipe CLI checks passed. The initial pytest invocation failed in an unrelated auto-loaded xonsh console plugin; disabling plugin auto-loading allowed the actual tests to run. These are admission checks, not native accuracy evidence.

Frozen experimental executable: `source_bias_difference_sigma_admission_fixed.exe`, SHA-256 `359ab743913f678e143cbf7f014bd18018a4c0b2f30088122f1f51060b5c0899`. Evidence is in `gsdc2023_bias_difference_sigma_admission.json`. The ongoing corrected train40 uses its existing d73 executable and frozen plan.

Prepared `train_expanded_bias_tdcp_sigma`: all eleven existing train drives of those four phones, eleven controls plus eleven sigma-only candidates using the same new executable. Control plan SHA-256 `9d2b82e35d68bc6518f81c38d67ed3d4a03f334efa813783efb0b044d7facbea`; candidate plan SHA-256 `c8d22ed873da10b0d906e80bd018fe5b5e15d75bd3d4ae4112a43ed2a8553218`. Each pair has identical arguments except the sigma flag and output paths. Route/Huber/Doppler settings remain fixed to isolate sigma. The prepared watcher requires audited completion of the parent all40 replay, then runs controls, verifies byte-identical solution/seed reproduction, runs candidates and checks positive actually inserted bias-difference TDCP factors, weights, complete native coverage, P50/P95 and runtime. It is not started yet. All inputs are exposed development data; no promotion or external submission is authorized by these experiments.

### Corrected train40: all four old coverage failures recovered

Slot 35 (`2023-05-25-20-11-us-ca-sjc-he2/pixel7pro`) passed native output and independent raw-clock cleanup audits under the unified d73 executable. All 1,259 raw keys are output, 1,258 truth keys are scored, and coordinate interpolation/hold and missing keys are zero. The clock audit reconstructs all 1,259 exported values from the original CSV and verifies five filled clock-drift values with zero jump masks. These are clock measurement repairs, not interpolated position outputs. Local score is 0.7998717050807369 m (P50 0.7145458230395427 m; P95 0.8851975871219311 m), and solution bytes match the previously audited isolated repair. Wall time is 1069.36567902565 seconds.

The snapshot now has 37 scored drives, two pending and one not started. All four failed old-baseline inputs have valid corrected native outputs and scores; their old accuracy deltas remain null. There are 33 valid baseline pairs. This proves recovery of those four inputs, not all40 completion or taroz accuracy parity. Slot 36 September 5 Pixel5 also reproduced the old solution exactly (0.7583777376274515 m; 1,564 raw/truth keys; no missing/interpolated output).

### Corrected train40 terminal audit: all forty native and scored

All forty runs completed and passed the fixed audit, grouped into 29 coincident-route evaluation groups. There are 65,702 native raw-key outputs and 65,520 scored truth keys; the 182 extra native keys are retained in raw output and excluded only from truth scoring. Missing raw keys and coordinate interpolation/hold are zero. Eighty-three temporal initial guesses remain counted. Mean local drive score is 0.9743933949452525 m; mean per-drive P50 0.6741798630000844 m and P95 1.2746069268904205 m. Mean route-group score is 0.933090889324813 m. Summed per-run wall time is 31,295.490482091904 seconds, not elapsed batch time because two workers overlap.

The last Pixel7Pro output reproduces the old fixed output exactly (1.0551524261827625 m). All four formerly failed inputs recover; the comparison has 36 valid pairs and four null baseline deltas. Paired-subset mean score delta is -0.5785803300455481 m and must not be reported as an all40 baseline delta. Audit evidence and hashes are in `gsdc2023_train40_corrected_recipe_completion_audit.json`. These are exposed development data using the local sphere/linear-percentile metric, not official score or heldout evidence. The official test candidate remains Public 1.396 m / Private 1.333 m, below the requested target performance; the overall goal is still unachieved.

Started `watch_gsdc_train_expanded_bias_tdcp_sigma.py` after the parent final audit. Its first stage runs all eleven same-binary controls before candidate scoring; source-sigma candidate results are not yet available. No external submission is part of this pipeline.

### Corrected train40: remaining route-setting differences

The completed forty-run summaries were checked against their recorded hashes and the original archive's `settings_train.csv` (SHA-256 `3e6ae65388b2809088b16732b87744e673f860c24a1fe0f709ef903a87397f39`). All frozen plan settings match those source rows. Actual native environment labels differ on 26 drives: 25 source Highway drives use Street, and one source Street drive uses Highway. Four Highway and ten Street labels match. Explicit source type/L5 selection is disabled, so even matching labels do not prove factor/noise parity. This metadata audit does not establish that changing settings improves accuracy.

Six of the eleven drives in the active expanded sigma experiment have these environment-label differences. Its frozen controls and candidates retain identical parent route settings to isolate sigma; the experiment cannot establish full source-setting parity. The remaining route-setting gap requires a separate fixed comparison after the sigma results, without per-drive best-result selection. Evidence: `records/gsdc2023_corrected_train40_route_settings_audit.json`. No active plan, submitted artifact or solver configuration was changed by this audit.

The frozen 359ab743 executable also passed forty CLI admission checks using each corrected parent recipe plus the archive Type/L5 metadata. Each case terminated at an intentionally missing GNSS input with no output written, proving argument compatibility only, not native execution or accuracy. Exact arguments and diagnostics are retained in `E:/rtklib_v2_ws_output/gsdc_native/train40_explicit_route_admission_cli_v1/report.json`; the checker is `E:/rtklib_v2_ws_tmp/test_gsdc_train40_explicit_route_admission.py`.

The expanded sigma control audit has verified three of eleven drives so far (July G988B, May Pixel6 Pro and November Pixel7 Pro). Each passed native output provenance and reproduced both parent solution and initialization bytes. Detailed partial evidence is in `records/gsdc2023_train_expanded_bias_tdcp_sigma_partial_control_check.json`. Candidate sigma accuracy remains unmeasured.

Prepared the separate all40 explicit Type/L5 comparison, not started. Plan `train40_explicit_source_route_plan.json` SHA-256 `91962fc37b1fb925b23f02fbed545003e83c3c6c6dcde89e053ce1e087256f97` reuses the corrected parent's d73 executable, all inputs and all inference arguments except adding source Type/L5 and changing output paths. No sigma/Phase184 change or per-drive selection is included. Evaluation groups use the existing conservative May16 grouping, totaling 29. Forty admission checks also passed on this exact d73 executable; these stop at deliberately missing raw input and do not prove accuracy. Runner and validator scripts compile; the comparison must be reviewed and run after the ongoing expanded sigma pipeline is terminal. Evidence: `records/gsdc2023_train40_explicit_source_route_evaluation.json`. The prepared experiment changes several source-mapped thresholds together, so its future score difference must not be attributed solely to the environment label.

### Expanded sigma comparison: all eleven controls verified

All eleven controls completed and passed native output auditing plus the full comparator. The new 359ab743 executable reproduces every corrected-parent solution and initialization file byte-for-byte with sigma disabled. There are 17,477 native raw keys and 17,399 scored truth keys, zero missing raw keys and zero interpolated/held output coordinates. Mean local drive score for this eleven-drive scope is 0.8807192620178949 m, not an all40 or official score. Evidence: `records/gsdc2023_train_expanded_bias_tdcp_sigma_control_check.json` and the control validation record. Earlier partial checkpoints remain historical.

The existing watcher advanced to `run_candidate`, beginning the eleven sigma-only native runs under the frozen candidate plan. G988B July and Pixel6 Pro May processes are live. Candidate accuracy is still unverified; no promotion or external submission occurred. The overall goal remains unachieved.

### Expanded sigma: first audited candidate regresses slightly

July14 G988B passes native output auditing and the strict matched comparison: inputs, truth keys, initialization, route settings, Huber and 22,408 inserted TDCP factors are unchanged. Sigma-only local score changes from 0.6261038489634385 to 0.6525597560701942 m (+0.026455907106755716 m). P50 changes from 0.4546566128864969 to 0.4716991610975178 m; P95 from 0.7975510850403802 to 0.8334203510428706 m. Representative sigma is 0.001153494181723202 m instead of fixed 0.03 m; this is not a constant sigma for all factors. Shared-load wall time is 910.603296995163 s versus control 497.54633355140686 s, not a controlled timing benchmark.

Only one of eleven candidate comparisons is complete. The slight regression is retained and the remaining fixed runs continue; there is no per-drive selection or promotion. Evidence: `records/gsdc2023_train_expanded_bias_tdcp_sigma_partial_comparison.json`. Full-scope accuracy remains unverified.

### Expanded bias TDCP sigma: second audited candidate (2026-09-24)

The May 13 Pixel 6 Pro candidate completed successfully and passed the same strict paired provenance checks as the first candidate. The local development score decreased from 0.945450951725 m to 0.647182111590 m (delta -0.298268840135 m); candidate P50/P95 are 0.493387880563/0.800976342616 m. All 2,180 raw keys remain native, with 2,162 truth keys scored. Seed metadata, inputs, route configuration, Huber threshold, and 33,122 inserted TDCP factors match the control; only the requested TDCP sigma policy changes. Shared-load wall time was 2583.447 s versus control 708.885 s, not a controlled speed benchmark.

The partial matched comparison now covers 2/11 candidates: the July SM-G988B case regressed by 0.026455907107 m and this Pixel 6 Pro case improved. The completed-subset mean delta is -0.135906466514 m, while the full eleven-drive mean remains null. No promotion, heldout claim, official score claim, or new submission follows from this checkpoint. Evidence: `docs/use_cases/records/gsdc2023_train_expanded_bias_tdcp_sigma_partial_comparison.json`.

Before starting the separate explicit Type/L5 replay, corrected its temporary validator to verify grouping evidence against the pinned parent plan, then require the candidate plan's embedded 40-case/29-group assignments to match. The summary now uses those embedded assignments instead of passing a parent-only grouping audit as candidate evidence. Compilation, plan-only comparison, and execution of the actual grouping validation block passed; numerical execution remains pending completion and review of the sigma experiment.

Prepared `E:/rtklib_v2_ws_tmp/watch_gsdc_train40_explicit_source_route.py` for an explicit later launch. It rejects launch until the eleven-drive sigma pipeline is audited-and-compared and all eleven validations are scored; it verifies the frozen route comparison plan, reserves the supervisor state exclusively, then sequences native execution, validation, and comparison with separate logs. Failures preserve the failed stage without retries. Compilation passed, and invoking it during the current sigma run correctly rejected launch before creating either the output directory or pipeline state. The accepted launch path is not yet exercised; no new native batch was started.

### Expanded bias TDCP sigma: all eleven candidates compared — not promoted (2026-09-24)

All eleven sigma-only candidates completed, passed native output auditing and the strict matched comparison (`records/gsdc2023_train_expanded_bias_tdcp_sigma_comparison.json`, status `all-eleven-matched-sigma-comparisons-complete`). Paired mean delta: phone score -0.0017 m, P50 -0.0895 m, P95 +0.0862 m. Per-drive deltas span -0.474 m (May13 SM-G988B) to +0.414 m (May24 Pixel7 Pro); 5 improve, 6 regress. The policy tightens the median but widens the tail, so it is neutral on the phone score and is not promoted. No per-drive selection, heldout claim, official score or submission follows.

Launched the prepared explicit Type/L5 source-route replay (`watch_gsdc_train40_explicit_source_route.py`, supervisor PID 53224, 40 runs, 2 workers) now that its gate (`audited-and-compared`) is satisfied. The overall goal remains unachieved: latest official score is Public 1.396 m / Private 1.333 m versus the taroz reference Public 0.789 m / Private 0.928 m.

### Stage comparison against archived taroz outputs: missing phone offset found (2026-09-24)

Scored the archived taroz `result_gnss.mat` / `result_gnss_imu.mat` (dataset_2023.zip, SHA `bda30ab4...`) against the eleven expanded-sigma control outputs on common truth keys. Mean P50/P95 score: taroz GNSS-only 0.695 m, taroz GNSS+IMU 0.617 m, native final 0.881 m. The archived results are diagnostic references only; generation settings remain unproven and nothing from them enters native inference.

Error decomposition in each drive's along/cross-track frame shows native with a speed-independent forward bias of +0.4 to +0.6 m on 8/11 drives, versus about +0.1 m for taroz. A regression against speed has near-zero slope, so this is not a timestamp offset. Every control run and every submitted all40 test run has `native_upstream_position_offset: false`, whereas upstream `fgo_gnss.m` and `fgo_gnss_imu.m` always call `add_position_offset`. A truth-free post-hoc simulation using the pinned per-phone offsets with trajectory heading changes the eleven-drive mean from 0.881 to 0.771 m (9/11 improve). The reversed sign worsens it to 1.094 m, confirming orientation.

After the simulated offset, the 31-epoch moving-average split shows equal high-frequency error (native about 0.06-0.15 m vs taroz 0.05-0.15 m). The remaining gap is low-frequency wander (for example SM-S908B 0.78 vs 0.46 m, May13 SM-G988B 0.80 vs 0.56 m), which points to GNSS-stage absolute pseudorange modeling rather than IMU smoothing.

Launched a same-binary paired rerun of the eleven controls that adds only `--native-upstream-position-offset` (`E:/rtklib_v2_ws_tmp/run_gsdc_train11_upstream_offset.py` → `train11_upstream_offset_v1`; scorer `compare_gsdc_train11_upstream_offset.py`). Development drives previously exposed; no heldout or official claim.

GNSS-stage export (`--native-export-gnss-stage`, plus offset, diagnostic only; `train4_gnss_stage_export_v1`) for May13 Pixel6 Pro: native GNSS-first score 0.969 m without offset / 0.809 m with the heading-based offset, versus archived taroz `result_gnss.mat` 0.548 m. The gap is therefore already present before IMU. Minute-binned errors show the same sign pattern as taroz but amplified and persistent for several minutes, with step changes (for example minutes 4-15: native mean EN (-0.6, +0.45) m vs taroz (-0.25, +0.05) m). This is consistent with a biased satellite/signal subset or weighting retained natively. Next: per-satellite residual comparison over those windows.

May13 SM-G988B GNSS stage: native 1.327 m (no offset) / 1.024 m (offset) vs taroz 0.771 m. The low-pass native−taroz mean difference is E -0.13 / N +0.20 m, matching the same-trip Pixel6 Pro (E -0.12 / N +0.21 m). A trip-common displacement shared by two different phones implicates route-level inputs, most likely the base pseudorange correction. Base station and year-specific coordinates match upstream (`SLAC` 2022; upstream `base_offset.csv` is added only after residuals and is therefore inert upstream). Next: reproduce upstream `correct_pseudorange` (satellite-wise base residual, `movmean` 151 at 1 s / 11 at 15 s, linear interpolation to rover time) and diff it against native per-satellite corrections for this route.

### Upstream phone position offset: eleven-drive paired result (2026-09-24)

The same-binary rerun adding only `--native-upstream-position-offset` completed 11/11 with return code 0. Mean phone score changes from 0.880719 to 0.775571 m (-0.105148 m), with 9/11 improving. The two regressions are small Pixel7 Pro changes (+0.021 m and +0.081 m); the largest gain is May13 SM-G988B (-0.284 m). The result matches the truth-free heading simulation (0.771 m). This restores a pinned upstream post-processing step absent from every submitted native run, and the per-phone constants are the upstream table, not tuned values. Development drives were previously exposed, so no heldout claim is made. Evidence: `records/gsdc2023_train11_upstream_offset_comparison.json`.

GNSS-stage diagnostic (offset applied, vs archived taroz `result_gnss.mat`): Pixel6 Pro May13 0.809 vs 0.548; SM-G988B May13 1.024 vs 0.771; SM-S908B 1.174 vs 0.743; Pixel7 Pro he2 0.732 vs 0.488 m. Most of the remaining gap to taroz is present before IMU, as low-frequency wander. May13 shares a trip-common displacement across phones; S908B shows time-varying differences with a larger height bias (+1.98 vs +1.02 m).

### Test candidate: submitted artifact plus post-hoc upstream phone offset (2026-09-24)

A same-recipe test rerun with the flag was attempted and immediately rejected (return code 2 on 7 drives, no outputs): the submitted binary `leading_count_phone_noise_fixed.exe` predates Phase171 offset admission. The runner was stopped. Instead, `scripts/analysis/apply_gsdc_phone_offset.py` applies the pinned upstream per-phone constants using heading derived from each drive's own native trajectory (no truth, WLS or external coordinates). On the eleven train controls it gives 0.7692 m, versus 0.7756 m for the in-solver flag and 0.8807 m for no offset.

Applied to the submitted artifact `c318140e...`, it gives `E:/rtklib_v2_ws_output/gsdc_native/all40_c318140e_phone_offset_posthoc_v1/submission.csv`, SHA-256 `a63bb22f3719e13a8fd57dbd987e2999dcf5d577fb9db84ba080badec17ffe4d`: 71,936 rows, 40 drives, identical key order, maximum shift 0.431 m. It is not submitted; external submission awaits explicit user instruction.

Submitted on explicit user instruction (2026-09-24): Kaggle ref 56507225, candidate `a63bb22f...`, status COMPLETE. Official **Public 1.251 m / Private 1.219 m**, versus the previous 1.396 / 1.333 m (-0.145 / -0.114 m), consistent with the train development estimate. The goal is still not achieved: taroz reference is 0.789 / 0.928 m. Receipt: `all40_c318140e_phone_offset_posthoc_v1/kaggle_receipt.json`.

### GPS L5 silently removed by base-band selection (2026-09-24)

Added diagnostic-only `--native-export-gnss-stage-factors` (writes `<summary>.gnss-first-factors.csv`; binary `binaries/gnss_stage_factor_export.exe`, SHA `bb410d9c...`, built from 359ab743 plus this export). For May13 Pixel6 Pro, the GNSS-first factors contain GPS L1CA, GLO L1CA, GAL E1 and GAL E5a, but **zero GPS L5**, although the route setting is `L5=1` and raw data has 3,980 GPS_L5_Q rows. The source miss-mask taxonomy shows all 3,897 adopted GPS L5 rows dropped (3,036 no exact base stream, 861 out of domain). The base model holds only 123 GPS L5 rows / 1 stream, while the SLAC RINEX 3.03 file carries C5X for G06/G24/G25/G18 on essentially every epoch. Cause: the RINEX reader keeps one primary and one secondary band per satellite, and the GPS secondary is L2, so base L5 is discarded unless `--native-base-pseudorange-preserve-additional-frequency-bands` is set. Upstream keeps FTYPE L1+L5 with sigtype factor 0.5 for L5, so it uses these rows at the highest weight. The existing Phase109 admission for that flag was only structurally validated and never used in any accuracy recipe or submission.

Truth-referenced snapshot residuals (diagnostic) also show persistent high-elevation GPS L1 biases, for example G25 at 62° with a mean of +5.4 m over minutes 4-15, which the missing L5 rows would dilute.

Launched a paired eleven-drive rerun: offset argv plus only `--native-base-pseudorange-preserve-additional-frequency-bands` (`train11_offset_extra_bands_v1`; scorer `compare_gsdc_train11_offset_extra_bands.py`, control = offset run).

### Explicit Type/L5 route replay result; extra-band interim (2026-09-24 13:10 JST)

The explicit source Type/L5 route replay ended `evaluated-with-failures`: 39/40 paired (2021-01-04 e1highway280 Pixel5 returned 1), and the paired-subset mean phone-score delta is **+0.0367 m** (P50 +0.033, P95 +0.040). Not promoted.

Extra-band (GPS L5 retained) versus offset-only, 10/11 complete: mean delta **-0.046 m**, with 7 improved, 2 regressed (2023 Pixel6 Pro mtv-u +0.071, routen +0.066) and 1 flat. GPS L5 rows are now fully retained on every drive. A test all40 rerun with binary 359ab743 plus `--native-upstream-position-offset --native-base-pseudorange-preserve-additional-frequency-bands` is in progress (`test40_offset_extra_bands_v1`, 40/40 admission verified beforehand, 5/40 complete). Note that this changes the test binary from the submitted `leading_count_phone_noise_fixed.exe`.

### Extra-band result complete; new test candidate (2026-09-24 evening)

Extra-band (GPS L5 retained) versus offset-only, 11/11: mean **0.775571 → 0.729124 m (-0.046447 m)**, 8/11 improving, evidence `records/gsdc2023_train11_offset_extra_bands_comparison.json`. Cumulative against the original control: 0.8807 → 0.7291 m.

Test all40 rerun complete: 40/40 return 0, every summary reports the offset enabled. The assembled candidate `E:/rtklib_v2_ws_output/gsdc_native/all40_offset_extra_bands_submission_v1/submission.csv` (SHA-256 `eb11130d23f3a192aba87d8a0e4d79d4dec9fd7f2d8fd2d6056981ede679c1da`) has 71,936 rows with the exact official key order of the parent. Every coordinate is the native solution at the identical key, with 114 non-official native rows dropped, no interpolation and no sample coordinates. Median displacement versus the submitted a63bb22f is 0.22 m (largest trip median: 2021-08-31 SM-G988B, 1.01 m). Not submitted; awaiting explicit instruction.

Submitted on explicit user instruction: Kaggle ref 56517871, candidate `eb11130d...`, COMPLETE. Official **Public 1.138 m / Private 1.088 m**, versus 1.251 / 1.219 m for a63bb22f and 1.396 / 1.333 m for c318140e. The goal is still not achieved: taroz reference is 0.789 / 0.928 m, leaving a Private gap of 0.160 m.

### Post-L5 GNSS-stage audit (2026-09-24 night)

GNSS-first export with offset and extra bands (diagnostic, binary bb410d9c), versus archived taroz `result_gnss.mat`: May13 Pixel6 Pro 0.701 m (was 0.809; taroz 0.548), SM-S908B 1.020 m (was 1.174; taroz 0.743), routen Pixel6 Pro 1.144 m (taroz 0.936). S908B median height bias now matches upstream (+0.98 vs +1.02 m, previously +1.98). Native height scatter remains larger (for example 1.10 vs 0.71 m on S908B), and low-pass horizontal differences correlate with height error (0.23-0.56).

Checks with no discrepancy found:
- BeiDou rows on May13 SM-G988B are all dropped because SLAC carries no BeiDou; upstream `sameSat` does the same.
- MultipathIndicator, SNR<20, code-lock and TOW/TOD status masks already match `exobs.m`.
- Exported pseudorange sigma equals upstream `10^(-(S-P85)/20)*sigtype_factor` up to one constant per band (0.984 L1-family / 0.942 L5-family on May13; 0.943 / 0.908 on routen), because of the percentile population. The relative weights are therefore at parity.

Routen Pixel6 Pro regression with L5 is a near-constant southward shift of the final solution (mean N -0.16 → -0.51 m), consistent with a satellite-dependent L5 code bias. That is not yet explained.

### Height constraints: upstream uses a train-GT height map (2026-09-25)

Upstream `fgo_gnss_imu.m` (non-init pass) adds, for every phone, either an absolute-height prior from `<course>/ref_hight.mat` (nearest point within 15 m of the initial trajectory; sigma 0.1 m, Huber 0.5) or, when no map exists, relative equal-height pairs (distance < 15 m, cumulative speed sum > 100; sigma 0.1 m, Huber 0.5). The archive has `ref_hight.mat` for 26/40 test courses and 1 train course. It is a train ground-truth trajectory: where our pooled Kaggle-train GT overlaps it, heights agree to 0.00 m. Three courses (mtv-g, mtv-m, mtv-de1) have upstream maps that the 2023 train GT does not cover.

User decision (2026-09-25): build our own height map from train ground truth rather than use `ref_hight.mat`. Downloaded all 156 Kaggle train `ground_truth.csv` files (existing competition data, manifest in `E:/rtklib_v2_ws_data/gsdc2023/kaggle_train_gt/`). Pooled test coverage within 15 m: mean 40%, with 18 drives above 30%.

Native changes (opt-in; default behavior unchanged):
- Lifted the Pixel5-only admission of `--native-relative-height-pairs`; the backend already accepted all phones.
- New `--native-height-map <csv lat_deg,lon_deg,height_m>`: an `AbsoluteHeightPoseFactor` (unary local-up prior on the antenna position) on main-graph epochs whose seed is within 15 m horizontally of a map point. It is cleared for GNSS-first and initial passes, mutually exclusive with relative pairs, and summary telemetry reports `height_map_*`. Binary `binaries/height_map.exe` SHA `9386777e...`.
- Map builder `E:/rtklib_v2_ws_tmp/build_gsdc_height_maps.py`: train mode excludes every GT file of the evaluated course (all phones). Leave-course-out coverage for the train11 drives is at least 60% on 7 drives.

Running: train11 relative-height pairs (all 11; first result May13 Pixel6 Pro 0.669 → 0.649 m) and train height map on the 7 covered drives, both paired against the extra-band run.

Height-map result (leave-course-out train GT, 7 covered train drives, versus extra-band): mean **0.700614 → 0.667955 m (-0.032659)**, 5/7 improved. The largest gains are routen Pixel6 Pro -0.077, mtv-a Pixel7 Pro -0.064 and May13 Pixel6 Pro -0.051; the regressions are xe1 Pixel7 Pro +0.020 and May13 SM-G988B +0.002. Relative-height interim: 5/5 improved, mean -0.010 m. Test all40 run launched (`run_gsdc_test40_height.py` → `test40_height_v1`): the height map on 25 drives with all-train-GT coverage ≥10%, relative pairs on the other 15; 40/40 admission verified.

Relative-height pairs, all phones (train11 versus extra-band): complete 11/11, mean **0.729124 → 0.714226 m (-0.014898)**, 10/11 improving (only xe1 Pixel7 Pro +0.025). Evidence `records/gsdc2023_train11_relheight_comparison.json`. Test all40 height run in progress (7/40 at 22:xx JST, no failures).

Test all40 height run (2026-09-25 10:10 JST): 39/40 complete with return 0. The remaining `2022-02-24-15-10-us-ca-lax-p/pixel5` (relative mode, about 1,262 estimated pairs, 4,515 epochs) has run 9.5 h versus 2.5 h without height pairs, and is left running. Candidate `all40_height_submission_v1/submission.csv` (SHA-256 `cf4a73e35852c0554ab8ecb3d65e62f7f268812dc3df5168a62c812313a695cb`) uses 25 map drives, 14 relative drives and that one drive from the offset+extra-band native run (declared in its manifest). 71,936 official rows, all native, no interpolation. Not submitted.

The lax-p height process was stopped on user instruction after about 9.8 h (memory had grown 0.8 → 5.6 GB, most likely from long-range relative pairs densifying the elimination). The candidate was unchanged. Submitted on explicit user instruction: Kaggle ref 56536540, `cf4a73e3...`, COMPLETE, **Public 1.133 m / Private 1.055 m** (previous 1.138 / 1.088). The goal is not achieved: Private gap to taroz 0.928 m is 0.127 m. Follow-up: bound relative-height pair topology on long drives before relying on it.

### IMU-stage gain parity; Doppler residual screen bug (2026-09-25)

GNSS-first → final gain on 5 drives (offset+extra bands, no height): native mean -0.097 m versus taroz `result_gnss`→`result_gnss_imu` -0.076 m. The native IMU stage is not the deficit; the gap is established at GNSS-first (native mean 0.90 m vs taroz 0.70 m).

With the Doppler rows exported (`.gnss-first-doppler.csv`), only 29,246 of 54,792 raw Doppler rows (about 50% on every signal) reach the May13 Pixel6 Pro GNSS-first graph. The builder's Doppler median screen always uses `upstream::residualThreshold(...,'D') = 3 m/s`, the upstream final-pass value, around the native SPP seed. The existing `nativeImuDopplerResidualThreshold` (20 m/s initialization / 3 m/s final) is never called there. Upstream `result_gnss.mat` is the initflag pass: 20 m/s Doppler and 50/30 m code screens around the WLS baseline.

Added opt-in `FGOConfig::native_doppler_residual_threshold_override_mps` and CLI `--native-gnss-first-doppler-threshold <mps>` (applied only to the GNSS-first staging config; default unchanged). The MSVC C1061 nesting limit required a standalone `if (...) continue;` parser block. Binary `binaries/doppler_threshold.exe` SHA `7adef1cf...`. With 20 m/s, May13 Pixel6 Pro retains 45,643 Doppler factors and GNSS-first improves 0.701 → 0.689 m (height std 0.57 → 0.48 m). Train11 paired run in progress (`train11_dthr20_v1`, control = extra-band run).

Result: the 20 m/s GNSS-first Doppler screen leaves the **final** solution unchanged (max 0.05 mm on May13 Pixel6 Pro and SM-G988B). The main IMU graph re-solves from its own factor set, and the GNSS-first result acts only as a seed. The experiment was stopped after 2/11. Conclusion: GNSS-first accuracy is not the lever for the final score. What matters is the main-graph factor set: P screened around the SPP seed, TDCP, IMU, no Doppler. Upstream's final pass instead re-screens P/D around its own initial GNSS+IMU trajectory and includes Doppler.

Main-graph Doppler (Phase213) on train11, versus extra-band: (A) 3 m/s screen, mean 0.7291 → 0.9525 m; (B) plus 20 m/s GNSS-first screen, 0.7291 → 0.9088 m. Both are dominated by a breakdown on July14 SM-G988B (0.559 → 3.015 / 2.560 m). The other ten drives change within ±0.02 m (A: 6 improved), which is neutral. Not promoted. Evidence: `records/gsdc2023_train11_maindop_v1_comparison.json`, `records/gsdc2023_train11_maindop_dthr20_v1_comparison.json`.

Upstream-like three-pass structure on train11, versus extra-band. Common flags: `--native-source-imu-initialization --native-imu-refinement-pass --native-phase213-main-doppler`; R2 adds observation/stop stages.
- R1 mean 0.7291 → 0.7764 m.
- R2 mean 0.7291 → 0.7402 m.
- Both improve 7/11. The other ten drives improve by about 0.01 m on average.
- Both are sunk by July14 SM-G988B (R1 +0.593, R2 +0.196 m), the same drive that breaks with main-graph Doppler.
Not promoted. Next: diagnose Doppler on July14 SM-G988B rather than excluding it by phone.

### Final-pass structural audit: main-graph XXVV motion (2026-09-25)

A line-by-line reading of upstream `fgo_gnss_imu.m` (non-init) against the native main graph found one more missing term: upstream adds `MotionFactor_XXVV` (x2 = x1 + (v1+v2)/2·dt) beside the IMU factor, with `prm.sigma_motion` = 0.05 m for Street, 0.01 m otherwise, 0.1 m for mi8. Native had this only as Pixel5-only Phase217 with a fixed 0.05 m. The native change lifts the Pixel5 gate and adds `--native-phase217-motion-sigma` (binary `motion.exe` SHA `a44e59d4...`). Train11 with the upstream per-route sigma, versus extra-band, 10/11 complete: mean -0.004 m, with 7 improving (routebb1 -0.081, S908B -0.017) but mtv-a Pixel7 Pro +0.112. July14 SM-G988B ran over 2 h (baseline ~0.5 h) and was stopped. Neutral and costly in runtime; not promoted.

Other upstream final-pass items verified present natively: P (SNR-sigma, Huber), TDCP, IMU preintegration with bias random walk, stop velocity/pose factors, height factors (added), and the post-solve phone offset (added). Main-graph Doppler remains absent. Adding it via Phase213 or the refinement pass breaks July14 SM-G988B, whose raw Doppler is clean against truth (GPS L1 MAD 0.055 m/s), so the failure lies in native processing around two short stops at minutes 4-9. It is not yet diagnosed.

July14 SM-G988B main-Doppler breakdown characterized (truth used for diagnosis only). Relative to truth, the Phase213 solution carries an approximately constant **world-frame position offset of about 5 m** over the whole segment between the U-turn stop at t≈255 s and the next U-turn stop at t≈550 s. Its along/cross components flip sign with the 180° heading reversals while the ENU vector stays the same, so this is not an along-track Doppler time-tag effect (correlation with acceleration +0.06) or a heading error. It vanishes after the third U-turn. The initial IMU pass of the refinement run already shows it (3.75 m). Interpretation: with main-graph Doppler (VD + receiver drift), a stop-bounded segment slides as a rigid block that Huber-weighted pseudorange does not pull back (local minimum). Candidate causes to check next: segment-wise drift/clock coupling through the VD factor at stop boundaries, and stop pose factors interacting with the Doppler velocity. Not yet resolved.

### Checkpoint and pause (2026-09-25 evening)

Paused on user instruction. **The goal is not achieved.** Best official: ref 56536540, `cf4a73e3...`, **Public 1.133 m / Private 1.055 m** (taroz 0.789 / 0.928; Private gap 0.127 m). Progress this cycle: Private 1.333 → 1.219 (upstream phone offset) → 1.088 (base GPS L5 retained) → 1.055 (train-GT height map + relative height).

Additional G988B main-Doppler checks: removing stop constraints does not change the breakdown (3.016 m); raw epochs are clean (one 1001 ms step); the Phase213 VD factor uses an absolute receiver-only range rate with ENU LOS, matching upstream `DopplerFactor_VD`. Best explanation: a stop/U-turn-bounded segment converging to a rigidly shifted local minimum once velocities are strongly constrained. Unresolved.

Open items when resuming:
1. Main-graph Doppler / three-pass refinement: roughly -0.01 m on 10/11 drives, but blocked by July14 SM-G988B (above).
2. Relative-height pairs on long drives (test lax-p) densify elimination and do not finish; bound or subsample long-range pairs.
3. Main-graph XXVV motion: neutral (-0.004 m on 10 drives) and slow; kept as an opt-in only.
4. Test-side height map coverage is 40% (three upstream-covered courses lack 2023 train GT).
Scripts and binaries for every experiment are under `E:/rtklib_v2_ws_tmp` and `E:/rtklib_v2_ws_output/gsdc_native/binaries`.

### Resumed: main-Doppler residual localization (2026-09-26)

Resumed on user instruction toward official Private <= 0.928 m; the last
recorded official result remains 1.055 m. Work branch:
`fix/gsdc-doppler-segment-drift`, based on merged revision `6ef6514`.
No native experiment process was running when the work resumed.

Read-only inspection found a previously unlocalized TDCP residual anomaly on
July14 SM-G988B: maximum residual 3.4365 m without main Doppler versus
23,363.2208 m with it, with the same 22,408 inserted factors. The backend and
CLI reconstructed RMS agree (0.03929 m versus 156.07422 m). R1 and the
no-stop Doppler ablation also have the extreme residual; R2 does not, despite
its remaining position regression. This is evidence for investigation, not
proof of the cause or a complete explanation of the segment displacement.
Frozen summary hashes and values are in
`use_cases/records/gsdc2023_doppler_tdcp_residual_audit_20260926.json`.

Added report-only maximum-residual endpoint indices, system/signal, signed
residual and carrier difference. The canonical MSVC Release native target
built successfully. Started a sequential same-binary control/Doppler replay
using `scripts/analysis/run_gsdc_doppler_segment_diagnostic.py`, with pinned
raw-input and executable hashes and no truth input. Output root:
`E:/rtklib_v2_ws_output/gsdc_native/doppler_segment_diagnostic_20260926`.
The runner records the live child PID and final status in each `run.json`.
Results and position-stream preservation are pending; no solver correction,
promotion or official submission has been made.

Raw carrier/Doppler cross-check: 24,303 same-satellite/signal adjacent pairs
with valid ADR and neither reset nor cycle-slip at either endpoint have a
maximum absolute `delta ADR - trapezoidal integrated Doppler` of 1.6654 m.
This check uses only raw measurements; its population is not asserted to be
the 22,408 admitted native TDCP factors. It argues against a raw 23 km carrier
jump in this valid population, but cannot establish whether geometry,
measurement preparation, clock states, or optimization causes the native
residual. Reproduce with `scripts/analysis/audit_gsdc_raw_carrier_doppler.py`;
input hash and largest pairs are retained in
`use_cases/records/gsdc2023_g988b_raw_carrier_doppler_20260926.json`.

The diagnostic no-D control completed successfully in 478.74 s and exactly
reproduces the historical position CSV SHA-256 `9f94aaf9...`. Its largest
TDCP residual is 3.4365 m on epochs 568->569. The paired main-D replay is
running; its largest-residual location remains pending.

Prepared a separate continuation experiment:
`--native-refinement-no-doppler-initialization` requires the existing complete
IMU-refinement recipe, solves the initial IMU pass with a copy of the problem
whose main-D rows are omitted, and then executes the existing final observation
rebuild and Doppler solve. Default behavior is unchanged. This is not a claim
of upstream parity or a fixed-final-factor-set comparison: final admission
can change because it is rebuilt around the different initial trajectory.
Telemetry records initial main-D factor count and the selected option.
MSVC build and three negative CLI admission checks passed.

A same-binary R1 control/candidate pair is running under
`E:/rtklib_v2_ws_output/gsdc_native/doppler_continuation_20260926`, executable
SHA-256 `82ef0163...`. The candidate is selected by one additional flag;
inputs, source recipe, and executable are pinned. The companion
`compare_gsdc_doppler_diagnostic.py` audits hashes, sole-flag difference, all
raw UTC keys, native output contracts and (for continuation) identical
GNSS-first stage output and the initial/final Doppler factor counts before
scoring existing development truth. It has been syntax checked; the pending
pair has not yet exercised its complete comparison path.
Plan and source hashes: `use_cases/records/gsdc2023_doppler_continuation_20260926.json`.

### Main-D replay completed; endpoint isolated (2026-09-26)

Both diagnostic arms completed, with byte-identical position streams to their
historical counterparts. The audited same-binary comparison covers all 1,165
raw UTC keys and all 1,165 existing development truth keys. No output key is
interpolated, held or unresolved. P50/P95 phone score changes from
0.559146909 m to 3.015469947 m; candidate P95 is 5.390315051 m.
Evidence: `use_cases/records/gsdc2023_doppler_segment_comparison_20260926.json`.

The -23,363.220845 m residual is at epochs 556->557, GPS L1, with prepared
carrier difference 704.670450 m. Raw GPS 5 is the sole continuous valid GPS-L1
ADR pair across these endpoints, with delta ADR about 704.684420 m and
hardware clock-discontinuity count unchanged at 55. The graph has all 1,164
C0/D clock links and zero clock-jump skips. The archived R1 initial position
step across this second is 12.379 m. The active exact-range/code-clock TDCP
factor and reconstructed diagnostic use the same equation. These checks
localize the anomaly but do not yet distinguish prepared satellite geometry
from optimized clock change.

The continuation control also completed: final, GNSS-first, and initial-IMU
CSVs are byte-identical to archived R1. Its initial main-D count is 12,762;
the no-D-initialization candidate is running. In parallel, a report-only replay
under `E:/rtklib_v2_ws_output/gsdc_native/doppler_residual_components_20260926`
adds maximum-residual PRN, geometric range change and code-clock change. The
diagnostic explicitly labels integrated-drift phones, whose code-clock change
is not their TDCP clock term. Its build succeeded after correcting a summary
scope error. Results remain pending; no accuracy improvement is established.

### Clock-jump decomposition and continuation v2 (2026-09-26)

The report-only decomposition replay completed successfully. At GPS 5 L1,
epochs 556->557, the signed TDCP residual is -23363.220845 m. Its prepared
carrier delta is 704.670450 m, geometric range change 594.859019 m and
optimized code-clock change -23253.409415 m. The output position stream
matches the earlier main-D replay. The anomaly is therefore in the optimized
clock term; the underlying cause is not yet established.

Continuation v1 failed before its final solve: rebuilding from the no-D
initial IMU states discarded all Doppler rows. No final solution or score
exists for that arm. V2 reuses the complete same-run GNSS-first drift vector
when rebuilding the final observations. Eight focused handoff tests pass,
including an alternating-drift rejection/replacement case and strict source
identity checks. Both v2 native arms are running with frozen binary SHA-256
`5fcee465a87646971e887725072c2e430fa37b8d2e4e1f2aa83f33792b9d4d13`.
The candidate has exported its initial IMU clocks: around the diagnostic
endpoint, C0 advances normally by approximately 110.4 m/s while the no-D
drift alternates near -33896 and +34117 m/s. This confirms the problematic
unobserved drift mode on real data, without yet proving a position benefit.

An independent small diagnostic links the existing native base correction
model and uses the exact raw RINEX/navigation inputs, explicit station and
ordinary 1 s / 151-sample smoothing recipe. GPS 5 correction changes from
-1.320568 m to -1.317266 m at the TDCP endpoint (about +0.003302 m).
The maximum adjacent GPS-L1 correction change over the sampled route is
0.039703 m. This rejects a large base-correction jump as a direct explanation
of the optimized 23 km clock jump. Source, executable, input and output
hashes and initial clock excerpt are recorded in
`use_cases/records/gsdc2023_base_clock_audit_20260926.json`.

A uniform 15-Pixel5 source-TDCP/main-D comparison has also been prepared, but
has not launched while the two v2 native processes occupy the experiment
slots. All cases use the same three candidate flags; maps exclude the entire
evaluated course across phones. No candidate is promoted and the recorded
official Private remains 1.055 m.

V2 control subsequently completed (rc=0, wall 852.172 s). Its final position
CSV is byte-identical to the v1 control. The v2 candidate remains live; no
candidate score is claimed. `score_gsdc_frozen_pairs.py` is now prepared for
the subsequent Pixel5 benchmark: it verifies completion of the entire frozen
plan, exact planned command/input hashes, and each comparison's native/raw
key contracts before writing the full mean and regression count. Only syntax
and CLI parsing have been checked so far; the real batch is not yet launched.

### V2 rejected; observation-based drift candidate prepared (2026-09-26)

Both v2 arms completed and passed the frozen-input/binary/argv, native-key,
convergence and identical GNSS-first stage audits. All 1,165 truth timestamps
were scored. The control phone score is 1.151969228 m (P50 0.482294481,
P95 1.821643974); no-D initialization plus cached GNSS drift gives
2.824781066 m (P50 0.626959020, P95 5.022603111). Delta +1.672811838 m:
a regression, not promoted. Candidate final main-D count is 31,895, but its
maximum TDCP residual remains 23,336.379 m at the same 556->557 GPS 5 pair.
Evidence: `use_cases/records/gsdc2023_doppler_continuation_v2_comparison_20260926.json`.

The stage audit also found GNSS-only position spikes at epochs 264, 389, 557,
with adjacent 7.7, 18.7, 20.8 km steps. The corresponding GNSS drift has large
outliers. No-D IMU suppresses those position spikes (maximum step 36.65 m),
but has a 46.65 km clock step at epoch 423. See
`use_cases/records/gsdc2023_doppler_stage_clock_audit_20260926.json`.

An explicit, default-off `--native-refinement-observed-clock-drift` candidate
now estimates drift from corrected raw D minus LOS-projected IMU velocity,
before the final Doppler mask. It takes per-satellite medians before the
across-satellite median, requires at least three satellites at every epoch,
and has no time fill or GNSS drift fallback. It reuses the existing native
orbit/atmosphere geometry and residual thresholds. The original v2 path
remains available and unchanged when the new selector is off. Ten focused
tests pass; native build and real-data evaluation are pending.

The full frozen 15-Pixel5 batch is now running with one worker (session 98677,
runner PID 18808), leaving one native slot for the Samsung investigation.
The immutable plan hash is unchanged. No batch score is available yet.

V3 build completed and was frozen as
`E:/rtklib_v2_ws_output/gsdc_native/binaries/doppler_continuation_v3_20260926.exe`,
SHA-256 `0ebe47f530bfc5dff57f815f78f86a35ebd61a369a02e28fb029bd7e1890656a`.
The CLI rejects the new selector without no-D initialization (exit 2).
The candidate-only replay now runs from the unchanged R1 control recipe with
exactly the two declared additional flags, under
`E:/rtklib_v2_ws_output/gsdc_native/doppler_continuation_v3_20260926/candidate_arm/observed_drift`
(session 91065). Same-binary v3 control is still pending; do not claim an
improvement until the native output and comparison audits complete.
Provenance: `use_cases/records/gsdc2023_doppler_continuation_v3_20260926.json`.

A supervisor now waits on the verified live v3 candidate process (PID 27492,
creation time pinned), then launches the same-binary control only after a
successful candidate exit and terminal run record. It audits/scores the pair
after the control completes. Supervisor session 95272; authoritative gate
state is `doppler_continuation_v3_20260926/control_gate.json`. Its source hash
and process identity are stored in the v3 record. Do not start a third native
run while this gate and the one-worker Pixel5 batch own the two slots.

Read-only follow-up on relative-height cost confirmed that the native source
selector emits every qualifying pair (<15 m separation, cumulative speed
sample sum >100, neither endpoint stopped), without an edge budget. The
interrupted test lax-p artifact has no exported seed coordinates or completed
summary, so its exact inserted graph cannot be reconstructed from that record
alone. No topology change or claimed runtime fix has been made.

V3's GNSS-first position CSV, clock CSV and initialization metadata are
byte-identical to v2, confirming no pre-refinement seed difference in the
candidate replay. Its initial IMU/final stages are still running. Pixel5
case 00 control completed successfully in 1015.971 s (2,002 native epochs);
the batch advanced automatically to case 00 candidate. No pair score or
whole-batch mean is available yet.

### V3 pre-screen failure fixed; v4 and first Pixel5 pair (2026-09-26)

V3 stopped before its final solve (rc=1, 741.096 s), reporting fewer than
three satellites at epoch 0. Its IMU-initial position/clock exports exactly
match v2. Source inspection identifies the actual cause: passing receiver
velocities makes `refinement` true, and the builder applies its absolute
Doppler screen regardless of `use_upstream_absolute_doppler_residual_screen`.
The no-D drift therefore removes measurements before drift estimation.
This is not evidence that epoch zero requires a special exception.

V4 keeps the three-satellite requirement. Only its temporary geometry build
uses the maximum finite residual threshold, retaining finite SNR/elevation-
qualified Doppler before clock estimation. The subsequent mask and final
build retain the original thresholds; the temporary graph is never optimized.
A new integration regression test actually calls the builder/rebuild path:
legacy +/-60000 m/s drift rejects all rows; the new path estimates 110 m/s,
retains 8 valid rows and rejects the two injected 50 m/s outliers, including
correct handling of the first epoch. All 3 builder tests and 10 handoff tests
pass. V4 executable SHA-256:
`a73457093141c87cb4689a8debe9c14c2f8de8a467b75d419c1ddc0d2f26d639`.
Candidate runs first, followed automatically by the control and paired audit
on success (session 81051). Record:
`use_cases/records/gsdc2023_doppler_continuation_v4_20260926.json`.

Pixel5 first pair (2021-01-04 highway280) completed with all 2,002 native UTC
keys and all 2,001 available truth keys verified. Control 0.503787651 m,
candidate 0.503595939 m, delta -0.000191712 m: practically neutral on this
one route. The fixed 15-case plan continues on case 01; no whole-batch mean
or promotion is claimed. Partial evidence:
`use_cases/records/gsdc2023_pixel5_source_recipe_progress_20260926.json`.

A read-only Kaggle score-list attempt could not authenticate in the current
CLI environment. No login or submission was attempted. The local submitted
CSV still hashes to the recorded `cf4a73e3...` receipt (Private 1.055 m).
This is a recorded score, not a fresh server verification. Readback record:
`use_cases/records/gsdc2023_official_score_readback_20260926.json`.

### Native-effect audit and live official readback (2026-09-26)

The all-pair scorer now requires native telemetry to confirm the requested
TDCP metre-sigma mode, source Huber k for the resolved route type, and actual
main-D factor insertion. Case 00 passes: TDCP sigma is data-derived rather
than fixed 0.03 m, source Highway Huber k is 0.5, and candidate main-D count
is 21,941 versus zero in control. This does not change its nearly neutral
score or justify any promotion. The whole 15-case aggregate remains pending.

The earlier CLI authentication failure was resolved using the same saved
OAuth credential/refresh mechanism as the previously successful submission
script. Browser login was disabled, credentials were not printed, and only
the submission-list endpoint was read. Server readback confirms latest
submission ref 56536540 is COMPLETE, Public 1.133 m / Private 1.055 m.
No submission was made. Evidence:
`use_cases/records/gsdc2023_official_score_oauth_readback_20260926.json`.
The earlier failed-read record remains as history and is superseded by this
successful server readback; authentication is not a current blocker.

### Conditional test plan reconciled with the published artifact (2026-09-26)

Prepared (not executed) a paired test plan for all 17 Pixel5 phones, retaining
the 23 other phones from the published native sources. All 71,936 published
coordinates exactly match their declared completed native output rows. Raw
input hashes are checked against the historical native run manifests; every
existing input-file argument is pinned. The 17 selected phones preserve the
published height policy: 8 maps, 8 relative-height recipes, and lax-p's
explicit no-height fallback. No stopped/failed lax-p height output is used.
Plan SHA-256 `0c22f0f0b045aeb95a364876bf9226e1a8ec442a1f13bf7c3cfb6cdcd633de4f`.
Execution requires review of the complete frozen development comparison;
assembly would additionally require control replay parity against the
published native outputs. Record:
`use_cases/records/gsdc2023_pixel5_submitted_test_recipe_plan_20260926.json`.

V4 now has completed initial IMU exports that are byte-identical to v3 and
has proceeded past the previous reconstruction failure into the final solve.
The live process continues; the completed summary and score are not yet
available. Both native experiment slots remain occupied.

### Native input clock-reference reset localized (2026-09-26)

V4 candidate completed and passed the 1,165-key native output audit, scoring
2.609281705 m on the exposed development route. The approximately 23 km
TDCP residual remains, so the candidate is not promoted. Its same-binary
control remains live. Pixel5 case 01 control completed successfully and its
candidate is live; only the first pair has a completed paired audit.

A small executable linked to the actual native Android loader now isolates
the origin of the large initial-IMU clock step: at epoch 423, UTC
1626296265000, the loader changes its FullBias reference because the raw
TimeNanos interval exceeds one second. HardwareClockDiscontinuityCount
remains unchanged. The resulting input code-clock change is -46649.805012 m,
matching the optimized initial-IMU change (-46649.75666 m) within 5 cm.
This is an input reference discontinuity, not evidence of a physical clock
jump or a solver prior. It does not yet prove the cause of the later epoch
557 residual or the five-metre position displacement. The next experiment
should explicitly compare continuous raw-clock reference handling against
legacy ingestion, retaining default upstream parity and testing actual
hardware/time discontinuities. Evidence:
`use_cases/records/gsdc2023_native_input_clock_reset_20260926.json`.

The conditional test submission assembler has passed syntax and negative
incomplete-development gating, and all 23 retained native sources passed
hash/output-contract preflight. No complete positive assembly or test-plan
execution is claimed. Evidence:
`use_cases/records/gsdc2023_pixel5_test_assembly_preflight_20260926.json`.

### Continuous raw-clock experiment built and tested (2026-09-26)

Added default-off `--android-continuous-clock-reference` (requires raw-clock
Android input). It retains the first FullBias reference across forward gaps
and rejects backward TimeNanos or changed hardware clock counts. Legacy
upstream reference-reset behavior is unchanged unless explicitly enabled.
All 27 Android-loader tests pass, including unchanged default reset behavior,
continuous pseudorange/time/clock consistency and discontinuity rejection.
On the actual 1,165-epoch G988B input, all UTC keys match; the epoch-423 clock
step changes from -46649.805012 m to +110.623417 m, with maximum adjacent step
111.223002 m and no reference resets. These are input diagnostics, not a
position score or proof that the five-metre position error is fixed.

Frozen binary and source/test hashes are recorded in
`use_cases/records/gsdc2023_continuous_raw_clock_20260926.json`.
Supervisor session 29398 waits for verified native v4-control PID 49040 to
finish, then runs candidate first and same-binary control against the original
main-D recipe (no refinement), differing only by the new clock flag. A full
native-contract and truth scoring audit follows successful pair completion.
Pixel5 continues in the other slot; at most two inference processes run.

V4 paired audit has now completed: control 1.151969228 m, observed-drift
candidate 2.609281705 m (regression +1.457312477 m), not promoted.
The continuous-clock candidate has started, verified live PID 17168, after
v4 control completed. Pixel5 candidate PID 60536 remains live. The queued
supervisor's final scoring command contains an incorrect truth-file path;
inference is unaffected. Re-run the comparator after both native runs finish
with the pinned `scoring/train/.../sm-g988b/ground_truth.csv` path recorded
in the experiment manifest; do not repeat inference because of this scoring
path error.

### Continuous-clock candidate evidence and applicability (2026-09-26)

The original main-D recipe with continuous reference completed in 86.644 s.
All 1,165 native UTC and truth keys pass the output audit. Development score
is 0.560819250 m (P50 0.373595689, P95 0.748042811), near the historical
no-D 0.5591469 m baseline. Maximum TDCP residual falls to 3.436765 m and RMS
0.039275478 m, versus approximately 23 km / 156 m previously with main D.
The same-binary legacy control remains live as PID 56232, so the paired
comparison and default output parity still need verification. Candidate
record: `use_cases/records/gsdc2023_continuous_raw_clock_candidate_20260926.json`.

A hash-verified inventory of the 40 published native summaries finds six
phones with loader clock-discontinuity counts. Read-only raw scans show
five have only forward TimeNanos gaps and unchanged hardware counters. The
2023-06-06 Pixel5 has an actual counter change plus backward TimeNanos and
must not use the new single-clock-segment mode. Raw scans do not reproduce
native quality gates; they are diagnostics, not an inference eligibility
certificate. Inventory:
`use_cases/records/gsdc2023_test_clock_reset_inventory_20260926.json`.

Supervisor session 43903 waits for the current legacy control, reruns the
original pair audit with the correct separate scoring truth path, then runs
the refinement recipe with continuous reference, candidate first followed
by its own same-binary control. No truth is passed to either inference run.
This also repairs the earlier supervisor's scoring-only path error without
repeating inference. The refinement plan is recorded in
`use_cases/records/gsdc2023_continuous_raw_clock_refinement_20260926.json`.

Pixel5 case 01 now passes the frozen-plan and native-effect audits:
0.551327829 -> 0.482478924 m (delta -0.068848905 m). Together with the nearly
neutral case 00 this is 2/15 completed pairs, not a whole-plan estimate or
promotion. Case 02 control PID 4316 is the next running native process.

### Segment displacement check (2026-09-26)

A truth-free comparison of all 1,165 output positions against the previously
audited no-D trajectory confirms that the metre-scale displacement is absent
in the continuous-clock candidate: whole-route P95 displacement falls from
5.492681 m (legacy main D) to 0.155023 m, and maximum from 5.723469 m to
0.232699 m. In the second chronological quarter, the median displacement
falls from 4.594186 m to 0.060574 m. These are distances between solutions,
not errors against truth. Historical reference runs use their previously
frozen binary; the current same-binary control is still pending.
Record: `use_cases/records/gsdc2023_clock_segment_displacement_20260926.json`.
The plotted trajectories were visually checked at
`E:/rtklib_v2_ws_output/gsdc_native/continuous_raw_clock_20260926/displacement.png`.
The current legacy control PID 56232 and Pixel5 case-02 control PID 4316 were
both verified live; no restart or third inference job was issued.

### Native admission and wider development coverage (2026-09-26)

Completed loader-only paired checks on all 15 fixed Pixel5 development cases
and all six submitted test phones with native clock-discontinuity telemetry.
All 15 Pixel5 development clock exports are byte-identical between modes.
Five affected test phones preserve their observation UTC keys under continuous
reference; the June06 Pixel5 with a real hardware counter change/backward
TimeNanos is rejected, as required. This is input admission, not optimizer
validation or a test-side accuracy claim. Evidence:
`use_cases/records/gsdc2023_clock_admission_20260926.json`.

An inventory of all 40 completed corrected-development summaries identifies
three affected phones: July14 G988B and Oct06 SM-A205U / SM-A325F. The latter
two pass the loader's continuous mode. A205U currently resets its reference
at all 1,208 inter-epoch intervals; continuous mode retains the reference and
all observation keys. A325F still has an approximately 600 km input-bias step
under continuous mode; that event is not resolved by removing reference resets
and must not be described as fixed. Source counts and native loader evidence:
`use_cases/records/gsdc2023_train40_clock_reset_inventory_20260926.json` and
`use_cases/records/gsdc2023_train_samsung_clock_admission_20260926.json`.

Prepared, not executed, a frozen comparison of both additional affected
Samsung phones, selected by input diagnostics only, with no score filtering.
Their original corrected no-main-D recipes differ only by the clock flag.
A205U's application adds four native leading clock states beyond the helper's
1,209 observation epochs; final inference must pass all 1,213 raw UTC keys.
Plan SHA-256 `70d173439a77c40959a548cbb755998d089b9168d46e79018f594da8d169cc17`.
Run with one worker only after the queued July14 refinement comparison ends.
Record: `use_cases/records/gsdc2023_continuous_raw_clock_train_samsung_plan_20260926.json`.

### Same-binary clock repair confirmed (2026-09-26)

The original main-D comparison is complete and fully audited on all 1,165
raw UTC/truth keys: legacy reference 3.015469947 m, continuous reference
0.560819250 m, delta -2.454650697 m. Only the continuous-clock flag differs.
The new-binary legacy control's position CSV is byte-identical to the
historical component diagnostic (`066cf1b6...`), confirming default output
parity on this route. No official-score claim or recipe promotion follows
from this exposed single-route result. Paired evidence:
`use_cases/records/gsdc2023_continuous_raw_clock_comparison_20260926.json`.

The original supervisor ended with its known scoring-path error after both
successful native runs; corrected supervisor 43903 subsequently completed
the audit using the separate scoring path. No inference was repeated. It
has now started the refinement candidate as native PID 9484; Pixel5 case 02
candidate PID 64080 remains the other native run. The prepared two-phone
Samsung no-D plan is not yet launched.

### Refinement candidate and third Pixel5 result (2026-09-26)

The continuous-clock refinement candidate completed successfully in 152.65 s,
with all 1,165 native keys audited. Development score 0.561445425 m, TDCP RMS
0.039249053 m and max residual 3.436653 m. This is essentially neutral against
the continuous non-refined candidate (0.560819250 m); no additional accuracy
gain is claimed. The matched legacy refinement control is live as PID 56624.
Stage audit confirms the GNSS-first kilometre-scale spikes are gone: maximum
ECEF step is 36.653 m and clock step 117.675 m; initial-IMU max clock step is
110.940 m. Both stages have zero >100 m position or >1000 m clock steps.
These thresholds are diagnostic only. Records:
`use_cases/records/gsdc2023_continuous_raw_clock_refinement_candidate_20260926.json`
and `use_cases/records/gsdc2023_continuous_clock_stage_audit_20260926.json`.

The two additional Samsung pairs are now queued via session 93925, waiting
for the verified refinement supervisor PID 31068 and requiring its complete
paired audit before launching one worker. This preserves the two-inference
limit while Pixel5 runs in the other slot.

Pixel5 case 02 (March10) completed with a -0.065107889 m score delta. The
new `scripts/analysis/record_gsdc_frozen_pairs_progress.py` re-audits native
artifacts, fixed plan arguments, actual setting effects and pinned truth
hashes before recording partial progress. It passes on all 3/15 completed
pairs and deliberately emits no partial aggregate. The whole-plan scorer
now also verifies scoring truth files against the frozen GT manifest when
provided. Pixel5 case 03 control is running as PID 23668.

### A325F remaining clock changes localized (2026-09-26)

The approximately 599.6 km input-bias step is at startup, observation epoch
0->1 (a 0.5 s interval), with a 2,000,128 ns FullBias update while the hardware
counter stays zero. Constant-reference raw GPS L1 code differences also show
the roughly 599 km change. None of the five common GPS L1 satellites has
valid, reset-free, cycle-slip-free ADR at both startup endpoints, so these
rows do not establish a valid TDCP continuity violation. Several later
approximately -299.8 km FullBias-derived steps remain in both modes.
Continuous reference removes the separate reference reset at epoch 9 but
does not purport to remove these other raw clock changes. No extra data mask
or inference correction was introduced from this diagnostic.
Evidence: `use_cases/records/gsdc2023_a325f_raw_clock_event_20260926.json`.
The paired A325F/A205U experiment remains queued behind the current refinement
comparison; its scope and frozen inputs have not changed.

### Fixed eleven-case policy reconstruction (2026-09-26)

Recomputed scores from hash-verified saved native outputs for all eleven
original offset/extra-band cases. Replace only July14 G988B, the sole case
with reference-reset telemetry, with its completed continuous-clock output;
retain the other ten saved trajectories. Selection is input-diagnostic based,
not per-route score selection. The result is a reconstructed policy audit,
not a fresh all-eleven same-binary experiment, not held out, and not the
height recipe or official test set:

- No-main-D reference mean: 0.729124212 m.
- Clock-repaired main D: 0.729344433 m (+0.000220222 m; essentially neutral).
- Clock-repaired refinement: 0.722669496 m (-0.006454716 m; modest).

Thus repairing the catastrophic main-D case does not itself establish the
0.127 m official gap has been closed. Keep broader Pixel5/noise-recipe and
Samsung comparisons as the next evidence sources; do not promote main D
alone from its dramatic single-case recovery. Artifact hashes and per-case
recomputed values are in
`use_cases/records/gsdc2023_train11_clock_repaired_policy_20260926.json`.

The matched refinement control was confirmed live as PID 56624 and the
supervisor session 43903 remains running. Pixel5 case 03 has advanced from
control to candidate, PID 66168. No failed or live job was restarted.

### Refinement comparison completed; Samsung pairs started (2026-09-26)

Same-binary refinement pair completed and passed the full native-contract
and scoring audit: legacy clock 1.151969228 m, continuous clock 0.561445425 m,
delta -0.590523802 m. The control position CSV is byte-identical to the
historical train11 refinement output. Record:
`use_cases/records/gsdc2023_continuous_raw_clock_refinement_comparison_20260926.json`.
No further July14 inference is queued from this result; its regression is
resolved in the tested recipes, while broad accuracy gains remain modest.

The gated Samsung plan has started with one worker: parent PID 42228,
SM-A205U control native PID 6548. Pixel5 case 03 candidate PID 66168 is the
other native solve. The plan remains exactly its frozen hash `70d17343...`;
no concurrent third inference or task restart was introduced.

### IMPORTANT: same-drive alias found in one unexecuted Pixel5 height map (2026-09-26)

The existing raw-UTC route-group audit identifies May16 xe1 Pixel5 (19:54)
and Pixel7 Pro (19:55) as the same evaluation group (99.93% temporal overlap).
The old height builder excluded only the literal course-directory name.
Pixel5 case 12's map contained 2,318 exact coordinate/height rows from the
same-drive Pixel7 Pro truth file. Case 12 had NOT started inference. This
violates the intended exclusion of all phones from the evaluated drive.

To prevent evaluation-data reuse, only that unused map was renamed to
`.../pixel5_source_recipe_20260926/maps/2023-05-16-19-54-us-ca-mtv-xe1__pixel5.invalid_same_group.csv`.
Its bytes/hash are preserved. The immutable original runner will now fail
closed before inference for case 12, then continue cases 13/14. This failure
is intentional; do not restore the unsafe map or restart that original pair.
The other fourteen maps and live processes are unchanged.

Built a replacement map excluding BOTH aliases: 16,247 points, 65.6048%
coverage. Replacement one-pair plan SHA `26fa7bf4...`; composite 15-case
`evaluation_plan.json` SHA `00854b27...`, both under
`E:/rtklib_v2_ws_output/gsdc_native/pixel5_height_group_repair_20260926`.
The composite audit reuses exactly fourteen original pairs and the one
predetermined replacement, with explicit per-entry source-plan hashes.
No score-based route selection or fabricated execution manifest is used.

New `gsdc_development_plan_audit.py` checks group exclusions, the fixed cohort,
and source-plan provenance. Original metadata rejects exactly case 12; all
fifteen corrected entries pass. The full scorer supports this explicit repair
and requires every selected native pair to finish; the original batch's
intentional failed case is never included. Submission assembly now requires
`height_map_group_independence_verified`. The old prepared test17 plan must
be rebased to the corrected development evidence before execution/promotion.
Evidence: `use_cases/records/gsdc2023_pixel5_height_group_overlap_20260926.json`.

Replacement supervisor session 21361 waits for verified Samsung supervisor
PID 48012 (created 12:51:37.607798 JST), accepts its terminal failed experiment
as a completed slot, verifies no prior native process is live, then runs the
replacement pair with one worker. Binary remains `82ef0163...`, matching the
other fourteen Pixel5 pairs. The new current progress record is
`use_cases/records/gsdc2023_pixel5_group_safe_progress_20260926.json` (4/15).
Case 03 completed with delta -0.183039876 m; case 04 control PID 43944 runs.

SM-A205U continuous-clock candidate failed in 6.41 s before IMU/main inference:
`temporal-initialization-ineligible-failure`; raw seed status `time-gap`,
reason `epoch-time-gap-exceeds-limit`, 1208/1213 independent SPP epochs.
Do not treat the observation-only loader admission as full application
admission (four native leading epochs are added by this recipe). No retry or
threshold relaxation has been made. SM-A325F's paired computation continues.
Investigate the precise GPST/UTC gap and raw-P limit before any fix.

Historical review also confirms sigma-only weighting on the eleven modern
phones was roughly neutral (-0.00166 m), and explicit Type/L5 overrides
regressed the 39 completed pairs. These old ablations should not be repeated
unchanged; the joint sigma/Huber/main-D recipe remains a distinct experiment.

### Continuous clock v2: raw-P gap admission correction under validation (2026-09-26)

Located A205U's rejection at a 2.000001405016519 s receiver-reference gap.
Its raw receiver-clock offset changes by 1.28 us; the clock-corrected interval
is 2.000000125016519 s, within the existing 2 s plus 1 us equality tolerance.
Evidence: `use_cases/records/gsdc2023_a205u_clock_gap_admission_20260926.json`.

Added default-off `receiver_clock_corrected_gap_checks` to raw-P seed Config,
activated at the three app seed call sites only by the continuous-reference
Android option. Only gap admission subtracts the raw receiver clock change;
epochs, observations, and thresholds remain unchanged. Invalid corrected
intervals fail closed. Added synthetic regression coverage for default
rejection, corrected admission with preserved timestamps, true excessive
gaps, nonmonotonic intervals, and nonfinite clock metadata.

Native build session 40126 is still running. The external raw_p_clock_test
target is configured but has not yet been built/run against the new libraries.
No v2 frozen executable or native replay exists yet. Original Pixel5 case 04
control completed successfully (727.397 s), candidate PID 27984 is running;
Samsung A325F candidate PID 22032 remains active. Group-safe audited progress
remains 4/15. The queued group-safe height-map replacement retains priority
for the next available Samsung inference slot.

### Additional Samsung clock comparison terminal (2026-09-26)

The Samsung supervisor exited 1 as expected because A205U failed admission;
A325F completed both arms successfully and its pair was audited independently.
A325F score is 2.005371153 -> 2.014966128 m (delta +0.009594975 m), with all
1,230 truth/native keys present. The single loader reference reset disappears,
but this does not establish a benefit for the no-main-D recipe. No blanket
promotion is justified. Pair evidence:
`use_cases/records/gsdc2023_continuous_clock_a325f_comparison_20260926.json`.
The planned group-safe Pixel5 height replacement started automatically in the
freed slot (control PID 14096); original case 04 candidate PID 27984 continues.

### V2 validation queued behind the verified native build (2026-09-26)

Validation supervisor session 91527 runs
`E:/rtklib_v2_ws_tmp/validate_continuous_clock_v2.ps1`. It waits for native
CMake PID 21224 (creation 13:28:53.660732 JST), then verifies the incremental
native build exit status, builds and runs the complete raw_p_clock_test suite,
and only on success freezes a new, unique continuous_raw_clock_v2_20260926.exe.
It will record source hashes, executable hash, and test-report hash in
`use_cases/records/gsdc2023_continuous_raw_clock_v2_build_20260926.json`.
This supervisor does not launch inference. Existing v1 and Pixel5 binaries are
unchanged. Next replay should be the previously failed A205U same-binary pair,
using the exact original recipe and inputs with v2; reserve its inference slot
only after the group-safe height replacement finishes. No v2 result is claimed
until the supervisor and subsequent native replay actually pass.

### Continuous-clock v2 built; full raw-P suite passes (2026-09-26)

Native build session 40126 and validation session 91527 both completed with
exit code 0. All 40 tests in test_raw_p_seed.cpp passed, including the new
clock-corrected gap admission test and existing time boundary/leading-state
checks. Frozen v2 SHA is
`7c5417bbc3a2d29f1ed7c7ace3ffbb91650fb8a8ce7595eac23813644b5d1316`.
The build/source/test hash record is
`use_cases/records/gsdc2023_continuous_raw_clock_v2_build_20260926.json`.
This proves synthetic coverage, not A205U native success or score improvement.

Prepared `continuous_raw_clock_v2_a205u_20260926/plan.json`, SHA
`ebb2c312f970751c6e7902943b69f333aef73c85ada481c17dff2d4721b95f90`.
It replays the predetermined A205U v1 admission failure using exactly the old
inputs/recipe with v2, both arms sharing the new executable. Supervisor session
54898 (`E:/rtklib_v2_ws_tmp/launch_continuous_clock_v2_a205u.ps1`) waits for
verified height-replacement supervisor PID 34424 (creation 13:21:22.233502 JST),
checks terminal state and a free native slot, then runs one worker, audits the
pair, and checks v2 default trajectory byte parity with the completed v1
control. The plan is prepared but native inference has NOT started yet.

Pixel5 case 04 comparison completed: score delta +0.010377030 m. Group-safe
progress is now 5/15 audited pairs, with no partial aggregate. Case 05 control
PID 32748 and repaired-height case 12 control PID 14096 are running. This
mixed result reinforces waiting for the complete fixed cohort before policy
promotion. No official submission or score change occurred.

### Test17 preparation now rejects unsafe development provenance (2026-09-26)

The Pixel5 test-plan preparer now requires the complete 15-case Pixel5 cohort
metadata and audits every height-map exclusion and source-plan provenance
before preparing a test plan. The original unsafe development plan is rejected
before creating output; the corrected composite passes. Prepared, but did NOT
execute, `pixel5_submitted_test_recipe_group_safe_20260926/plan.json`, SHA
`22ba1769b4fc3f9471a3a946250ee95392dafbd285b4b48fbf108e8998a0b74e`.

All seventeen paired inference recipes/input manifests match the old test
plan after normalizing only output paths; all twenty-three retained published
sources are unchanged, and all 71,936 submitted rows reconcile. The assembler
also rejects the current partial 5/15 development report before producing
output. Record: `use_cases/records/gsdc2023_pixel5_group_safe_test_plan_20260926.json`.
The old test plan record is marked superseded-never-executed, with its frozen
plan bytes retained. Neither test plan is authorized for automatic execution
or promotion by these preparatory checks; inspect the full fixed development
comparison first.

The group-safe height replacement control completed successfully in 635.406 s;
its candidate is next/running under the existing supervisor. A205U v2 remains
queued behind that supervisor. Original Pixel5 case 05 control PID 32748 is
still live. Official Private remains 1.055 m.

### Fixed eleven-phone joint source recipe prepared and conditionally queued (2026-09-26)

Prepared all eleven cases from the pre-existing expanded-bias cohort (G988B,
Pixel6 Pro, Pixel7 Pro, S908B), without score-dependent selection. This is a
new joint sigma/source-Huber/main-D experiment; the historical sigma-only
ablation was nearly neutral and is not being repeated unchanged. Both arms
use v2 continuous-reference clocks, upstream offset, extra bands, and the
same group-excluded height policy. The input-clock fix is therefore common,
not confounded with the three candidate flags. Eight maps and three relative
height cases were chosen using the established 10% coverage rule. May16
Pixel7 Pro's map excludes both same-drive aliases, unlike the old maps.

Plan `modern11_source_recipe_20260926/plan.json`, SHA
`7a69e33ad1fc34977c37052a2fc87c91a72e9a17d8f871c074fe858a4264ae8c`;
all eleven paired argv/input hashes/group exclusions pass preflight. The
shared native auditor now verifies actual continuous-clock telemetry whenever
that option is present in either arm; the existing A325F pair passes this
strengthened audit. No modern11 inference or result exists yet.

Supervisor session 37542 (`E:/rtklib_v2_ws_tmp/launch_modern11_source_recipe.ps1`)
waits for A205U v2 supervisor PID 24288 (13:42:58.152688 JST), and requires a
successful complete native pair, score audit, and byte-identical default
control before starting one worker. Any failure stops the queue. It also
checks no previous native child is live and at most one other native process
exists. Pixel5 remains the other worker; never start a third native inference.
Record: `use_cases/records/gsdc2023_modern11_source_recipe_plan_20260926.json`.

### Group-safe height replacement completed; A205U v2 starts (2026-09-26)

Replacement supervisor session 21361 completed with exit code 0. The fixed
May16 Pixel5 pair improved from 1.144799484 to 1.016689746 m (delta
-0.128109738 m), with all raw native keys and 2,322 evaluation truth keys
accounted for. The actual sigma/Huber/main-D effects and height-map evaluation
group exclusion pass the full one-pair scorer. This is development evidence,
not held-out or official performance. Report:
`use_cases/records/gsdc2023_pixel5_height_group_replacement_comparison_20260926.json`.
Composite progress is now 6/15 audited pairs; no partial aggregate is reported.
The unsafe original map remains quarantined and must never be restored.

A205U v2 supervisor 54898 started its control (PID 33268) in the freed slot;
its plan/executable remain the previously frozen ebb2c312/7c5417bb versions.
Original Pixel5 case 05 control completed successfully in 1062.819 s and its
candidate is now PID 18524. Two native processes remain active; modern11
supervisor 37542 still waits for the complete A205U v2 audit and default parity.
Official Private is unchanged at 1.055 m.

### A205U v2 clears the previous admission failure (2026-09-26)

V2 control completed in 187.748 s. Actual solution.csv AND summary.json hashes
match the v1 control exactly, establishing default behavior parity for this
real-data replay. The v2 continuous-reference candidate (PID 42724) passed the
previously failing seed admission: 1,209 independent SPP epochs plus the same
four explicitly supported native leading guesses account for all 1,213 states.
Its initialization metadata is accepted/graph-compatible, without imported
seeds or truth. raw_p_seed_ok remains false because the four leading epochs
still lack independent SPP; the existing supported adapter handles them, not
any new filling policy. Native factor construction proceeds. Final convergence,
raw-key coverage, and score are still pending; modern11 remains gated.
Evidence: `use_cases/records/gsdc2023_continuous_clock_v2_a205u_execution_20260926.json`.

### A205U v2 full pair passes; modern11 computation starts (2026-09-26)

Supervisor 54898 completed successfully. Candidate wall time 249.455 s; all
1,213 native raw keys and all 1,213 truth keys are audited, graph convergence
passes, and actual continuous-clock telemetry reports 0 reference resets
versus 1,208 in the control. Score: 1.916556366 -> 1.912326145 m, delta
-0.004230221 m. P50 improves by 0.009283 m but P95 worsens by 0.000822 m.
This validates the admission correction and near-neutral accuracy; it does
not substantiate a large official-score gain or blanket clock-only promotion.
Both control solution and summary byte parity with v1 were verified.
Full report: `use_cases/records/gsdc2023_continuous_clock_v2_a205u_comparison_20260926.json`.

The pre-registered modern11 supervisor 37542 then started one native worker:
runner PID 61500, first G988B control PID 65456. The frozen plan remains SHA
7a69e33a..., binary 7c5417bb..., with common continuous-clock/height/offset/extra
bands and candidate sigma+source-Huber+main-D. Original Pixel5 case 05 candidate
PID 18524 continues in the other slot. Neither cohort is fully scored; no
partial aggregate or official improvement is claimed.

### Pixel5 composite evaluation watcher active (2026-09-26)

Added `scripts/analysis/watch_gsdc_completed_pairs.py`. It never launches native
inference: it scores only complete selected pairs, reuses existing comparison
artifacts, updates the audited progress record on newly completed pairs, and
runs the full scorer only after all fifteen pairs and their source execution
manifests are complete. It checks frozen-plan identity and map/source-group
provenance, fails on any selected native failure, and holds a Windows handle
to the original runner so a stopped batch cannot be mistaken for a live wait.
The intentionally failed original case 12 is not a selected source pair.

Syntax check and a one-pass check against the actual 6/15 composite passed.
Watcher session 48833 is now active, observing original runner PID 18808.
Full output will be
`use_cases/records/gsdc2023_pixel5_group_safe_full_comparison_20260926.json`.
Do not start a duplicate watcher or manually rewrite comparison artifacts it
is producing. This automates audits only; it does not execute test17, assemble
a submission, publish, or change candidate policies. Native inference remains
two processes: Pixel5 case05 candidate and modern11 case00 control.

### Modern11 first completed pair: G988B joint recipe regresses (2026-09-26)

Both July14 G988B arms completed and passed native/output/hash/group-exclusion
and actual-factor audits. With the same continuous-clock and group-safe height
recipe in both arms, the joint source sigma/source Huber/main-D candidate scores
0.606051984 m versus control 0.564669084 m (delta +0.041382900 m).
P50: 0.384617709 -> 0.389747741 m; P95: 0.744720459 -> 0.822356227 m.
All 1,165 raw/truth epochs are accounted for. Candidate telemetry confirms
sigma 0.001153494 m (control 0.03), Huber 0.2 (control 4), and 12,762 main-D
factors (control 0). This is a real small regression after the clock fix, not
the old roughly 5 m clock-reset failure. Do not promote the joint policy from
this result or select route winners; continue the predetermined eleven cases.

Evidence: `modern11_source_recipe_20260926/runs/00/comparison.json` and
`use_cases/records/gsdc2023_modern11_source_recipe_progress_20260926.json` (1/11,
no partial aggregate). This pair was scored with the watcher in one-pass mode;
no second persistent watcher was started, avoiding a race with the modern11
supervisor's final full scorer. Next Pixel6 Pro control PID 25956 is live;
original Pixel5 case05 candidate PID 18524 continues and its existing watcher
48833 owns its comparison output. Official Private remains 1.055 m.

### Pixel5 case 05 completed and watcher verified seven pairs (2026-09-26)

August24 Pixel5 joint recipe improves 0.644545664 -> 0.521612640 m (delta
-0.122933024 m). P50 0.438325349 -> 0.388337419 m, P95 0.850765978 ->
0.654887862 m; all native raw keys and 3,139 truth keys pass the comparator.
Candidate wall time was 1510.986 s. Watcher 48833 produced the comparison and
completed the full provenance/hash/native-effect/group-exclusion progress
audit: 7/15 pairs, with no partial aggregate. Evidence remains the current
`use_cases/records/gsdc2023_pixel5_group_safe_progress_20260926.json` and
`pixel5_source_recipe_20260926/runs/05/comparison.json`.

Original case06 January26 Pixel5 control PID 58336 is now running. Modern11
case01 May13 Pixel6 Pro control PID 25956 is the second native process. The
previous modern11 G988B comparison remains the sole completed modern11 pair;
no policy promotion or official-score change has been made.

### Offline forty-case directional-error diagnostic (2026-09-26)

Added and ran `scripts/analysis/diagnose_gsdc_directional_error.py` against all
40 completed frozen corrected-recipe outputs. Every solution/summary hash,
convergence marker, native raw-key count, and truth-key alignment is checked.
Velocity/direction comes only from adjacent native coordinates and timestamps;
truth is used only to measure errors afterward. No native inference was run,
no coordinates were changed, and no fitted coefficient enters any solver.

The baseline predates the upstream phone position-offset and height policy,
so its constant directional errors cannot be treated as new current-policy
errors. Descriptive along-error versus speed slopes have mixed signs: Pixel5
has 5 positive and 10 negative route slopes (median -0.00603 s); Pixel7 Pro has
2 positive and 3 negative (-0.00478 s); Mi8 has 3 positive and 2 negative
(+0.00073 s). One/two-route Samsung estimates are insufficient to identify a
phone-wide timing delay. This does not support adding a blanket time shift.
Route/multipath/antenna-offset confounding remains; continue the already frozen
joint-factor comparisons rather than applying these descriptive fits.
Record: `use_cases/records/gsdc2023_train40_directional_error_20260926.json`.
Current inference remains Pixel5 case06 control and modern11 case01 control.

### Modern11 Pixel6 Pro control reproduces historical height trajectory (2026-09-26)

Case01 May13 Pixel6 Pro control completed successfully in 774.258 s. Native
artifact audit passes with 2,180 exact raw keys, converged graph, truth_used
false, and zero continuous-reference resets. Its actual solution.csv is byte
identical to `train11_heightmap_v1/01/solution.csv` (SHA 9e6f6bed...), confirming
that the common v2 clock convention and regenerated group-safe height policy
preserve this no-reset case's historical trajectory. Candidate PID 584 is now
running. Pixel5 case06 control PID 58336 remains the other native process.
No new paired score or promotion is implied by this single-arm check.

### Pixel6 Pro and January26 Pixel5 pairs improve (2026-09-26)

Modern11 case01 May13 Pixel6 Pro joint recipe passed the complete native,
clock-mode, factor-effect, input/output-hash, and height-group audit. Score
0.618045078 -> 0.482040421 m (delta -0.136004657 m); P50 0.455041321 ->
0.376047906, P95 0.781048835 -> 0.588032936. All 2,180 native raw keys and
2,162 truth keys are present. Candidate wall time 816.377 s. Modern11 progress
is 2/11 audited pairs with no partial aggregate. Next case02 May13 G988B
control PID 29140 runs under the existing supervisor.

Pixel5 case06 January26 also completed and was independently processed by
watcher 48833: 0.377190017 -> 0.357164006 m (delta -0.020026011 m), with all
1,698 raw/truth epochs. P50 slightly worsens 0.257998702 -> 0.258509152, while
P95 improves 0.496381332 -> 0.455818861. Candidate wall time 747.669 s. The
watcher audited the composite progress to 8/15; no partial mean is reported.
Next original case07 February24 lax-o Pixel5 control PID 65708 is running.

Evidence: the two cohorts' runs/01 and runs/06 comparison.json respectively,
and the existing modern11_source_recipe_progress and pixel5_group_safe_progress
records. These exposed development improvements do not alter the official
Private 1.055 m score; both fixed cohorts must finish before policy decisions.

### Modern11 May13 G988B control also reproduces historical output (2026-09-26)

Case02 control completed in 718.650 s; native artifact audit verifies 2,182
exact raw keys, convergence, and truth_used=false. Its actual trajectory is
byte identical to `train11_heightmap_v1/02/solution.csv` (SHA 1fcbbb38...).
This is a second no-reset real-data parity check for the common v2 clock and
regenerated height setup. Candidate PID 34448 now runs; Pixel5 case07 control
PID 65708 remains live. No new paired score is available yet.

### Modern11 May13 G988B joint recipe improves (2026-09-26)

Case02 candidate completed successfully in 829.849 s. The comparison and
frozen-plan progress audit passed: score 0.892321729 -> 0.740056246 m
(delta -0.152265483 m), P50 0.615723745 -> 0.612194937 m, and P95
1.168919714 -> 0.867917555 m. Both runs retain all 2,182 native raw keys;
evaluation covers 2,163 truth keys. Input/output hashes, factor effects,
continuous-clock telemetry, and height-group independence were validated.

Evidence: modern11_source_recipe_20260926/runs/02/comparison.json and
use_cases/records/gsdc2023_modern11_source_recipe_progress_20260926.json.
Progress is 3/11 audited pairs, with no partial aggregate or policy promotion.
The existing supervisor advanced to case03 November15 Pixel7 Pro control
(PID 38384); Pixel5 case07 control (PID 65708) remains the other native run.
The official Private score is unchanged at 1.055 m.

### Modern11 November15 Pixel7 Pro control parity (2026-09-26)

Case03 control completed in 264.816 s. The native artifact auditor verifies
all 1,231 raw UTC keys, convergence, truth_used=false, continuous-clock
telemetry, and input/output hashes. Its solution.csv is byte identical to
train11_heightmap_v1/03/solution.csv. Candidate PID 62176 has started under
the existing supervisor; Pixel5 case07 control PID 65708 remains live.
This is a single-arm parity result, not a new paired score or promotion.

### Modern11 November15 Pixel7 Pro joint recipe regresses (2026-09-26)

Case03 candidate completed in 491.171 s and passed the native artifact,
factor-effect, continuous-clock, input/output-hash, and height-group audits.
Score worsens 0.464387009 -> 0.527052500 m (delta +0.062665491 m).
P50 improves 0.319768966 -> 0.307917901 m, but P95 worsens
0.609005051 -> 0.746187098 m. Both trajectories contain all 1,231 raw keys;
evaluation uses 1,193 truth keys. This is a real tail-error regression in
the fixed joint-factor comparison, not missing epochs or failed convergence.

Evidence: modern11_source_recipe_20260926/runs/03/comparison.json and
use_cases/records/gsdc2023_modern11_source_recipe_progress_20260926.json.
Progress is 4/11 audited pairs; no partial aggregate or route-specific
selection is used. The supervisor continues case04 March08 Pixel6 Pro
control (PID 51688). Pixel5 case07 control PID 65708 remains live.
Official Private remains 1.055 m; no policy has been promoted or submitted.

### Modern11 March08 Pixel6 Pro control audit (2026-09-26)

Case04 control completed in 682.930 s. The native artifact auditor verifies
all 1,102 raw UTC keys, convergence, truth_used=false, continuous-clock
telemetry, and input/output hashes. No historical byte-parity claim is made:
train11_heightmap_v1/04/solution.csv does not exist at the checked location.
Candidate PID 65456 started under the existing supervisor. Pixel5 case07
control PID 65708 remains live. Paired progress remains 4/11; this single-arm
audit does not establish an accuracy change or alter the official score.

### Modern11 March08 Pixel6 Pro joint recipe improves (2026-09-26)

Case04 candidate completed in 387.415 s. Comparison and frozen-plan progress
audits passed, including native raw keys, convergence, factor effects,
continuous-clock telemetry, input/output hashes, and height-group independence.
Score improves 0.947582764 -> 0.817610733 m (delta -0.129972032 m).
P50 improves 0.714737163 -> 0.603049559 m and P95 improves
1.180428366 -> 1.032171906 m. Both trajectories and evaluation contain all
1,102 epochs. Evidence: modern11_source_recipe_20260926/runs/04/comparison.json
and use_cases/records/gsdc2023_modern11_source_recipe_progress_20260926.json.

Progress is 5/11 audited pairs, with no partial aggregate or route selection.
The supervisor advanced to case05 May16 Pixel7 Pro control (PID 50096), using
the frozen height map that excludes both May16 same-drive aliases. Pixel5
case07 control PID 65708 remains live. Official Private remains 1.055 m;
no policy has been promoted or submitted.

### Modern11 May16 Pixel7 Pro control audit (2026-09-26)

Case05 control completed in 940.663 s. The native artifact auditor verifies
all 2,323 raw UTC keys, convergence, truth_used=false, continuous-clock
telemetry, and input/output hashes. The frozen-plan height-independence and
provenance checks also pass for this May16 case, with both same-drive aliases
excluded. Candidate PID 29968 started under the existing supervisor; Pixel5
case07 control PID 65708 remains live. Paired progress remains 5/11. This
single-arm check establishes no accuracy improvement or official score change.

### Modern11 May16 Pixel7 Pro joint recipe improves (2026-09-26)

Case05 candidate completed in 820.628 s. Comparison and frozen-plan progress
audits passed, including native raw keys, convergence, factor effects,
continuous-clock telemetry, input/output hashes, and height-group independence.
Score improves 0.636489016 -> 0.577922279 m (delta -0.058566738 m).
P50 improves 0.440379438 -> 0.365591994 m and P95 improves
0.832598595 -> 0.790252563 m. Both trajectories and evaluation contain all
2,323 epochs. Both arms use the same group-safe map excluding both May16
same-drive aliases; this comparison does not reuse the contaminated old map.

Evidence: modern11_source_recipe_20260926/runs/05/comparison.json and
use_cases/records/gsdc2023_modern11_source_recipe_progress_20260926.json.
Progress is 6/11 audited pairs, without partial aggregate or route selection.
The supervisor advanced to case06 May24 Pixel7 Pro control (PID 20252).
Pixel5 case07 control PID 65708 remains live. Official Private remains
1.055 m; no policy has been promoted or submitted.

### Modern11 May24 Pixel7 Pro control audit (2026-09-26)

Case06 control completed in 335.266 s. The native artifact auditor verifies
all 1,384 raw UTC keys, convergence, truth_used=false, continuous-clock
telemetry, and input/output hashes. Candidate PID 41740 started under the
existing supervisor; Pixel5 case07 control PID 65708 remains live. Paired
progress remains 6/11. This single-arm check establishes no accuracy change
or official score improvement.

### Modern11 May24 Pixel7 Pro joint recipe slightly improves (2026-09-26)

Case06 candidate completed in 480.776 s and passed the comparison and
frozen-plan progress audits, including keys, convergence, factor effects,
continuous-clock telemetry, input/output hashes, and height-group independence.
Score improves 0.443964063 -> 0.431920802 m (delta -0.012043261 m).
P50 improves 0.340356560 -> 0.291452312 m, while P95 worsens
0.547571565 -> 0.572389291 m. Both trajectories retain all 1,384 raw epochs;
evaluation covers 1,383 truth epochs. Evidence is the case06 comparison.json
under modern11_source_recipe_20260926 and the modern11 progress record.

Progress is 7/11 audited pairs, with no partial aggregate or route selection.
The supervisor advanced to case07 May25 SM-S908B control (PID 64496).
Pixel5 case07 control PID 65708 remains live. Official Private remains
1.055 m; no policy has been promoted or submitted.

### Complete development cohorts audited; Pixel5 test execution started (2026-09-27)

Both development supervisors finished while the conversation was idle. The
Pixel5 watcher and modern11 supervisor exited successfully. All selected
development pairs were re-audited against current artifacts on September27.
Pixel5 fixed15: mean 0.736483004 -> 0.644028218 m, delta -0.092454786 m,
13 improved and two regressed. Modern fixed11: mean 0.699758890 ->
0.628204040 m, delta -0.071554850 m, nine improved and two regressed.
These are exposed development results, not held-out or official scores.
Both complete reports verify native factor effects and height-group
independence. Pixel5's original case12 preflight failure is expected: the
selected composite uses the previously completed group-safe replacement.

Evidence: use_cases/records/gsdc2023_pixel5_group_safe_full_comparison_reaudit_20260927.json
and use_cases/records/gsdc2023_modern11_source_recipe_reaudit_20260927.json.
The full Pixel5 cohort supports proceeding with the already frozen test17
plan, without choosing per-route winners. Plan SHA 22ba1769b4fc3f9471a3a946250ee95392dafbd285b4b48fbf108e8998a0b74e
is unchanged. After confirming no native processes remained, launched the
test pairs with two workers under exec session67526. The supervisor script
is E:/rtklib_v2_ws_tmp/launch_pixel5_test_group_safe_20260927.ps1.

After all17 pairs succeed, the supervisor invokes the existing assembler,
which requires byte-identical reproduction of published control sources,
23 unchanged native sources, and exact 71,936 output keys. Its output is
E:/rtklib_v2_ws_output/gsdc_native/pixel5_joint_submission_20260927.
This is local inference and candidate preparation under the continuing goal;
the supervisor has no submission command. Official Private remains 1.055 m.

### Pixel5 test first four pairs audited (2026-09-27)

The frozen test17 runner (PID58468, supervisor session67526) remains active.
Four pairs completed: August17 mtv-g, February08 sjc-r, February23 lax-n,
and February23 lax-m. The native artifact and factor-effect audits pass for
all four. Five completed controls, including February24 lax-i, reproduce
their published native source solution.csv byte for byte. Current processes
are February24 lax-p control PID42212 and lax-i candidate PID32628.

Candidate-versus-control displacement P95 values are 0.329791, 0.248122,
0.704829, and 0.611975 m respectively; maximum displacement across these
four is 1.560016 m. These are trajectory differences, not accuracy estimates.
No test truth is loaded or scored. Evidence:
use_cases/records/gsdc2023_pixel5_test_progress_20260927.json, produced by
E:/rtklib_v2_ws_tmp/audit_pixel5_test_progress_20260927.py. Final assembly
still requires all17 pairs and the full existing assembler audit.

### Pixel5 test lax-i pair audited (2026-09-27)

February24 lax-i candidate completed in 1,350.665 s. Native artifact and
factor-effect checks pass with all 3,582 raw keys, convergence, no truth use,
and a control trajectory byte identical to its published source. Candidate
versus control displacement is P50 0.223103 m, P95 0.742002 m, maximum
1.229382 m; no accuracy claim follows from these differences. The test
progress record now contains 5/17 audited pairs. The same runner continues
February24 lax-p control PID42212 and March22 mtv-pe1 control PID51932.
Supervisor session67526 remains live. No submission has been assembled yet.

### Pixel5 test March22 control reproduced (2026-09-27)

March22 mtv-pe1 control completed in 670.766 s. The refreshed test progress
artifact audit passes: six completed controls reproduce their published
native source solutions byte for byte, with five complete candidate pairs.
The same two-worker runner has advanced March22 to its candidate arm while
February24 lax-p control remains running. No test accuracy is evaluated;
official Private remains 1.055 m. Evidence is the refreshed
use_cases/records/gsdc2023_pixel5_test_progress_20260927.json.

### Modern test cohort source audit prepared (2026-09-27)

While the Pixel5 two-worker queue remains active, audited all nine test
phones matching the four models in the fixed modern development cohort.
The new audit_gsdc_modern_test_sources.py checks recorded published source
hashes, executable hashes, convergence, no truth use, and exact raw UTC keys
without interpolation or holds. All nine pass. This does not establish
replay parity or candidate accuracy and starts no additional inference.

November05 Pixel6 Pro is the only one of these nine with a legacy loader
reference reset. Its existing loader-only admission artifacts were rehashed:
both modes preserve the same 1,446 keys, while maximum input clock step
changes from 21,770.629 to 157.391 m. Full continuous-mode solver execution
is still required. A subsequent frozen experiment must distinguish the
continuous-clock change from the three recipe flags, retaining a published
control replay; the Pixel5 plan and candidate assembly remain unchanged.
Evidence: use_cases/records/gsdc2023_modern9_test_source_audit_20260927.json.

### Modern9 three-arm test plan frozen, not started (2026-09-27)

Prepared modern9_submitted_test_three_arm_20260927/plan.json under the native
output root; SHA256 6c21ba66ac1692c7a763118b9e392e480e9baf0de25ad2fe0248199c70663565.
All nine model-matched test phones receive three same-binary arms:
published settings, continuous-clock only, then continuous-clock plus the
three uniformly selected source-recipe flags. Binary is frozen continuous
raw clock v2 (7c5417bbc3a2d29f1ed7c7ace3ffbb91650fb8a8ce7595eac23813644b5d1316).
This separates clock changes from recipe changes, including November05
Pixel6 Pro's loader reset. Height settings retain published per-phone policy.

prepare_gsdc_modern_test_recipe.py verified complete development evidence,
height-group exclusion, the source audit evidence, published source hashes,
and all declared native inputs before writing the plan. An independent argv
check verified all 27 commands differ only by declared arm flags and output
paths. No execution.started.json exists. A three-arm-aware runner and complete
native/parity audits are still needed; the existing runner supports pairs only.
Do not pass this plan to it unchanged. The active Pixel5 two-worker experiment
continues unmodified, with no third native process launched and no submission.

### Three-arm runner support verified (2026-09-27)

run_gsdc_frozen_pairs.py now honors an explicitly declared control,
clock_only, candidate arm order, retaining the existing two-arm default.
It rejects undeclared or mismatched arms before creating the execution
marker and requires every declared arm for completion. This supersedes the
previous note that a three-arm-aware runner was still missing. Existing
Pixel5 Python workers already loaded their code and continue unchanged.

Five simulated-child lifecycle tests pass in tests/test_gsdc_frozen_runner.py:
legacy pair completion and exclusive restart rejection, three-arm ordering
and completion, clock-only failure preventing candidate launch, changed
input rejection before child launch, and undeclared third-arm rejection.
These tests launch no GNSS inference and establish no positioning accuracy.
Modern9 remains prepared, not started; capacity is still occupied by Pixel5.

### Pixel5 test March22 pair audited (2026-09-27)

March22 mtv-pe1 candidate completed in 806.916 s. The refreshed artifact
and factor-effect audits pass for all 2,112 native raw UTC keys, convergence,
no truth use, and a control solution byte identical to its published source.
Candidate versus control displacement is P50 0.166683 m, P95 0.501901 m,
maximum 0.630442 m. These trajectory differences do not establish accuracy.
The progress record now contains 6/17 audited pairs. The same two-worker
runner continues; no submission has been assembled or sent. Official
Private remains 1.055 m. Evidence: use_cases/records/gsdc2023_pixel5_test_progress_20260927.json.

### Modern9 three-arm output auditor prepared (2026-09-27)

Added audit_gsdc_modern_test_recipe.py. It requires the complete nine-case,
three-arm terminal manifest, frozen provenance and exact argv/input/binary
hashes, full native artifact audits, published-control solution hash parity,
continuous-clock telemetry and native recipe effects. It reports clock-only
versus control, recipe versus clock-only, and total candidate displacement
separately, with no test truth or accuracy claim. It creates no submission.

The actual not-yet-started plan is rejected with "native experiment not
finished" and no output report is created. This checks only the incomplete
execution gate; successful full auditing remains pending the 27 real runs.
Pixel5's live two-worker run is still occupying both native inference slots.

### Pixel5 test lax-p published control reproduced (2026-09-27)

February24 lax-p control finished successfully in 7,168.777 s. The refreshed
native artifact audit confirms full raw key coverage, convergence, no truth
use, and a solution byte identical to the published source. Seven completed
controls now reproduce their published solutions; complete pairs remain 6/17.
The existing runner advanced lax-p to its candidate arm while April04 lax-x
control continues. Evidence: use_cases/records/gsdc2023_pixel5_test_progress_20260927.json.
No official accuracy claim or submission follows from this control replay.

### Pixel5 test lax-x published control reproduced (2026-09-27)

April04 lax-x control completed successfully in 4,185.967 s. The refreshed
native artifact audit confirms full raw key coverage, convergence, no truth
use, and a solution byte identical to its published source. Eight completed
controls now reproduce published solutions; complete pairs remain 6/17.
The same two-worker runner advanced lax-x to its candidate while lax-p
candidate continues. Evidence: use_cases/records/gsdc2023_pixel5_test_progress_20260927.json.
No candidate assembly or new official result exists yet.

### Pixel5 test lax-x pair audited (2026-09-27)

April04 lax-x candidate completed in 1,156.991 s and passes the native and
factor-effect audits, with all 2,171 raw UTC keys, convergence, no truth use,
and a published-control byte-identical replay. Progress is now 7/17 complete
pairs and eight reproduced controls. Candidate solution SHA256 is
493e54a978b3953d3afd01ad970cdcfaf230cb3f2687cc6337c438999c2726ff.

Candidate-versus-control displacement is P50 0.329908 m, P95 3.301476 m,
maximum 4.043247 m. This is larger than earlier completed pairs and should
receive trajectory/solver-diagnostic review before assembly is presented.
It is not test accuracy and does not justify choosing a per-route winner.
Evidence: use_cases/records/gsdc2023_pixel5_test_progress_20260927.json.
The existing runner continues lax-p candidate and the next planned control;
no submission or official score change has occurred.

### Lax-x larger trajectory disagreement diagnosed without truth (2026-09-27)

The read-only diagnostic gsdc2023_pixel5_test_laxx_displacement_diagnostic_20260927.json
re-audits both native artifacts. Maximum disagreement 4.043247 m occurs at
epoch130 (UTC1649089992434). Disagreement above2 m is confined to epochs1-3
and10-202, chiefly the first203 seconds. All2,171 epochs are exactly one second
apart. The largest adjacent change in the displacement vector is0.406809 m,
so the4 m magnitude is not a single-epoch4 m step.

Both arms have zero loader clock resets, zero C0D clock/gap skips, and exactly
identical raw-P initialization bytes. Both stop by outer convergence tolerance
with zero indeterminate solves. This does not reproduce July14's artificial
FullBias reset failure. GNSS-first summaries differ in iterations and costs;
the combined recipe also changes TDCP weighting, so it does not isolate main
Doppler causality. Costs across differently weighted graphs are not comparable.
TDCP RMS is0.214769 ->0.216267 m; both have their largest TDCP residual at
2107->2108, far from the early large-disagreement segment. Output maximum
speed is27.518399 ->27.519378 m/s. These diagnostics establish neither test
accuracy nor a preferred arm. No settings, route selection, or candidate
coordinates were changed; the frozen experiment continues.

### Pixel5 test ebf-y published control reproduced (2026-09-27)

April22 ebf-y control completed successfully in454.702 s. The refreshed
native artifact audit confirms exact raw UTC keys, convergence, no truth use,
and a solution byte identical to its published source. Nine completed
controls now reproduce published solutions; complete pairs remain7/17.
The same two-worker runner advanced ebf-y to its candidate while lax-p
candidate continues. Evidence: use_cases/records/gsdc2023_pixel5_test_progress_20260927.json.
No candidate assembly or new official result exists yet.

### Pixel5 test ebf-y pair audited (2026-09-27)

April22 ebf-y candidate completed in453.153 s. Native artifact and factor-effect
audits pass with all1,400 raw UTC keys, convergence, no truth use, and published
control byte parity. Progress is8/17 pairs and nine reproduced controls.
Candidate-versus-control displacement is P50 0.198197 m, P95 0.520647 m,
maximum0.898955 m; these are not accuracy estimates. Candidate solution SHA256
b3443bf3c8fdd7d1ededb297284df57f493a4c134fc3bddeba1a0db9f374e7d1.
Evidence: use_cases/records/gsdc2023_pixel5_test_progress_20260927.json.
The same runner continues lax-p candidate and the next planned control.
No candidate assembly or new official result exists yet.

### Pixel5 test ebf-z published control reproduced (2026-09-27)

April25 ebf-z control completed successfully in320.901 s. The refreshed
native artifact audit confirms exact raw UTC keys, convergence, no truth use,
and a solution byte identical to its published source. Ten completed
controls now reproduce published solutions; complete pairs remain8/17.
The existing two-worker runner advanced ebf-z to its candidate while lax-p
candidate continues. Evidence: use_cases/records/gsdc2023_pixel5_test_progress_20260927.json.
No candidate assembly or new official result exists yet.

### Pixel5 test lax-p pair audited; large tail disagreement needs investigation (2026-09-27)

Lax-p candidate completed in5,049.526 s. Native artifact and factor-effect
audits pass with all4,515 raw UTC keys, convergence, no truth use, and published
control byte parity. Progress is9/17 pairs and ten reproduced controls.
Candidate SHA256 fd752bea5499963abbb046a4e2a907d401da511cb9dc808fb87679fb266ddc17.
Disagreement P50 0.529192 m, P95 6.258649 m, maximum350.199980 m is material:
passing provenance/key checks alone is not sufficient for submission review.

The read-only lax-p diagnostic localizes the maximum to epoch4499; the final
265 epochs (4250-4514) have disagreement above2 m, with other smaller mid-run
segments. All epochs are one second apart, both loader reset counts and C0D
gap/clock skips are zero, and raw-P initialization bytes are identical. Thus
this is not the July14 FullBias-reset failure. Both arms converge with no
indeterminate solves, but maximum optimized acceleration bias is already
18.493567 m/s2 in the published control and18.625443 m/s2 in the candidate.
Maximum output step speed is54.394738 and43.643655 m/s respectively. TDCP
RMS is1.257488 and1.035460 m, with residual maxima near the tail. These facts
do not identify the more accurate trajectory. Investigate tail IMU/observable
support before presenting a candidate; do not select a per-route winner from
trajectory differences. The active frozen test queue continues unchanged.
Evidence: gsdc2023_pixel5_test_progress_20260927.json and
gsdc2023_pixel5_test_laxp_displacement_diagnostic_20260927.json in use_cases/records.

### Lax-p raw sensor tail and stationary pose contract (2026-09-27)

Read-only raw sensor summaries rehash both input files against the completed
candidate run. No truth or device positions are consumed. During the last
35 seconds, median satellite count is21, median C/N0 is32.8 dB-Hz, and ADR
valid fraction is0.858; IMU sample gaps remain below30 ms. Acceleration norm
P95 is10.221 m/s2, while gyro norm P95 rises to1.371 rad/s and mean measured
acceleration changes from predominantly the Y axis toward the Z axis. This
supports a late phone reorientation hypothesis, not a demonstrated cause of
the trajectory disagreement (which starts earlier, at epoch4250).

Code inspection finds full identity Pose3 between-factors for admitted stops
in src/algorithms/fgo_gtsam_backend.cpp. However, upstream_stop_constraints.hpp
requires acceleration/gyro norm moving standard deviations below adaptive
thresholds AND instantaneous gyro norm below0.05 rad/s. Thus reorientation
does not by itself establish erroneous stop admission. Exact epoch-level
stop admission and attitude/bias behavior remain to be checked before any
solver change. The frozen queue, binary, and route policy are unchanged.
Evidence: use_cases/records/gsdc2023_pixel5_test_laxp_sensor_tail_20260927.json.

### Pixel5 test ebf-z pair and ebf-zz control audited (2026-09-27)

The refreshed artifact audit passes10/17 complete pairs and11 reproduced
published controls. April25 ebf-z candidate completed in508.021 s with1,587
raw UTC keys; disagreement P50 is0.183000 m, P95 0.491593 m, maximum0.541775 m.
Candidate SHA256 is67343bf2324d7c7719510a13156b7cc87d9038724f7e0e13942fec89603c932a.
April27 ebf-zz control completed in379.966 s and reproduces its published
solution bytes. The existing two-worker queue continues ebf-zz candidate
and ebf-xx control. No accuracy was evaluated on test data and no submission
was made. Evidence: use_cases/records/gsdc2023_pixel5_test_progress_20260927.json.

### Lax-p stationary detector replay narrows the hypothesis (2026-09-27)

A read-only Python replay uses raw IMU measurements with first-UTC deduplication,
gyro-clock acceleration interpolation, the500-sample centered N-1 standard
deviation, and nearest UTC epoch mapping. All229,839 paired samples and1,619
stop epochs match native aggregate telemetry; both adaptive thresholds agree
within1e-9. No truth is consumed. This is a detector replay before the speed
gate, not a reconstruction of all admitted pose factors.

There are no stop epochs4250-4349 or4450-4479. In the final35 seconds, only
epochs4498-4505 pass. These measurements are already nearly motionless in the
new orientation (acceleration predominantly positive Z, gyro norm below0.01
rad/s at those epoch samples). Thus the maximum trajectory difference at4499
coincides with a plausible post-reorientation stop, not demonstrated freezing
through the rotation itself. Removing stop constraints is not justified by
this evidence. Inspect attitude initialization and bias evolution next; saved
summary telemetry alone does not expose their per-epoch optimized states.
Evidence: use_cases/records/gsdc2023_pixel5_test_laxp_stop_replay_20260927.json.
Replay helper: E:/rtklib_v2_ws_tmp/replay_laxp_stop_20260927.py.

### Lax-p initialization and saved-state limitations (2026-09-27)

The raw replay now reconstructs rotation-invariant norms from the first250
paired IMU samples, following alignStatic and the app's gravity9.80665 m/s2:
acceleration mean norm9.758790082722, initial acceleration bias norm0.047859917278
m/s2, initial gyro bias norm0.001011320508 rad/s. The gravity norm matches
the native summary. Therefore the roughly18.5 m/s2 optimized maximum is not
already present in the static bias seed.

The frozen run has source initialization and per-epoch velocity-heading
attitude seeds disabled. The app retains alignStatic biases but replaces
initial attitude with the first GNSS-first velocity-derived RPY; the backend
then propagates per-epoch attitude seeds through preintegrated deltaRij.
This identifies initialization sensitivity as an open hypothesis, not proof
of an incorrect optimum. The backend currently records only maximum optimized
bias norms; per-epoch optimized attitude exists in memory but is not saved
by this ordinary run. A diagnostic rerun/export is required to locate the
bias growth. No solver or frozen experiment changes were made.

### Opt-in optimized IMU state export prepared (2026-09-27)

Added --native-imu-state-diagnostic for the Phase171 main IMU entrypoint.
The flag requests post-solve bias vectors from the GTSAM backend and writes
optimized_imu_states in the ordinary summary: graph epoch index/raw UTC,
Rot3::rpy radians, ENU velocity, and body-FLU acceleration/gyro biases.
It is disabled by default, changes no factors or initial states, and never
feeds exported values back into inference. Serialization rejects incomplete,
nonfinite, fallback, or unconverged state exports.

Extended the existing synthetic Phase171 handoff test to solve with export
off/on, require exact position/attitude/cost/iteration parity, and compare
exported bias maxima to aggregate diagnostics. Build of gnss_fgo_imu_no_base
and gnss_run_tests started (session21622); log is
E:/rtklib_v2_ws_tmp/imu_state_diagnostic_build_20260927.log. Tests and real-data
export/trajectory parity remain pending. This new build is not eligible for
candidate inference until validated. Existing frozen binaries and the active
two-worker Pixel5 queue are unchanged. Diagnostic lax-p reruns must wait for
native capacity and use separate output directories.

### Diagnostic auditor prepared; ebf-zz pair audited (2026-09-27)

scripts/analysis/audit_gsdc_imu_state_diagnostic.py checks completed native
manifests, input hashes, inference options, exact frozen trajectory byte
parity, graph epoch indices/UTC keys, finite3-vectors, and bias maxima against
native aggregates. It reports first/last/maximum bias norms and first/last
epochs above fixed diagnostic thresholds; it has no truth input. A synthetic
valid export and four corruption cases (wrong UTC, nonfinite attitude,
inconsistent aggregate, estimator feedback enabled) passed their expected
accept/reject checks. Actual native export audit remains pending the build
and a capacity-safe diagnostic rerun. Build session21622 remains live.

The completed Pixel5 test audit is now11/17 pairs, with11 reproduced controls.
April27 ebf-zz candidate finished in540.600 s with1,315 keys, solution SHA256
ffbd7f68330912465127bb47d88b3c7f0ffa5531d8b875bebbabe2c050b3a6f4.
Disagreement P50=0.189854 m, P95=0.658694 m, maximum=0.738682 m; these are
not accuracy estimates. Artifact audit session51847 completed successfully.
The frozen queue session67526 remains live; no submission was performed.

### Pixel5 ebf-xx control reproduced; diagnostic build verified live (2026-09-27)

April27 ebf-xx control completed in765.980 s. Artifact audit session53845
finished successfully:11/17 complete pairs and12 controls reproducing the
published solution bytes, with exact raw keys and no truth use. The existing
runner advanced to its candidate. The diagnostic build session21622 remains
live in code generation; compiler PID12012 CPU advanced from308.66 to516.89 s.
No restart was attempted. Synthetic C++ and real-data diagnostic checks remain
pending build completion. The export auditor now also requires the replay's
source_run_sha256 to match the exact reference run manifest.

### Pixel5 April2023 mtv-pe1 control reproduced (2026-09-27)

2023-04-27-19-25-us-ca-mtv-pe1/pixel5 control completed in459.214 s.
Artifact audit session12267 completed successfully, bringing reproduced
published controls to13 and complete pairs to11/17. The existing frozen
runner continues the candidate. Diagnostic build session21622 is still
live in compiler code generation; new-binary C++ tests have not yet run.

### Pixel5 development bias inventory (2026-09-27)

Read all30 completed summaries from the pinned group-safe15-case development
plan; run argv, completion, summary hashes, no-truth flags, and convergence
were checked. No truth was loaded or rescored. Every optimized acceleration
bias maximum is below0.081 m/s2. The largest is0.080705 m/s2 for January04
highway candidate; the long lax-o pair is0.051883 ->0.053883 m/s2. Thus these
development cases do not reproduce lax-p's roughly18.5 m/s2 state anomaly.
Do not interpret the15-case development improvement as validation of that
failure mode. Evidence: use_cases/records/gsdc2023_pixel5_development_bias_inventory_20260927.json.

### Diagnostic build parser limit corrected (2026-09-27)

Build session21622 ended with exit1 after successfully compiling/linking
gnss_lib_solvers. MSVC C1061 in the app reported excessive block nesting:
the added diagnostic else-if crossed the existing long parser chain limit.
Moved only the new flag to a standalone if/continue beside other standalone
flags. Rebuild session5807 is running; log
E:/rtklib_v2_ws_tmp/imu_state_diagnostic_build_retry_20260927.log.
The original failure log is retained. No new binary has passed validation yet.
The frozen test queue continues; ebf-xx candidate completed in653.766 s and
its artifact audit is now running.

### Diagnostic app built; targeted test rebuild underway (2026-09-27)

The parser fix compiled and linked gnss_fgo_imu_no_base successfully.
Invoking --native-imu-state-diagnostic alone rejects missing Phase171 main
graph with exit2 and the intended message. The binary is frozen at
E:/rtklib_v2_ws_output/gsdc_native/binaries/imu_state_diagnostic_20260927.exe,
SHA2560630b4eb46432f96db589ee7deba2dc17dddc001f7b34581ae27c9fbf2332575.
This is built but not yet validated for real-data diagnostic use.

The combined build session5807 ended exit1 because gnss_run_tests explicitly
depends on unrelated native apps, including gnss_pos_vel_pdc.cpp, whose
unistd.h include fails on Windows. After that build terminated, started
gnss_run_tests with /p:BuildProjectReferences=false against the already-built
libraries (session68021, log E:/rtklib_v2_ws_tmp/imu_state_diagnostic_tests_build_20260927.log).
No unrelated source changes were made to accommodate this build dependency.

Artifact audit6393 passed ebf-xx:12/17 pairs and13 reproduced controls,
1,382 raw keys, candidate SHA1d7275922bcfbdb355c41224e6e2ab42429700c53997d56e9804c3157847fc0f,
disagreement P50=0.142640 m, P95=0.665057 m, max=2.793219 m (not accuracy).
April2023 mtv-pe1 candidate also finished in651.711 s; its audit is pending.

### Pixel5 April2023 mtv-pe1 pair audited; diagnostic launch prepared (2026-09-27)

Audit session99996 passed:13/17 pairs and13 reproduced controls. April2023
mtv-pe1 has1,357 exact raw keys, candidate SHA256
0a637e8be755a0bf40085f2e495545bdd315225aff87a396b4c846d640783ed2,
disagreement P50=0.113798 m, P95=0.262500 m, max=0.311032 m (not accuracy).

Prepared but did not launch E:/rtklib_v2_ws_tmp/launch_laxp_imu_state_diagnostic_20260927.ps1.
PowerShell syntax parsing passes. It requires a passed diagnostic test record
and hashed JUnit XML for the frozen binary, finished Pixel5 execution manifest,
and a free native slot. It replays both lax-p arms sequentially with only the
diagnostic flag added, in separate directories, auditing exact frozen output
byte parity and state exports after each arm. Expected test evidence path is
use_cases/records/gsdc2023_imu_state_diagnostic_tests_20260927.json; this does
not exist yet because test build session68021 is still running. Modern9 and
diagnostic jobs must continue respecting the global two-native-process cap.

### Pixel5 April2023 sjc-q control reproduced (2026-09-27)

2023-04-27-20-55-us-ca-sjc-q/pixel5 control completed in590.119 s.
Artifact audit56491 completed successfully:13/17 pairs and14 controls
reproducing published solution bytes with exact raw keys and no truth use.
The frozen runner continues the candidate. Diagnostic test build68021 is
live in code generation; no new-binary C++ test result exists yet.

### Optimized IMU export synthetic tests passed (2026-09-27)

Targeted test build68021 completed exit0. Ran
FGOGtsamPhase171NoDopplerImuMainTest.* on the new test binary:2/2 passed,
zero failures/errors/disabled tests. The handoff test exercises diagnostic
off/on parity across its valid synthetic branches, checking position and
attitude components, final cost, iteration count, bias vector finiteness,
and agreement with existing aggregate bias maxima. The missing/nonfinite
handoff rejection test also passed. JUnit and source/binary hashes are pinned
in use_cases/records/gsdc2023_imu_state_diagnostic_tests_20260927.json.
Real-data trajectory parity and JSON export audit remain pending; the
prepared lax-p launcher must wait for the frozen Pixel5 queue to finish.

### Partial Pixel5 test bias inventory (2026-09-27)

Read27 completed arms from the frozen17-case test plan, checking run argv,
completion, summary hashes, convergence, and no-truth flags. Lax-p remains
the clear outlier:18.493567 /18.625443 m/s2 maximum acceleration bias.
Ebf-xx is next at0.372469 /0.406264 m/s2; every other completed arm is below
0.052 m/s2. This inventory is incomplete (27/34 arms), does not evaluate
accuracy, and does not change route selection or inference settings.
Evidence: use_cases/records/gsdc2023_pixel5_test_bias_inventory_20260927.json.
The two-worker queue remains live; no additional native inference was started.

### Pixel5 April2023 sjc-q pair audited (2026-09-27)

Candidate completed in564.946 s. Audit83110 passed:14/17 pairs and14
reproduced published controls. The sjc-q pair has1,380 exact raw keys,
candidate SHA25615c314816302fffe31df65a0865da5142e00c231cf8439b15d6412c71068ff6a,
disagreement P50=0.091300 m, P95=0.210376 m, maximum=0.376642 m.
These are trajectory differences, not accuracy. The existing queue continues
unchanged. Evidence: use_cases/records/gsdc2023_pixel5_test_progress_20260927.json.

### Pixel5 May2023 mtv-de1 control reproduced (2026-09-27)

2023-05-23-21-06-us-ca-mtv-de1/pixel5 control completed in1,114.313 s.
Artifact audit42306 passed:14/17 pairs and15 reproduced published controls,
with exact raw keys, converged native solutions, and no truth use. The
existing two-worker queue continues mtv-de1 candidate and sjc-be2 control.
No new official submission or additional inference queue was started.

### Pixel5 May2023 sjc-be2 control reproduced (2026-09-27)

2023-05-26-21-23-us-ca-sjc-be2/pixel5 control completed in322.361 s.
Artifact audit86357 passed:14/17 pairs and16 reproduced published controls,
with exact raw keys, convergence, and no truth use. The same two-worker
queue continues mtv-de1 and sjc-be2 candidates. No diagnostic or Modern9
native inference has started yet; the two slots remain occupied.

### Pixel5 May2023 mtv-de1 and sjc-be2 pairs audited (2026-09-27)

Candidates completed in669.898 and495.249 s respectively. Audit66601 passed:
16/17 complete pairs and16 reproduced controls. Mtv-de1 has1,975 keys,
candidate SHA690ec08acada6519046f3fb4b5d64a8d5c7f26ea238653ff2f24792049231328,
disagreement P50=0.117892 m, P95=0.650861 m, max=0.726472 m. Sjc-be2 has
1,482 keys, SHA3d6ec5146039ef736f88d927b7a80c01c81e2a7276f427f822702d40a9203e59,
disagreement P50=0.194857 m, P95=0.588616 m, max=0.626398 m. No test accuracy
was evaluated. Evidence: use_cases/records/gsdc2023_pixel5_test_progress_20260927.json.

Only last-case June06 sjc-he2 control remains active (PID46372 at observation).
Although a native slot is free, the frozen Modern9 execution gate explicitly
requires waiting until the active Pixel5 queue finishes. It was re-read and
no Modern9 launch occurred. The prepared diagnostic launcher has the same
finished-Pixel5 precondition. Keep both plans intact.

### Pixel5 June2023 sjc-he2 control reproduced (2026-09-27)

Last-case control completed in816.050 s. Audit50473 passed:all17 published
controls reproduce their solution bytes, while complete pairs remain16/17.
Exact raw UTC keys, convergence, and no-truth contracts pass. The existing
runner advanced to sjc-he2 candidate (PID64924 at observation); it is the
only remaining native arm. Diagnostic and Modern9 launches still wait for
the authoritative completed Pixel5 manifest.

### Pixel5 test17 complete and locally assembled; next jobs launched (2026-09-27)

Sjc-he2 (2023-06-06-22-43-us-ca-sjc-he2/pixel5) candidate completed in370.531 s.
Final audit89858 passes17/17 pairs and all17 reproduced controls. Last case
has1,608 raw keys, candidate SHA0d10bd10f8fb86f69a6d9cc1279f88fcc2f93614dcec50dce9d565645c9215e3,
disagreement P50=0.398675 m, P95=3.482897 m, max=3.636268 m (not accuracy).
Pixel5 execution.done.json confirms native_execution_complete=true for the
frozen plan. Supervisor67526 finished exit0 after the assembler passed.

Local candidate is E:/rtklib_v2_ws_output/gsdc_native/pixel5_joint_submission_20260927/submission.csv,
SHA2568faeea3e1bc91e8c6a3ba07617b28ca0f1ad2d0451ef21ef8807f5ee6dc9c245,
with71936 rows/40 drives,17 replaced and23 retained. Manifest status is
assembled-locally-not-submitted, all rows native, all controls reproduced,
no evaluation truth used. Lax-p's350 m disagreement and large bias remain
unresolved: local assembly is not evidence of accuracy or readiness to submit.

After native Pixel5 execution ended, launched lax-p diagnostic replay with
the prepared gated script (session37013; control PID65392 at observation),
then Modern9 frozen three-arm plan with one worker (session33150; runner40552,
first native PID1396). Modern launcher:
E:/rtklib_v2_ws_tmp/launch_modern9_test_20260927.ps1. It verifies pinned plan,
binary, development report/plan, source audit, reference submission, completed
Pixel5 manifest and capacity. Global native count is two: one diagnostic,
one Modern9. No submission command was run. Both supervisors perform audits
after completion; do not restart either from an observation timeout.

### Sjc-he2 disagreement localized without lax-p bias anomaly (2026-09-27)

Read-only audited trajectory diagnostic places all differences above2 m
in epochs0-201, maximum3.636268 m at epoch30. Maximum adjacent change in the
disagreement vector is0.330615 m, with exact1 s raw UTC intervals. Raw-P
initialization bytes match. Both arms converge with zero indeterminate solves;
maximum acceleration bias is0.019419 ->0.019911 m/s2 and gyro bias remains
about0.00015 rad/s. TDCP RMS is0.012302 ->0.013963 m; its largest residual is
at1000->1001, away from the early disagreement. One loader reset is reported
in both arms, but this does not establish causality or justify changing the
frozen clock policy. This case does not reproduce lax-p's large bias anomaly.
No test truth was consumed, accuracy inferred, or per-route selection made.
Evidence: use_cases/records/gsdc2023_pixel5_test_sjche2_displacement_diagnostic_20260927.json.

### Modern9 first control and clock-only arms audited (2026-09-27)

August31 SM-G988B control completed in164.583 s and reproduces the published
solution bytes across1,141 exact raw keys. Clock-only completed in156.262 s;
audited argv differs only by --android-continuous-clock-reference, both
reset counts are zero, and its solution is byte-identical to control.
Thus this no-reset test case shows no trajectory change from clock-only.
The candidate with the additional three recipe flags is next in the frozen
queue; no accuracy is evaluated. Evidence: use_cases/records/
gsdc2023_modern9_first_control_audit_20260927.json and
gsdc2023_modern9_first_clock_only_audit_20260927.json.
Lax-p diagnostic supervisor37013 remains live; Modern9 supervisor33150 remains live.

### Complete Pixel5 bias inventory and first Modern9 triplet (2026-09-27)

The completed Pixel5 plan now has all34 saved summaries hash-checked in
use_cases/records/gsdc2023_pixel5_test_bias_inventory_complete_20260927.json.
Lax-p alone has acceleration bias maxima18.493567/18.625443 m/s2.
Ebf-xx follows at0.372469/0.406264; all other15 cases remain below0.052.
This supersedes the earlier27-arm partial inventory without changing it.
No test truth was loaded and these magnitudes do not measure accuracy.

Modern9 first August31 SM-G988B candidate completed in401.800 s; all three
arms pass input/argv/binary/raw-key/convergence and recipe-effect audits.
The control reproduces published solution bytes; clock-only is identical.
Adding the frozen three recipe flags changes positions by P50=0.236246 m,
P95=0.483284 m, max=0.606517 m across1141 keys. These are disagreements,
not accuracy estimates. Evidence: use_cases/records/
gsdc2023_modern9_first_three_arm_audit_20260927.json.
Modern9 supervisor33150 continues the remaining eight triplets;
lax-p diagnostic supervisor37013 continues its control replay. Official
Private remains1.055 m; target<=0.928 m remains active. No submission.

### Lax-p bias-prior and interval coverage inspection (2026-09-27)

The frozen control summary reports first_imu_bias_priors_inserted=1,
first_imu_bias_priors_omitted=0, and imu_intervals=4514 for4515 epochs.
Phase209 separate factors are disabled. Current source constructs a Gaussian
first bias prior (accel sigma0.1 m/s2) and CombinedImuFactors with accel bias
random-walk sigma0.00025. The app's initial acceleration bias norm is only
0.04786 m/s2 (earlier raw replay). Thus an absent first-bias prior is not
supported by this evidence; the time evolution of the optimized state is
still required before choosing a fix. Both native supervisors were polled
live; no diagnostic replay has yet completed and no solver changes were made.

### Modern9 November05 Pixel6Pro control reproduced (2026-09-27)

The second Modern9 case,2021-11-05-18-28-us-ca-mtv-m/pixel6pro,
completed its control in2873.800 s. Input/argv/binary/provenance and native
solution audits pass across1446 keys. Solution SHA256
8b2e5037a262d009667e19fde9101fdd93ec70881170eb5c32d05be0100a1a17
matches the published source bytes. The legacy loader reports one clock
discontinuity. Clock-only and candidate outcomes remain pending; the raw
loader jump diagnostic must not be confused with an optimized accuracy gain.
Evidence: use_cases/records/gsdc2023_modern9_nov05_control_audit_20260927.json.
No truth was consumed and no submission was made. Lax-p diagnostic remains
running under supervisor37013; Modern9 supervisor33150 continues its plan.

### Modern9 November05 Pixel6Pro clock-only audited (2026-09-27)

Clock-only completed in4075.844 s. The audit verifies unchanged inputs,
frozen argv/binary/plan provenance,1446 raw keys and converged native output.
Clock discontinuities drop1->0. Control-to-clock-only displacement is
P50=0.006903 m, P95=0.133501 m, max=0.221190 m; this is not accuracy.
Control/candidate here refers only to this clock ablation, not the pending
three-flag candidate. Control iterations166, clock-only255. Initial cost
changes8.896291806e11->8.091748918e6, final cost13785.988->14459.294;
TDCP finite residual count24403->24416, so costs are not identical-objective
comparisons. TDCP RMS0.023736->0.023819 m; max acceleration bias
0.102333->0.102341 m/s2. No lax-p-scale bias anomaly is present.
Clock-only solution SHA256c285ee55ee989578adb2915f89b4901b4c26a908d9fc69e8edfe6b6db2bf51e4.
Evidence: use_cases/records/gsdc2023_modern9_nov05_clock_only_audit_20260927.json.
The frozen queue proceeds to the additional three recipe flags; lax-p
state diagnostic remains running. No evaluation truth or submission used.

### Lax-p control state export passes real-data parity (2026-09-27)

The control diagnostic completed in 8169.904 s. The new binary exports
4515 optimized IMU states and its solution bytes exactly match the frozen
control. The automated audit checks input/argv/source provenance, raw epoch
keys, finite state vectors and aggregate bias consistency. Summary SHA256:
589d253c8a0db251b9cb62061c78c6411098da3c69d959ae1ba9ed5205134fae.
Evidence: use_cases/records/gsdc2023_laxp_imu_state_control_20260927.json.
The supervisor subsequently started the candidate diagnostic; its parity
and state comparison remain pending. Modern9 November05 candidate also
continues, with the two-native-process limit maintained.

Optimized control acceleration bias is already 1.009872 m/s2 at epoch 0
(distinct from the initial seed norm 0.04786), crosses 5 at epoch 1543 and
10 at epoch 2441, and peaks at 18.493567 at epoch 4343. The largest adjacent
bias-vector change is only 0.007760 m/s2. This is an extended state drift,
not a bias jump first appearing during the final phone reorientation.
Selected low-speed epochs show roll progressing from 8.18 degrees at 0
to 87.59 at 3000 and 135.18 at 4000. These are coordinate summaries, not
rotation distances or independently observed physical attitude.

Read-only raw-IMU checks use +/- 2 s UTC windows and the source's frozen
RzRyRx mounting rotation; they do not replay native admitted IMU factors.
At epoch 4000, mean acceleration norm is 9.776 m/s2 and gyro norm P95
is 0.006104 rad/s. Raw mean specific force differs by 130.85 degrees from
the gravity direction implied by optimized attitude. The vector discrepancy
is 17.809 m/s2, but falls to 0.0863 after subtracting the optimized bias.
Earlier selected quiet windows exhibit the same growing compensation.
This supports investigating coupled attitude/bias drift; it does not prove
the underlying solver/initialization cause or establish a remedy.
Evidence: use_cases/records/gsdc2023_laxp_control_state_evolution_20260927.json;
reproducer: E:/rtklib_v2_ws_tmp/analyze_laxp_control_state_evolution_20260927.py.
No inference settings changed, truth consumed, accuracy claim, or submission.

### Historical lax-p initialization evidence rechecked (2026-09-27)

Rechecked the two completed September23 source-initialization/multipass
lax-p runs: binary, every recorded output hash, argv against the prior
experiment record, and all four raw-input hashes against today's diagnostic.
Their maximum optimized acceleration bias is 0.0543865/0.0543842 m/s2,
with 4515 per-epoch attitude seeds. They use different composite settings
and an older executable, so this is evidence for an initialization ablation,
not isolated causality or a replacement submission route.
Evidence: use_cases/records/gsdc2023_laxp_historical_initialization_audit_20260927.json.

The existing --native-epoch-heading-attitude-seeds option admits this Pixel5
recipe and changes attitude initialization/nearest heading filling without
changing bias initialization or factor/noise settings (lever-arm translation
seeds follow the selected rotation). After the active control/candidate
diagnostic pair finishes and passes parity, isolate this option against a
frozen diagnostic reference. Preserve the two-native-process limit.
The prior H development experiment was effectively neutral (+0.000100 m),
and the separate stationary-gyro initializer was also neutral; neither is
an established general accuracy improvement. Do not tune against H or
promote a test-route winner. Any useful ablation requires fixed-group
development transfer before changing the submitted recipe.

### Modern9 November05 Pixel6Pro full triplet audited (2026-09-27)

The candidate completed in 799.638 s. All three arms pass native output,
input/argv/binary/plan provenance and recipe-effect checks for 1446 keys;
control reproduces the published solution bytes. Candidate SHA256:
4be238425d379455c7e66fc72105d5133aecdbfeef253da6df323a70a6dcc3dd.
Position differences (P50/P95/max metres): clock-only versus control
0.006903/0.133501/0.221190; recipe versus clock-only
0.156343/0.417355/0.546482; candidate versus control
0.161247/0.458990/0.547933. These differences do not measure accuracy.
Candidate max acceleration bias is 0.104255 m/s2, with no lax-p-scale
anomaly. Evidence: use_cases/records/gsdc2023_modern9_nov05_three_arm_audit_20260927.json.
Modern9 now has two completed triplets (6/27 native runs); the supervisor
advanced to case02 control. Lax-p candidate diagnostic remains live.
No truth accessed or submission made; official Private remains 1.055 m.

### Heading-only ablation prepared, not started (2026-09-27)

Prepared E:/rtklib_v2_ws_tmp/launch_laxp_heading_seed_ablation_20260927.ps1.
It requires both current diagnostic arms to complete and pass trajectory
byte parity, then runs heading-only ablations sequentially against each
same-binary diagnostic reference. It adds only the existing
--native-epoch-heading-attitude-seeds flag, retains state export, and refuses
to launch when two native processes are present or an output already exists.
It does not use historical multipass solutions as solver input.

scripts/analysis/audit_gsdc_heading_seed_ablation.py verifies exact argument
delta, binary/input/source hashes, complete raw keys, convergence, diagnostic
states, unchanged raw initialization bytes, graph/TDCP/stop/height factor
counts and bias-prior counts. It reports trajectory differences and bias
evolution without reading truth. Python and PowerShell syntax checks pass;
real-data ablation validation is pending, not claimed passed. No launcher
execution or extra native process was started while the current jobs run.

### Modern9 March17 SM-G988B control reproduced (2026-09-27)

Case02, 2022-03-17-20-16-us-ca-sjc-q/sm-g988b, completed its control
in 312.041 s. The audit verifies frozen plan/argv/binary/input provenance,
native coverage of all 1172 raw epochs and exact published solution bytes.
Solution SHA256 d46002dd620e724eb630d2142c54d2cee246cd23101a5e83467f922cfc1aebc6.
The loader reports zero clock discontinuities. Clock-only is running;
its result must be checked rather than inferred from this reset count.
Evidence: use_cases/records/gsdc2023_modern9_mar17_control_audit_20260927.json.
Modern9 has 7/27 native runs complete. Lax-p candidate diagnostic is still
live; the heading ablation launcher remains unstarted. No accuracy claim
or submission follows from this control reproduction.

### Modern9 March17 clock-only reproduces control bytes (2026-09-27)

Case02 clock-only completed in 311.708 s. The frozen argument/input/binary
and raw-key audits pass; all 1172 output rows are byte-identical to control
(solution SHA256 d46002dd620e724eb630d2142c54d2cee246cd23101a5e83467f922cfc1aebc6).
Both arms report zero clock discontinuities, 19 iterations and identical
costs and TDCP residual statistics. Acceleration bias max is 0.253067 m/s2.
This establishes no trajectory change for this case, not accuracy improvement.
Evidence: use_cases/records/gsdc2023_modern9_mar17_clock_only_audit_20260927.json.
The supervisor proceeds to the additional recipe flags; Modern9 has 8/27
completed native runs. Lax-p candidate diagnostic remains running.

### Modern9 March17 SM-G988B triplet audited (2026-09-27)

Case02 candidate completed in 428.759 s. All three arms pass pinned
argv/input/binary/plan, native coverage and recipe-effect audits for 1172
raw epochs. Candidate SHA256:
5242e55607789d782cc811b4e84d2782bdf6749500a0df1f01144e2c714bf948.
Clock-only is byte-identical to control. Additional recipe versus either
has displacement P50 0.200795 m, P95 0.372750 m, max 0.470681 m.
These are trajectory differences, not accuracy measurements.
Evidence: use_cases/records/gsdc2023_modern9_mar17_three_arm_audit_20260927.json.
The fixed queue has now completed 3/9 triplets (9/27 runs) and started
case03 control. Lax-p candidate diagnostic is still live. No submission.

### Modern9 May02 Pixel7Pro control reproduced (2026-09-27)

Case03, 2023-05-02-19-24-us-ca-sjc-we1/pixel7pro, completed its control
in 3525.351 s. Frozen plan/argv/binary/input provenance and native coverage
audits pass for all 2110 raw epochs. Solution bytes reproduce the published
source, SHA256 c011b806b646a71cf3a8221b50e19a88281d7308b28c5538c076a3d2f6357131.
The loader reports zero clock discontinuities. Clock-only has started;
its optimized result remains pending. Evidence:
use_cases/records/gsdc2023_modern9_may02_control_audit_20260927.json.
Modern9 has 10/27 completed runs. Lax-p candidate diagnostic remains live
under supervisor37013; heading ablation is still unstarted. No truth used,
accuracy evaluated, or official submission made.

### Lax-p candidate state export completed; heading admission corrected (2026-09-27)

Diagnostic candidate completed in 5033.516 s and passed trajectory byte
parity with the frozen candidate. All 4515 state rows pass provenance,
keys, finite-value and aggregate consistency checks. Acceleration bias norm
starts at 0.937704, crosses 5 at epoch1556 and 10 at epoch2422, and peaks
at 18.625443 at epoch4321. Thus both arms exhibit long-duration bias drift.
Evidence: use_cases/records/gsdc2023_laxp_imu_state_candidate_20260927.json.
Diagnostic supervisor37013 is terminal with exit0; do not poll or restart it.

The prepared heading launcher was attempted after that completion, but
exited2 before inference: sparse-P staging separately rejected epoch heading
seeds. Earlier notes claiming this full combination was admitted were
incomplete. Preserve the failed output under laxp_heading_seed_ablation_20260927;
do not rerun that old launcher. Source review shows sparse staging preserves
epoch identities/admits sparse GNSS initialization, whereas heading seeds
are disabled in GNSS-first and applied only to the IMU initialization.
Removed only this CLI conflict; the raw/all-epoch/Phase171 and Pixel5 heading
guards remain. No solver factors or initialization implementation changed.

New frozen binary heading_sparse_admission_20260927.exe SHA256:
533f678b9e351e805b4424b8d4aa1b52f57bea48c48f755c88f638e53cffaa53.
Release app build succeeded. Seven CLI tests pass, including reaching
missing-input ingress for the combination and rejecting non-all-epoch input.
Pytest plugin autoload was disabled after unrelated xonsh console failure;
the negative test was corrected to assert the earlier Phase171 guard.
Evidence: use_cases/records/gsdc2023_heading_sparse_admission_build_20260927.json.

Started supervisor84907: new-binary control-only replay from the completed
diagnostic control, output laxp_heading_sparse_admission_20260927/control_baseline/control.
Require byte parity against the diagnostic control before running heading
from this new reference. The later heading run must use the same new binary
and source-run hash chain; existing heading ablation auditor then applies.
The candidate recipe requires its own same-binary baseline subsequently.
Modern9 clock-only remains live under33150, using its original frozen binary;
the two-native-process limit is preserved. No truth or submission used.

### Lax-p exported state pair compared (2026-09-27)

Both completed exports were rechecked for raw input/binary hashes, exact
three-recipe-flag difference, complete 4515 keys, diagnostic byte-parity
evidence and state consistency. Both arms show similar growing acceleration
bias well before the final reorientation: epoch2441..4249 median bias norm
15.126/15.395 m/s2. Additional recipe flags do not remove the anomaly.
The maximum absolute optimized vertical velocity is 99.075/94.719 m/s at
the final epoch4514. These are optimized-state diagnostics, not measured
vehicle velocities or truth errors. Evidence:
use_cases/records/gsdc2023_laxp_imu_state_pair_20260927.json.
The new-binary baseline replay remains live under84907; the original
Modern9 supervisor33150 continues May02 clock-only. No extra native job,
new inference setting, or official submission was introduced by this audit.

### Baseline replay audit prepared (2026-09-27)

Added scripts/analysis/audit_gsdc_imu_baseline_replay.py for the pending
new-binary replay. It checks source-run chaining, binary/input/output hashes,
identical inference arguments, native epoch coverage, byte-identical trajectory
and raw initialization, and exact exported IMU states with aggregate validation.
Python syntax check passed; the real replay audit is pending completion.
Both supervisors remain live:84907 baseline PID21492 and33150 May02
clock-only PID60960. Latest CPU samples increased to941.344/1303.359 seconds;
neither run is complete. Do not launch a third native process or reuse the
failed old heading launcher. Official Private remains1.055 m; goal active.

### Gated heading continuation started (2026-09-27)

Prepared and syntax-checked E:/rtklib_v2_ws_tmp/continue_laxp_heading_sparse_20260927.ps1.
Supervisor26254 is live, waiting for the existing control baseline process;
it does not start another native process while that baseline is running.
Its current stage is stored in laxp_heading_sparse_admission_20260927/continuation.json.
It verifies both original diagnostic audits, frozen binary and source chain,
then requires the new baseline replay audit before starting control heading.
Only after that heading audit passes does it run the candidate recipe's new
baseline, require its parity, and run/audit candidate heading. All outputs
use fresh folders control_heading, candidate_baseline, candidate_heading.
The maximum remains two native jobs, including Modern9. A persistent
exclusive creation lock prevents duplicate launch; failures stop without
retry. Do not manually launch these followups while26254 remains active.
Existing84907 and33150 remain live; no inference results or official scores
have changed yet. This is scheduling of the already planned paired ablation,
not automatic submission or candidate promotion.

### May02 Pixel7Pro clock-only completed (2026-09-27)

Modern9 case03 clock_only completed in3606.209 seconds, return0. Audited
exact frozen argv/plan/binary/input hashes, all2110 native keys, convergence,
and published-control provenance. The clock-only solution is byte-identical
to control/published output (SHA256 c011b806b646a71cf3a8221b50e19a88281d7308b28c5538c076a3d2f6357131).
Both arms have zero clock discontinuities,150 iterations and identical
costs/TDCP residuals/bias aggregates. Displacement is zero; this verifies
neutrality on this case, not test accuracy. Clock-only summary SHA256:
e358e99d8ba77ab98bffca4c32cb70671d8d47ced78ebb734f8c3d3d5471d535.
Evidence: use_cases/records/gsdc2023_modern9_may02_clock_only_audit_20260927.json;
reproducer: E:/rtklib_v2_ws_tmp/audit_modern9_may02_clock_only_20260927.py.
Modern9 now has11/27 native runs complete; case03 candidate is running
under the original supervisor33150, new native PID45876. Previous clock-only
PID60960 is terminal. Lax-p baseline21492/84907 and gated continuation26254
remain live. No additional native job or official submission was started.

### May02 sjc-we1 Pixel7Pro three-arm comparison completed (2026-09-27)

Case03 candidate completed in1022.468 seconds, return0. All three arms
pass frozen plan/argv/binary/input provenance, native2110-key coverage,
convergence and published-control byte-parity checks. The three candidate
recipe effects were verified in the summaries. Clock-only remains identical
to control; candidate displacement from either is P50=0.172486 m,
P95=0.403508 m,max=0.710477 m. These are trajectory differences, not truth
errors or evidence of improved test accuracy. Candidate solution SHA256:
6758a1ba1ace089226aabc4bf1c65fdb2af3a0c1b25aec6567738ee97c198780;
summary SHA256 c4d65da37ceb762a49147fbabcc8760aaebc0ce54e799bdb821afcbb3ff6c266.
Evidence: use_cases/records/gsdc2023_modern9_may02_three_arm_audit_20260927.json;
reproducer: E:/rtklib_v2_ws_tmp/audit_modern9_may02_three_arm_20260927.py.
Modern9 now has12/27 native runs complete (four complete triplets).
Supervisor33150 advanced to case04 control:
2023-05-02-20-33-us-ca-mtv-xe1/pixel7pro, native PID56000.
Case03 candidate PID45876 is terminal. Lax-p baseline21492/84907 and
continuation26254 remain live. No truth evaluation or official submission.

### May02 mtv-xe1 Pixel7Pro control reproduced (2026-09-27)

Modern9 case04 control completed in946.889 seconds, return0. Audited frozen
plan/argv/binary/input hashes, native2145-key coverage and convergence, and
byte-identical published trajectory. Solution SHA256:
be26652733f3e13b35e8d93406b5633bf4a9c4526b8330c7c9fdd28d9fc6706c;
summary SHA256 c1c9ee51f0c8cb7c76a325830e95492c78c3f4422f44baa4b02ace4417afa9c7.
Control has zero raw clock discontinuities. Evidence:
use_cases/records/gsdc2023_modern9_may02_mtv_control_audit_20260927.json;
reproducer: E:/rtklib_v2_ws_tmp/audit_modern9_may02_mtv_control_20260927.py.
Modern9 now has13/27 native runs complete. Supervisor33150 is running
case04 clock_only, native PID12808; previous control PID56000 is terminal.
Lax-p baseline21492/84907 and continuation26254 remain live. No truth
evaluation, candidate promotion or official submission was performed.

### May02 mtv-xe1 Pixel7Pro clock-only reproduced (2026-09-27)

Case04 clock_only completed in953.011 seconds, return0. The paired audit
passes frozen plan/argv/binary/input hashes, native2145-key coverage,
convergence and published-control provenance. Clock-only is byte-identical
to control (solution SHA256 be26652733f3e13b35e8d93406b5633bf4a9c4526b8330c7c9fdd28d9fc6706c).
Both arms have zero clock discontinuities,39 iterations and identical
cost/residual/bias aggregates. Clock-only summary SHA256:
1ee018be7ea987f061df50680f1eca942433ae2caca69cb0b1cd84ea1fd680c3.
Evidence: use_cases/records/gsdc2023_modern9_may02_mtv_clock_only_audit_20260927.json;
reproducer: E:/rtklib_v2_ws_tmp/audit_modern9_may02_mtv_clock_only_20260927.py.
Modern9 now has14/27 native runs complete. Supervisor33150 advanced to
case04 candidate, native PID55216; clock-only PID12808 is terminal.
Lax-p baseline21492/84907 and continuation26254 remain live. This proves
trajectory neutrality for this case, not test accuracy. No official submission.

### May02 mtv-xe1 Pixel7Pro three-arm comparison completed (2026-09-27)

Case04 candidate completed in1071.473 seconds, return0. All three arms pass
frozen plan/argv/binary/input provenance, native2145-key coverage,
convergence and published-control byte-parity checks. The requested recipe
effects were verified. Clock-only remains identical to control; candidate
displacement from either is P50=0.223901 m,P95=0.381069 m,max=0.480045 m.
These are trajectory differences, not truth errors. Candidate solution SHA256:
1f54be74ddc79d1738ac907650014cc73a7a67944168218b01b4f75d34d40105;
summary SHA256 d5bd8cfb4b30c30e29b0eccd0c684aee6f0e26292aaa34d4a573d83142652000.
Evidence: use_cases/records/gsdc2023_modern9_may02_mtv_three_arm_audit_20260927.json;
reproducer: E:/rtklib_v2_ws_tmp/audit_modern9_may02_mtv_three_arm_20260927.py.
Modern9 now has15/27 native runs complete (five complete triplets).
Supervisor33150 advanced to case05 control:
2023-05-23-22-16-us-ca-mtv-ie2/pixel6pro, native PID39344.
Case04 candidate PID55216 is terminal. Lax-p baseline21492/84907 and
continuation26254 remain live. No truth evaluation or official submission.

### May23 Pixel6Pro control reproduced (2026-09-27)

Modern9 case05 control completed in278.128 seconds, return0. Audited frozen
plan/argv/binary/input hashes, native1020-key coverage, convergence and
byte-identical published trajectory. Solution SHA256:
913debe206e43939251e0a1cf2a06505d066226de0c795cd15a904dc5834a370;
summary SHA256 d28a7aca3e1e4ebc8ac15f8e0b93aec82025996ddca6f9ea708aacd0f53d262d.
Control has zero raw clock discontinuities. Evidence:
use_cases/records/gsdc2023_modern9_may23_control_audit_20260927.json;
reproducer: E:/rtklib_v2_ws_tmp/audit_modern9_may23_control_20260927.py.
Modern9 now has16/27 native runs complete. Supervisor33150 is running
case05 clock_only, native PID29180; previous control PID39344 is terminal.
Lax-p baseline21492/84907 and continuation26254 remain live. No truth
evaluation, candidate promotion or official submission was performed.

### May23 Pixel6Pro clock-only reproduced (2026-09-27)

Case05 clock_only completed in281.243 seconds, return0. The paired audit
passes frozen plan/argv/binary/input hashes, native1020-key coverage,
convergence and published-control provenance. Clock-only is byte-identical
to control (solution SHA256 913debe206e43939251e0a1cf2a06505d066226de0c795cd15a904dc5834a370).
Both arms have zero clock discontinuities,17 iterations and identical
cost/residual/bias aggregates. Clock-only summary SHA256:
c1be6d655bc1fac6af1a66da8f4dc57f1a23cbeac99ba1c6ac62850cbd96f9f9.
Evidence: use_cases/records/gsdc2023_modern9_may23_clock_only_audit_20260927.json;
reproducer: E:/rtklib_v2_ws_tmp/audit_modern9_may23_clock_only_20260927.py.
Modern9 now has17/27 native runs complete. Supervisor33150 advanced to
case05 candidate, native PID24156; clock-only PID29180 is terminal.
Lax-p baseline21492/84907 and continuation26254 remain live. This proves
trajectory neutrality for this case, not test accuracy. No official submission.

### May23 Pixel6Pro three-arm comparison completed (2026-09-27)

Case05 candidate completed in505.523 seconds, return0. All three arms pass
frozen plan/argv/binary/input provenance, native1020-key coverage,
convergence and published-control byte-parity checks. Recipe effects pass.
Clock-only remains identical to control; candidate displacement from either
is P50=0.174751 m,P95=0.326762 m,max=0.409326 m. These are trajectory
differences, not truth errors. Candidate solution SHA256:
8bd4d93d62472c5e7c3ea80f82a476d57524a0e5868d13baaf6d478e58cfd354;
summary SHA256 3a4c1feb93ae96e054d0d1406e1951b7d302ba26012dfa9199cb20b6602c1ca3.
Evidence: use_cases/records/gsdc2023_modern9_may23_three_arm_audit_20260927.json;
reproducer: E:/rtklib_v2_ws_tmp/audit_modern9_may23_three_arm_20260927.py.
Modern9 now has18/27 native runs complete (six complete triplets).
Supervisor33150 advanced to case06 control:
2023-05-25-17-32-us-ca-pao-j/pixel6pro, native PID43312.
Case05 candidate PID24156 is terminal. Lax-p baseline21492/84907 and
continuation26254 remain live. No truth evaluation or official submission.

### Lax-p new-binary control replay passed exact parity (2026-09-27)

The heading-admission binary control replay completed in9236.265 seconds,
return0. The baseline auditor verified all4515 native keys, frozen input
and source provenance, trajectory byte parity, raw-initialization byte
parity and exact equality of every exported optimized IMU state against
the original diagnostic control. Both summary hashes are identical:
589d253c8a0db251b9cb62061c78c6411098da3c69d959ae1ba9ed5205134fae.
Replay run SHA256:
eabc04c69c5200b30227c3d77fbcac3ebd8e1b14bce5391fa2168c96343ae97f.
Solution SHA256:
1dc1773eee36296944a163c634dc06f54aba188effc88f9be8bec7a4a7be4bf8.
Evidence: use_cases/records/gsdc2023_laxp_heading_baseline_control_20260927.json;
auditor: scripts/analysis/audit_gsdc_imu_baseline_replay.py.
The CLI admission change is therefore neutral for this control replay;
the candidate baseline replay remains pending. Original large optimized
acceleration-bias drift is reproduced, not fixed by the admission change.
Continuation26254 passed this gate and started control_heading/heading,
native PID44440, adding only --native-epoch-heading-attitude-seeds to the
new baseline argv. Existing baseline84907/PID21492 is terminal0 and must
not be polled/restarted. Modern9 remains18/27 complete, case06 control
PID43312 live. No accuracy claim or official submission.

### May25 Pixel6Pro control reproduced (2026-09-27)

Modern9 case06 control completed in1644.900 seconds, return0. The audit
passes frozen plan/argv/binary/input hashes, native1292-key coverage,
convergence and byte-identical published trajectory. Solution SHA256:
cc67e73e774b3609afd096ad36cd3422d7a62722b1ba80969038c206dc091757;
summary SHA256 2f40776faa99276c2e74f67e5b17356f8fc3418d1ca3c9a0bfe9edc424dc5d9e.
Control has zero raw clock discontinuities. Evidence:
use_cases/records/gsdc2023_modern9_may25_control_audit_20260927.json;
reproducer: E:/rtklib_v2_ws_tmp/audit_modern9_may25_control_20260927.py.
Modern9 now has19/27 native runs complete. Supervisor33150 advanced to
case06 clock_only, native PID58236; previous control PID43312 is terminal.
Lax-p control-heading PID44440/continuation26254 remains live.
No truth evaluation, candidate promotion or official submission.

### Lax-p control heading ablation removes large optimized bias drift (2026-09-27)

Control-heading completed in1204.135 seconds, return0, with4515 native keys.
The one-option audit passes identical binary/inputs, exact baseline argv
plus --native-epoch-heading-attitude-seeds, raw-initialization byte parity,
convergence and invariant factor counts/noise. Graph222438 factors,
stop velocity1608, stop pose1555, TDCP80373 with sigma0.03, one first bias
prior, no height factors. This option changes attitude/lever-arm seeds and
low-speed heading fill; no zero-bias initialization was enabled.
Optimized acceleration-bias maximum18.493567 -> 0.053551 m/s2; all heading
states are below0.1 m/s2. Last bias18.077615 -> 0.032515 m/s2.
Optimized gyro-bias maximum0.003431428 -> 0.000469876 rad/s.
Trajectory displacement P50=0.293697 m,P95=8.842109 m,max=292.052068 m.
These are changes from an anomalous baseline, not measured accuracy.
Additional audited state inspection shows max absolute optimized vertical
velocity99.074619 -> 0.815822 m/s and horizontal maximum54.984264 ->
20.782192 m/s. These are optimizer states, not measured physical velocity.
Heading solution SHA256:
4137e61b89de201fe21a555a6152edc3823ef95830577282d6a3affe64ce257d;
summary SHA256 437ce05492a76432ba88cbd4b8acceb87a5af1fcb0b77551fc0c0bf5c1b897a5;
run SHA256 a2d5783a6944e2cb2f0fb4508c897d69301932d243af99b68004d85d1ab4c111.
Evidence: use_cases/records/gsdc2023_laxp_heading_seed_control_20260927.json
and gsdc2023_laxp_control_heading_velocity_20260927.json.
Velocity reproducer: E:/rtklib_v2_ws_tmp/analyze_laxp_control_heading_states_20260927.py.
This supports initialization sensitivity as a cause of the large optimized
state drift. It does not yet prove candidate-pair stability or test accuracy.
Continuation26254 advanced to candidate_baseline/control, native PID59840,
source run SHA256 f542e21a047c2e2ba5cc32fc44d48ae39ca034fec7f6b4cd3d6a99ff1c030d56.
Previous heading PID44440 is terminal. Modern9 clock-only PID58236 remains
live,19/27 complete. No route-selected promotion or official submission.

### Fixed Pixel5 heading development comparison prepared (2026-09-27)

Prepared all15 previously exposed Pixel5 development cases from the audited
height-group-safe candidate recipe. No route filtering or winner selection.
Plan: E:/rtklib_v2_ws_output/gsdc_native/pixel5_heading_development_20260927/plan.json
SHA256 669ca34e677db460f44f2652946bdfd560787a28881000e1295f0d890ff65586.
The30 planned native runs use the frozen heading-admission binary. Control
replays the prior candidate recipe plus report-only IMU export; candidate
adds only epoch-heading seeds. Each old candidate source was audited and
pinned for later trajectory parity. Height-map group exclusions, input
hashes and maps are preserved. Source zero-bias initialization is absent
in every argv and original summary. This is exposed development, not heldout.
Preparation script: scripts/analysis/prepare_gsdc_pixel5_heading_development.py;
record: use_cases/records/gsdc2023_pixel5_heading_development_plan_20260927.json.
Not launched: existing Modern9 and lax-p jobs occupy both native slots.
Before launch, provide a development heading auditor for runner plan-hash
provenance (the lax-p auditor expects direct baseline-run provenance),
then require source parity, heading telemetry/invariants and whole-set
truth scoring separately after inference. No candidate selection or submission.

### Pixel5 heading development auditor verified (2026-09-27)

Added scripts/analysis/audit_gsdc_pixel5_heading_development.py for the
runner's plan-hash provenance. It requires all15 completed native pairs,
parent-plan/source hashes, unchanged maps and evaluation-group exclusions,
exact old-candidate control trajectory parity, same-binary heading-only
argv changes, raw initialization parity, heading telemetry, factor/noise
invariants and finite exported IMU states. Only after all native audits
pass does it open pinned development truth and aggregate every case.
Seven checks pass: the completed real lax-p control-heading contract;
rejection of missing heading request/seed, changed sigma/factor count,
zero-bias initialization and an unstarted experiment before truth access.
Auditor SHA256 01b56eccfc5f0756e115d7b8b1bbf2063e4f937b3b254445a7ec46a8430de9fa.
Evidence: use_cases/records/gsdc2023_pixel5_heading_development_auditor_tests_20260927.json;
reproducer: E:/rtklib_v2_ws_tmp/test_pixel5_heading_development_audit_20260927.py.
The full15-case development audit/scoring remains unrun pending inference.
No extra native job was launched; Modern9 and lax-p candidate baseline
remain live. Next: verify the development truth-root paths, then queue
one-worker execution after an existing native slot is released.

### Fixed15 heading development queued behind Modern9 (2026-09-27)

Verified all15 scoring-only truth files under
E:/rtklib_v2_ws_data/gsdc2023/scoring/train against the frozen plan hashes.
These scoring paths are separate from the frozen inference arguments; no
truth coordinates were loaded for inference. Launched a gated continuation, session51207, waiting
on the existing Modern9 launcher PID3008 (command identity checked).
Launcher: E:/rtklib_v2_ws_tmp/launch_pixel5_heading_development_20260927.ps1
SHA256 e67a2b483a71293ad6fefa443eaf585d2507ac6f57f61e0db78e37bc065d1a33.
PowerShell syntax validation passed. Exclusive creation lock prevents
relaunch. The continuation pins plan, binary, parent evidence, runner and
auditor; it requires Modern9 execution completion and full27-arm audit,
then checks fewer than2 native processes before starting one worker.
It runs30 fixed development inferences and only afterward invokes the
heading-specific auditor/scorer. It fails without automatic retry and
contains no assembly, selection or submission action.
Current stage waiting-modern9-launcher; no additional native process yet.
Do not manually launch this plan or duplicate session51207.

### May25 Pixel6Pro clock-only reproduced (2026-09-27)

Case06 clock_only completed in1406.931 seconds, return0. The paired audit
passes frozen plan/argv/binary/input hashes, native1292-key coverage,
convergence and published-control provenance. Clock-only is byte-identical
to control (solution SHA256 cc67e73e774b3609afd096ad36cd3422d7a62722b1ba80969038c206dc091757).
Both arms have zero clock discontinuities,69 iterations and identical
cost/residual/bias aggregates. Clock-only summary SHA256:
31d89bdbbb2605f164fa16d946341fa353d5b40af95adf028b00c4e27382f090.
Evidence: use_cases/records/gsdc2023_modern9_may25_clock_only_audit_20260927.json;
reproducer: E:/rtklib_v2_ws_tmp/audit_modern9_may25_clock_only_20260927.py.
Modern9 now has20/27 native runs complete. Supervisor33150 advanced to
case06 candidate, native PID23520; clock-only PID58236 is terminal.
Lax-p candidate-baseline PID59840/continuation26254 remains live.
Fixed15 development continuation51207 remains waiting for Modern9.
This establishes trajectory neutrality for this case, not accuracy.
No candidate promotion or official submission.

### May25 Pixel6Pro three-arm comparison completed (2026-09-27)

Case06 candidate completed in467.315 seconds, return0. All three arms pass
frozen plan/argv/binary/input provenance, native1292-key coverage,
convergence and published-control byte-parity checks. Recipe effects pass.
Clock-only remains identical to control; candidate displacement from either
is P50=0.285395 m,P95=0.634017 m,max=0.836197 m. These are trajectory
differences, not truth errors. Candidate solution SHA256:
b67a972296138a7d92b0a9d1d504df33501de980df7e286540fa0071af321dcd;
summary SHA256 6e74c7a5642aa8416bcd4ca7bc1363aec983c27daf741a5186b10a4120e36a88.
Evidence: use_cases/records/gsdc2023_modern9_may25_three_arm_audit_20260927.json;
reproducer: E:/rtklib_v2_ws_tmp/audit_modern9_may25_three_arm_20260927.py.
Modern9 now has21/27 native runs complete (seven complete triplets).
Supervisor33150 advanced to case07 control:
2023-05-25-21-50-us-ca-sjc-ke2/sm-s908b, native PID63980.
Case06 candidate PID23520 is terminal. Lax-p candidate baseline59840 and
continuations26254/51207 remain live. No official submission.

### May25 SM-S908B control reproduced (2026-09-27)

Modern9 case07 control completed in1719.230 seconds, return0. The audit
passes frozen plan/argv/binary/input hashes, native1728-key coverage,
convergence and byte-identical published trajectory. Solution SHA256:
a7ed0de0e336d34b955eff3037a3ea78b75ba65d5d16608b18cb2f7a56272fd5;
summary SHA256 bb5de09ca33ca562f5b06ba7f3d8fb8d53f8ea6db08e1e96b0afd7a2f779d1e4.
Control has zero raw clock discontinuities. Evidence:
use_cases/records/gsdc2023_modern9_may25_samsung_control_audit_20260927.json;
reproducer: E:/rtklib_v2_ws_tmp/audit_modern9_may25_samsung_control_20260927.py.
Modern9 now has22/27 native runs complete. Supervisor33150 advanced to
case07 clock_only, native PID22268; previous control PID63980 is terminal.
Lax-p candidate-baseline PID59840/continuation26254 remains live.
Fixed15 development continuation51207 remains waiting for Modern9.
No accuracy claim, candidate promotion or official submission.

### Lax-p candidate baseline replay passed exact parity (2026-09-27)

Candidate baseline completed in4016.327 seconds, return0. All4515 native
keys, frozen inputs/source provenance, trajectory bytes, raw initialization
bytes and every optimized IMU state match the original diagnostic candidate.
Both summaries have SHA256:
0633861e8c2142fc1e1f414cd7a36f058aff3b0c85489d7471b36d6e0829cd88.
Replay run SHA256:
f6efda0ba37ce89b0da79b6cd2e2ec6e1edaa73ad841c454494a4b4174ac475c.
Solution SHA256:
fd752bea5499963abbb046a4e2a907d401da511cb9dc808fb87679fb266ddc17.
Evidence: use_cases/records/gsdc2023_laxp_heading_baseline_candidate_20260927.json.
Both baseline replay audits now pass; output hashes were rechecked and
linked in gsdc2023_heading_sparse_admission_real_parity_20260927.json.
This completes the CLI admission change's two-arm lax-p parity evidence,
not accuracy validation. Candidate acceleration-bias maximum18.625443 m/s2
is reproduced. Continuation26254 advanced to candidate_heading/heading,
native PID21596, adding only the heading option. Candidate baseline
PID59840 is terminal. Modern9 clock-only PID22268 and fixed15 waiting
continuation51207 remain live;22/27 Modern9 runs complete. No submission.

### May25 SM-S908B clock-only parity verified (2026-09-27)

Case07 clock_only completed in1665.106 seconds, return0. Frozen plan,
binary/input/source provenance, native1728-key coverage and convergence
audit pass. Its solution is byte-identical to control (SHA256
a7ed0de0e336d34b955eff3037a3ea78b75ba65d5d16608b18cb2f7a56272fd5),
with zero clock discontinuities in both arms. Summary SHA256:
0ca615d2ecd6b1e552c6df89ee19677b6e08320bc5de09731b45d789d3bf7a1b.
Evidence: use_cases/records/gsdc2023_modern9_may25_samsung_clock_only_audit_20260927.json.
Modern9 now has23/27 runs complete; supervisor33150 advanced to case07
candidate. Clock-only PID22268 is terminal. Lax-p candidate-heading
PID21596 remains live; fixed15 supervisor51207 still awaits Modern9.
Prepared case08 Jun15 Pixel7Pro partial-audit reproducers in the temp folder.

Added scripts/analysis/audit_gsdc_heading_recipe_quartet.py to audit both
heading ablations and compare the recipe pair before/after heading seeds.
Syntax and the exact three-flag recipe argv delta pass on recorded runs.
The complete quartet audit is pending candidate-heading completion; it
reports trajectory displacement and optimized velocities without truth.
No accuracy claim, promotion or official submission. Official Private
remains1.055 m; the0.928 m goal remains active.

### Quartet audit queued behind live heading supervisor (2026-09-27)

The new quartet auditor correctly rejects the currently incomplete
candidate-heading run with `run incomplete`, without writing a result.
Auditor SHA256 e4800497408d2682a2b937043965038856239fffdb15179c5965149e97b964c5.
Queued audit-only launcher
E:/rtklib_v2_ws_tmp/audit_laxp_quartet_after_heading_20260927.ps1
is live under session39816, waiting for verified-live supervisor65412.
It requires complete-both-heading-pairs-audited, verifies the auditor hash,
and uses an exclusive CreateNew lock before writing
use_cases/records/gsdc2023_laxp_heading_recipe_quartet_20260927.json.
No native inference, retry, scoring-truth read or submission is added.
Native jobs51392 (Modern9 case07 candidate) and21596 (lax-p heading)
remain active; fixed15 supervisor51207 is waiting for Modern9 completion.

### Lax-p heading quartet complete; fixed15 development started (2026-09-27)

Candidate-heading completed in1643.786 seconds, return0. Both heading
ablations and the four-run recipe comparison pass native4515-key coverage,
convergence, identical input/binary checks and exact recipe argv checks.
Candidate heading solution SHA256:
cdee067b0527607041bc9a0b5e07f0d2f611500d874cde56d194d4ed1864e5fb;
summary fc0a4f302cb81db83578b16cb7e24eb6cc8310bb2d404c58b5fc7a1fa0f9db51.
Candidate acceleration-bias maximum18.625443 becomes0.055132 m/s2;
absolute optimized vertical velocity maximum94.718965 becomes0.842726 m/s.
Control-versus-candidate displacement before/after heading in both arms:
P50 0.529192 ->0.328019 m, P95 6.258649 ->0.732266 m,
maximum350.199980 ->0.971174 m. These are trajectory differences and
optimized states, not truth errors or physical velocity measurements.
This supports initialization sensitivity as the cause of the large
recipe-dependent discrepancy; it does not establish test accuracy.
Records: gsdc2023_laxp_heading_seed_candidate_20260927.json and
gsdc2023_laxp_heading_recipe_quartet_20260927.json under use_cases/records.
Quartet record SHA256 f87b1c1450db561fb2234afa7fff29c9432e5b5a396863a75824afc06785bf5c.
Sessions26254 and39816 exited0; native21596 is terminal.

One inference slot became free. Replaced only the verified unstarted
fixed15 waiting supervisor62576 (session51207, intentional exit-1) to avoid
waiting on unrelated Modern9 completion. execution.started.json was absent
before and after stopping that queue; its original state is preserved in
continuation_original_wait_preserved.json. Original launcher/lock preserved.
New launcher E:/rtklib_v2_ws_tmp/launch_pixel5_heading_development_after_laxp_20260927.ps1
SHA256 5e67ec3622294cfc9c1ff2e6bc52948bd56496580350e6d4494a4a4843747161
requires completed lax-p audits, pinned quartet record, terminated old queue,
unchanged plan/binary/runner/auditor/parent hashes and fewer than2 native jobs.
The same fixed15 plan now runs under session65602, supervisor39092,
first native35688. Modern9 case07 candidate51392 remains live,23/27 complete.
Only scheduling changed; no case/recipe selection, new truth input,
promotion or submission. Official Private remains1.055 m and goal active.

### May25 SM-S908B three-arm comparison completed (2026-09-27)

Case07 candidate completed in1457.944 seconds, return0. All three arms
pass frozen plan/argv/binary/input/source checks, native1728-key coverage,
convergence and published-control byte parity. Clock-only equals control;
candidate displacement from either is P50=0.143700 m,P95=0.258259 m,
maximum=0.323285 m. Recipe-effect checks pass. These are trajectory
differences, not truth errors. Candidate solution SHA256:
b990413fcd31afb130d7881c981eff8de930bbc8f9d8b3082d3cd8629cfff79a;
summary d6a342e532417dfa5d8e918d8f0306bfe517320e619791dfe313cad9b3410ff1.
Evidence: use_cases/records/gsdc2023_modern9_may25_samsung_three_arm_audit_20260927.json.
Reproducer: E:/rtklib_v2_ws_tmp/audit_modern9_may25_samsung_three_arm_20260927.py.
Modern9 has24/27 runs complete (eight full triplets). Supervisor33150
advanced to final case08 control:2023-06-15-18-49-us-ca-sjc-ce1/pixel7pro.
Case07 native51392 is terminal. Fixed15 development supervisor65602
and first native35688 remain live. No promotion or official submission.

### Fixed15 heading development first control reproduced (2026-09-27)

First case2021-01-04-21-50-us-ca-e1highway280driveroutea/pixel5 control
completed in595.489 seconds, return0. Partial audit verifies frozen
plan/binary/parent hashes, input/map group independence, exact source argv
plus diagnostic option, convergence, native2002-key coverage and byte-equal
source trajectory. Solution SHA256:
858e4ef703489c87a80b93ac35d99c371182b89ade2e1475d0c9eb2fa3df8864;
summary7bb82bafb8d5915c90309a332b6c07c720a64c1fe2e8f1abec5d6b3b3653b082.
All optimized acceleration biases below0.1 m/s2; maximum0.0807046.
Evidence: use_cases/records/gsdc2023_pixel5_heading_development_first_control_20260927.json.
Reproducer: E:/rtklib_v2_ws_tmp/audit_pixel5_heading_development_first_control_20260927.py.
Fixed15 supervisor65602 now runs the first heading candidate, native13308;
native35688 is terminal.1/30 development runs complete. Modern9 final
case08 control21600 remains live,24/27 complete. No partial truth scoring
or recipe selection; full15 scoring awaits all native audits. No submission.

### First heading development pair passed native audit (2026-09-27)

Case00 heading candidate completed in679.573 seconds, return0. The pair
passes frozen plan/parent/binary/input/map provenance, source-control byte
parity, native2002-key coverage, convergence, exact one-option heading
delta, all-epoch seeds and unchanged graph/noise/first-bias-prior checks.
Raw initialization remains byte-identical (SHA256
b450d8867430f9efac58e6999fea861f0b0d0761c1660b8350da597545a307aa).
Trajectory displacement P50=0.000201895 m,P95=0.000840728 m,
maximum=0.002869090 m; this is not a truth error. Acceleration-bias
maximum remains approximately0.080705 m/s2 in both arms.
Candidate solution SHA256:
9ec3d142282fe49eead9fecf72494e24ee03dac848b1b37e3f095cebddeed6a7;
summary e891971d674479b2a1cc5177f0453ca5146d061aee13e6d8f4bdb2db6a98ce40.
Evidence: use_cases/records/gsdc2023_pixel5_heading_development_first_pair_20260927.json.
Reproducer: E:/rtklib_v2_ws_tmp/audit_pixel5_heading_development_first_pair_20260927.py.
Fixed15 has2/30 runs complete; supervisor65602 advanced to case01 control,
native9508. Case00 candidate13308 is terminal. Modern9 final control21600
remains live,24/27 complete. Full15 scoring pending; no submission.

### Jun15 Pixel7Pro control reproduced (2026-09-27)

Modern9 final case08 control completed in941.120 seconds, return0. Audit
passes frozen plan/argv/binary/input/source checks, native1495-key coverage,
convergence and byte-identical published-control trajectory. Solution SHA256:
c1bfeb93c3f085b78125fc4eba2fa9f01f4643f97e430b2dbd63ecd7826d237a;
summary60380290fdd70119ce0a310b691f0148c62052dd055f7d13241c72e2e8401146.
Zero raw clock discontinuities. Evidence:
use_cases/records/gsdc2023_modern9_jun15_control_audit_20260927.json.
Reproducer E:/rtklib_v2_ws_tmp/audit_modern9_jun15_control_20260927.py;
its copied status label was corrected to pixel7pro and rerun; actual
dataset/argv validation already targeted the correct Pixel7Pro case.
Modern9 now25/27 complete; supervisor33150 advanced to final clock_only.
Control21600 is terminal. Fixed15 case01 control9508/supervisor65602
remains live,2/30 complete. No truth scoring or official submission.

### Second heading development control reproduced (2026-09-27)

Case01 2021-01-04-22-40-us-ca-mtv-a/pixel5 control completed in787.079
seconds, return0. Frozen plan/binary/parent/input/map checks, native1855-key
coverage, convergence, diagnostic-only argv delta and exact source trajectory
parity pass. Acceleration-bias maximum0.05512335 m/s2, all states below0.1.
Solution SHA2568d21b77be693be7bc2df13148e096219e6ec598673708da336ea370d2117283e;
summary91a4d7f396e8ae4942e5b25d9ad261dbd83d835655363017cef5b24dcf07bc25.
Evidence: use_cases/records/gsdc2023_pixel5_heading_development_second_control_20260927.json.
Reproducer: E:/rtklib_v2_ws_tmp/audit_pixel5_heading_development_second_control_20260927.py.
Fixed15 now3/30 complete; supervisor65602 advanced to case01 candidate,
native23612. Previous control9508 is terminal. Modern9 final clock_only
27856 remains live,25/27 complete. No truth scoring or submission.

### Jun15 Pixel7Pro clock-only parity verified (2026-09-27)

Final case08 clock_only completed in1071.538 seconds, return0. Audit passes
frozen plan/argv/binary/input/source, native1495-key coverage, convergence
and published-control parity. Clock-only solution bytes equal control;
both have zero clock discontinuities and identical solver cost/residual/bias
aggregates. Solution SHA256:
c1bfeb93c3f085b78125fc4eba2fa9f01f4643f97e430b2dbd63ecd7826d237a;
clock-only summary04e29978f950774d77176724a29fbe2908cc943334c2c6df21ddd69a45316c8a.
Evidence: use_cases/records/gsdc2023_modern9_jun15_clock_only_audit_20260927.json.
Reproducer: E:/rtklib_v2_ws_tmp/audit_modern9_jun15_clock_only_20260927.py.
Modern9 has26/27 runs complete; supervisor33150 now runs final candidate,
native33388. Clock-only27856 is terminal. Fixed15 case01 candidate23612
remains live under65602,3/30 complete. No accuracy claim or submission.

### Second heading development pair passed native audit (2026-09-27)

Case01 candidate completed in768.613 seconds, return0. The pair passes
frozen provenance, source-control byte parity, native1855-key coverage,
convergence, exact heading-option delta, all-epoch seeds and unchanged
graph/noise/first-bias-prior checks. Raw initialization byte parity passes
(SHA25691d0034f24fb9eec33c7016439067a79792ca686e174566329996df30c52d242).
Trajectory displacement P50=0.000431472 m,P95=0.001095849 m,
maximum=0.001466649 m. These are differences, not truth errors.
Acceleration-bias maxima remain approximately0.055124 m/s2 in both arms.
Candidate solution SHA256:
352b1ae67fdf63622c6e24a4c4591dc1da7db24aa0f75c542de28c922d8d00f9;
summary020a8c3edf2390f089178b547e2b2fbef0caed0e9c878e96e0d9a1c7cb708626.
Evidence: use_cases/records/gsdc2023_pixel5_heading_development_second_pair_20260927.json.
Reproducer: E:/rtklib_v2_ws_tmp/audit_pixel5_heading_development_second_pair_20260927.py.
Fixed15 now4/30 complete; supervisor65602 advanced to case02 control.
Candidate23612 is terminal. Modern9 final candidate33388 remains live,
26/27 complete. Full15 truth scoring pending; no submission.

### Modern9 all27 native runs and full audit completed (2026-09-27)

Final Jun15 Pixel7Pro candidate completed in566.488 seconds, return0.
Its1495-key three-arm audit passes; candidate displacement from control
and clock_only is P50=0.294660 m,P95=0.771627 m,max=1.758642 m.
Candidate solution SHA256:
f5fa8c43d48ed2cce6215a239b7724367d9eace1ad45371c7265acf652577255;
summary88ec422f1908e4b9b93f6466fae641f9602db92473146e9e6f89b5b77df3a813.
Evidence: use_cases/records/gsdc2023_modern9_jun15_three_arm_audit_20260927.json.

Supervisor33150 then exited0 after the full audit passed9 cases/27 arms.
Full record: use_cases/records/gsdc2023_modern9_test_three_arm_audit_20260927.json.
execution.done.json SHA256:
90b9ca36965a574cbb15c54c6deeed8756aa6d4e57686dbdc0f673805be06718.
Reviewed auditor coverage: complete exact case/arm set, source/development/
reference hashes, exact plan argv/binary/input provenance, native raw keys,
convergence, published control byte parity and requested recipe effects.
All9 published controls reproduced. No test truth was read; these checks
establish reproducibility and trajectory differences, not official accuracy.
Modern native33388 is terminal. Fixed15 supervisor65602 continues case02
control59560,4/30 complete. No assembler, promotion or submission run.
Official Private remains1.055 m;0.928 m target remains active.

### Third heading development control reproduced (2026-09-27)

Case02 2021-03-10-23-13-us-ca-mtv-h/pixel5 control completed in510.272
seconds, return0. Partial audit passes frozen provenance/input/map checks,
native1465-key coverage, convergence, diagnostic-only argv delta and source
trajectory byte parity. Acceleration-bias maximum0.04076448 m/s2.
Solution SHA25615d3677d87c399af261d04baf0ea729cb864173b95ee437f101d6022ee41baad;
summary0508126a5889853c4e7b9b330d4a8e35dbfa5ff94f620de6156d66c62afc0787.
Evidence: use_cases/records/gsdc2023_pixel5_heading_development_third_control_20260927.json.
Reproducer: E:/rtklib_v2_ws_tmp/audit_pixel5_heading_development_third_control_20260927.py.
Fixed15 now5/30 complete; supervisor65602 advanced to case02 candidate,
native63132. Control59560 is terminal. Modern9 remains fully audited27/27.
Full15 scoring pending; no submission.

### Third heading development pair passed native audit (2026-09-27)

Case02 candidate completed in450.462 seconds, return0. Both arms pass
frozen provenance, source-control byte parity, native1465-key coverage,
convergence, exact heading-option delta, all-epoch seeds and unchanged
graph/noise/first-bias-prior checks. Raw initialization byte parity passes
(SHA2569859a6c7c92e87c026b492a6e3d2f6c86cbefd433004091ff56b3a69724d31db).
Trajectory displacement P50=0.000274913 m,P95=0.001084046 m,
maximum=0.001769216 m. These are differences, not truth errors.
Acceleration-bias maxima remain approximately0.0407644 m/s2 in both arms.
Candidate solution SHA256:
8544dbc74d72473d19d71877a606783907ed973a34002f3c1afc224856bdf879;
summary1dbe0ccecb84e061674517502d1b03d13285785347c798bf65983b2d20bca8ab.
Evidence: use_cases/records/gsdc2023_pixel5_heading_development_third_pair_20260927.json.
Reproducer: E:/rtklib_v2_ws_tmp/audit_pixel5_heading_development_third_pair_20260927.py.
Fixed15 now6/30 complete; supervisor65602 advanced to case03 control.
Candidate63132 is terminal. Modern9 remains fully audited27/27.
Full15 truth scoring pending; no submission.

### Fourth heading development control reproduced (2026-09-27)

Case03 2021-03-16-18-59-us-ca-mtv-a/pixel5 control completed in649.376
seconds, return0. Frozen provenance/input/map checks, native2159-key
coverage, convergence, diagnostic-only argv delta and source trajectory
byte parity pass. Acceleration-bias maximum0.04147853 m/s2.
Solution SHA256a5c782d61e2134ee6b6622b8d6fc4eb2c6b43ea73eaf6d412daf4c7467d30421;
summaryfd9da9fe363f39b367e6788ccc7641ff9bcd7e77a19d3423008807c8b6862e24.
Evidence: use_cases/records/gsdc2023_pixel5_heading_development_fourth_control_20260927.json.
Reproducer: E:/rtklib_v2_ws_tmp/audit_pixel5_heading_development_fourth_control_20260927.py.
Fixed15 now7/30 complete; supervisor65602 advanced to case03 candidate.
Control61816 is terminal. Full15 scoring pending; no submission.

### Fourth heading development pair passed native audit (2026-09-27)

Case03 candidate completed in517.143 seconds, return0. Both arms pass
frozen provenance, source-control byte parity, native2159-key coverage,
convergence, exact heading-option delta, all-epoch seeds and unchanged
graph/noise/first-bias-prior checks. Raw initialization byte parity passes
(SHA25678c4cb181bd934e626536abb908aabb7f44819bd32666ebd31b5b26daa9584e4).
Trajectory displacement P50=0.000310745 m,P95=0.001693967 m,
maximum=0.002142718 m. These are differences, not truth errors.
Candidate solution SHA256:
28f9d3b29fc5b2c9f8a91ffadc08d059a6ab61cdade7b4154c4011fb61e1b09c;
summary3a9d301362eddf0fe42646b90dec4e8c064978158f62649c4accac5c0da9a987.
Evidence: use_cases/records/gsdc2023_pixel5_heading_development_fourth_pair_20260927.json.
Reproducer: E:/rtklib_v2_ws_tmp/audit_pixel5_heading_development_fourth_pair_20260927.py.
Fixed15 now8/30 complete; supervisor65602 advanced to case04 control.
Candidate18144 is terminal. Full15 truth scoring pending; no submission.

### Fifth heading development control reproduced (2026-09-27)

Case04 2021-07-19-20-49-us-ca-mtv-a/pixel5 control completed in460.834
seconds, return0. Frozen provenance/input/map checks, native1897-key
coverage, convergence, diagnostic-only argv delta and source trajectory
byte parity pass. Acceleration-bias maximum0.03170458 m/s2.
Solution SHA2565ac7fdc439005e703d73240d90ba7dc9b515f9f03dbab6c14ffffbeb2562e9f5;
summaryfc9a44cec95fb03b9f6dedb16bb7b5b115d942eb2638d2702068d68393659d08.
Evidence: use_cases/records/gsdc2023_pixel5_heading_development_fifth_control_20260927.json.
Reproducer: E:/rtklib_v2_ws_tmp/audit_pixel5_heading_development_fifth_control_20260927.py.
Fixed15 now9/30 complete; supervisor65602 advanced to case04 candidate,
native45368. Control42932 is terminal. Full15 scoring pending; no submission.

### Fifth heading development pair passed native audit (2026-09-27)

Case04 2021-07-19-20-49-us-ca-mtv-a/pixel5 candidate completed in
476.590 seconds, return0. Both arms pass frozen provenance, source-control
byte parity, native1897-key coverage, convergence, exact heading-option
delta, all-epoch seeds and unchanged graph/noise/first-bias-prior checks.
Raw initialization byte parity passes; SHA256:
308fe37114a8105d5b157a565df182b8c2bc37240a0a17d242bee60887ee9d8b.
Trajectory displacement P50=0.000536284 m, P95=0.001694190 m,
maximum=0.002969972 m. These are differences, not truth errors.
Candidate solution SHA256:
f29f49ad3c8e1f5219d32347314d01f4465da4d2bddea6871b57c24ead37a3bf;
summary10499c9b77333137297eaf6824d0c025308a3a6452e0a97d7337b5c305ba9228.
Evidence: use_cases/records/gsdc2023_pixel5_heading_development_fifth_pair_20260927.json.
Reproducer: E:/rtklib_v2_ws_tmp/audit_pixel5_heading_development_fifth_pair_20260927.py.
Fixed15 now10/30 complete; candidate45368 is terminal.
Full15 truth scoring pending; no submission. Official Private remains1.055 m.

### Sixth heading development control reproduced (2026-09-27)

Case05 2021-08-24-20-32-us-ca-mtv-h/pixel5 control completed in1066.174
seconds, return0. Frozen provenance/input/map checks, native3140-key
coverage, convergence, diagnostic-only argv delta and source trajectory
byte parity pass. Acceleration-bias maximum0.05062132 m/s2.
Solution SHA256924ebf3b2f0ce2ccb859e945845365eb7fe7bf4babdc07a04b95cc0e089e5a59;
summaryc39e60e4699d21646d1b6c9998f1c9b9d44c0ad422e851f985cc11a948041535.
Evidence: use_cases/records/gsdc2023_pixel5_heading_development_sixth_control_20260927.json.
Reproducer: E:/rtklib_v2_ws_tmp/audit_pixel5_heading_development_sixth_control_20260927.py.
Fixed15 now11/30 complete; supervisor65602 advanced to case05 candidate.
Control4120 is terminal. Full15 scoring pending; no submission.

### Sixth heading development pair passed native audit (2026-09-27)

Case05 2021-08-24-20-32-us-ca-mtv-h/pixel5 candidate completed in1081.646
seconds, return0. Both arms pass frozen provenance, source-control byte
parity, native3140-key coverage, convergence, exact heading-option delta,
all-epoch seeds and unchanged graph/noise/first-bias-prior checks.
Raw initialization byte parity passes; SHA256:
b0110b9cf36b15743016b032122ebf8c5da40d865c927f255483f47871308200.
Trajectory displacement P50=0.000276323 m, P95=0.000844513 m,
maximum=0.001156340 m. These are differences, not truth errors.
Candidate solution SHA256:
b88c29e674fc1b6e280e4b72b8b4ff563ae0446659b1226a8680f80b15ebda38;
summary1ea46e524934fd5546ca79699e2d697f28ba891bfdfe0b7f9eb8e8ceed02c8a7.
Evidence: use_cases/records/gsdc2023_pixel5_heading_development_sixth_pair_20260927.json.
Reproducer: E:/rtklib_v2_ws_tmp/audit_pixel5_heading_development_sixth_pair_20260927.py.
Fixed15 now12/30 complete; supervisor65602 advanced to case06 control.
Candidate44892 is terminal. Full15 truth scoring pending; no submission.

### Seventh heading development control reproduced (2026-09-27)

Case06 2022-01-26-20-02-us-ca-mtv-pe1/pixel5 control completed in596.814
seconds, return0. Frozen provenance/input/map checks, native1698-key
coverage, convergence, diagnostic-only argv delta and source trajectory
byte parity pass. Acceleration-bias maximum0.007704132 m/s2.
Solution SHA25691f0a7039a849512e1bb5217c5398a879de87bd431ed21fae012134b8ee683b9;
summarye0c3a9fc53b094dd01bb8d0206dd791f77cf0bf622f8bc432267cb77c54b2a9d.
Evidence: use_cases/records/gsdc2023_pixel5_heading_development_seventh_control_20260927.json.
Reproducer: E:/rtklib_v2_ws_tmp/audit_pixel5_heading_development_seventh_control_20260927.py.
Fixed15 now13/30 complete; supervisor65602 advanced to case06 candidate,
native50756. Control58540 and audit session38318 are terminal.
Full15 scoring pending; no submission.

### Seventh heading development pair passed native audit (2026-09-27)

Case06 2022-01-26-20-02-us-ca-mtv-pe1/pixel5 candidate completed in698.749
seconds, return0. Both arms pass frozen provenance, source-control byte
parity, native1698-key coverage, convergence, exact heading-option delta,
all-epoch seeds and unchanged graph/noise/first-bias-prior checks.
Raw initialization byte parity passes; SHA256:
4f3abf57a64f5cd5aaebbede1cbc4ffe90bd90724960e94c97dd117e723d7f41.
Trajectory displacement P50=0.000316542 m, P95=0.001034451 m,
maximum=0.002133450 m. These are differences, not truth errors.
Candidate solution SHA256:
538f45ea6a70530facb925282ad001a7c103e56ddde3c6a83a9bfbb30af35f5e;
summary4491f553c3079dd434700d6c76fe582020c0fe296e77eaf94d5eb633e59f102d.
Evidence: use_cases/records/gsdc2023_pixel5_heading_development_seventh_pair_20260927.json.
Reproducer: E:/rtklib_v2_ws_tmp/audit_pixel5_heading_development_seventh_pair_20260927.py.
Fixed15 now14/30 complete; supervisor65602 advanced to case07 control.
Candidate50756 is terminal. Full15 truth scoring pending; no submission.

### Eighth heading development control reproduced (2026-09-27)

Case07 2022-02-24-18-29-us-ca-lax-o/pixel5 control completed in1226.566
seconds, return0. Frozen provenance/input/map checks, native2439-key
coverage, convergence, diagnostic-only argv delta and source trajectory
byte parity pass. Acceleration-bias maximum0.05388317 m/s2.
Solution SHA2564f278a0375e60b51322d9ee43cfa0b4f86b723fe37d6dd1add0e40a03817cc2a;
summary123dff41c76bbd20e9a6257c2db2e9bc77337a0fe9840ad192c80e2a9bd1434b.
Evidence: use_cases/records/gsdc2023_pixel5_heading_development_eighth_control_20260927.json.
Reproducer: E:/rtklib_v2_ws_tmp/audit_pixel5_heading_development_eighth_control_20260927.py.
Fixed15 now15/30 complete; supervisor65602 advanced to case07 candidate.
Control11240 is terminal. Full15 scoring pending; no submission.

### Eighth heading development pair passed native audit (2026-09-27)

Case07 2022-02-24-18-29-us-ca-lax-o/pixel5 candidate completed in805.337
seconds, return0. Both arms pass frozen provenance, source-control byte
parity, native2439-key coverage, convergence, exact heading-option delta,
all-epoch seeds and unchanged graph/noise/first-bias-prior checks.
Raw initialization byte parity passes; SHA256:
ac1707c874edeec09417d9af74296dd049415d534bcc4c06dea76e6be9a6dae3.
Trajectory displacement P50=0.323873280 m, P95=1.244246969 m,
maximum=5.245110187 m. Unlike the first seven pairs, this is a material
trajectory change; direction of accuracy effect remains unmeasured until
the planned full15 truth evaluation. No route-specific selection.
Acceleration-bias maximum changes0.05388317 to0.03677177 m/s2;
gyro-bias maximum changes0.004253433 to0.000134102 rad/s.
Candidate solution SHA256:
91851c439a6d83651e3ed0cb800facc47d137f324198ebfc4d0746ade133e263;
summary3dd8df7ac147b5fa51b5a13aa4aea80cb0f7cd053f4b2352d14737ddbe2515e5.
Evidence: use_cases/records/gsdc2023_pixel5_heading_development_eighth_pair_20260927.json.
Reproducer: E:/rtklib_v2_ws_tmp/audit_pixel5_heading_development_eighth_pair_20260927.py.
Fixed15 now16/30 complete; supervisor65602 advanced to case08 control.
Candidate59308 is terminal. Full15 truth scoring pending; no submission.

### Ninth heading development control reproduced (2026-09-27)

Case08 2022-04-01-18-22-us-ca-lax-t/pixel5 control completed in396.831
seconds, return0. Frozen provenance/input/map checks, native1466-key
coverage, convergence, diagnostic-only argv delta and source trajectory
byte parity pass. Acceleration-bias maximum0.01474573 m/s2.
Solution SHA256d13b8973314385ceff26f5b7a8de38efb2391eb08940430d022ce2337a76f2ba;
summary5accaa4a15c98b8fcec37320d3d83f498a9c8746d4d9815a4a9e408a642ff6ba.
Evidence: use_cases/records/gsdc2023_pixel5_heading_development_ninth_control_20260927.json.
Reproducer: E:/rtklib_v2_ws_tmp/audit_pixel5_heading_development_ninth_control_20260927.py.
Fixed15 now17/30 complete; supervisor65602 advanced to case08 candidate.
Control17208 is terminal. Full15 scoring pending; no submission.

### Ninth heading development pair passed native audit (2026-09-27)

Case08 2022-04-01-18-22-us-ca-lax-t/pixel5 candidate completed in439.742
seconds, return0. Both arms pass frozen provenance, source-control byte
parity, native1466-key coverage, convergence, exact heading-option delta,
all-epoch seeds and unchanged graph/noise/first-bias-prior checks.
Raw initialization byte parity passes; SHA256:
77f94eb6ed65832f8708877daf5bf025a2264f09ebb175dc9f513429c2728ef3.
Trajectory displacement P50=0.000300605 m, P95=0.001768808 m,
maximum=0.003390444 m. These are differences, not truth errors.
Candidate solution SHA256:
7b0340b3a65cf667575bb2f950aa0d142d45141b9c4934c214e4b5fd5b2b9853;
summary0bab7a8fdba59e21c5aa83d918ed16f8d436c01b08b8523386f1f0db04e4e872.
Evidence: use_cases/records/gsdc2023_pixel5_heading_development_ninth_pair_20260927.json.
Reproducer: E:/rtklib_v2_ws_tmp/audit_pixel5_heading_development_ninth_pair_20260927.py.
Fixed15 now18/30 complete; supervisor65602 advanced to case09 control.
Candidate63772 is terminal. Full15 truth scoring pending; no submission.

### Tenth heading development control reproduced (2026-09-27)

Case09 2022-08-04-20-07-us-ca-sjc-q/pixel5 control completed in599.579
seconds, return0. Frozen provenance/input/map checks, native1450-key
coverage, convergence, diagnostic-only argv delta and source trajectory
byte parity pass. Acceleration-bias maximum0.04511097 m/s2.
Solution SHA256b8db5c96ead16b901cdb56885f34f13f2ac16d1e09ba0a37bbef7f370ccf563a;
summary50885ab6f51306d3b5897d21ecb77d439a7317c695f85d58b034bc1aa4be46eb.
Evidence: use_cases/records/gsdc2023_pixel5_heading_development_tenth_control_20260927.json.
Reproducer: E:/rtklib_v2_ws_tmp/audit_pixel5_heading_development_tenth_control_20260927.py.
Fixed15 now19/30 complete; supervisor65602 advanced to case09 candidate.
Control2460 and audit session79459 are terminal. Full15 scoring pending;
no submission.

### Tenth heading development pair passed native audit (2026-09-27)

Case09 2022-08-04-20-07-us-ca-sjc-q/pixel5 candidate completed in725.011
seconds, return0. Both arms pass frozen provenance, source-control byte
parity, native1450-key coverage, convergence, exact heading-option delta,
all-epoch seeds and unchanged graph/noise/first-bias-prior checks.
Raw initialization byte parity passes; SHA256:
149c71ad455519a14ffef095db5fe05b93027e295e36cebf3e5f10724d3e856c.
Trajectory displacement P50=0.000196193 m, P95=0.000583407 m,
maximum=0.001105500 m. These are differences, not truth errors.
Candidate solution SHA256:
a5d9cc5f485478463e98d01883164d49328d78aca0090be4857cafd42bf9a5dd;
summaryb689a5e68b6c5609350572967a64ed8ae2ad0a9b21565f542a696e28e6e47e7e.
Evidence: use_cases/records/gsdc2023_pixel5_heading_development_tenth_pair_20260927.json.
Reproducer: E:/rtklib_v2_ws_tmp/audit_pixel5_heading_development_tenth_pair_20260927.py.
Fixed15 now20/30 complete; supervisor65602 advanced to case10 control.
Candidate26524 is terminal. Full15 truth scoring pending; no submission.

### Eleventh heading development control reproduced (2026-09-27)

Case10 2023-03-08-21-34-us-ca-mtv-u/pixel5 control completed in494.463
seconds, return0. Frozen provenance/input/map checks, native1102-key
coverage, convergence, diagnostic-only argv delta and source trajectory
byte parity pass. Acceleration-bias maximum0.04326483 m/s2.
Solution SHA256b3d58b5cebf23dc17f9b0d3ce07c54ac129cb40a76a8530341aaaf55a425ce34;
summary002365ebcfeed72094da6e45e0cc18c2ac485e683094bc5edbcb5eb8414e30c1.
Evidence: use_cases/records/gsdc2023_pixel5_heading_development_eleventh_control_20260927.json.
Reproducer: E:/rtklib_v2_ws_tmp/audit_pixel5_heading_development_eleventh_control_20260927.py.
Fixed15 now21/30 complete; supervisor65602 advanced to case10 candidate,
native24144. Control36620 is terminal. Full15 scoring pending; no submission.

### Eleventh heading development pair passed native audit (2026-09-27)

Case10 2023-03-08-21-34-us-ca-mtv-u/pixel5 candidate completed in538.265
seconds, return0. Both arms pass frozen provenance, source-control byte
parity, native1102-key coverage, convergence, exact heading-option delta,
all-epoch seeds and unchanged graph/noise/first-bias-prior checks.
Raw initialization byte parity passes; SHA256:
b50981425e533f04982e4d626d671583b4c5d92fe3746ae80b35dd0ff567f31f.
Trajectory displacement P50=0.000494724 m, P95=0.000826367 m,
maximum=0.000911137 m. These are differences, not truth errors.
Candidate solution SHA256:
d4c3bea06ab36e136c4c09d677c9a2aa0cff1358ee2920ced13da24fdc8999aa;
summaryf5d3e50d0c6a86c3f69145e1b43b914c430916da21bcb7e62548abe4076f5009.
Evidence: use_cases/records/gsdc2023_pixel5_heading_development_eleventh_pair_20260927.json.
Reproducer: E:/rtklib_v2_ws_tmp/audit_pixel5_heading_development_eleventh_pair_20260927.py.
Fixed15 now22/30 complete; supervisor65602 advanced to case11 control.
Candidate24144 is terminal. Full15 truth scoring pending; no submission.

### Twelfth heading development control reproduced (2026-09-27)

Case11 2023-05-09-21-32-us-ca-mtv-pe1/pixel5 control completed in831.091
seconds, return0. Frozen provenance/input/map checks, native2132-key
coverage, convergence, diagnostic-only argv delta and source trajectory
byte parity pass. Acceleration-bias maximum0.05261637 m/s2.
Solution SHA25669ab31859d9f7236eaa321e03c19a3ef12f29131b822329dee9c3e5ee4c06dae;
summary0a0701e62be9899d33ff232c6fc0b1a8cf619089824edf43357b0dd0093489c3.
Evidence: use_cases/records/gsdc2023_pixel5_heading_development_twelfth_control_20260927.json.
Reproducer: E:/rtklib_v2_ws_tmp/audit_pixel5_heading_development_twelfth_control_20260927.py.
Fixed15 now23/30 complete; supervisor65602 advanced to case11 candidate.
Control37248 is terminal. Full15 scoring pending; no submission.

### Twelfth heading development pair passed native audit (2026-09-27)

Case11 2023-05-09-21-32-us-ca-mtv-pe1/pixel5 candidate completed in959.629
seconds, return0. Both arms pass frozen provenance, source-control byte
parity, native2132-key coverage, convergence, exact heading-option delta,
all-epoch seeds and unchanged graph/noise/first-bias-prior checks.
Raw initialization byte parity passes; SHA256:
68f6beca28d462b863b8ba890b068444cc09f6810a9d394f7625abf38d5ef637.
Trajectory displacement P50=0.035061727 m, P95=1.212155867 m,
maximum=3.788395989 m. These are differences, not truth errors.
Acceleration-bias maximum changes from0.052616369 to0.018909419 m/s2.
Candidate solution SHA256:
7923c1212802338e6370f703cd72b2b6a38fbc5b7c48f0e126013f913505d4c3;
summary27055d04068c336b310435e8a7e937f6fffabbcf5e5584fc39507048a64229a9.
Evidence: use_cases/records/gsdc2023_pixel5_heading_development_twelfth_pair_20260927.json.
Reproducer: E:/rtklib_v2_ws_tmp/audit_pixel5_heading_development_twelfth_pair_20260927.py.
Fixed15 now24/30 complete; supervisor65602 advanced to case12 control,
native48016. Candidate25928 is terminal. Full15 scoring pending; no submission.

### Thirteenth heading development control reproduced (2026-09-28)

Case12 2023-05-16-19-54-us-ca-mtv-xe1/pixel5 control completed in809.904
seconds, return0. Frozen provenance/input/map independence checks,
native2323-key coverage, convergence, diagnostic-only argv delta and
source trajectory byte parity pass. Acceleration-bias maximum0.025678516 m/s2.
Solution SHA256d25dc6ab5024f738b33f6d32e75fbc97a7b97e8db4d9572a0b47a5a47d0ac695;
summarye06ff0d25a2a227b5183d493ddcfded995c519d132b0e31a30e9c60521d24007.
Evidence: use_cases/records/gsdc2023_pixel5_heading_development_thirteenth_control_20260927.json.
Reproducer: E:/rtklib_v2_ws_tmp/audit_pixel5_heading_development_thirteenth_control_20260927.py.
Fixed15 now25/30 complete; supervisor65602 advanced to case12 candidate,
native65740. Control48016 is terminal. Full15 scoring pending; no submission.

### Thirteenth heading development pair passed native audit (2026-09-28)

Case12 2023-05-16-19-54-us-ca-mtv-xe1/pixel5 candidate completed in737.364
seconds, return0. Both arms pass frozen provenance, source-control byte
parity, native2323-key coverage, convergence, exact heading-option delta,
all-epoch seeds and unchanged graph/noise/first-bias-prior checks.
Raw initialization byte parity passes; SHA256:
63722d41b536813794f426b07731845aa467e1230e82f7a37577e0f404728e7d.
Trajectory displacement P50=0.000189238 m, P95=0.001310487 m,
maximum=0.002045795 m. These are differences, not truth errors.
Candidate solution SHA256:
07f9288b10bbf9a9cdee248871b778111d0cb25f3f4351ffd3fc692e2a0c5c46;
summaryfd49adda36353114aa6f2ce603a7e2eb03684df0d8652825d0ce0bcff560e5c5.
Evidence: use_cases/records/gsdc2023_pixel5_heading_development_thirteenth_pair_20260927.json.
Reproducer: E:/rtklib_v2_ws_tmp/audit_pixel5_heading_development_thirteenth_pair_20260927.py.
Fixed15 now26/30 complete; supervisor65602 advanced to case13 control,
native11012. Candidate65740 is terminal. Full15 scoring pending; no submission.

### Fourteenth heading development control reproduced (2026-09-28)

Case13 2023-09-05-23-07-us-ca-routen/pixel5 control completed in524.264
seconds, return0. Frozen provenance/input/map checks, native1564-key
coverage, convergence, diagnostic-only argv delta and source trajectory
byte parity pass. Acceleration-bias maximum0.018178498 m/s2.
Solution SHA25662f7a8b803a7fe92d0800ed7526bc1d03484ecb400093ede11562158ac67f2ac;
summary1702b9e7fd144d40536f0dbf266a06a6e93e93ed6df6b8533b9e87432cf7a304.
Evidence: use_cases/records/gsdc2023_pixel5_heading_development_fourteenth_control_20260927.json.
Reproducer: E:/rtklib_v2_ws_tmp/audit_pixel5_heading_development_fourteenth_control_20260927.py.
Fixed15 now27/30 complete; supervisor65602 advanced to case13 candidate,
native11992. Control11012 is terminal. Full15 scoring pending; no submission.

### Fourteenth heading development pair passed native audit (2026-09-28)

Case13 2023-09-05-23-07-us-ca-routen/pixel5 candidate completed in535.075
seconds, return0. Both arms pass frozen provenance, source-control byte
parity, native1564-key coverage, convergence, exact heading-option delta,
all-epoch seeds and unchanged graph/noise/first-bias-prior checks.
Raw initialization byte parity passes; SHA256:
bce5ae97ee5261322dffb3073e4a61ddefcf0ace9a4f97c06affc63a53b1bc44.
Trajectory displacement P50=0.000387748 m, P95=0.001490927 m,
maximum=0.002170421 m. These are differences, not truth errors.
Candidate solution SHA256:
6310fc48046437d74a94312353859e905c75840f92e309ff7e8e94a1241727e5;
summary905793162b69ca715c36c0cca0c83471876bf98037bd362c61f2fb8aaa9e01d5.
Evidence: use_cases/records/gsdc2023_pixel5_heading_development_fourteenth_pair_20260927.json.
Reproducer: E:/rtklib_v2_ws_tmp/audit_pixel5_heading_development_fourteenth_pair_20260927.py.
Fixed15 now28/30 complete; supervisor65602 advanced to case14 control,
native53252. Candidate11992 is terminal. Full15 scoring pending; no submission.

### Fifteenth heading development control reproduced (2026-09-28)

Case14 2023-09-07-18-59-us-ca/pixel5 control completed in336.800
seconds, return0. Frozen provenance/input/map checks, native1172-key
coverage, convergence, diagnostic-only argv delta and source trajectory
byte parity pass. Acceleration-bias maximum0.017881355 m/s2.
Solution SHA2565b956d589525156b655eb736761c9626b18eed7587dd5e71f1b6bd9aa19b1fbe;
summary727f487b46fdc8a43994f02d926220bb3eaa10a8705902b5efdc447d732d9aea.
Evidence: use_cases/records/gsdc2023_pixel5_heading_development_fifteenth_control_20260927.json.
Reproducer: E:/rtklib_v2_ws_tmp/audit_pixel5_heading_development_fifteenth_control_20260927.py.
Fixed15 now29/30 complete; supervisor65602 advanced to case14 candidate,
native30700. Control53252 is terminal. Full15 scoring pending; no submission.

### Fifteenth heading development pair passed native audit (2026-09-28)

Case14 2023-09-07-18-59-us-ca/pixel5 candidate completed in341.240
seconds, return0. Both arms pass frozen provenance, source-control byte
parity, native1172-key coverage, convergence, exact heading-option delta,
all-epoch seeds and unchanged graph/noise/first-bias-prior checks.
Raw initialization byte parity passes; SHA256:
09b52f3aa2d02fa59a2e8fcee0e90f9deb4ef891efa96346cb2e39b2ae08a11c.
Trajectory displacement P50=0.000365015 m, P95=0.001081798 m,
maximum=0.002983077 m. These are differences, not truth errors.
Candidate solution SHA256:
f1e49616c1aaf407841e23043d8aee73df95c891411f3191ee89019e63f20551;
summarya8e41b396780aa78797a9bb8ca06a6784ad6bd474bb7900e22f24d9e19c3f9ec.
Evidence: use_cases/records/gsdc2023_pixel5_heading_development_fifteenth_pair_20260927.json.
Reproducer: E:/rtklib_v2_ws_tmp/audit_pixel5_heading_development_fifteenth_pair_20260927.py.
Fixed15 now30/30 complete; supervisor65602 advanced to full audit and
exposed-development scoring. Candidate30700 is terminal. No submission.

### Full fixed15 heading development evaluation completed (2026-09-28)

Supervisor65602 exited0 after all30 native runs and the pinned full auditor.
All15 source controls reproduced; heading contracts, native coverage,
convergence, provenance and height-map group independence passed.
Evaluation truth was consumed only after inference audits, never for inference.
Unweighted mean per-phone (P50+P95)/2: control0.6440282180321103 m,
heading candidate0.590451182416754 m; delta-0.05357703561535632 m.
Five cases improved and ten numerically regressed. All ten regressions are
below0.000291 m; largest+0.000290414375772763 m (July19).
Material improvements: lax-o1.5908205687504549 ->1.154353415180311 m;
May09 mtv-pe1 1.0632277705475712 ->0.6951783239676044 m.
This is previously exposed development, not heldout or official accuracy.
Evidence: use_cases/records/gsdc2023_pixel5_heading_development_comparison_20260927.json.
Record SHA256428228e9e5b3af7991ad84ce97e2ae5b249bd0e31c1fe79d15b88cd777e05c36.
Execution done SHA256ebcf4c6b7321bfc4b9581a19db9a3fbe8e77a2f2684d3e24820ea23e452699be.
Next: prepare uniform heading evaluation for all17 Pixel5 test cases with
frozen provenance and no route-specific winners; audit before any assembly.
No official submission occurred; recorded Private remains1.055 m and goal active.

### Uniform Pixel5 test17 heading comparison launched (2026-09-28)

Full fixed15 heading development audit supports evaluating the same option
uniformly on all17 Pixel5 test cases. No route-specific selection or truth
input is used. New control is the prior recipe candidate plus diagnostic;
new candidate adds only --native-epoch-heading-attitude-seeds. Height and
all input options are preserved exactly. Both arms use the pinned heading
binary533f678b9e351e805b4424b8d4aa1b52f57bea48c48f755c88f638e53cffaa53.
Preparer audited all17 source candidate runs before freezing the plan.
Plan: E:/rtklib_v2_ws_output/gsdc_native/pixel5_heading_test_20260928/plan.json.
Plan SHA2560e9145acc1abc1f1da8922398d512f6684a8138941e491152e0be6de9f9ee6ca.
Preflight passed exact17 cases, binary pin, all normalized argv deltas and
rejection of an unstarted plan by the auditor. No native workers were active.
Supervisor session73504 launched34 runs with workers2. Exclusive runner start
marker prevents duplicate launch. On success it invokes the full test auditor.
Auditor scripts/analysis/audit_gsdc_pixel5_heading_test.py SHA256:
c8e1eabce190b487234ab5f682ae42dd4b841ebd03d7b4a211aa5a5bc099f0dc.
Runner SHA25692cfaf0ed25018350f635935457342d96738c762d5f39ae33f886d76837a848a.
Expected audit: use_cases/records/gsdc2023_pixel5_heading_test_comparison_20260928.json.
The development supervisor65602 is terminal exit0; do not restart it.
No assembly, official submission or official score update occurred.

### Pixel5 test heading sjc-r control reproduced (2026-09-28)

2022-02-08-22-04-us-ca-sjc-r/pixel5 control completed in843.287 seconds.
Frozen provenance, exact diagnostic-only argv delta, native1665-key coverage,
convergence and source-candidate trajectory byte parity pass.
Acceleration-bias maximum0.030669248 m/s2. No truth consumed.
Evidence: use_cases/records/gsdc2023_pixel5_heading_test_sjcr_control_20260928.json.
Reproducer: E:/rtklib_v2_ws_tmp/audit_pixel5_heading_test_sjcr_control_20260928.py.
Supervisor73504 remains active; sjc-r candidate18484 and first-case control37336
running. One of34 runs complete. No assembly or submission.

### Pixel5 test heading mtv-g control reproduced (2026-09-28)

2021-08-17-20-37-us-ca-mtv-g/pixel5 control completed in1366.784 seconds.
Frozen provenance, exact diagnostic-only argv delta, native1676-key coverage,
convergence and source-candidate trajectory byte parity pass.
Acceleration-bias maximum0.050249170 m/s2. No truth consumed.
Evidence: use_cases/records/gsdc2023_pixel5_heading_test_mtvg_control_20260928.json.
Reproducer: E:/rtklib_v2_ws_tmp/audit_pixel5_heading_test_mtvg_control_20260928.py.
Supervisor73504 active; sjc-r candidate18484 and mtv-g candidate31508 running.
Control37336 terminal. Two of34 runs complete. No assembly or submission.

### Pixel5 test heading sjc-r pair audited (2026-09-28)

2022-02-08-22-04-us-ca-sjc-r/pixel5 candidate completed in700.340 seconds.
Both arms pass provenance, native1665-key coverage, control-source parity,
raw initialization parity and heading contract checks (exact option delta,
all-epoch seeds, unchanged graph/noise/first-bias-prior, no zero-bias init).
Trajectory displacement P50=0.000307964 m, P95=0.001356695 m,
maximum=0.001891059 m. These are differences, not accuracy errors.
Evidence: use_cases/records/gsdc2023_pixel5_heading_test_sjcr_pair_20260928.json.
Reproducer: E:/rtklib_v2_ws_tmp/audit_pixel5_heading_test_sjcr_pair_20260928.py.
Supervisor73504 active; mtv-g candidate31508 and next control63000 running.
Candidate18484 terminal. Three of34 runs complete. No assembly or submission.

### Pixel5 test heading mtv-g pair audited (2026-09-28)

2021-08-17-20-37-us-ca-mtv-g/pixel5 candidate completed in1259.959 seconds.
Both arms pass provenance, native1676-key coverage, control-source parity,
raw initialization parity and heading contract checks (exact option delta,
all-epoch seeds, unchanged graph/noise/first-bias-prior, no zero-bias init).
Trajectory displacement P50=0.000634780 m, P95=0.001226713 m,
maximum=0.002886176 m. These are differences, not accuracy errors.
Evidence: use_cases/records/gsdc2023_pixel5_heading_test_mtvg_pair_20260928.json.
Reproducer: E:/rtklib_v2_ws_tmp/audit_pixel5_heading_test_mtvg_pair_20260928.py.
Supervisor73504 active; controls63000 and22944 running.
Candidate31508 terminal. Four of34 runs complete. No assembly or submission.

### Pixel5 test heading lax-n control reproduced (2026-09-28)

2022-02-23-17-46-us-ca-lax-n/pixel5 control completed in1197.244 seconds.
Frozen provenance, exact diagnostic-only argv delta, native2407-key coverage,
convergence and source-candidate trajectory byte parity pass.
Acceleration-bias maximum0.047156147 m/s2. No truth consumed.
Evidence: use_cases/records/gsdc2023_pixel5_heading_test_laxn_control_20260928.json.
Reproducer: E:/rtklib_v2_ws_tmp/audit_pixel5_heading_test_laxn_control_20260928.py.
Supervisor73504 active; control22944 and lax-n candidate61572 running.
Control63000 terminal. Five of34 runs complete. No assembly or submission.

### Pixel5 test heading lax-n pair audited (2026-09-28)

2022-02-23-17-46-us-ca-lax-n/pixel5 candidate completed in 1179.523 seconds.
Both arms pass provenance, native 2407-key coverage, control-source parity,
raw initialization parity and heading contract checks (exact option delta,
all-epoch seeds, unchanged graph/noise/first-bias-prior, no zero-bias init).
Trajectory displacement P50=0.000179388 m, P95=0.000746814 m,
maximum=0.000900083 m. These are differences, not accuracy errors.
Acceleration-bias maximum remains 0.04716 m/s2. No truth consumed.
Evidence: use_cases/records/gsdc2023_pixel5_heading_test_laxn_pair_20260928.json.
Reproducer: E:/rtklib_v2_ws_tmp/audit_pixel5_heading_test_laxn_pair_20260928.py.
Supervisor 73504 active; lax-m control 22944 and lax-p control 61484 running.
Candidate 61572 terminal. Six of 34 runs complete. No assembly or submission.
Official Private remains 1.055 m; goal <=0.928 m remains active.

### Combined heading/modern local assembler prepared (2026-09-28)

Added scripts/analysis/assemble_gsdc_heading_modern_test.py for the fixed 17
Pixel5 heading candidates plus 9 modern clock/recipe candidates and 14 retained
native sources. It requires both execution.done records, reruns both full native
audits, verifies disjoint case sets, provenance, published Pixel5 parent controls,
reference hash and exact native lookup for all 71936 reference keys / 40 drives.
Retained output coordinates must equal the reference. No inference or submission
is performed by this assembler; test truth is never loaded.
Validation: Python syntax compilation passed. Running against the current plans
failed as expected at the missing Pixel5 execution.done.json; the proposed output
E:/rtklib_v2_ws_output/gsdc_native/heading_modern_joint_submission_20260928
remained absent. Full end-to-end assembly remains pending all native runs.
Supervisor 73504 remains live; current native PIDs 22944 and 61484.

### Retained14 preflight and lax-m heading control audited (2026-09-28)

Combined-assembly retained14 preflight passed for 23986 reference rows. Source
solution/run/summary hashes, dataset identities, native/converged/no-truth
contracts, and exact coordinate equality at every reference UTC key passed.
Evidence: use_cases/records/gsdc2023_heading_modern_retained14_preflight_20260928.json.
Reproducer: E:/rtklib_v2_ws_tmp/audit_heading_modern_retained14_20260928.py.
This is an independent partial check, not proof of complete combined assembly.

2022-02-23-22-35-us-ca-lax-m/pixel5 control completed in 1698.453 seconds.
Frozen provenance, diagnostic-only argv delta, native 2805-key coverage,
convergence and source-candidate trajectory byte parity pass.
Acceleration-bias maximum 0.043727052 m/s2. No truth consumed.
Evidence: use_cases/records/gsdc2023_pixel5_heading_test_laxm_control_20260928.json.
Reproducer: E:/rtklib_v2_ws_tmp/audit_pixel5_heading_test_laxm_control_20260928.py.
Supervisor 73504 active; lax-m candidate 61588 and lax-p control 61484 running.
Control 22944 terminal. Seven of 34 runs complete. No assembly or submission.

### Pixel5 test heading lax-m pair audited (2026-09-28)

2022-02-23-22-35-us-ca-lax-m/pixel5 candidate completed in 1394.704 seconds.
Both arms pass provenance, native 2805-key coverage, control-source parity,
raw initialization parity and heading contract checks (exact option delta,
all-epoch seeds, unchanged graph/noise/first-bias-prior, no zero-bias init).
Trajectory displacement P50=0.001021880 m, P95=0.001834063 m,
maximum=0.002243017 m. These are differences, not accuracy errors.
Acceleration-bias maxima: control 0.043727052, candidate 0.043728059 m/s2.
No truth consumed.
Evidence: use_cases/records/gsdc2023_pixel5_heading_test_laxm_pair_20260928.json.
Reproducer: E:/rtklib_v2_ws_tmp/audit_pixel5_heading_test_laxm_pair_20260928.py.
Supervisor 73504 active; lax-p control 61484 and next control 62236 running.
Candidate 61588 terminal. Eight of 34 runs complete. No assembly or submission.
Official Private remains 1.055 m; goal <=0.928 m remains active.

### Pixel5 test heading lax-i control reproduced (2026-09-28)

2022-02-24-22-14-us-ca-lax-i/pixel5 control completed in 1301.199 seconds.
Frozen provenance, diagnostic-only argv delta, native 3582-key coverage,
convergence and source-candidate trajectory byte parity pass.
Acceleration-bias maximum 0.039761220 m/s2. No truth consumed.
Evidence: use_cases/records/gsdc2023_pixel5_heading_test_laxi_control_20260928.json.
Reproducer: E:/rtklib_v2_ws_tmp/audit_pixel5_heading_test_laxi_control_20260928.py.
Supervisor 73504 active; lax-p control 61484 and lax-i candidate 22556 running.
Control 62236 terminal. Nine of 34 runs complete. No assembly or submission.

### Pixel5 test heading lax-p control reproduced (2026-09-28)

2022-02-24-15-10-us-ca-lax-p/pixel5 control completed in 3855.926 seconds.
Frozen provenance, diagnostic-only argv delta, native 4515-key coverage,
convergence and source-candidate trajectory byte parity pass.
The known large acceleration bias is reproduced: maximum 18.625443499 m/s2,
2093 epochs above 10 m/s2. Passing provenance is not an accuracy/stability claim.
No truth consumed. The uniform heading candidate is now running.
Evidence: use_cases/records/gsdc2023_pixel5_heading_test_laxp_control_20260928.json.
Reproducer: E:/rtklib_v2_ws_tmp/audit_pixel5_heading_test_laxp_control_20260928.py.
Supervisor 73504 active; lax-p candidate 63228 and lax-i candidate 22556 running.
Control 61484 terminal. Ten of 34 runs complete. No assembly or submission.

### Pixel5 test heading lax-i pair audited (2026-09-28)

2022-02-24-22-14-us-ca-lax-i/pixel5 candidate completed in 1242.655 seconds.
Both arms pass provenance, native 3582-key coverage, control-source parity,
raw initialization parity and heading contract checks (exact option delta,
all-epoch seeds, unchanged graph/noise/first-bias-prior, no zero-bias init).
Trajectory displacement P50=0.000164403 m, P95=0.000993722 m,
maximum=0.001776964 m. These are differences, not accuracy errors.
Acceleration-bias maxima remain below 0.039762 m/s2. No truth consumed.
Evidence: use_cases/records/gsdc2023_pixel5_heading_test_laxi_pair_20260928.json.
Reproducer: E:/rtklib_v2_ws_tmp/audit_pixel5_heading_test_laxi_pair_20260928.py.
Supervisor 73504 active; lax-p candidate 63228 and next control 52508 running.
Candidate 22556 terminal. Eleven of 34 runs complete. No assembly or submission.

### Pixel5 test heading March22 control reproduced (2026-09-28)

2022-03-22-18-44-us-ca-mtv-pe1/pixel5 control completed in 739.119 seconds.
Frozen provenance, diagnostic-only argv delta, native 2112-key coverage,
convergence and source-candidate trajectory byte parity pass.
Acceleration-bias maximum 0.015108385 m/s2. No truth consumed.
Evidence: use_cases/records/gsdc2023_pixel5_heading_test_march22_control_20260928.json.
Reproducer: E:/rtklib_v2_ws_tmp/audit_pixel5_heading_test_march22_control_20260928.py.
Supervisor 73504 active; lax-p candidate 63228 and March22 candidate 22796 running.
Control 52508 terminal. Twelve of 34 runs complete. No assembly or submission.

### Pixel5 test heading lax-p pair audited (2026-09-28)

2022-02-24-15-10-us-ca-lax-p/pixel5 candidate completed in 1646.471 seconds.
Both arms pass provenance, native 4515-key coverage, control-source parity,
raw initialization parity and heading contract checks (exact option delta,
all-epoch seeds, unchanged graph/noise/first-bias-prior, no zero-bias init).
Acceleration-bias maximum falls from 18.625443499 to 0.055132341 m/s2;
candidate has no epochs above 0.1 m/s2. Candidate solution and summary hashes
exactly match candidate_heading in the earlier audited lax-p quartet record.
Trajectory displacement P50=0.521081611 m, P95=5.989445782 m,
maximum=269.939508158 m. These are output differences, not accuracy errors.
No test truth consumed. The bias change supports the initialization diagnosis;
accuracy and official Private improvement remain unproven.
Evidence: use_cases/records/gsdc2023_pixel5_heading_test_laxp_pair_20260928.json.
Reproducer: E:/rtklib_v2_ws_tmp/audit_pixel5_heading_test_laxp_pair_20260928.py.
Supervisor 73504 active; March22 candidate 22796 and next control 28744 running.
Candidate 63228 terminal. Thirteen of 34 runs complete. No assembly or submission.

### Pixel5 test heading March22 pair audited (2026-09-28)

2022-03-22-18-44-us-ca-mtv-pe1/pixel5 candidate completed in 699.470 seconds.
Both arms pass provenance, native 2112-key coverage, control-source parity,
raw initialization parity and heading contract checks (exact option delta,
all-epoch seeds, unchanged graph/noise/first-bias-prior, no zero-bias init).
Trajectory displacement P50=0.000316016 m, P95=0.001344719 m,
maximum=0.001947538 m. These are differences, not accuracy errors.
Acceleration-bias maxima remain below 0.015110 m/s2. No truth consumed.
Evidence: use_cases/records/gsdc2023_pixel5_heading_test_march22_pair_20260928.json.
Reproducer: E:/rtklib_v2_ws_tmp/audit_pixel5_heading_test_march22_pair_20260928.py.
Supervisor 73504 active; controls 28744 and 33880 running.
Candidate 22796 terminal. Fourteen of 34 runs complete. No assembly or submission.

### Pixel5 test heading ebf-y control reproduced (2026-09-28)

2022-04-22-20-11-us-ca-ebf-y/pixel5 control completed in 371.001 seconds.
Frozen provenance, diagnostic-only argv delta, native 1400-key coverage,
convergence and source-candidate trajectory byte parity pass.
Acceleration-bias maximum 0.027350049 m/s2. No truth consumed.
Evidence: use_cases/records/gsdc2023_pixel5_heading_test_ebfy_control_20260928.json.
Reproducer: E:/rtklib_v2_ws_tmp/audit_pixel5_heading_test_ebfy_control_20260928.py.
Supervisor 73504 active; control 28744 and ebf-y candidate 55128 running.
Control 33880 terminal. Fifteen of 34 runs complete. No assembly or submission.

### Pixel5 test heading lax-x control reproduced (2026-09-28)

2022-04-04-16-31-us-ca-lax-x/pixel5 control completed in 933.355 seconds.
Frozen provenance, diagnostic-only argv delta, native 2171-key coverage,
convergence and source-candidate trajectory byte parity pass.
Acceleration-bias maximum 0.013398412 m/s2. No truth consumed.
Evidence: use_cases/records/gsdc2023_pixel5_heading_test_laxx_control_20260928.json.
Reproducer: E:/rtklib_v2_ws_tmp/audit_pixel5_heading_test_laxx_control_20260928.py.
Supervisor 73504 active; lax-x candidate 48464 and ebf-y candidate 55128 running.
Control 28744 terminal. Sixteen of 34 runs complete. No assembly or submission.

### Pixel5 test heading ebf-y pair audited (2026-09-28)

2022-04-22-20-11-us-ca-ebf-y/pixel5 candidate completed in 388.208 seconds.
Both arms pass provenance, native 1400-key coverage, control-source parity,
raw initialization parity and heading contract checks (exact option delta,
all-epoch seeds, unchanged graph/noise/first-bias-prior, no zero-bias init).
Trajectory displacement P50=0.000237620 m, P95=0.000508983 m,
maximum=0.000969928 m. These are differences, not accuracy errors.
Acceleration-bias maxima remain below 0.027351 m/s2. No truth consumed.
Evidence: use_cases/records/gsdc2023_pixel5_heading_test_ebfy_pair_20260928.json.
Reproducer: E:/rtklib_v2_ws_tmp/audit_pixel5_heading_test_ebfy_pair_20260928.py.
Supervisor 73504 active; lax-x candidate 48464 and next control 58708 running.
Candidate 55128 terminal. Seventeen of 34 runs complete. No assembly or submission.

### Pixel5 test heading ebf-z control reproduced (2026-09-28)

2022-04-25-22-36-us-ca-ebf-z/pixel5 control completed in 381.920 seconds.
Frozen provenance, diagnostic-only argv delta, native 1587-key coverage,
convergence and source-candidate trajectory byte parity pass.
Acceleration-bias maximum 0.033115515 m/s2. No truth consumed.
Evidence: use_cases/records/gsdc2023_pixel5_heading_test_ebfz_control_20260928.json.
Reproducer: E:/rtklib_v2_ws_tmp/audit_pixel5_heading_test_ebfz_control_20260928.py.
Supervisor 73504 active; lax-x candidate 48464 and ebf-z candidate 16884 running.
Control 58708 terminal. Eighteen of 34 runs complete. No assembly or submission.

### Pixel5 test heading ebf-z pair audited (2026-09-28)

2022-04-25-22-36-us-ca-ebf-z/pixel5 candidate completed in 330.042 seconds.
Both arms pass provenance, native 1587-key coverage, control-source parity,
raw initialization parity and heading contract checks (exact option delta,
all-epoch seeds, unchanged graph/noise/first-bias-prior, no zero-bias init).
Trajectory displacement P50=0.135122627 m, P95=0.876175800 m,
maximum=0.951427299 m. These are differences, not accuracy errors.
Acceleration-bias maximum increases from 0.033115514 to 0.214557085 m/s2;
candidate exceeds 0.1 m/s2 for 858 epochs (0 through 857), with none above 1.
This merits joint review; it does not establish an accuracy regression and
must not drive route-specific selection. No truth consumed.
Evidence: use_cases/records/gsdc2023_pixel5_heading_test_ebfz_pair_20260928.json.
Reproducer: E:/rtklib_v2_ws_tmp/audit_pixel5_heading_test_ebfz_pair_20260928.py.
Supervisor 73504 active; lax-x candidate 48464 and next control 52228 running.
Candidate 16884 terminal. Nineteen of 34 runs complete. No assembly or submission.

### Pixel5 test heading lax-x pair audited (2026-09-28)

2022-04-04-16-31-us-ca-lax-x/pixel5 candidate completed in 895.744 seconds.
Both arms pass provenance, native 2171-key coverage, control-source parity,
raw initialization parity and heading contract checks (exact option delta,
all-epoch seeds, unchanged graph/noise/first-bias-prior, no zero-bias init).
Trajectory displacement P50=0.000349979 m, P95=0.001804423 m,
maximum=0.002455626 m. These are differences, not accuracy errors.
Acceleration-bias maxima remain below 0.013399 m/s2. No truth consumed.
Evidence: use_cases/records/gsdc2023_pixel5_heading_test_laxx_pair_20260928.json.
Reproducer: E:/rtklib_v2_ws_tmp/audit_pixel5_heading_test_laxx_pair_20260928.py.
Supervisor 73504 active; controls 52228 and 37980 running.
Candidate 48464 terminal. Twenty of 34 runs complete. No assembly or submission.

### Pixel5 test heading ebf-zz control reproduced (2026-09-28)

2022-04-27-18-16-us-ca-ebf-zz/pixel5 control completed in 395.648 seconds.
Frozen provenance, diagnostic-only argv delta, native 1315-key coverage,
convergence and source-candidate trajectory byte parity pass.
Acceleration-bias maximum 0.027119385 m/s2. No truth consumed.
Evidence: use_cases/records/gsdc2023_pixel5_heading_test_ebfzz_control_20260928.json.
Reproducer: E:/rtklib_v2_ws_tmp/audit_pixel5_heading_test_ebfzz_control_20260928.py.
Supervisor 73504 active; ebf-xx control 37980 and ebf-zz candidate 61204 running.
Control 52228 terminal. Twenty-one of 34 runs complete. No assembly or submission.

### Pixel5 test heading ebf-xx control reproduced (2026-09-28)

2022-04-27-19-23-us-ca-ebf-xx/pixel5 control completed in 453.612 seconds.
Frozen provenance, diagnostic-only argv delta, native 1382-key coverage,
convergence and source-candidate trajectory byte parity pass.
Acceleration-bias maximum 0.406263790 m/s2; all 1382 epochs exceed 0.1,
none exceed 1 m/s2. This reproduces the existing control behavior.
No truth consumed; candidate comparison remains pending.
Evidence: use_cases/records/gsdc2023_pixel5_heading_test_ebfxx_control_20260928.json.
Reproducer: E:/rtklib_v2_ws_tmp/audit_pixel5_heading_test_ebfxx_control_20260928.py.
Supervisor 73504 active; ebf-xx candidate 32480 and ebf-zz candidate 61204 running.
Control 37980 terminal. Twenty-two of 34 runs complete. No assembly or submission.

### Pixel5 test heading ebf-zz pair audited (2026-09-28)

2022-04-27-18-16-us-ca-ebf-zz/pixel5 candidate completed in 396.453 seconds.
Both arms pass provenance, native 1315-key coverage, control-source parity,
raw initialization parity and heading contract checks (exact option delta,
all-epoch seeds, unchanged graph/noise/first-bias-prior, no zero-bias init).
Trajectory displacement P50=0.000184841 m, P95=0.000998266 m,
maximum=0.001458786 m. These are differences, not accuracy errors.
Acceleration-bias maxima remain below 0.027121 m/s2. No truth consumed.
Evidence: use_cases/records/gsdc2023_pixel5_heading_test_ebfzz_pair_20260928.json.
Reproducer: E:/rtklib_v2_ws_tmp/audit_pixel5_heading_test_ebfzz_pair_20260928.py.
Supervisor 73504 active; ebf-xx candidate 32480 and next control 52748 running.
Candidate 61204 terminal. Twenty-three of 34 runs complete. No assembly or submission.

### Pixel5 test heading ebf-xx pair audited (2026-09-28)

2022-04-27-19-23-us-ca-ebf-xx/pixel5 candidate completed in 456.174 seconds.
Both arms pass provenance, native 1382-key coverage, control-source parity,
raw initialization parity and heading contract checks (exact option delta,
all-epoch seeds, unchanged graph/noise/first-bias-prior, no zero-bias init).
Trajectory displacement P50=0.000289642 m, P95=0.002436857 m,
maximum=0.002717955 m. These are differences, not accuracy errors.
Acceleration-bias maximum 0.406263789 -> 0.406270333 m/s2, effectively
unchanged; both arms exceed 0.1 for all epochs and never exceed 1 m/s2.
No truth consumed.
Evidence: use_cases/records/gsdc2023_pixel5_heading_test_ebfxx_pair_20260928.json.
Reproducer: E:/rtklib_v2_ws_tmp/audit_pixel5_heading_test_ebfxx_pair_20260928.py.
Supervisor 73504 active; controls 52748 and 21744 running.
Candidate 32480 terminal. Twenty-four of 34 runs complete. No assembly or submission.

### Pixel5 test heading April27 mtv control reproduced (2026-09-28)

2023-04-27-19-25-us-ca-mtv-pe1/pixel5 control completed in 452.808 seconds.
Frozen provenance, diagnostic-only argv delta, native 1357-key coverage,
convergence and source-candidate trajectory byte parity pass.
Acceleration-bias maximum 0.023696434 m/s2. No truth consumed.
Evidence: use_cases/records/gsdc2023_pixel5_heading_test_apr27mtv_control_20260928.json.
Reproducer: E:/rtklib_v2_ws_tmp/audit_pixel5_heading_test_apr27mtv_control_20260928.py.
Supervisor 73504 active; sjc-q control 21744 and mtv-pe1 candidate 39712 running.
Control 52748 terminal. Twenty-five of 34 runs complete. No assembly or submission.

### Pixel5 test heading sjc-q control reproduced (2026-09-28)

2023-04-27-20-55-us-ca-sjc-q/pixel5 control completed in 507.308 seconds.
Frozen provenance, diagnostic-only argv delta, native 1380-key coverage,
convergence and source-candidate trajectory byte parity pass.
Acceleration-bias maximum 0.019758628 m/s2. No truth consumed.
Evidence: use_cases/records/gsdc2023_pixel5_heading_test_sjcq_control_20260928.json.
Reproducer: E:/rtklib_v2_ws_tmp/audit_pixel5_heading_test_sjcq_control_20260928.py.
Supervisor 73504 active; sjc-q candidate 30384 and mtv-pe1 candidate 39712 running.
Control 21744 terminal. Twenty-six of 34 runs complete. No assembly or submission.

### Pixel5 test heading April27 mtv pair audited (2026-09-28)

2023-04-27-19-25-us-ca-mtv-pe1/pixel5 candidate completed in 451.721 seconds.
Both arms pass provenance, native 1357-key coverage, control-source parity,
raw initialization parity and heading contract checks (exact option delta,
all-epoch seeds, unchanged graph/noise/first-bias-prior, no zero-bias init).
Trajectory displacement P50=0.000412276 m, P95=0.000940310 m,
maximum=0.001257384 m. These are differences, not accuracy errors.
Acceleration-bias maxima remain below 0.023697 m/s2. No truth consumed.
Evidence: use_cases/records/gsdc2023_pixel5_heading_test_apr27mtv_pair_20260928.json.
Reproducer: E:/rtklib_v2_ws_tmp/audit_pixel5_heading_test_apr27mtv_pair_20260928.py.
Supervisor 73504 active; sjc-q candidate 30384 and next control 17772 running.
Candidate 39712 terminal. Twenty-seven of 34 runs complete. No assembly or submission.

### Pixel5 test heading sjc-q pair audited (2026-09-28)

2023-04-27-20-55-us-ca-sjc-q/pixel5 candidate completed in 458.677 seconds.
Both arms pass provenance, native 1380-key coverage, control-source parity,
raw initialization parity and heading contract checks (exact option delta,
all-epoch seeds, unchanged graph/noise/first-bias-prior, no zero-bias init).
Trajectory displacement P50=0.000386167 m, P95=0.001759285 m,
maximum=0.002409850 m. These are differences, not accuracy errors.
Acceleration-bias maxima remain below 0.019761 m/s2. No truth consumed.
Evidence: use_cases/records/gsdc2023_pixel5_heading_test_sjcq_pair_20260928.json.
Reproducer: E:/rtklib_v2_ws_tmp/audit_pixel5_heading_test_sjcq_pair_20260928.py.
Supervisor 73504 active; controls 17772 and 18472 running.
Candidate 30384 terminal. Twenty-eight of 34 runs complete. No assembly or submission.

### Pixel5 test heading mtv-de1 control reproduced (2026-09-28)

2023-05-23-21-06-us-ca-mtv-de1/pixel5 control completed in 634.160 seconds.
Frozen provenance, diagnostic-only argv delta, native 1975-key coverage,
convergence and source-candidate trajectory byte parity pass.
Acceleration-bias maximum 0.022847224 m/s2. No truth consumed.
Evidence: use_cases/records/gsdc2023_pixel5_heading_test_mtvde1_control_20260928.json.
Reproducer: E:/rtklib_v2_ws_tmp/audit_pixel5_heading_test_mtvde1_control_20260928.py.
Supervisor 73504 active; sjc-be2 control 18472 and mtv-de1 candidate 19160 running.
Control 17772 terminal. Twenty-nine of 34 runs complete. No assembly or submission.

### Pixel5 test heading sjc-be2 control reproduced (2026-09-28)

2023-05-26-21-23-us-ca-sjc-be2/pixel5 control completed in 468.319 seconds.
Frozen provenance, diagnostic-only argv delta, native 1482-key coverage,
convergence and source-candidate trajectory byte parity pass.
Acceleration-bias maximum 0.036933377 m/s2. No truth consumed.
Evidence: use_cases/records/gsdc2023_pixel5_heading_test_sjcbe2_control_20260928.json.
Reproducer: E:/rtklib_v2_ws_tmp/audit_pixel5_heading_test_sjcbe2_control_20260928.py.
Supervisor 73504 active; sjc-be2 candidate 23520 and mtv-de1 candidate 19160 running.
Control 18472 terminal. Thirty of 34 runs complete. No assembly or submission.

### Pixel5 test heading sjc-be2 pair audited (2026-09-28)

2023-05-26-21-23-us-ca-sjc-be2/pixel5 candidate completed in 443.466 seconds.
Both arms pass provenance, native 1482-key coverage, control-source parity,
raw initialization parity and heading contract checks (exact option delta,
all-epoch seeds, unchanged graph/noise/first-bias-prior, no zero-bias init).
Trajectory displacement P50=0.000551218 m, P95=0.002163530 m,
maximum=0.002301790 m. These are differences, not accuracy errors.
Acceleration-bias maxima remain below 0.036935 m/s2. No truth consumed.
Evidence: use_cases/records/gsdc2023_pixel5_heading_test_sjcbe2_pair_20260928.json.
Reproducer: E:/rtklib_v2_ws_tmp/audit_pixel5_heading_test_sjcbe2_pair_20260928.py.
Supervisor 73504 active; mtv-de1 candidate 19160 and sjc-he2 control 59584 running.
Candidate 23520 terminal. Thirty-one of 34 runs complete. No assembly or submission.

### Pixel5 test heading mtv-de1 pair audited (2026-09-28)

2023-05-23-21-06-us-ca-mtv-de1/pixel5 candidate completed in 619.091 seconds.
Both arms pass provenance, native 1975-key coverage, control-source parity,
raw initialization parity and heading contract checks (exact option delta,
all-epoch seeds, unchanged graph/noise/first-bias-prior, no zero-bias init).
Trajectory displacement P50=0.000146962 m, P95=0.001274897 m,
maximum=0.002244323 m. These are differences, not accuracy errors.
Acceleration-bias maxima remain below 0.022848 m/s2. No truth consumed.
Evidence: use_cases/records/gsdc2023_pixel5_heading_test_mtvde1_pair_20260928.json.
Reproducer: E:/rtklib_v2_ws_tmp/audit_pixel5_heading_test_mtvde1_pair_20260928.py.
Supervisor 73504 active; final case sjc-he2 control 59584 running.
Candidate 19160 terminal. Thirty-two of 34 runs complete. No assembly or submission.

### Pixel5 test heading sjc-he2 control reproduced (2026-09-28)

2023-06-06-22-43-us-ca-sjc-he2/pixel5 control completed in 334.423 seconds.
Frozen provenance, diagnostic-only argv delta, native 1608-key coverage,
convergence and source-candidate trajectory byte parity pass.
Acceleration-bias maximum 0.019911058 m/s2. No truth consumed.
Evidence: use_cases/records/gsdc2023_pixel5_heading_test_sjche2_control_20260928.json.
Reproducer: E:/rtklib_v2_ws_tmp/audit_pixel5_heading_test_sjche2_control_20260928.py.
Supervisor 73504 active; final candidate 39208 running.
Control 59584 terminal. Thirty-three of 34 runs complete. No assembly or submission.

### Pixel5 heading test full audit complete (2026-09-28)

All 34 runs completed; supervisor 73504 exited 0 after the full 17-pair audit.
All controls reproduce their frozen sources, and all heading contracts pass.
No evaluation truth consumed; no official score or test accuracy established.
Execution done SHA256: 63f120d5922cb32ccc5770b3ed337f76996f1b1b3ce5b79527817682208b3047.
Full evidence: use_cases/records/gsdc2023_pixel5_heading_test_comparison_20260928.json.
Final sjc-he2 pair: 1608 native keys, candidate 293.341 seconds;
trajectory difference P50=0.000715594 m, P95=0.001931174 m, max=0.003235528 m.
Pair evidence: use_cases/records/gsdc2023_pixel5_heading_test_sjche2_pair_20260928.json.
Combined local assembly started in session 51901 using
scripts/analysis/assemble_gsdc_heading_modern_test.py: all 17 Pixel5 candidates,
all 9 modern candidates, and 14 retained native sources. No submission.

### Combined heading + modern candidate assembled (2026-09-28)

Session 51901 exited 0. The assembler reran all Pixel5 and modern audits and
verified provenance, published control parity, retained source hashes,
native output keys, and exact retained coordinate parity.
Output: E:/rtklib_v2_ws_output/gsdc_native/heading_modern_joint_submission_20260928/submission.csv
SHA256: cbd1fde10f0f317f7803871b7d00b64a73398498f967b86f2130d8ba8acc26f8.
Manifest: same directory, manifest.json.
71936 rows / 40 drives, 26 replaced (17 Pixel5 + 9 modern), 14 retained.
All rows native; no evaluation truth or reference coordinates used for inference.
Development evidence remains exposed development, not an official score:
Pixel5 heading fixed15 mean 0.644028218 -> 0.590451182 m;
modern fixed11 mean 0.699758890 -> 0.628204040 m.
Review caveat: ebf-z heading raises estimated accel-bias max to 0.214557085 m/s2;
this is not proof of accuracy regression. Uniform recipe retained across all 17.
No submission performed. Official Private remains 1.055 m; goal remains active.
Concrete candidate is ready for user review and explicit submission approval
under the previously recorded conversation preference.

### User authorizes submission and removes confirmation preference (2026-09-28)

User explicitly said: 提出していいよ。その希望いらない
This authorizes submission of the reviewed heading + modern candidate and
revokes the prior preference to ask before official submissions.
Do not ask for that confirmation again for continued work toward this goal.
Submission script: E:/rtklib_v2_ws_tmp/submit_gsdc_heading_modern_20260928.py.
Exact authorized SHA256: cbd1fde10f0f317f7803871b7d00b64a73398498f967b86f2130d8ba8acc26f8.
Submission session 49839 started; check its result/receipt before any retry.

### Official heading + modern result confirmed (2026-09-28)

Submission 56625084 accepted and COMPLETE, verified from Kaggle submissions API.
Private: 1.055 -> 0.984 m (improvement 0.071 m).
Public: 1.133 -> 0.915 m (improvement 0.218 m).
Goal <=0.928 m remains NOT achieved; remaining Private gap 0.056 m.
Receipt: E:/rtklib_v2_ws_output/gsdc_native/heading_modern_joint_submission_20260928/kaggle_receipt.json.
Readback: use_cases/records/gsdc2023_heading_modern_official_readback_20260928.json.
Submission session 49839 exited 0. Do not resubmit this exact candidate.
The user's prior submission-confirmation preference is revoked; future
submissions within this goal do not require repeating that permission request.
