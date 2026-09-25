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
