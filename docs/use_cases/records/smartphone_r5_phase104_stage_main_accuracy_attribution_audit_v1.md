# Phase104 stage-vs-main accuracy attribution audit

- Execution label: `Luna Max`
- Audit date: `2026-09-03`
- Status: `sealed-read-only-audit-qualified-for-one-evaluation-only-stage-export-freeze`
- Scope: source text and sealed Phase82/Phase100/Phase103 aggregate metadata only
- Raw, truth, MAT, solver, and Kaggle execution: not performed

## Finding

The sealed accuracy records establish a final-main regression relative to the
same-route Phase82 reference, but they do not establish whether the regression
was already present in GNSS-first or was introduced by the IMU/main solve. The
two-route values are:

| Route | Phase82 same-route | Phase100 QR scalar-clock | Phase103 corrected evaluator |
|---|---:|---:|---:|
| `2021-03-16-18-59-us-ca-mtv-a/pixel5` | `1.1139384500152307 m` | `1.147436714982201 m` | `1.139793072101309 m` |
| `2022-04-01-18-22-us-ca-lax-t/pixel5` | `0.9389644134001871 m` | `3.3204375254720286 m` | `3.310820065300743 m` |

The Phase103 two-route macro is `2.225306568701026 m`, versus
`1.026451431707709 m` for the same-route Phase82 aggregate and
`2.2339371202271145 m` for Phase100. Both Phase103 route gates failed closed;
the strict `0.782 m` macro gate also failed. These are final native solution
scores. They cannot be used as GNSS-first scores, and GNSS-first cost reduction
cannot be substituted for the Haversine truth metric.

The Phase103 sealed telemetry contains GNSS-first iterations, costs, C0D factor
counts, and finite optimized-D counts, but no GNSS-first position/timestamp
trajectory that can be parsed by the Phase82 metric contract. Consequently,
stage-vs-main accuracy attribution is currently unobserved, not disproven.

## Source and native staging evidence

Only source text was inspected; `load(...)` statements in the official MATLAB
files were not executed and no MAT payload was opened.

1. `fgo_gnss.m` declares the preprocessed-data load at lines 22--23, builds
   epoch-local `x`, `v`, seven-component `c`, and `d` initial states at
   lines 34--48 and 89--94, adds the pseudorange/Doppler and CCDD factors at
   lines 103--201, optimizes with Levenberg--Marquardt at lines 203--214,
   retrieves `xest`, `vest`, `clkest`, and `dclkest` at lines 220--230, and
   saves the GNSS-first result at line 252.
2. `fgo_gnss_imu.m` declares its preprocessed-data load at lines 22--23 and,
   when `initflag` is true, loads `result_gnss.mat` and copies `posest`,
   `velest`, `clkest`, and `dclkest` at lines 40--48. Its Pose3, position,
   velocity, clock, drift, and bias states are inserted at lines 165--187;
   the IMU/main graph is optimized at lines 325--339 and all estimated state
   vectors are retrieved at lines 342--357. This identifies the official
   stage-to-main boundary, but it is not an allowed native input path for this
   phase.
3. In the native Android raw path, the Phase101 GNSS-first result is retained
   in `gnss_first_result`. After exact-key/D/C validation, existing code copies
   the stage position and receiver-clock values into `problem.epochs` and
   hands off optimized D/C vectors at `apps/native/gnss_fgo_imu_no_base.cpp`
   lines 6753--6791. This is the existing in-memory handoff and must remain
   untouched.
4. The main IMU solve then runs at lines 6871--6930. The normal output path
   constructs `output_positions` from `result.solution.solutions` at
   lines 7227--7273 and publishes only that main solution at lines 7275--7310.
   There is no stage trajectory export at this boundary. The safe observation
   point for the frozen candidate is therefore after the existing handoff and
   after main optimization, while `gnss_first_result` is still in scope: copy
   stage timestamps/positions and main positions into evaluation-only
   diagnostics, without writing either copy back to `problem`, `result`, or
   any factor/initializer.

## Fact versus inference

Facts supported by the sealed artifacts and source:

- Phase103's candidate CSV and truth scoring describe the main native output;
  no stage CSV was scored.
- The native graph already performs a same-run GNSS-first solve and in-memory
  handoff before the IMU/main solve in the Phase101 opt-in path.
- The official source has a GNSS-only result that is consumed as the IMU
  initialization, while the native raw-only implementation represents that
  transfer in memory.
- The current public metric is exact `(phone, UnixTimeMillis)` matching,
  spherical Haversine distance, and the fixed route/alignment rules sealed by
  Phase82/Phase100/Phase103.

Inferences that are not yet proven:

- The LAX-T increase from `0.938964... m` to `3.31--3.32 m` may originate in
  GNSS-first, the main handoff, or the IMU/main optimization; the current
  aggregate cannot distinguish them.
- A stage-vs-main comparison may localize the degradation, but it does not
  guarantee a fix or promotion.

## Exactly one diagnostic candidate

Freeze one default-off, evaluation-only export:

`phase104-evaluation-only-gnss-first-stage-export-v1`

On a later fresh Phase101 raw-only run for exactly MTV-A and LAX-T, after the
existing GNSS-first validation and in-memory C/D/position/velocity handoff and
after main optimization, copy the same-run GNSS-first position and timestamp
sequence into a private stage sidecar. Use exact retained source/UTC keys and
the existing output schema; do not interpolate, reorder by row, re-solve, or
read a MAT/precomputed trajectory. In the same post-solve observer, derive
compact main displacement statistics from consecutive main ECEF positions and
their retained timestamps. The statistics are diagnostics only and contain no
coordinate rows.

The later truth-only evaluator must open each route's official truth exactly
once, after both the sealed main solution and stage sidecar have passed schema,
hash, exact-key, finite, Earth-valid, duplicate, and coverage checks. It must
score stage and main with the identical Phase82 metric and alignment contract,
then report whether the observed degradation is present before or after the
handoff. It must not feed the sidecar or any score back into inference.

Rejected alternatives are not additional candidates: modifying the handoff or
graph, running a second GNSS-first solver, reading official MAT outputs, or
using truth during native execution would change the experiment boundary.

## Read/process accounting

| Activity | Count |
|---|---:|
| Native solver invocations | `0` |
| Raw GNSS/IMU/navigation byte reads | `0` |
| Truth-file reads | `0` |
| Phase100/Phase103 solution payload reads | `0` |
| Phase82/100/103 sealed aggregate metadata reads | `3` |
| Official source-code reads | `2` MATLAB files plus native source |
| MAT/base/PDC/precomputed-coordinate reads | `0` |
| Accuracy calculations | `0` |
| Kaggle/token access | `0` |
| Reruns/fallbacks/tuning | `0` |

This audit does not authorize raw execution, truth evaluation, solution
publication, validation, release, or Kaggle submission. A later run requires a
separate freeze-compliant implementation/manifest and one-shot authorization.

## Pinned audit authorities

| Authority | Commit/path | SHA-256 |
|---|---|---|
| Phase82 sealed result | `docs/use_cases/records/smartphone_r5_phase82_phase80_direct_quality_accuracy_result_v1.json` | `39c5a38b7e3e61fda0306582fffb961dad3e6fb0bd49fcfe70e66674e2c90873` |
| Phase100 sealed result | `docs/use_cases/records/smartphone_r5_phase100_phase99_qr_accuracy_result_v1.json` | `7acd77e948f17de96f8dd24a7c137e708f9a7f5329dbef52c3a78d8cca40b2b7` |
| Phase103 sealed result | commit `3b9ca8c` / `docs/use_cases/records/smartphone_r5_phase103_phase102_truth_only_evaluator_correction_result_v1.json` | `1a909a9b91ea90517fac3f138faa363fd5a601e33e4361b2017e4873e8873688` |
| Official GNSS source | `output/reproducibility-cache/gsdc2023/fgo_gnss.m` | `5368e4056f70f448b728792c5e4c124b7f2afc1e00653492b807083bd3ccf0c3` |
| Official GNSS/IMU source | `output/reproducibility-cache/gsdc2023/fgo_gnss_imu.m` | `c70090ccb8b27fc8ac7fd2929e2f995a14cfe7f089bb1fef067370e22051c3e3` |
| Native staging source | `apps/native/gnss_fgo_imu_no_base.cpp` | `72269b057073c0b0134b069a7436c8c3979504807b94ec7b477b7c4fa6f99339` |

