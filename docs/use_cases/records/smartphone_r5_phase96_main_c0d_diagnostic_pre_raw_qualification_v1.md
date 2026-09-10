# Phase96 main C0/D diagnostic pre-raw qualification

- Execution label: `Luna Max`
- Qualification status: `pre-raw qualified`
- Qualification date: `2026-09-03`
- Raw route execution in this record: **0**
- Native route solver invocations in this record: **0**
- Accuracy, truth, MAT, base, precomputed-coordinate, Kaggle/token activity: **0**
- Additional large build artifact: **none**; the existing `build/` binary was reused.

## Freeze and implementation audit

The manifest pins Phase96 freeze commit `d4c640312568bffc01f21282767d33756a1ef1a9`
(`d4c6403`) and freeze SHA-256
`9605f3ce32ad08ae3f17254834df50a65cd93918cee593b487a02ad7b97ebc65`.
The implementation under qualification is commit
`bcdcd952943623650cdf77410c17e9ec87f45d7f` (`bcdcd95`).  Its native source
SHA-256 is
`8e71268904fde04db14db8d9357aea97a9b6eed62b67ffd15fc2acbd7865799f` and its
existing binary SHA-256 is
`60e244d27fa48bdc962dfeb63047314f16cbed2ce09fc93040bced0bcd790009`.

`git diff --check d4c6403..bcdcd95` is clean.  The implementation diff is
limited to the Phase96 public diagnostic types/configuration, read-only graph
and existing-LM telemetry, native diagnostic serialization/opt-in validation,
and focused tests.  The official C0/D equation, Jacobian order, metre and
metres-per-second units, sigma `0.1 m`, filters, LM parameters, acceptance
branches, fallback policy, and legacy default remain pinned unchanged.  The
Phase96 selector is default-off and is not enabled by any legacy command.

## Qualification tests

The already-built targets were used without a rebuild:

| Check | Result | Raw route execution |
|---|---:|---:|
| `LD_LIBRARY_PATH=/home/sasaki/.local/lib:build:build/tests build/tests/run_tests` | 1104 run; 1046 passed, 58 fixture skips, 0 failed; exit 0 | no |
| Focused C++ `FGOGtsamPhase93Test.*`, active-solve, and `FGOGtsamPhase96DiagnosticTest.*` filter | 6 passed, 0 failed; exit 0 | no |
| `python3 -m unittest tests.test_smartphone_phase94_source_clock_c0d_stage_diagnostics` | 6 passed, 0 failed; exit 0 | no |
| `python3 -m unittest tests.test_smartphone_phase96_main_c0d_diagnostic` | 6 passed, 0 failed; exit 0 | no |
| Phase96 evaluator `--verify-pre-raw` | passed; raw reads 0, native invocations 0 | no |

The 58 C++ skips are fixture-dependent live/kinematic tests whose bundled
data is unavailable; they are not Phase96 route failures.  No raw route file
was opened or hashed by these qualification checks.

## Archive path record

The two old temporary build archives were moved reversibly to make room for
the existing build.  The original paths are now absent and the current paths
are present.  No user or repository data was deleted.

| Artifact | Original path | Current path | Original bytes | Current bytes |
|---|---|---|---:|---:|
| GNSS library archive | `/tmp/phase38-debug.PhRaoH/libgnss_lib.a` | `/dev/shm/phase96-build-space/libgnss_lib.a` | 445044674 | 445044674 |
| solver library archive | `/tmp/phase38-debug.PhRaoH/libgnss_lib_solvers.a` | `/dev/shm/phase96-build-space/libgnss_lib_solvers.a` | 539687964 | 539687964 |

The manifest records `user_data_deleted: false`, original-path absence, and
current-path presence.  These archive paths are not inputs to the diagnostic
execution.

## Raw path qualification (metadata only)

The Phase96 manifest uses role placeholders under `raw/phase93/`.  After the
separate authorization, the wrapper will resolve each selected route and raw
filename against the pinned Phase91 manifest, call only `stat`, and pass the
resolved path to the native process.  It will not copy, transform, hash, or
open raw bytes.

| Route | Raw file | Pinned Phase91 path | Current bytes |
|---|---|---|---:|
| MTV-A | `device_gnss.csv` | `output/smartphone-r5/phase25-raw-clock-eval-v1/raw/2021-03-16-18-59-us-ca-mtv-a/pixel5/device_gnss.csv` | 57715495 |
| MTV-A | `device_imu.csv` | `output/smartphone-r5/phase25-raw-clock-eval-v1/raw/2021-03-16-18-59-us-ca-mtv-a/pixel5/device_imu.csv` | 34393802 |
| MTV-A | `brdc.nav` | `output/smartphone-r5/phase25-raw-clock-eval-v1/raw/2021-03-16-18-59-us-ca-mtv-a/pixel5/brdc.nav` | 9955040 |
| LAX-T | `device_gnss.csv` | `output/smartphone-r5/phase37-pixel5-repeatability-v1/routes/2022-04-01-18-22-us-ca-lax-t/pixel5/inputs/device_gnss.csv` | 33837317 |
| LAX-T | `device_imu.csv` | `output/smartphone-r5/phase37-pixel5-repeatability-v1/routes/2022-04-01-18-22-us-ca-lax-t/pixel5/inputs/device_imu.csv` | 23649244 |
| LAX-T | `brdc.nav` | `output/smartphone-r5/phase37-pixel5-repeatability-v1/routes/2022-04-01-18-22-us-ca-lax-t/pixel5/inputs/brdc.nav` | 10635773 |

Only MTV-A and LAX-T are in scope.  MTV-H and MTV-U remain excluded by the
Phase96 freeze and are neither staged nor executed.

## Execution boundary

This qualification seals the manifest only.  A separate one-shot
authorization must pin the final manifest hash before the wrapper can launch
exactly two sequential native invocations.  The future result may contain
factor-family costs, per-variable/family gradient and normal-diagonal norms,
and at most ten existing LM trials per route.  It must contain no solution
rows or coordinates, and must stop before truth, accuracy, or submission.
