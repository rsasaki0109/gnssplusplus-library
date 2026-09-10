# Smartphone R5 Phase133 launch-free qualification

- Execution label: `Luna Max`
- Qualification date: `2026-09-04`
- Candidate: `phase133-runner-native-selector-boundary-v1`
- Scope: static selector ownership, deterministic argv validation, synthetic
  fake-binary/help checks, and build-only qualification.

## Result

Phase133 launch-free qualification passed.  The fixed command snapshot
contains each native selector for Phase118, 126, 127, 128, 129, and 131
exactly once.  The Phase130 runner-only selector occurs zero times, as do the
Phase117 dynamic-sigma, Phase120 atmosphere, and additional-frequency
selectors.  The synthetic fake binary reports Phase130 as the only unknown
option when it is inserted into the historical Phase132 argv.

No real native `--help` or solver process was launched.  The native source
usage/parser was read and hash-checked, and the same ownership rule was
tested with synthetic help text.

## Checks

| check | result |
|---|---|
| Phase133 focused Python tests | `10/10 passed` |
| `py_compile` validator, facade, and tests | passed |
| `--verify-freeze` | passed |
| `--verify-manifest` | passed |
| `--verify-pre-raw` | passed |
| `cmake --build build --target gnss_run_tests -j2` | passed |
| native source changed | `false` |
| target binary hash changed | `false` |

The existing Phase132 full-CTest qualification baseline is retained without
rerunning solver-bearing tests: `166/192` passed and `26` remained failed.
The two `run_tests` failures were loader failures for missing
`libmetis-gtsam.so`; the other `24` were pre-existing historical sealed
source/binary pin expectations or the existing strict-doc warning.  These
issues are not hidden or reclassified as Phase133 failures.  The target build
above passed, and Phase133 adds no C++ source.

## Pins

| artifact | commit | SHA-256 |
|---|---|---|
| audit | `52d397880baea7854919d00bb69c39900af5b48d` | `43ba702f45f446e0c2268b6a638fd3972d87e14812ce9adc9409e82b979a6bad` |
| freeze | `8b1f75be0aaaadaedfffc5671be706dabd1106043` | `4dbf7bff3d9dc2537fd5fd88bf930d0c88a832bdd7f655dc47c4f5ed6b6110f6` |
| implementation | `f32e0132aa144cd583a93799f320779c9e399a76` | validator `f77ebfc5482e423939dec3eb84e5d33bc5ed23c4cb4dad57c94bfc7032616dde` |
| focused tests | `8a20e02b1625315c12ad6bc578a253801c1a759a` | `e1a20f31254a33f5fe5a0de3327cba2e050d2c759b393b56d45732f664723c5a` |
| manifest | `2ff98c1d1b751b1405fcc4c83e7f9893650672a4` | `ca90d461759589c8cab7df6a328f9ac6f450f77edb6ec85fda6bbaae90161a2f` |
| pre-raw accounting | `3e086ac6907e049c760071919e3a5533d336cec1` | `f1f17573e12527aa6ac96b811d2d7448f83e0b992ac1586b0805c71312cb156d` |
| native source | unchanged | `6fcee581af70535b8af09a674a234352abaaf83a34b722d4e91577abaf352170` |
| target binary | unchanged | `ed24b57b29f679123c3a52dbb04fad9b979092564abc4a35905bdb2abfaa0c8e` |

## Read accounting and boundary

Phase133 read accounting is zero for phone GNSS, phone IMU, broadcast nav,
raw-base payload/header, native solver, solution rows, truth, accuracy,
MAT/PDC/precomputed coordinates, Kaggle/token access, and rerun/fallback/
repair/sweep.  Only source text and sealed metadata were read.

An independent authorization is still required before materializing either
route's raw inputs.  If authorized later, the order is exactly MTV-A then
LAX-T, one run each, with raw phone GNSS/IMU, broadcast nav, and sealed raw
base RINEX only.  Unknown native options, preflight failure, or any structural
failure must stop before solver invocation; solution content remains opaque,
and truth/accuracy remain separate authorizations.
