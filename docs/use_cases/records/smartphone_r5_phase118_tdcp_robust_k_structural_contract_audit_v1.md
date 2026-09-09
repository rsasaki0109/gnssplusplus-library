# Phase118 TDCP robust-k structural raw contract audit

Status: contract-only audit after the Phase118 implementation.  This record
does not authorize a raw read, native solver invocation, solution inspection,
truth evaluation, MAT access, accuracy calculation, or Kaggle action.  The
future matrix is exactly one candidate on the two primary routes MTV-A and
LAX-T, one invocation per route, sequentially.

## Decision and boundary

The implementation and source-parity freeze are pinned, but execution remains
closed until an independent raw authorization is created later.  The only
new selector is:

```text
--native-phase118-official-tdcp-huber-k
```

It is default-off.  The structural contract composes it with the already
sealed Phase112 champion recipe:

* Android raw UTC-key GNSS and IMU input, all epochs, raw clock-only mode, and
  UTC wall-clock fallback;
* the existing no-bridge, direct source P+D quality, source `ClockFactor_CCDD`
  meter-state, active-solve diagnostic, GNSS-first in-memory C7/D handoff,
  epoch-local C-vector, and Phase99 `MULTIFRONTAL_QR` selectors;
* the existing raw-base RINEX correction, exact-stream miss mask, and
  additional-frequency-band admission selectors;
* the existing final Pixel5 position offset selector.

Phase116 incidence telemetry is deliberately omitted from this contract.  It
is not part of the Phase112 recipe and is not needed to select or apply the
Phase118 scalar.  Phase117 dynamic TDCP sigma is also omitted and is forbidden
by the implementation admission checks.

The candidate changes one value only: the robust threshold passed to the
existing ordinary adjacent same-satellite/same-signal TDCP noise wrapper.
The fixed Phase112 TDCP factor sigma remains `0.03 m`.  Route setting.Type is
the only selector input:

| route | pinned source Type | expected ordinary TDCP Huber k |
| --- | --- | ---: |
| `2021-03-16-18-59-us-ca-mtv-a/pixel5` | `Highway` | `0.5` |
| `2022-04-01-18-22-us-ca-lax-t/pixel5` | `Highway` | `0.5` |

The implementation's unit mapping also covers `Street` and `Mix` as `0.2`,
and rejects an empty or unknown Type.  No route score, truth value, result
coordinate, or data payload is consulted for Type selection.

## Pins and read boundary

The contract must pin all of the following before authorization:

| item | pin |
| --- | --- |
| Phase118 source-parity freeze | commit `5fcc06ab64189dd5dfb8001664bdb8496f85224f`; SHA-256 `c6f4fda2abe170417610d4fd1a4ae8a1a618d07096ff0432669081ab06a1cf77` |
| Phase118 implementation | commit `7f339ccc8f0fb58e3dbcb7f3fc24ba2b04acbf40` |
| Phase112 source manifest | `docs/use_cases/records/smartphone_r5_phase112_main_output_offset_manifest_v1.json`; SHA-256 `d8be91f42f07196e07b293558cc6b83dd261226af577c1b26b167eead75902e4` |
| Phase112 structural baseline | `docs/use_cases/records/smartphone_r5_phase112_main_output_offset_structural_result_v1.json`; SHA-256 `087b1bf51b4e584b86a7b558560df4ceb78e19b01e572f2d60aeef4c3d759ba0` |
| built native target used for qualification | `build/apps/gnss_fgo_imu_no_base`; SHA-256 `fe6c0b501921a8628fb813abe380b84c89824409e731bf7cfd01920d900dea34` |

The implementation source hashes at the pin are:

| source | SHA-256 |
| --- | --- |
| `apps/native/gnss_fgo_imu_no_base.cpp` | `43e9fbbe8a0043f648b52b02e63be36b8960e412a989d21cfd345c8b8a9312d5` |
| `include/libgnss++/algorithms/fgo.hpp` | `efa415e3e09a43f354cba1251e0d92d2fd9c21db70a945bfccf08219773bd90c` |
| `include/libgnss++/algorithms/fgo_config.hpp` | `38ae28a5bdb398b0a764f9f09f8456107ddc0168654b1d1791879377cca24748` |
| `src/algorithms/fgo.cpp` | `32f83b213f2e30b0d4a8200a8b4a0793eeb86721013693194ba48381db0a28c9` |
| `src/algorithms/fgo_gtsam_backend.cpp` | `43b80e75638f1b2d70907f1a9b36f6ee486c070c765ba733d852a022f2a58d2b` |
| `tests/test_fgo.cpp` | `da1f92b2ae5565da0ad6382caa780a1195e165c100ba221a2faac7a1a65380ac` |
| `tests/test_smartphone_phase118_tdcp_robust_k.py` | `1920b5a7cfac8d71349e43750ae638b676cf85fe82c3b07a8eae2e9fe22fd9c3` |
| `tests/CMakeLists.txt` | `e8fd5a9787b1e872ac1fbe42561a89aaac666af0b281237790f02aa814e14f4e` |

The pre-raw contract and validator must read sealed JSON metadata and tracked
source only.  Before an independent authorization, all payload activity must
remain zero:

| resource/action | pre-raw count |
| --- | ---: |
| phone `device_gnss.csv` reads | 0 |
| phone `device_imu.csv` reads | 0 |
| broadcast `brdc.nav` reads | 0 |
| raw-base RINEX reads or hashes | 0 |
| native solver invocations | 0 |
| solution rows opened or published | 0 |
| truth/MAT/accuracy activity | 0 |
| phone or precomputed coordinates | 0 |
| PDC/Kaggle/token access | 0 |
| reruns/fallbacks | 0 |

Raw path and base metadata are inherited from the sealed Phase112 manifest;
the contract must not stat, hash, open, copy, or transform those members
while it is being verified.

## Exact future matrix

The future manifest has two route records, in this order, with
`runs_per_route = 1`, `controls = 0`, `reruns = 0`, and `fallbacks = 0`:

| dataset ID | domain rows | expected problem epochs | expected output epochs |
| --- | ---: | ---: | ---: |
| `2021-03-16-18-59-us-ca-mtv-a/pixel5` | 2158 | 2159 | 2159 |
| `2022-04-01-18-22-us-ca-lax-t/pixel5` | 1465 | 1466 | 1466 |

The exact command template for each route is the Phase112 command with the
single Phase118 selector inserted once:

```text
build/apps/gnss_fgo_imu_no_base
  --dataset-id <route>
  --android-gnss __PHASE118_RAW_DEVICE_GNSS__
  --android-imu __PHASE118_RAW_DEVICE_IMU__
  --nav __PHASE118_RAW_BROADCAST_NAV__
  --all-epochs
  --android-raw-utc-keys --android-raw-clock-only
  --android-utc-wall-clock-fallback
  --native-pdc-imu-tdcp-no-bridge
  --native-source-direct-observable-quality
  --native-source-clock-c0d-factor
  --native-source-clock-c0d-meter-state-parity
  --native-source-clock-c0d-active-solve-diagnostic
  --native-source-clock-c0d-gnss-first-meter-state-handoff
  --native-source-clock-c0d-epoch-vector-parity
  --native-source-clock-c0d-phase99-main-multifrontal-qr-solver
  --native-base-pseudorange-compensation
  --native-base-pseudorange-source-miss-mask
  --native-base-pseudorange-preserve-additional-frequency-bands
  --native-upstream-position-offset
  --native-phase118-official-tdcp-huber-k
  --native-base-rinex __PHASE118_RAW_BASE_RINEX__
  --native-base-rinex-sha256 __PHASE118_RAW_BASE_SHA256__
  --out output/smartphone-r5/phase118-tdcp-robust-k-v1/<route>__pixel5/withheld_solution_output.csv
  --summary-json output/smartphone-r5/phase118-tdcp-robust-k-v1/<route>__pixel5/summary.json
```

The literal manifest should use one token per argv item and exact placeholder
replacement only after authorization.  It must reject `--obs`, the Phase117
dynamic sigma selector, all alternate seed/quality/diagnostic switches, any
truth/MAT/PDC/precomputed/coordinate/Kaggle path term, duplicate flags, and
any command differing from the pinned template.

## Proposed runner and validator changes

These are contract requirements for the next implementation step, not changes
authorized by this audit commit.

1. Copy the Phase117 launch-free contract shape to a Phase118 contract module.
   Pin the Phase118 freeze, implementation commit, Phase112 manifest, Phase112
   structural result, evaluator, wrapper, focused test, and target binary.
   `verify_freeze()` must require one candidate, default-off, raw execution
   false, fixed sigma `0.03`, official mapping `Street/Mix=0.2` and
   `Highway/other official type=0.5`, and Phase117 dynamic sigma false.

2. `verify_implementation()` must hash the pinned source/build artifacts and
   require markers for the CLI selector, config resolver, ordinary TDCP noise
   call, fixed-sigma wiring, result metadata, and the Phase118 mapping test.
   It must not execute the native target.

3. `command_template()` and `validate_command()` must enforce the exact
   two-route Phase112 argv plus one Phase118 selector.  They must require
   Android raw placeholders, broadcast navigation, raw-base placeholders,
   exact output/summary locations, and no Phase116/117 selector.  A source
   route's Type must resolve from the pinned route table only; unknown route or
   Type fails closed.

4. `verify_pre_raw()` must return zero payload reads, zero hashes of raw/base
   members, zero solver/evaluator invocations, no solution publication, and
   the freeze/implementation hashes.  It must not call `stat`, `open`, or
   `sha256` on raw phone/base/solution members.

5. The future wrapper, after a separate authorization only, may stat the
   exact raw phone members, hash the sealed base RINEX exactly once for
   identity, and pass the original paths directly to the child.  It must not
   copy/transform payloads or expose truth/MAT/coordinate environment
   variables.  The child environment should be a minimal fixed environment
   with the pinned trusted library path.

6. The future structural result may seal native return code, summary hash,
   opaque withheld-output hash/header/row count, and structural telemetry.  It
   must not parse or publish solution coordinates.  A failed run is sealed
   unchanged and is not retried.

## Structural gates

The result validator must fail closed unless every route satisfies the
following.  “Unchanged” means compared to the sealed Phase112 recipe and the
implementation/source contract, not inferred from truth.

### Selector, Type, and noise contract

* Phase118 selector is present exactly once and the default-off path remains
  unchanged.
* `native_phase118_official_tdcp_huber_k` and
  `official_tdcp_huber_k_enabled` are true in the candidate result.
* `official_setting_type` is `Highway` and `official_huber_k` is exactly
  `0.5` for both authorized routes.
* `official_snr_type_sigma_enabled` is false; no Phase117 dynamic sigma is
  active.
* `fixed_sigma_m` and every ordinary TDCP factor sigma are finite and exactly
  `0.03 m`.
* The robust threshold applies only to ordinary TDCP factors.  P, Doppler,
  carrier, single-difference TDCP/Doppler, C0D, and all other robust paths
  retain their pinned thresholds.

### TDCP topology and admission

* Ordinary TDCP factor family, equation, metre units, `(satellite, signal)`
  adjacent key order, and endpoint convention are unchanged.
* For MTV-A, the sealed baseline count is 31,269 built/inserted factors,
  33,025 candidate pairs, 0 gap rejects, 1,756 code-phase-jump rejects,
  0 clock-discontinuity/invalid-measurement/invalid-weight/loss-of-lock
  rejects, and 4,282 missing-previous rows.
* For LAX-T, the sealed baseline count is 14,012 built/inserted factors,
  14,961 candidate pairs, 0 gap rejects, 949 code-phase-jump rejects,
  0 clock-discontinuity/invalid-measurement/invalid-weight/loss-of-lock
  rejects, and 2,373 missing-previous rows.
* Candidate and inserted/reject counts match the sealed baseline exactly;
  all residuals and robust costs are finite.  A count or admission change is
  a structural failure, not a reason to tune or retry.

### GNSS-first and main graph

* GNSS-first is attempted with finite initial/final cost and strict decrease,
  at least one accepted outer iteration, exact retained-epoch identity, full
  finite optimized C7 and D exports, and the existing in-memory handoff.
* Main uses the selected `MULTIFRONTAL_QR` branch, reaches at least one
  accepted iteration, has finite initial/final cost with strict decrease, and
  receives the exact C7/D handoff and existing position/velocity states.
* No global ISB double state, fallback initializer, alternate solver, or
  failed-step recovery is admitted.
* Position/clock values and output rows are finite, earth-valid, and cover all
  expected epochs; no solution coordinate is exposed in the structural result.

### Base, offset, provenance, and release boundary

* Raw-base correction is active exactly once, with the sealed raw RINEX path,
  bytes/header/hash accounting, source miss-mask and no duplicate correction.
* Pixel5 final position offset is applied exactly once at the final output
  boundary and never to GNSS-first staging or main initialization.
* Raw inputs are only phone GNSS/IMU, broadcast navigation, and the separately
  sealed raw base RINEX.  No MAT, truth, base-derived phone coordinate,
  precomputed coordinate, PDC, accuracy, or Kaggle resource is read.
* Structural GO does not authorize truth evaluation or accuracy scoring.
  Solution rows remain isolated/withheld; a later truth-only authorization is
  required.  The matrix has exactly two native invocations, no control runs,
  reruns, or fallbacks.

## Qualification already completed

The implementation pin was qualified before this contract draft: the existing
GTSAM Release target built successfully, the full C++ suite reported 1,131
tests from 184 suites with 1,073 passed, 58 fixture skips, and 0 failures, and
the Phase118 focused Python contract test passed 4/4 (CTest 1/1).  Those tests
were launch-free with respect to raw data and truth.  This contract audit adds
no execution or result claim.

## Authorization procedure

Authorization must be a separate commit after this audit and its freeze JSON
are pinned:

1. Verify the audit commit, the Phase118 source-parity freeze hash, the
   implementation commit/hash, Phase112 manifest/baseline hashes, and the
   exact target binary hash.
2. Run the launch-free Phase118 validator and focused tests; require all
   pre-raw counters to be zero and the exact argv templates to pass.
3. Create one independent raw structural authorization naming only MTV-A and
   LAX-T, one sequential invocation each, and no truth/accuracy/solution
   release.  The authorization must be a new commit and must pin the final
   manifest, validator, wrapper, focused tests, and binary.
4. Only after that authorization may the wrapper materialize exact raw paths
   and invoke the native target.  On either route failure, seal the failure
   telemetry and stop; do not rerun, fallback, inspect truth, or publish
   solution rows.
