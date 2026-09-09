# Phase139 structural raw result seal

This sidecar seals the single authorized Phase139 run.  It reports scalar
structural telemetry and opaque output hashes only; no solution coordinate,
truth, MAT, PDC, precomputed-coordinate, accuracy, or Kaggle content was
read or published.

## Authorization and execution

- Independent authorization commit:
  `9fd1c65e44bf83c48d69febecbc2f96c23e060da`
- Authorization JSON SHA-256:
  `35cb6bb1d31248b4232a9d7efa53248c2f7344d2af2da249924c11aa66a28232`
- Wrapper entry point was invoked exactly once.  The native binary was
  invoked exactly once for MTV-A and exactly once for LAX-T, in that order.
- Phase135 affine, Phase138 anchor correction, Phase118 fixed TDCP
  sigma/Huber, and Phase107 raw-base compensation were active.  Phase117,
  Phase120, Phase126--134, and additional-frequency selectors were inactive.

## Route telemetry

Both native processes returned zero.  The structural validator then failed
closed on the same representation-only contract mismatch: native emitted
`tdcp_native-(rho_current_initial-rho_previous_initial)`, while the frozen
validator requires
`tdcp_phase138 = tdcp_native - (rho_current_initial - rho_previous_initial)`.
No retry, repair, fallback, or rerun was performed.

### MTV-A

- Native invocation: 1; raw GNSS/IMU/nav/base hashes: 1 each; return code: 0.
- Phase135: affine configuration valid; pseudorange 43259, Doppler 20748,
  ordinary TDCP 31269, Pose3-X bridge 2159, geometry rows 95276; fixed LOS,
  single-Sagnac witness, and partial-family=false.
- Phase138: range constants 31269, adjusted measurements 31269, application
  passes 1, adjusted exactly once, factor count unchanged, fixed endpoint and
  satellite-state geometry.  The equation text mismatch above caused the
  validator NO-GO.
- Raw base: correction exactly once; adopted 57281, corrected 43259,
  missing-stream misses 12894, out-of-domain misses 1128, corrected factor
  count consistent, no extrapolation/endpoint hold.
- Clock: C7 enabled/exported, D exported, dimension 7, exact epoch alignment.
- Solver: `MULTIFRONTAL_QR`/`EliminateQR`; GNSS-first 111 accepted iterations,
  cost `5626399.6192836035 -> 23229.685424845655`; main 12 iterations,
  cost `22174262.911254589 -> 28127.4619404445` (native accepted-count field
  was absent in the summary).
- Output: 2158 target and 2158 exact epochs, zero interpolated/edge-hold/
  unresolved, finite=true, Pixel5 offset applied once to 2159 epochs.
- Opaque solution seal only: SHA-256
  `e1f8211b16a413622c95903e73d5b5f17fff0ee9acdb28bdce5bdd15708b229c`,
  172694 bytes, 2159 newline records; content interpreted=false.

### LAX-T

- Native invocation: 1; raw GNSS/IMU/nav/base hashes: 1 each; return code: 0.
- Phase135: affine configuration valid; pseudorange 30664, Doppler 8942,
  ordinary TDCP 14012, Pose3-X bridge 1466, geometry rows 53618; fixed LOS,
  single-Sagnac witness, and partial-family=false.
- Phase138: range constants 14012, adjusted measurements 14012, application
  passes 1, adjusted exactly once, factor count unchanged, fixed endpoint and
  satellite-state geometry.  The equation text mismatch above caused the
  validator NO-GO.
- Raw base: correction exactly once; adopted 30798, corrected 30664,
  missing-stream misses 0, out-of-domain misses 134, corrected factor count
  consistent, no extrapolation/endpoint hold.
- Clock: C7 enabled/exported, D exported, dimension 7, exact epoch alignment.
- Solver: `MULTIFRONTAL_QR`/`EliminateQR`; GNSS-first 116 accepted iterations,
  cost `6924674.3471763274 -> 14673.816581930669`; main 12 iterations,
  cost `49844452.31604182 -> 18147.802211554335` (native accepted-count
  field was absent in the summary).
- Output: 1465 target and 1465 exact epochs, zero interpolated/edge-hold/
  unresolved, finite=true, Pixel5 offset applied once to 1466 epochs.
- Opaque solution seal only: SHA-256
  `85f5610f3deffae9912ed9ea026be2b4a4e0824064516e6c93e591255c1f4fcd`,
  117254 bytes, 1466 newline records; content interpreted=false.

## Sealed artifacts and accounting

- Result JSON:
  `docs/use_cases/records/smartphone_r5_phase139_affine_tdcp_structural_raw_result_v1.json`
- Native summary SHA-256: MTV-A
  `7a84d6aa4ea25faf1560f15813c894db3e75c858fde36b0588ae851129540ec6`;
  LAX-T `4c6f6f4a221ccbcd9146cc6236d2c20491e03daad01338c4a5268d051d6ba727`.
- Aggregate reads: raw phone GNSS 2, raw phone IMU 2, broadcast navigation 2,
  raw base RINEX 2, native solver invocations 2.  Truth, solution-coordinate,
  MAT/PDC/precomputed, accuracy, Kaggle/token, and rerun/fallback/repair/
  sweep counts are all zero.
- Overall structural gate: **NO-GO**, preserved without modification or
  retry.  A future run would require a new authorization if any wrapper or
  validator changes; this result is historical and immutable.
