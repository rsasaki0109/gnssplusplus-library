# Phase120 TDCP equation-normalization structural contract audit

Status: launch-free contract audit.  The source candidate is pinned by the
Phase120 freeze at `8e5dd41bebef64a93361a25db6275799452720f2`; its implementation
is pinned by `3ea6b2caf5c6f5be78c18e388e49301b961d3036`.  This audit creates no
raw execution, solver, truth, accuracy, MAT, coordinate, PDC, or Kaggle
authority.

## Decision and exact boundary

The Phase120 candidate is admitted for one future structural matrix only:

* exactly one candidate, with the selector
  `--native-phase120-official-tdcp-resl-atmosphere-cancellation`;
* Phase118 official TDCP Huber-k selector enabled, Phase117 dynamic sigma
  disabled, and fixed ordinary TDCP sigma `0.03 m`;
* MTV-A followed by LAX-T, one invocation per route, with no controls,
  retries, reruns, fallback, or tuning;
* phone `device_gnss.csv`, phone `device_imu.csv`, broadcast `brdc.nav`, and
  the already sealed raw-base RINEX are the only future input members;
* solution output is sealed only as opaque metadata/hash; solution rows are
  not interpreted or published, and truth remains a separate later lane.

The implementation changes the ordinary TDCP measurement preparation only.
The legacy corrected carrier and every non-ordinary carrier consumer remain
unchanged.  For an accepted adjacent pair, the future Phase120 measurement is

```text
tdcp_measurement_m = carrier_phase_cycles * retained_wavelength_m
                     + satellite_clock_m
delta_carrier_m = current.tdcp_measurement_m - previous.tdcp_measurement_m
```

The existing pair/reject contract is evaluated with the historical prepared
carrier delta (`corrected_carrier_m`), not the new factor measurement.  This
keeps the same-satellite/same-signal lookup, gap, clock-discontinuity,
loss-of-lock, finite, and code-phase-jump predicates and therefore makes the
factor/reject population an explicit structural gate.  No atmosphere term is
reintroduced through fallback: missing/nonfinite core carrier or satellite
clock values are fail-closed.

The preserved legacy expression is:

```text
corrected_carrier_m = carrier_phase_cycles * wavelength_m
                      + satellite_clock_m
                      - troposphere_delay_m + ionosphere_delay_m
```

Only the ordinary TDCP factor's observed delta is switched when the opt-in is
true.  Standalone carrier factors, double-difference carrier factors,
pseudorange, Doppler, C7/D/C0D state topology, endpoint geometry, QR solver,
raw-base correction, Pixel5 final offset, IMU/preintegration, filtering,
initialization, LM settings, robust-k selection, and all output alignment are
outside the candidate boundary.

## Pinned provenance

The source-only Phase120 freeze is:

* `docs/use_cases/records/smartphone_r5_phase120_tdcp_equation_normalization_freeze_v1.json`;
* commit `8e5dd41bebef64a93361a25db6275799452720f2`;
* SHA-256 `5087cbcd052de828b703659d4eed847ca7711481a7b7656af80013b9d0fc6359`.

The implementation commit is `3ea6b2caf5c6f5be78c18e388e49301b961d3036`.
The sealed Phase118 recipe/result provenance used by the future contract is:

| artifact | SHA-256 |
| --- | --- |
| `docs/use_cases/records/smartphone_r5_phase118_tdcp_robust_k_structural_manifest_v1.json` | `e0a4a0e879a56d7336fe1e2cbf041e5a72920d8103dd22dfdc9dffddfe62ec1d` |
| `docs/use_cases/records/smartphone_r5_phase118_tdcp_robust_k_structural_result_v1.json` | `88e8799050fd396eeb14e83d4f5923339279f46d0219a385e303d3cb50731538` |
| `docs/use_cases/records/smartphone_r5_phase112_main_output_offset_manifest_v1.json` | `d8be91f42f07196e07b293558cc6b83dd261226af577c1b26b167eead75902e4` |
| `docs/use_cases/records/smartphone_r5_phase112_main_output_offset_structural_result_v1.json` | `087b1bf51b4e584b86a7b558560df4ceb78e19b01e572f2d60aeef4c3d759ba0` |
| `docs/use_cases/records/smartphone_r5_phase117_tdcp_weighting_structural_result_v1.json` | `2517dcc805146dc34e790c78a392caf000c79eaba837fe099b6d522c147e9cf2` |

The future runner must use only the sealed metadata above before its
independent authorization.  It must not stat, hash, open, copy, transform,
or otherwise touch any raw member while checking this contract.

## Structural gates

The future result must record each predicate per route, including failures:

1. selector active exactly once; default-off isolation and Phase117 dynamic
   sigma absence;
2. ordinary measurement formula exactly `carrier_m + satellite_clock_m`,
   fixed sigma `0.03 m`, Phase118 Type metadata `Highway`, and Huber `k=0.5`;
3. ordinary TDCP factor count, insertion count, finite residual count,
   candidate-pair count, and every reject-reason count equal the sealed
   Phase118 baseline;
4. GNSS-first attempted with accepted iterations `>0`, finite initial/final
   costs and strict decrease, followed by complete finite exact-key C7/D
   handoff;
5. main graph selects `MULTIFRONTAL_QR`, has accepted iterations `>0`, and
   has finite strict cost decrease;
6. raw-base correction is active exactly once; Pixel5 offset is applied
   exactly once at the final output boundary; no fallback is taken;
7. expected output/problem epoch coverage is exact, all reported state/output
   values are finite and earth-valid, and no forbidden input or output lane is
   touched.

The sealed Phase118 TDCP population supplies the route-level invariant
baseline:

| route | candidate pairs | built/inserted/finite | code-phase rejects | missing previous |
| --- | ---: | ---: | ---: | ---: |
| MTV-A | 33025 | 31269 / 31269 / 31269 | 1756 | 4282 |
| LAX-T | 14961 | 14012 / 14012 / 14012 | 949 | 2373 |

The future validator is structural only.  It checks commands, pins, schema,
and declared gates; it does not run the solver and does not read a solution or
truth row.

## Exact future matrix

The route order is fixed to:

1. `2021-03-16-18-59-us-ca-mtv-a/pixel5` — 2158 domain rows, 2159 expected
   problem/output epochs;
2. `2022-04-01-18-22-us-ca-lax-t/pixel5` — 1465 domain rows, 1466 expected
   problem/output epochs.

The command is the sealed Phase118 raw recipe plus exactly one Phase120 flag.
The raw path values remain placeholders until a later authorization.  The
future contract must reject truth, MAT, PDC, precomputed-coordinate,
coordinate, accuracy, Kaggle, and token path/flag terms, and must reject any
Phase117 selector or alternate handoff.

## Read accounting at this audit boundary

No payload was read by this audit.  Counts are fixed at zero for phone GNSS,
phone IMU, broadcast navigation, raw-base bytes/headers/hashes, solution rows,
truth, MAT, coordinates, PDC, native solver invocations, accuracy
calculations, Kaggle/token access, reruns, and fallbacks.  Reading the pinned
source and sealed metadata files for contract construction is permitted and
does not constitute payload activity.

This audit is not an execution authorization.  A separate runner/validator
commit and a separate pre-raw accounting commit must be created before any
independent raw authorization can be considered.
