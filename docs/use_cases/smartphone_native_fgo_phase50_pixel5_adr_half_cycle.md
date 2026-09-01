# Phase 50 Pixel5 ADR half-cycle ambiguity/resolution audit

Phase 50 audits the raw Android per-satellite carrier-phase
half-cycle ambiguity/resolution state lane selected by Phase 49.  This is a
truth-free, audit-only phase.  The four frozen Pixel5 `device_gnss.csv` files
were read exactly once each in one evaluator process.  Truth, the Phase 49
metric payload, navigation, solver/trajectory, coordinates,
validation/holdout, archive, MAT, WLS, and correction paths were not opened.
The Phase 49 record is pinned only as factor-selection/policy metadata; its
metric fields are not an input.

The input freeze was pushed first in `4610b61` (SHA
`31d63edbb5394513d3300a9e9a283e2ef77d47cde4930f5501282bcf0ab8d259`).  The
evaluator, focused tests, CMake registration, and manifest were then sealed
and pushed in `6ef6399`.  The evaluator manifest SHA is
`194d66918d977e163ae6e98b9841645aecdd88136bad422afc68a62a971b75f9`.

## Fixed raw-only contract

Phase 50 uses the Phase 49 ordinary TDCP pair contract: same system/SVID/
signal adjacent raw rows, `0 < dt <= 1.5 s`, unchanged
`HardwareClockDiscontinuityCount`, finite in-range Phase 25 reconstructed
pseudorange, existing code/CN0/multipath masks clear, valid ADR, and reset /
cycle-slip bits clear.  Invalid, reset, cycle-slip, gap, HCDC, existing-mask,
and missing/nonfinite transitions stay in separate accounting buckets.

The source-pinned Android ADR bits are:

```text
VALID                 = 1
RESET                 = 2
CYCLE_SLIP            = 4
HALF_CYCLE_RESOLVED   = 8
HALF_CYCLE_REPORTED   = 16
```

Ordinary pairs are classified into `resolved_to_resolved`,
`unresolved_reported_stable`, `resolved_toggle`, `reported_toggle`, or
`resolved_and_reported_toggle`.  A pair is implicated when either resolved or
reported status changes.  Stable pairs are the clean comparator.  The fixed
ADR/rate residual is

```text
r_k = (signed_ADR_k - signed_ADR_prev)
      - 0.5 * (PseudorangeRateMetersPerSecond_prev
               + PseudorangeRateMetersPerSecond_k) * dt_seconds
```

Pixel5 preserves the adapter ADR sign; only the five published Samsung models
are negated.  Residuals are centered by the median ordinary residual in each
endpoint `utcTimeMillis + HCDC` group.  For each ordinary pair the centered
residual is also divided by `0.5*c/CarrierFrequencyHz`.  A toggle is in the
half-cycle cluster when its distance to the nearest integer half-cycle is at
most 0.15.  Android has no direct ADR uncertainty field, so no unobserved
uncertainty is invented.

## One-shot observations

The machine-readable output is
`output/smartphone-r5/phase50-pixel5-adr-half-cycle-v1/`.  The result record is
[`smartphone_r5_phase50_pixel5_adr_half_cycle_result_v1.json`](records/smartphone_r5_phase50_pixel5_adr_half_cycle_result_v1.json).

| Route | Raw rows | Epochs | Transitions | Ordinary pairs | Satellites | Signal families | Resolved-stable | Unresolved/toggle | Implicated |
|---|---:|---:|---:|---:|---:|---|---:|---:|---:|
| MTV-a | 87,705 | 2,159 | 87,645 | 7,508 | 7 | GAL_E1 | 7,508 | 0 | 0 |
| MTV-h | 112,833 | 3,140 | 112,778 | 8,763 | 6 | GAL_E1 | 8,763 | 0 | 0 |
| LAX-t | 51,243 | 1,466 | 51,191 | 4,176 | 10 | GAL_E1 | 4,176 | 0 | 0 |
| MTV-u | 35,810 | 1,102 | 35,767 | 3,728 | 11 | GAL_E1 | 3,728 | 0 | 0 |

All four routes meet the frozen ordinary-pair/satellite minimum and overlap
ordinary current-TDCP proxy pairs.  Every ordinary pair is classified as
`resolved_to_resolved`; no unresolved stable or resolved/reported toggle pair
survives the ordinary contract.  Therefore implicated fraction is 0 in every
route, toggle p95 excess/ratio and half-cycle cluster fraction are not
material (reported as 0), and no toggle event can support an arc reset.
Ordinary clean p95 absolute residual is 0.038575–0.062970 m across the four
routes; route median absolute residuals are 0.030918, 0.017376, 0.041272, and
0.033028 m, with aggregate median 0.031973 m and route MAD 0.005177 m.

The static source audit confirms that the current adapter parses ADR state,
preserves Pixel5 sign, and uses reset/cycle-slip/valid bits for carrier mask
and LLI/loss-of-lock.  Half-cycle bits are not included in those gates, and
the current TDCP arc reset follows loss-of-lock/LLI.  This source observation
is policy context only; Phase 50 does not alter the loader or estimator.

## Integrity disposition and gates

The one-shot result is **no-go-evaluator-integrity-failure**.  The frozen
presentation-integrity check for state-class counts failed on all four routes:
the evaluator emitted `pair_state_classes` containing both the ordinary class
and the excluded-transition class, then compared their total with the
ordinary pair count.  Pair-reason sums, signal/satellite group sums, all four
route medians, aggregate median/MAD, and event count passed.  The state-class
sum failure is a hard gate, so the emitted no-toggle observation is retained
for audit traceability but is not promoted as an identification result.

The state population, toggle materiality, half-cycle cluster, route direction,
and clean-retention gates are consequently not an authorization path: the
toggle population is absent.  LOO fixed-class decision stability and current
ordinary-TDCP overlap pass, but cannot override the integrity failure.  No
second evaluator run or raw reread was performed after the failure.

No correction, mask, weighting, or arc reset is authorized.  The exactly one
next source-supported raw physical factor is **raw Android per-satellite
carrier-phase ADR carrier-frequency/antenna phase-bias residual**.  It is not
implemented here.  The 0.782 target is not evaluated without truth.

## Accounting, artifacts, and tests

Read accounting is four raw CSV reads total (one per frozen route, one process):
truth, past-truth/Phase 49 metric payload, BRDC navigation, solver/trajectory,
correction implementation, validation/holdout, archive reopen,
rematerialization, Kaggle/token, MAT, WLS, and coordinates are all zero.  The
raw event table contains 251,316 retained transition events.  Output hashes
are:

| Artifact | Bytes | SHA-256 |
|---|---:|---|
| `phase50_pixel5_adr_half_cycle.json` | 287,857 | `69d8be21ba601f97524708c2201fb1d6ee091ca239c9e7bf853e0f4ba4137094` |
| `phase50_pixel5_adr_half_cycle.routes.json` | 273,211 | `f54d9b55b0007f872cd0d19c1b371b40f05ffe4252d4f05c84ef7519d99f2d5e` |
| `phase50_pixel5_adr_half_cycle.events.json` | 83,690,404 | `9061d9e48f511d1642b23e9fb55ec45418ff44d734cea53375fbeedbb4d8731c` |
| `phase50_pixel5_adr_half_cycle.manifest.json` | 1,767 | `d28c2375ae1cc29ea0bfc5ce2d569c803c5c06d2efc1b9790a0df47d552cfb96` |

The focused Python suite had 7 passing tests before the raw seal; the
registered CTest `python_smartphone_phase50_pixel5_adr_half_cycle_audit_tests`
also passed before the raw seal.  Synthetic coverage includes the corrected
ADR bit values, stable/toggle class partitioning, sign and half-wavelength
normalization, reset/HCDC/gap boundaries, stale-group/collapsed-route
integrity, and one-read hashing.
