# Phase 49 Pixel5 ADR cycle-slip/lock-loss residual audit

Phase 49 audits the single raw physical factor selected by Phase 48: the
per-satellite Android carrier-phase accumulated-delta-range (ADR)
cycle-slip/lock-loss residual.  It is a raw-only, truth-free diagnostic.  The
four frozen Pixel5 `device_gnss.csv` files were read exactly once each by one
evaluator process.  No truth, Phase 45/48 metric payload, navigation, solver,
trajectory, coordinate, validation/holdout, archive, MAT, WLS, or correction
path was opened.

The input freeze was pushed before any Phase 49 raw read in `d90b698`.
The evaluator, focused tests, CMake registration, and manifest were sealed and
pushed in `0221c2e`.  The freeze SHA is
`c540656c66d055b9787ee5d84776a8df8b20aa585bea2d7d48ec34052d80619b`; the
evaluator manifest SHA is
`13547a0bb486d1a923f3cfb4f83e7d5e8a6d0867ed87b97c8e112c67186e7843`.

## Fixed raw-only contract

The evaluator reconstructs the Phase 25 raw pseudorange with integer/Decimal
segment-base arithmetic and constellation time mapping.  It does not use the
pseudorange as a position or solver input.  A scored ordinary transition is a
same-system/SVID/signal adjacent raw pair with `0 < dt <= 1.5 s`, unchanged
`HardwareClockDiscontinuityCount`, finite in-range reconstructed pseudorange,
existing code/CN0/multipath masks clear, and valid ADR with reset/cycle-slip
bits clear.  Reset, cycle-slip, invalid, HCDC, gap, existing-mask, and missing
or non-finite transitions remain separate boundary buckets and are not scored
as ordinary pairs.

The adapter's Pixel5 ADR sign is preserved.  The only sign changes are the
five published Samsung models already present in the pinned adapter policy.
Using the Phase 41 direct range-rate contract, the fixed residual is

```text
r_k = (signed_ADR_k - signed_ADR_prev)
      - 0.5 * (PseudorangeRateMetersPerSecond_prev
               + PseudorangeRateMetersPerSecond_k) * dt_seconds
```

The evaluator subtracts the median residual in each endpoint
`utcTimeMillis + HardwareClockDiscontinuityCount` group to remove receiver
common mode, while retaining both raw and centered residuals.  A diagnostic
candidate is an ordinary transition whose absolute centered residual exceeds
the source-pinned Phase 13 threshold of 1.5 m and whose immediately preceding
and following transitions for the same satellite/signal are ordinary.  This
two-sided rule is diagnostic only; it does not mask or change an estimator.

Android exposes no direct ADR uncertainty.  Where present, positive adjacent
`PseudorangeRateUncertaintyMetersPerSecond` is integrated over `dt` and used
only for normalized residual reporting.  ADR state labels retain VALID,
RESET, CYCLE_SLIP, HALF_CYCLE_REPORTED, HALF_CYCLE_RESOLVED, and their
combinations.  Carrier wavelength `c / CarrierFrequencyHz`, state bits,
arc lengths, and raw uncertainty availability are reported per route.

## One-shot result

The machine-readable result is
`output/smartphone-r5/phase49-pixel5-adr-cycle-slip-lock-loss-v1/`.  The
result record is
[`smartphone_r5_phase49_pixel5_adr_cycle_slip_lock_loss_result_v1.json`](records/smartphone_r5_phase49_pixel5_adr_cycle_slip_lock_loss_result_v1.json).

| Route | Raw rows | Epochs | Transitions | Ordinary pairs | Satellites | Signal families | Candidates | Candidate fraction | Ordinary clean p95 (m) | Ordinary clean max (m) |
|---|---:|---:|---:|---:|---:|---|---:|---:|---:|---:|
| MTV-a | 87,705 | 2,159 | 87,645 | 7,508 | 7 | GAL_E1 | 0 | 0 | 0.059884 | 0.285211 |
| MTV-h | 112,833 | 3,140 | 112,778 | 8,763 | 6 | GAL_E1 | 0 | 0 | 0.038575 | 0.300647 |
| LAX-t | 51,243 | 1,466 | 51,191 | 4,176 | 10 | GAL_E1 | 0 | 0 | 0.062970 | 0.281933 |
| MTV-u | 35,810 | 1,102 | 35,767 | 3,728 | 11 | GAL_E1 | 0 | 0 | 0.054942 | 0.220024 |

The ordinary-pair minimum is 10,000 per route; the observed range is
3,728–8,763.  All four routes have valid unflagged coverage and at least six
ordinary satellites, but every route has only one canonical signal family
(`GAL_E1`) against the frozen minimum of two.  The fixed 1.5 m two-sided
candidate score finds no candidate in any route, so its route fractions are
all 0 rather than the frozen 0.1–10% identification range.  Consequently no
candidate p95 excess or ratio is material, no two-sided event is observed, and
there is no current-TDCP proxy candidate overlap.

The ordinary clean residuals are small and stable (p95 absolute
0.038575–0.062970 m), while large all-transition tails are concentrated in
the separately reported reset/cycle-slip/invalid/mask/gap buckets.  This is
the strongest raw structure: the available large tails are already flagged or
excluded by the existing ADR/TDCP contract, and the retained unflagged pairs
do not identify a reproducible jump.  The route median absolute residuals are
0.030918 m (MTV-a), 0.017376 m (MTV-h), 0.041272 m (LAX-t), and 0.033028 m
(MTV-u); their aggregate median is 0.031973 m and route MAD is 0.005177 m.
All six pairwise route-median distances are retained in the result.

The leave-one-route-out fixed 1.5 m diagnostic is direction-stable only in the
negative sense: every fold has zero candidates and a non-material candidate
p95 excess (−0.051242 to −0.059948 m).  This does not authorize a correction.

## Gates and decision

The thresholds were sealed before the raw read and were not changed after the
result.  Route count, clean retention, stable-arc false-positive proxy, fixed
threshold LOO consistency, and all presentation-integrity checks pass.
Coverage, candidate population, candidate materiality, two-sided event
reproducibility, routewise direction, two-signal-family composition, and
current-TDCP candidate-impact gates fail.  Presentation integrity is fully
closed: pair-reason counts, signal/satellite group counts, all four route
medians, aggregate median/MAD, and event count recompute exactly.  The event
table contains 251,316 raw-only transition events, including the separately
labeled excluded boundary transitions.

Decision: **No-Go for an ADR/Doppler consistency reset or any mask, weighting,
or correction.**  This is an audit result, not an estimator change.  The
exactly one next source-supported raw physical factor is **raw Android
per-satellite carrier-phase half-cycle ambiguity/resolution residual**.  It is
not implemented here.  The 0.782 target is not evaluated without truth.

The pinned static source audit confirms that the current adapter parses ADR
state, applies the published sign policy, preserves Pixel5 sign, masks reset/
cycle-slip/invalid ADR, and raises loss-of-lock/LLI.  The current TDCP key,
gap, LLI, and HCDC contract is also present.  Phase 28/31 champion and Phase
14 reset/no-go records, together with the Phase 13 1.5 m threshold, are policy
pins only; no earlier result payload is an inference input.

## Accounting, artifacts, and verification

Read accounting is four raw reads total (one per frozen route, one process),
with truth, past-truth payload, Phase 48/47 metric payload, BRDC navigation,
solver/trajectory reruns, correction implementations, validation/holdout,
archive reopen, rematerialization, MAT, device-WLS/precomputed coordinates,
and Kaggle/token access all zero.  Raw input hashes and sizes are pinned in the
freeze and repeated in the result record.

| Artifact | Bytes | SHA-256 |
|---|---:|---|
| `phase49_pixel5_adr_cycle_slip_lock_loss.json` | 316,498 | `2ca43bf7dc1fa713eaf2a6e3256f79039557e5fe78b96bf0eb7d05372e977318` |
| `phase49_pixel5_adr_cycle_slip_lock_loss.routes.json` | 302,510 | `5e916cca2f8514839ce8f59a1d7d7e51697ee6d7d9c48d8ff3cefb1fb1d91b75` |
| `phase49_pixel5_adr_cycle_slip_lock_loss.events.json` | 85,462,206 | `1b00c9b98afb03d06bee84d765afc291fecd086a5d2e6432270abb6cd4905e31` |
| `phase49_pixel5_adr_cycle_slip_lock_loss.manifest.json` | 1,876 | `bb3c7088791a3d1570894becc532db5784ceeb0930bf119b5805b9c1eb0b3fd7` |

The focused Python suite has 8 passing tests; the registered CTest
`python_smartphone_phase49_pixel5_adr_cycle_slip_lock_loss_audit_tests` has 1
passing test.  The synthetic tests cover ADR state bits, reset/cycle-slip and
HCDC/gap boundaries, Pixel5 sign and trapezoidal-rate residual sign, two-sided
candidate masking, one-read hashing, and stale-loop/collapsed-route
presentation failures.
