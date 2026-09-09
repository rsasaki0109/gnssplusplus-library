# Phase 43 fallback-seed quality-anchor recovery

Phase 43 is a truth-free, opt-in recovery for the generalized no-quality-anchor
case identified by the [Phase 42 raw-row attrition audit](smartphone_native_fgo_phase42_doppler_row_attrition.md).
The immutable Phase 43 freeze is the [freeze record](records/smartphone_r5_phase43_native_fallback_seed_quality_anchor_recovery_freeze_v1.json);
the binary, runner, raw roots, and zero-truth read contract are pinned in the
[evaluator manifest](records/smartphone_r5_phase43_native_fallback_seed_quality_anchor_recovery_evaluator_manifest_v1.json).

## Candidate contract

The candidate flag is:

```text
--native-fallback-seed-quality-anchor-recovery
```

The normal quality-anchor SPP reconnaissance runs completely first with its
ordinary elevation gate and all existing SNR, health, broadcast-navigation,
geometry, GDOP, residual, and satellite-count gates.  Recovery is exclusive:
it runs only when normal reconnaissance has no eligible candidate, using the
same raw pseudorange and broadcast navigation at an elevation gate of −90°.
The existing deterministic ranking remains satellites descending, GDOP
ascending, normalized residual RMS ascending, then input index ascending.

When recovery selects an anchor, its raw/nav-derived ECEF position is the
first state for ordinary 0° forward/backward replay.  The factor builder stays
on the ordinary 0° path.  Recovery failure is fail-closed; it never activates
a sentinel factor-level bypass.  The flag defaults to false, and flag-off
summary/submission bytes remain the existing frozen baseline.

The candidate summary exposes normal and recovery candidate counts, trigger,
selected anchor index/satellite count/GDOP/normalized residual, replay valid
and invalid epochs, and `sentinel_factor_bypass: false`.  Candidate-only fields
are removed only for the declared other-route projection comparison.

## Reproduction

The runner reads only the six frozen `device_gnss.csv`, `device_imu.csv`, and
`brdc.nav` files.  It validates their SHA-256 hashes, runs the candidate twice
per route, runs flag-off controls, checks keyed finite output, raw UTC identity,
convergence, TDCP build/insert equality, speed, deterministic repeatability,
and projected identity:

```bash
python3 apps/commands/benchmarks/gnss_smartphone_phase43_fallback_seed_quality_anchor_recovery.py \
  --run-matrix
```

The machine-readable run is written under the ignored output directory
`output/smartphone-r5/phase43-native-fallback-seed-quality-anchor-recovery-v1/`.
The focused contract test is registered as
`python_smartphone_phase43_fallback_seed_quality_anchor_recovery_tests`.

## Structural result

The [sealed result](records/smartphone_r5_phase43_native_fallback_seed_quality_anchor_recovery_result_v1.json)
is `sealed-truth-free-structural-go`.

| route | normal candidates | recovery candidates | trigger | selected index | replay valid | TDCP built = inserted | max speed / >70 count |
|---|---:|---:|---|---:|---:|---:|---:|
| MTV-h / Pixel 5 | 0 | 3,140 | true | 904 | 3,140 | 41,383 = 41,383 | 20.5607 / 0 |
| MTV-a / Pixel 5 | 2,159 | 0 | false | 1,791 | — | 31,488 = 31,488 | 44.3292 / 0 |
| MTV-b / Pixel 4 | 1,678 | 0 | false | 1,371 | — | 27,080 = 27,080 | 36.3172 / 0 |
| MTV-e / SM-G988B | 613 | 0 | false | 1,065 | — | 20,846 = 20,846 | 36.6897 / 0 |
| LAX-t / Pixel 5 | 1,466 | 0 | false | 740 | — | 14,062 = 14,062 | 35.9479 / 0 |
| MTV-u / Pixel 5 | 642 | 0 | false | 701 | — | 10,368 = 10,368 | 29.1832 / 0 |

On MTV-h, the selected anchor has 27 satellites, GDOP
0.9928008873297592, normalized residual RMS 2.3663773350268436, finite
Earth-valid position, and 3,140 valid replay epochs with zero invalid epochs.
Its graph converged with 3,140 raw epochs, 3,139 exact raw UTC output keys,
and 73,286 pseudorange factors.  All six routes had finite output and zero
transitions above 70 m/s.  Candidate submission and summary bytes were
identical across both runs on every route.

The five non-target routes kept recovery untriggered.  Their candidate
submission and summary matched flag-off after the declared candidate-specific
projection, while flag-off controls matched the frozen Phase31/Phase37 bytes.
The MTV-h flag-off control returned the expected fail-closed status without a
submission or summary.  The result records zero ground-truth, validation,
holdout, MAT, precomputed-coordinate, device-WLS, Kaggle, and token reads.

