# CLASLIB parity P0 baseline

This is the first scorecard produced with the canonical contract described in
`claslib_near_equivalence_plan.md`. It is a diagnostic baseline from the dirty
development tree, not a sign-off result.

## Contract

- Dataset: public CLASLIB 2019-08-27 sample
- Window: GPST 2068/230420–233999 (3580 one-second epochs)
- Warmup: 300 seconds
- CLASLIB: commit `23cfd363a2db6d8d8144e292c82e9d97ca2d3015`
- OBS SHA-256: `a54edc2dba3fa5cdcbbfb415766c9f3cb8ed551ce401c8d9440c30bd037fc169`
- NAV SHA-256: `6234177c9cf4d9357084f52503dffcc0b4eb80a320d2188fb5175ceda4d50312`
- L6 SHA-256: `9617fa4c03e700e6d0671323a3d3a3537dadedf001a2601bfe6d1f72debedc15`
- CLASLIB NMEA SHA-256: `2505fae86e5ba188fe3c27b19f6ddd5bfaed53ec5c11f4a26d628335918978f1`
- Native `.pos` SHA-256: `043c528dea4e42c714ee81e26cfeb484197cae0cf5b7328e1b2c3be9822e1c3a`

NMEA UTC receives the 18-second GPS–UTC offset before epoch matching. CLASLIB
FIX is NMEA quality `4`; native FIX is `.pos` `FixedAmbiguities > 0`.

## Result

All 3580 solution epochs matched. After warmup, CLASLIB was FIX for all 3280
evaluated epochs while this native baseline was intentionally run with AR off,
so status agreement was 0% and native FIX count was zero.

| Metric | Baseline | Gate |
| --- | ---: | ---: |
| Trajectory delta RMS 3D | 3.3905 m | 0.020 m |
| Trajectory delta P95 3D | 4.1245 m | 0.030 m |
| ZD common rows | 79,630 | — |
| ZD row-key coverage | 32.262% | 100% |
| PRC RMS delta | 0.01927 m | 0.010 m |
| CPC RMS delta | 1.63222 m | 0.010 m |

The full ZD comparison had 163,980 CLASLIB rows and 162,472 native rows. The
largest CPC delta is phase-bias driven; row-surface mismatch is also material.

## P1 first finding

CLASLIB keys GPS frequency slot 1 as `CODE_L2W`, including receiver epochs whose
raw observation is C2X/L2X. The native diagnostic now preserves the raw receiver
identity but requests the C2W/L2W correction slot. On the first 300 epochs this
reduced per-satellite L2 PRC RMS from 48–372 mm to 16–22 mm for G25/G26/G29/G31/G32.
The first phase investigation showed that the apparent lifecycle divergence was
actually a service-network selection error: the Chiba receiver belongs to CLAS
network 7, while native interpolation could consume a newer network-1 row.

## P1 GPS progress

P1 closed on the full 3580-epoch run in
`output/claslib_parity_p1_gps_surface_3580`. The comparison uses requested
CLASLIB slot identity, so a receiver C2X/L2X observation is keyed as the
CLASLIB C2W/L2W correction slot while retaining raw observation provenance.

| Metric | P0 | P1 final | Gate |
| --- | ---: | ---: | ---: |
| GPS code rows | — | 52,065 / 52,065 | 100% |
| GPS phase rows | — | 52,065 / 52,065 | 100% |
| GPS row-key coverage | incomplete | 100% | 100% |
| PRC RMS delta | 0.01927 m (all systems) | 0.002414 m | 0.010 m |
| CPC RMS delta | 1.63222 m (all systems) | 0.005058 m | 0.010 m |
| Phase-bias RMS delta | 2.54369 m (first GPS run) | 0.000000 m | — |

The closing changes were:

- exact network-7 phase-bias selection with the 15-second reception lag and a
  strict 30-second bank lifetime, reproducing the G26 90-second and G25
  30-second withdrawal windows;
- CLASLIB `adjust_r_dts()` IODE-boundary geometry compensation combined with
  SIS continuity, rather than applying the SIS term alone;
- literal phase wind-up, end-of-stream code-bias hold, and exact L2W bias-cell
  identity;
- an output-only GPS L5 slot. It emits CLASLIB's C5X/L5X component rows with
  zero PRC5/CPC5 without increasing the estimator's two-frequency measurement
  count or colliding with the L2 ambiguity state;
- canonical normalizer treatment of zero PRC5/CPC5 as an invalid closure
  sentinel, so it no longer invents an IODE geometry component for L5.

The refreshed scorecard passes dataset coverage and all GPS ZD gates. It still
fails trajectory, FLOAT/FIX, and status gates as expected because P2–P6 remain
open and this static diagnostic run has ambiguity resolution disabled.
