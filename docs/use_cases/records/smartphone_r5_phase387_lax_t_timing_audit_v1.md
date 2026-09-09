# Phase387 — LAX-T timing and output-offset audit

Read-only inspection of Phase382 LAX-T and Phase234 H native summaries,
`src/io/imu.cpp`, and the native CLI output-offset application site.
No truth, candidate coordinates, MAT files, or saved positioning inputs
were read. No new solver run, score, or production setting change.

Both runs report output position offset disabled, zero application passes,
and zero corrected epochs. Thus a difference in that postprocessing switch
does not explain their accuracy difference. Both use the UTC wall-clock
fallback with the requested and effective source offset of -20 ms. The
loader applies it before the affine UTC-to-GPS mapping; it does not change
accel/gyro pairing clocks. This does not establish absolute sensor latency.

| Aggregate | LAX-T Phase382 | H Phase234 |
| --- | ---: | ---: |
| Paired samples | 77758 | 166502 |
| Exact pairing timestamps | 2384 | 89150 |
| Interpolated samples | 75374 | 77352 |
| Endpoint-nearest / omitted | 0 / 0 | 0 / 0 |
| Median absolute pairing offset, ms | 5 | 0 |
| Maximum absolute pairing offset, ms | 14 | 10 |
| GNSS UTC mapping anchors | 1466 | 3140 |
| Mapping drift, ppm | 0.003928239837472196 | 0.0026718708051164802 |
| Maximum mapping fit residual, ms | 0.004217786692857742 | 0.004991516141295433 |
| Maximum anchor gap, ms | 1000 | 1000 |

These very small GNSS mapping residuals do not prove the IMU wall clock is
accurately synchronized to GNSS. More frequent interpolation on LAX-T is a
measurement-support difference, not evidence of a bug or causal attribution.
Next inspect raw accel/gyro timestamp cadence and the interpolation contract,
including synthetic staggered-clock tests. Do not fit latency against truth,
sweep offsets using these scores, or change the fixed -20 ms setting without
independent evidence. The 0.782-class / leaderboard objective remains unmet.
