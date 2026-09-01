# Phase 58 Pixel5 C/N0/Doppler residual calibration audit

Phase 58 audited Android per-satellite `Cn0DbHz` against a raw code/rate
closure residual on four route-disjoint Pixel5 recordings.  The audit was
truth-free and did not read navigation, coordinates, IMU, solver output,
MATLAB, validation/holdout data, archives, or an earlier metric payload.
Each pinned `device_gnss.csv` was read once in one process.

The fixed residual was

```text
e = (P_current - P_previous)
    - 0.5 * (PseudorangeRate_previous + PseudorangeRate_current) * dt
```

after subtracting the same `utcTimeMillis`/hardware-clock-group median.  The
source-supported shape was `10^(-(Cn0-40)/20)`.  All predeclared route,
coverage, monotonicity, leave-one-route-out calibration, non-common-mode,
factor-impact, and presentation gates passed.  The four sealed LOO scale
values were 0.7726573316542686, 0.7720231442247758, 0.7416064915523163, and
0.7453335259209156 m/s at 40 dB-Hz.  Their exact median fixes the implementation
scale at **alpha = 0.7586783350728457 m/s**, without manual rounding.

The audit itself made no native correction.  A separately frozen implementation
stage is authorized with the exact rule

```text
sigma = max(existing_champion_doppler_sigma_mps,
            0.7586783350728457 * 10^(-(Cn0DbHz - 40)/20))
```

It is FGO Doppler-only, opt-in, coefficient one, and has no upper clip or SPP
application.  The existing p85/12 upstream quality path is not reimplemented
or mutated.  Phase 43 remains champion and Phase 51 remains experimental;
`0.782` is not evaluated without truth.

| Route | transitions | satellites | median abs centered residual (m/s) | LOO alpha (m/s) |
|---|---:|---:|---:|---:|
| MTV-a | 13,431 | 7 | 1.3461344093084335 | 0.7726573316542686 |
| MTV-h | 15,630 | 7 | 1.3880830090492964 | 0.7720231442247758 |
| LAX-t | 11,732 | 11 | 1.8418421931564808 | 0.7416064915523163 |
| MTV-u | 9,612 | 11 | 1.7463475596159697 | 0.7453335259209156 |

Machine-readable details and hashes are in
[`smartphone_r5_phase58_pixel5_cn0_doppler_calibration_result_v1.json`](records/smartphone_r5_phase58_pixel5_cn0_doppler_calibration_result_v1.json),
with the pre-read [freeze](records/smartphone_r5_phase58_pixel5_cn0_doppler_calibration_freeze_v1.json)
and [evaluator manifest](records/smartphone_r5_phase58_pixel5_cn0_doppler_calibration_evaluator_manifest_v1.json).
