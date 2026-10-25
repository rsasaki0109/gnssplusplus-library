# Long-lock ambiguity overconfidence: what it is not

The [ambiguity consistency diagnostics](rtk_ambiguity_consistency_diagnostics.md)
found accepted AR fixes whose float ambiguities sit far off integer for their
Qb. Even correct fixes have a best cost per pair of 4.7 (Odaiba) and 64
(Shinjuku), where about 1 is expected.

The [base interpolation fix](rtk_base_interpolation_curvature.md) removed one
systematic error, but it barely moved those numbers. This round measures the
carrier and code residuals against the reference trajectory at the 10 Hz rover
rate and tests three explanations. **None of them is the dominant mechanism.**
No solver behaviour changes. Record:
[JSON](urbannav_long_lock_overconfidence.json).

The reference trajectory is used only for analysis residuals and 0.5 m labels.
It is never a solver input.

## Method

`scripts/analysis/truth_dd/export_truth_dd.cpp` writes the following at every
Trimble rover epoch:

- the rover and base carrier phase and code;
- the base aligned by `interpolateBaseEpoch(model_range_at_target)`, the
  corrected interpolation;
- the ranges from the reference rover position and from the surveyed base;
- the line-of-sight vector.

GLONASS is excluded and the elevation cut is 15°. For each system and
observation type, DD residuals are formed against the most-present satellite.
Arcs split at LLI or at steps larger than 0.3 cycle.

For the carrier, each epoch's common 3D offset is fitted out, which removes
the reference trajectory's own error. The model σ is the solver's `varerr`:
2 mm for phase and 0.3 m for code, elevation-scaled, DD = satellite + reference.

## Carrier and code residuals

| Drive | carrier σ / model | carrier τ p50 | code \|e\| p50 / p95 | code σ / model (robust / plain) | code τ p50 |
| --- | --- | ---: | --- | --- | ---: |
| Odaiba | 6.2 / 6.5 mm | 2.9 s | 0.49 / 5.3 m | 0.71 / 3.5 | 3.0 s |
| Shinjuku | 8.6 / 6.1 mm | 3.0 s | 0.57 / 4.9 m | 0.89 / 9.1 | 2.9 s |

τ is the integrated autocorrelation time, up to the first zero crossing, on
arcs of at least 60 s.

- **Carrier.** The amplitude matches the model, but the errors are time
  correlated. The running mean after L s does not shrink as σ/√n: its
  white-noise |z| p50 is 10/17/22 (Odaiba) and 6/13/22 (Shinjuku) at
  10/30/60 s.
- **Code.** The core matches the model, but the tail is heavy: 7% of rows
  exceed 3σ. A code-driven float ambiguity would sit 1.8–2.0 cycles off at the
  median over 10–60 s, and 4–8 cycles at p90.

## Three explanations tested

1. **Counting time-correlated errors as white at 10 Hz.**
   - The prediction was frozen before the replay:
     [JSON](urbannav_long_lock_overconfidence.json)
     `frozen_prediction_before_decimation_replay`.
   - The rover was decimated to integer seconds, carrying LLI from dropped
     epochs into the next kept one (`decimate_rinex.py`). If the explanation
     held, the correct-fix cost per pair would drop at least 3× on both drives.
   - **It did not.** Odaiba went 4.73 → 14.44, Shinjuku 64.3 → 24.8, and max |z|
     was unchanged (4.0 → 3.6, 6.6 → 7.2).
   - Caveat: 1 Hz processing fixes far less (integer-second FIX 416 → 43 on
     Odaiba, 216 → 66 on Shinjuku), so the 1 Hz sample is small and selected
     (39 and 56 correct AR fixes).
   - The max-|z| pair's lock is 7–16 s at either rate, only a few correlation
     times.
2. **Ionosphere** (baselines 4.5 and 9.5 km, `--iono off`). Removing the
   geometry-free dispersive term from the first-carrier residual *increases*
   its slow part (30 s average) on every system and drive. The inter-frequency
   difference is dominated by frequency-dependent multipath, not by the
   ionosphere. Few dual-carrier arcs last 90 s, so this is indicative only.
3. **Heavy-tailed code.** On correct AR fixes, the max-|z| pair is ranked among
   the pairs sharing its reference by the truth code error over its lock. It
   falls in the top third in 53% of fixes on Odaiba, against 39% under a
   uniform ranking. On Shinjuku it is 36% against 35%. The carrier gives 39%
   against 39% and 26% against 35%. Code errors explain part of Odaiba, not
   Shinjuku.

## What this leaves

The residuals behave as the model assumes in amplitude, with the following
caveats:

- the carrier is correlated over a few seconds and carries slow components;
- the code has a heavy tail;
- yet neither the update rate, the ionosphere nor the code tail accounts for
  the overconfidence.

The next suspects are inside the filter, not in the inputs:

- the kinematic position re-seed and its loss of position/ambiguity
  cross-covariance;
- ambiguity initialisation from code with a 0.3 m model at urban code errors;
- the reference satellite changes.

They should be tested in the same way as here: a frozen prediction first, and
the reference used for labels only. Because a Qb-based acceptance statistic
cannot be trusted until this is understood, no acceptance rule is proposed.

## Reproduce

```bash
c++ -std=c++20 -O2 -Iinclude -Iapps/native -I/usr/include/eigen3 \
  scripts/analysis/truth_dd/export_truth_dd.cpp build/libgnss_lib.a \
  build/libgnss_rtk_runtime.a build/libgnss_lib.a \
  build/third_party/ginan-iers2010/libginan_iers2010.a \
  build/third_party/sofa/libsofa.a -o export_truth_dd
D=data/driving/Tokyo_Data/Odaiba
./export_truth_dd $D/base.nav $D/rover_trimble.obs $D/base_trimble.obs \
  $D/reference.csv -3961904.9530 3348993.7578 3698211.7553 truth_dd.csv
python3 scripts/analysis/truth_dd/phase_residual_correlation.py truth_dd.csv
python3 scripts/analysis/truth_dd/code_residual_correlation.py truth_dd.csv
python3 scripts/analysis/truth_dd/dispersive_share.py truth_dd.csv
python3 -I scripts/analysis/truth_dd/decimate_rinex.py $D/rover_trimble.obs rover_1hz.obs
# gnss_solve on rover_1hz.obs and on the 10 Hz rover with
# --max-fixed-anchor-age 5 --max-fixed-doppler-consensus 3
# --lambda-shadow-candidates 2 --debug-epoch-log <csv>, then:
python3 scripts/analysis/truth_dd/ar_consistency_stats.py Odaiba <pos> <csv> [--integer-seconds]
# max-|z| ranking uses a run with --base-interp-curvature:
python3 scripts/analysis/truth_dd/maxz_truth_rank.py truth_dd.csv <csv> <pos> Odaiba
```

The SHA-256 of the inputs, the decimated rovers and the truth-DD CSVs are in
the JSON.
