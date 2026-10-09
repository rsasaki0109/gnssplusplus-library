# RTK FLOAT reported covariance

`PositionSolution::position_covariance` of an RTK FLOAT epoch was the constant
`0.01 m^2 * I` (`RTKProcessor::generateSolution`), independent of the filter and
of the epoch's quality. This note records why that is wrong on the PPC runs,
the default-OFF correction, and the measured effect. Evidence directory (local):
`rtklib_v2_ws_output/pva_floatcov_20261009/`.

## Option

`RTKConfig::reported_covariance_mode`: `LEGACY_FIXED_SIGMA` (default, unchanged
output), `FILTER_MARGINAL`, `FIRST_PASS_SCALED`, `SPP_CONSISTENCY_SCALED`.
Batch solver: `gnss_solve --rtk-reported-covariance
legacy|filter|first-pass|spp-consistency` and `--rtk-covariance-log FILE`.
Only the reported covariance of FLOAT epochs changes. Mechanism and formulas are
in [online_pva_candidate_v4.md](online_pva_candidate_v4.md).

## Measurement (six PPC runs, FLOAT epochs, offline reference)

Error is the ECEF difference to `reference.csv` rotated to ENU; z = error /
reported sigma per axis; NEES = e' C^-1 e (3 dof, 99.73 % bound 14.16). Online
path = `gnss_pva_replay` control (RTK + tight INS prior, base 1 Hz so about one
epoch in five is RTK, the rest SPP). The corrected covariance is evaluated on
the identical RTK stream (the isolated RTK prior makes the RTK output
independent of the reporting mode; positions and statuses compared equal).

| FLOAT epochs, pooled (n = 3834) | reported sigma median | RMS z | all axes < 3 sigma | NEES < 14.16 |
|---|---:|---:|---:|---:|
| legacy, all | 0.10 m | 307 | 16 % | 18 % |
| mode 3, all | 0.55 m | 6.1 | 56 % | 50 % |
| legacy, error < 1 m (n = 1620) | 0.10 m | 3.1 | 39 % | 42 % |
| mode 3, error < 1 m | 0.08 m | 5.7 | 36 % | 30 % |
| legacy, 1-10 m (n = 1012) | 0.10 m | 21 | 0 % | 0 % |
| mode 3, 1-10 m | 0.83 m | 4.6 | 55 % | 47 % |
| legacy, >= 10 m (n = 1202) | 0.10 m | 547 | 0 % | 0 % |
| mode 3, >= 10 m | 39.9 m | 7.5 | 84 % | 79 % |

Per run (legacy -> mode 3): RMS z of FLOAT 35-547 -> 1.7-11 per axis; share of
epochs inside 3 sigma per axis 8-61 % -> 48-95 %. SPP epochs (RMS z 0.3-2.3) and
FIXED epochs (RMS z 0.1-2.5) were already consistent and are unchanged.

Over time (Nagoya 1, FLOAT, per 120 s, legacy -> mode 3): 240-360 s median error
0.99 m with a 149 m maximum; legacy NEES inside 22 %, mode 3 100 % with sigma
32 m; 720-960 s legacy 0-1 % inside, mode 3 38-60 %. Tokyo 1 57-66 s: all
measurement rows suppressed, error 33 -> 125 m, reported sigma 0.1 m.

Batch RTK (`gnss_solve`, PPC native-replay recipe, 11,909 FLOAT epochs,
interpolated base): legacy RMS z 38 / 62 / 227 (E/N/U), mode 3 16 / 14 / 18;
NEES inside 19 % -> 30 %. FLOAT errors there are metre-level, not gross, so the
SPP cross-check rarely acts and the gain is smaller. Batch `.pos` output of the
unmodified binary, the new binary with the option OFF and with mode 3 is
md5-identical on all six runs, and so is the debug epoch log (checked on two
runs).

## Why it was over-confident, and what remains

1. The value was a constant (`rtk.cpp` `generateSolution`).
2. With the tight INS prior, an error of more than the 30 m outlier threshold
   makes the filter zero every row (`rtk_filter.cpp:458`, `rtk_update.cpp:256`):
   NIS 0, no measurement update, position carried by the INS prior that is
   re-anchored from the same posterior (`online_rtk_imu.cpp:231`).
3. Up to two iterations re-apply the same rows to the updated covariance
   (`rtk_epoch.cpp:308-313`).
4. Not fixed: for converged epochs the Kalman marginal is 3-6x too small versus
   the decimetre errors (time-correlated multipath modelled as white), and mode 3
   is slightly worse than the old constant for error < 1 m. A covariance floor or
   a correlated-noise model would address it; both change the model and were not
   tried. FIXED wrong-fix epochs in the batch path (NEES inside 91 %) are not
   addressed either.
