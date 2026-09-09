# Phase278: stage-dependent pseudorange admission

Source-code audit only; no raw/candidate/truth/MAT payload access.
Phase277 relative-height regression does not motivate a threshold sweep.

Cached gsdc2023/fgo_gnss_imu.m:41-61 chooses the previous stage's position,
velocity and clock estimates. At 65-75 it reapplies exobs, computes satellite
geometry/residuals at that estimate, applies exobs_residuals, and recomputes
residuals. exobs_residuals.m centers corrected pseudorange residuals by a
whole-matrix system/frequency median, then applies the configured threshold.
parameters.m:72-77 distinguishes initflag thresholds (50/30 m) from final
thresholds (20/15 m). run_fgo.m:46 and :55 call the IMU algorithm with true
and false respectively. The MAT-based persistence used by that source is
prohibited here; this audit does not execute those calls.

Native fgo_problems.cpp:1114-1156 centers upstream_seed_residual_m by
system/band and filters P rows during construction. In the current selected
CLI path, gnss_first_problem copies problem. The ECEF-D staging rebuild
around gnss_fgo_imu_no_base.cpp:12323 copies only D rows; its comment explicitly
retains the original P/TDCP admission. After successful GNSS-first, the code
around 12606 replaces main epoch positions/clocks but does not reconstruct
P admission from all eligible raw observations at those updated states.

Thus updating initial Values is not equivalent to the source's repeated
measurement selection. A GTSAM nonlinear factor can relinearize retained
rows, but cannot recover a row already discarded by a seed-residual mask.
This is an identified pipeline difference, not proof of an accuracy gain.

Next bounded work: synthetic tests of the actual grouped-residual admission
operator with biased and corrected seeds, demonstrating both rejection and
recovery while holding the raw observations and thresholds fixed. Audit
which pre-mask rows remain available in memory. Any implementation must
retain/rebuild eligible raw observations, not merely re-mask the already
filtered factor vector. Preserve exact source keys, and report old/new,
recovered/rejected counts without coordinates. Keep the same-run handoff;
no saved trajectory or MAT intermediate. Separate re-admission from an
extra IMU solve and changed threshold schedule in the first experiment.

Do not claim full source parity: base correction, source affine versus
native nonlinear geometry, and the source's outer-stage schedule remain
separate differences. H remains repeatedly used development data and the
0.782-class/leaderboard objective remains unachieved.
