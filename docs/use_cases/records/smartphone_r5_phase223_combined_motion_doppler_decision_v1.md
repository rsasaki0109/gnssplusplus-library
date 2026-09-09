# Phase223 combined motion/Doppler development decision

Primary agent. Evaluation frozen at 18733af before truth access; 13 tests
passed. One candidate read, one truth read, one score, no native rerun.

H scalar: 1.2680574850266653 m versus Phase219 1.2689788175187473 m,
delta -0.0009213324920820387 m. P50 worsened from 0.7606401104947066 to
0.7706427231288419 m; P95 improved from 1.7773175245427881 to
1.7654722469244886 m. All 3139 rows matched exactly, finite and Earth-valid;
zero over-70-m/s transitions. No interpolation, hold, offset reapplication.

Phase222 becomes best on the frozen H scalar only. The sub-millimetre scalar
gain is not evidence of a robust general improvement; do not change production
defaults or claim leaderboard/heldout performance. Target 0.782 m remains
unmet. No MAT or precomputed positioning input was used by the native solver.
The saved candidate was evaluator-only. Runtime 159.8143124129856 seconds
is a single observation, not a reproducible speed benchmark.

Next prioritize a source/native measurement-model audit over further tiny
H-only parameter searches: inspect raw correction and TDCP likelihood
differences with controlled synthetic tests, without porting truth-dependent
height priors. Broader route validation and a leakage/split audit remain
required before generalization or leaderboard claims.
