# Phase237: joint source noise decision

Phase236 completed once from frozen raw GNSS/IMU/nav inputs, without MAT or
precomputed positioning inputs. Both stages converged; structural record was
committed before evaluation. Phase237 evaluator and manifest were frozen at
7e5bd39 after 13 selected Python tests passed. Candidate and truth payloads
were each read once; one score was calculated.

H development score: 1.1412645286818073 m, versus Phase235 best
1.0769392017393964 m (delta +0.06432532694241089 m).
P50: 0.8264285272326909 m; P95: 1.4561005301309238 m.
All 3139 rows matched exactly, finite and Earth-valid; no interpolation,
hold, offset reapplication, or over-70-m/s violations.

Do not promote Phase236. Retain Phase234/235 as the current H development
reference: metre-domain dynamic TDCP sigma, Huber k=4, main Doppler and
motion enabled. The k=0.2 source combination improved P50 slightly but
worsened P95 and the prespecified scalar. Production defaults remain off.

This is repeatedly used training/development H, not independent validation
or leaderboard evidence. The 0.782 m objective remains unmet.

Next investigate source/native SNR percentile population and masking order
using raw-only/source-code evidence, before deciding on another experiment.
Do not sweep Huber thresholds against this truth. Source sigma units are
now aligned, but population and atmospheric residual parity remain unproven.
Broader route validation is still required before generalization claims.
