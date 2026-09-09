# Phase317: paired raw base/rover correction admission

After the freeze commit and all source/binary/raw hash checks, ran the native
diagnostic once on H. It computed base corrections and rover FGO factors in
one process, then called the existing finite-correction miss-mask transaction.
No corrections, seeds, or trajectories were saved or reloaded.

Native exit 0, stderr empty. Expected raw phone epochs (3140) and base state
count (68697) matched. Rover state count remained 74339, with no missing-nav
events or seedless epochs. Of 101536 adopted P factors, 81161 were corrected
exactly once, 19092 lacked a stream, 1283 had unavailable corrections, and
none had a nonfinite returned correction. Exact count conservation passed.

The unavailable count is callback failure, not independently proven to be
time-domain failure in this phase. Do not transfer Phase313's raw-row domain
breakdown to this post-admission population. The pre-correction count differs
from Phase316 because zero group delay is now enabled for rover factors and
SPP; this is a different diagnostic configuration, not an accuracy comparison.

This proves raw-only in-memory connection through factor correction, not
numerical source parity or performance. Native rover rotation and orbital
equations remain; raw header reference is not station-table parity. The CLI
operational recipe, IMU integration and optimizer were not exercised. No MAT,
truth, saved positioning input, candidate trajectory, or submission was used.

Next bring the paired path into the experimental operational CLI configuration
and its provenance output, then validate full native FGO/IMU optimization before
one frozen evaluation. Keep the established base-OFF recipe available. Neither
0.782-class performance nor leaderboard standing has been established.
