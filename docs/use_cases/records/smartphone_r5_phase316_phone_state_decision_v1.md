# Phase316: raw H phone admission through shared satellite states

Executed the Phase316 frozen native diagnostic once after verifying all four
source/binary/nav/phone hashes. Native exit 0; one aggregate stdout line and
zero stderr bytes. The expected 3140 raw epochs were consumed. The FGO builder
constructed 74339 satellite-epoch states, with zero missing-navigation events
and zero epochs lacking a seed, and admitted 101888 pseudorange factors.

This resolves whether the new receive-time selector/state propagation can
process this raw phone/navigation pair without exceptions or navigation-local
misses. It does not validate numerical orbit parity, estimator accuracy, all
routes, or leaderboard performance. SPP seeds were computed from raw GNSS in
the same process and were not saved or reloaded. No optimizer, base correction,
IMU, MAT payload, truth, or candidate trajectory was used.

Do not compare the factor count to the operational Phase234 recipe as a
regression check: this diagnostic uses default FGO settings plus the four
explicit flags in its source, raw parser epoch times, and no operational CLI
UTC preprocessing. It is an admission diagnostic, not that full recipe.

Next: expose a paired base/rover experimental recipe with explicit provenance
and missing-correction policy. Check rover group-delay/geometric conventions
and operational GLONASS annotation before claiming source-complete pairing.
Then freeze a native raw-only candidate and evaluate once. The operational
base-OFF recipe and accuracy result remain unchanged.
