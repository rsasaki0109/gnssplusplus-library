# Phase407 — actual graph replacement fixed

Root cause of the Phase406 zero-state failure: installed GTSAM
`FactorGraph.h:348` declares `sharedFactor back() const`, returning a copy.
Assigning `graph.back() = wrapper` only changed a temporary shared_ptr, not
the graph. The newly inserted residual-state prior existed, but neither TDCP
factor actually depended on that key. Counters alone masked this error.

Changed the backend assignment to mutable indexed access
`graph[graph.size()-1] = wrapper`; installed GTSAM returns a mutable reference
from the nonconst index operator. Existing factor geometry/noise and OFF
paths are untouched.

Rebuilt library/app successfully, recompiled current native tests and linked
against the rebuilt libraries. Focused Phase171 main suite passes 2/2,
including the strengthened synthetic frequency discrepancy, nonzero exported
correction, per-factor identity and reconstructed/backend RMS agreement.
This resolves the prior failing assertion without weakening it or adjusting
its prior/noise values. No raw GNSS solve, truth, MAT, score, or saved-position
input was used. All build/test sessions completed.

Phase402–405 counter-only enabled-path results did not prove state connectivity;
this regression supersedes that interpretation. CLI remains unexposed.
Next validate the actual app diagnostic failure cases and raw epoch-time
pairing provenance, then freeze a controlled raw experiment with an explicit
justified prior and disabled-output replay. No accuracy gain is established;
the native .782-class/LB objective remains unmet.
