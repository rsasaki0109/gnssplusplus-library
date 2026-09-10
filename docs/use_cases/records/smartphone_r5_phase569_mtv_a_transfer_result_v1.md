# Phase569: current baseline on MTV-A development

Phase568 native session 75280 completed exit 0 in 153.29674580006395 seconds.
Frozen source/binary/raw pins verified. Output SHA:
`76322d25a5399d5a1324ad8bc4dd9d8a36b2a9148d4cbf13b33f2445c9f209b5`.
Same binary and solver flags as Phase563 H baseline; changed dataset/raw/output
paths only. Main/GNSS-first converged with finite final costs, 2,159 modeled
epochs and same-run GNSS-first coordinate handoff. No MAT/base/saved solution
inference, no route-specific tuning, no output repair.

Separate evaluation manifest frozen after completion checks. Existing
Phase76/74 parser/metric reused, pinned against previous evaluation authority.
One candidate read and one truth read; evaluation does not invoke inference.
2,158 predictions match 2,158 of 2,159 truth rows. The single missing first
key 1615921153434 is exactly the historical Phase146 contract, fixed before
truth read, not chosen from errors. No interpolation or offset reapplication.

- Score (P50+P95)/2: **1.3613354169383562 m**
- P50: 1.241926785340521 m
- P95: 1.4807440485361916 m
- Mean: 1.2074261000949946 m; maximum: 1.5730654029516462 m
- Prediction-domain coverage: 1; finite; over-70-m/s count: zero.

This adds a fourth current native-baseline development route, not a new
algorithm improvement. MTV-A was evaluated in Phase146, so this is not fresh
validation/holdout or leaderboard evidence. Older configurations are not
matched controls for this run. The score remains above .782; do not claim
success or promote any recent diagnostic model from this result.

Next: compare raw-derived model/geometry diagnostics across the four current
development routes to select a shared implementation hypothesis; reserve
untouched data for a genuinely frozen candidate after truth-history audit.
Do not fit a route offset from this score or turn aggregate error into an
inference input. No submission; full CTest unrun; objective remains unmet.
