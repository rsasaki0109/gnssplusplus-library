# Phase444 — deficient LAX-T epoch topology

Phase442 completed once, exit 0, 221.09452460706234 s. Frozen manifest SHA
`ee77e823e2ffec88a9e14c57ff4551149ba905e3bb414bd9562dc1230804bbea`.
Verifier passed source/test/algorithm, raw/binary pins, baseline candidate SHA,
stage convergence, unchanged bias aggregates, 1466 information epochs and
exactly two distinct valid deficient-epoch log entries. No truth/scoring.

| Zero-based epoch | Admitted P | Clock rank | Incoming TDCP | Outgoing TDCP |
|---|---:|---:|---:|---:|
| 851 | 1 | 1 | 1 | 1 |
| 852 | 6 | 4 | 1 | 1 |

Minimum eigenvalues: 0 and 1.340271127241753e-18 m^-2 respectively.
The row-minus-clock dimension bound explains the single-epoch deficiency:
position rank at most 0 and 2, not 3. There is no complete absence of TDCP
edges, but one edge per side does not prove full position observability.

Raw MessageType/UTC-only follow-up: epochs 850–853 have 17,14,18,20 raw
GNSS rows respectively and all neighboring timestamp gaps are exactly 1000 ms.
Raw rows are NOT admitted P factors; these counts do not identify which
quality/geometry masks caused the loss or justify bypassing them.

The two adjacent sparse epochs are a localized support issue, not proof of
the route-wide error's cause. Next inspect admission reasons for raw rows at
these epochs before proposing a model change. Do not fill with saved positions,
remove timestamps, weaken gates blindly, or repeatedly score bias/prior tweaks.
