# Phase500 — candidate neighbor overlap is substantial

Extended native_code_edge_audit.cpp and rebuilt against current libraries.
H raw-only audit exit 0; previous 3140 epochs, 2148 P-D masked rows and 340
opposite-sign triples reproduced. Enriched pseudorange checking remains off;
no derived coordinates, FGO execution or truth used.

Unique (epoch,satellite,signal) identities:

- Reversal centres: 340.
- Their neighboring rows: 659 (not 680).
- Neighbors also appearing as reversal centres: 114.
- Neighbors with a bad edge to a non-centre adjacent identity: 106.
- Neighbors missing at least one non-centre adjacent edge: 106.

The last three sets may overlap; do not subtract their counts to infer a
recoverable count. Edges incident on any identified centre are excluded from
the non-centre check. A row flanked by centres can have no independent edge
left, so zero reported bad edges does not itself establish clean support.
Finite unique pairing and 1.5 s gap limits are enforced by the existing helper;
an explicit hardware-clock continuity audit and downstream-mask intersection
remain outstanding.

Evidence argues against the naive rule "keep neighbors, drop centres": 114
neighbor identities are themselves candidate centres. Next require an explicit
available outside-edge witness and disjoint candidate sets before considering
builder-level admission experiments. No weights or observation selection
changed. Counts are raw candidates, not validated clean/recoverable factors.
Overall accuracy/leaderboard objective remains unmet.
