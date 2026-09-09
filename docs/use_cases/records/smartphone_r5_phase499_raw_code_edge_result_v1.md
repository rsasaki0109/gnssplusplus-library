# Phase499 — raw H adjacent code-edge localization candidates

Built native_code_edge_audit.cpp with current native libraries (initial link
without solver dependencies failed; corrected link succeeded). One H raw
loader audit completed exit 0. Input SHA256 checked:
46482b82db0992c1f063dbd9cf697268605234d3e38bcbd23525fd4b60bc17a7.
verify_enriched_pseudorange=false; no derived coordinate inputs, nav/FGO or
truth read. Parser-admitted observations only, using default loader times.

3140 epochs; actual upstream P-D mask removes 2148 identity/epoch rows.
102491 valid unique-identity consecutive-epoch triples; 360 have both P-D
edges exceeding the existing band threshold. Of these, 340 have opposite
signs; in all 340 triples both neighboring rows are in the actual mask.

These are triples, not 680 unique clean neighbors or recoverable factors.
Triples overlap, opposite signs do not identify the true bad sample, and
later builder geometry/residual masks may also remove either neighbor.
Current audit does not separately test clock continuity beyond existing
loader/mask behavior. Do not treat these counts as an admission rule.

Next count unique neighboring identities and intersect other masks in the
same-run native builder, with explicit clock/timing continuity and an
independent consistency witness before considering readmission. No change
to production selection, no threshold tuning or scoring. Goal remains unmet.
