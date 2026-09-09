# Phase501 — 426 raw neighbor identities have outside-edge support

Extended native_code_edge_audit.cpp to require at least one present,
threshold-consistent edge to a non-centre neighbor, no missing/bad such edges,
and exclusion of identities also classified as reversal centres. H audit:
426 of 659 unique candidate neighboring rows pass. 233 have no consistent
outside edge. This is an intersection computed per identity, not subtraction
of the previously overlapping rejection counts.

Then added explicit epoch_hardware_clock_discontinuity_count coverage check,
rejecting triples and outside edges across count changes. Rebuilt/reran raw
audit, exit 0: 0 discontinuous triple windows and the same 426 supported rows.
The preceding counts (340 centres, 659 neighbors, 114 overlap) reproduce.
No FGO or truth evaluation; native loader ignores enriched pseudorange.

Support is temporal self-consistency, not an independent proof of unbiased
code. A persistent offset shared by a neighbor and its outside sample still
passes. Same-device Doppler is not statistically independent truth. These
426 rows remain subject to all downstream masks and geometry; do not call
them clean, recoverable or an accuracy improvement. Next integrate a
diagnostic-only candidate identity set with same-run builder row rejection
reasons before any explicit readmission experiment. Defaults unchanged.
