# Phase342: correction ordering and observation weights

Read-only source review after the low-elevation hypothesis weakened in
Phase341. Local source `gsdc2023/fgo_gnss_imu.m` calls `exobs`, computes
residuals, calls `exobs_residuals`, recomputes residuals, then subtracts
base correction from resPc. `obserrmodel` follows subtraction, but its
weights depend on elevation or SNR and signal-type factors, not resPc or
the magnitude of the base correction.

Native `fgo_problems.cpp` builds weights and applies the grouped centered
pseudorange residual mask before returning the problem. The CLI then calls
`source_pseudorange_miss_mask::apply`. That transaction changes corrected
pseudorange and its applied flag, preserving sigma and other factor data.
This ordering alone is therefore not a discovered source-parity bug: do
not move residual masking after base subtraction or adapt sigma to the
correction magnitude merely because the source computes weights later.

The native `upstream_seed_residual_m` field remains the pre-correction
admission residual. Searches of the GTSAM implementation and this CLI found
no downstream use of that field. This is not proof against every consumer,
but there is no evidence here that stale admission metadata drives the
optimized pseudorange factor. The reviewed correction transaction subtracts
from `corrected_pseudorange_m` once and rejects repeated application.

No numerical run, score, truth or candidate read in this review. The
remaining source/native differences include raw-derived initialization and
its admission residuals, base antenna/reference interpretation, and exact
multi-constellation measurement conventions. Neither this review nor
Phase341 supports a production change to atmosphere or mask ordering.
