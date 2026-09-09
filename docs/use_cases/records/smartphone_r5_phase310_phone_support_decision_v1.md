# Phase310 raw smartphone correction support

Frozen at ff590ac6 before reads. Base/nav/phone/source/binary hashes verified.
One hash scan plus one native read per real input; corrections built and
queried in the same process, never stored or imported. Enriched pseudorange
verification disabled, matching raw-clock-only policy. No truth, MAT, saved
positioning, receiver coordinate, station-table or Kaggle/token input.

3140 parser epochs, 108722 valid P rows queried by native satellite/signal
and raw-parser epoch time. Finite corrections 86731, missing stream 20172,
unavailable/nonfinite lookup 1819. Counts conserve exactly. Base build
matches Phase309: 3500 epochs, 112050 signal rows, 38 streams, no failure.

These are pre-FGO support counts: no main residual masks, first-epoch output
drop or subsequent UTC processing was applied. Not final factor admission,
canonical-band support, measurement improvement or positioning accuracy.
Unavailable count combines correctionAt failure and nonfinite output; it
does not alone prove a temporal-domain issue. Missing-stream constellation
breakdown is not recorded here. Do not fill these misses with zero correction.

Next expose per-system miss reasons and align this diagnostic with the
actual rover preparation before enabling the paired FGO path. Operational
baseline unchanged; no score calculation or submission. Diagnostic compiled
successfully; previous tests not rerun, no full CTest claim.
