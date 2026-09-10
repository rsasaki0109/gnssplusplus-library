# Phase312 exact correction-domain metadata

Starting HEAD 9eccb608; root-only. correctionAt returns false for missing
stream, invalid time, out-of-domain time, invalid spacing or nonfinite result.
Phase311's combined unavailable count cannot distinguish them. Existing
source_pseudorange_miss_mask labels callback false as out-of-domain; that
label is broader than a proven temporal cause and must not drive a fix.

Added read-only streamTimeDomain for exact satellite/signal keys. Returns
first/last timestamps of committed samples, not correction values; false
on absent streams. Interpolation, masks and published streams are unchanged.
Extended positive-model synthetic test verifies endpoint lookup success,
one millisecond outside rejection on both sides, and metadata absence after
a later failed rebuild clears streams.

Next use this metadata in a frozen diagnostic to separate the 1819 raw-phone
unavailable queries into before/after/in-domain failure. Do not extrapolate
or relabel numerical failure as temporal coverage. No raw/truth/candidate,
MAT, station-table/network/Kaggle input or score evaluation this turn.

Validation: gnss_run_tests target built; all 33 base-compensation tests passed.
Synthetic fixture I/O only. No full CTest, real-data or solver byte-parity claim.
