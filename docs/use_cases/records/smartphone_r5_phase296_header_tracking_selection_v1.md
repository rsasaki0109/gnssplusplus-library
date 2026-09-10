# Phase296 source header tracking selection

Starting HEAD `abf7493`; root-only source port, no production integration.

Pinned MALIB commit 159e150d4a54e6b7b15d81128289b8559523ca81:
- src/rinex.c SHA256 3d5f163d06a4b6286f51966f2a5c3a1b5a86c8af938ee676964f6f6980e04f6d
- src/rtkcmn.c SHA256 4176a2f24d184323d0dd5eb44dd2345dabf9b79b524847633f7901451a648fa6

Both read by in-memory HTTPS from raw.githubusercontent.com/JAXA-SNU/MALIB.
set_index assigns the greatest nonzero priority per frequency index from
header types; equal priority retains header order. All observation kinds
with the chosen code share its slot. Extended observations are separate.
getcodepri uses constellation/index tracking tables unless options override.

Added source_tracking_selection.hpp for fixed default priorities and header
selection. Input requires validated two-character codes from all header
observation kinds. This is not a complete obs2code validator, option parser,
RINEX2 translator or extended-observation implementation. In particular it
must not be called with only the currently nonmissing epoch P values.

Tests cover GPS Q/X/I preference, reversed unequal priority, BeiDou equal
priority/header-order ties and unsupported/default priority cases. Loader
integration must preserve header selection even when its measurement is
missing; this test is selection-only, not an epoch decoder proof.

No real raw, truth, candidate, MAT, station-table or Kaggle/token read.
Only algorithm sources fetched externally; no solver or score execution.

Validation: gnss_run_tests build succeeded; all 24 base-compensation tests
passed. Existing synthetic RINEX fixture I/O only; not full CTest or raw parity.
