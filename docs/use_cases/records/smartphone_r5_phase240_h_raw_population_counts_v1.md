# Phase240: H raw population counts

Read-only CSV audit of the Phase37-pinned H Pixel5 device_gnss.csv.
Only raw timing, constellation/SVID, signal/frequency, Cn0 and epoch-key
fields were interpreted. Enriched coordinates and pseudoranges were not
used; no MAT, truth, candidate, solver or network access.

Sequential counters from Python csv.DictReader:

- Input: 112833 rows.
- TimeNanos zero or ReceivedSvTimeNanos < 10000000000: 35 rows.
- Remaining BiasUncertaintyNanos > 10000: zero rows.
- Remaining constellation outside GPS/GLO/BDS/GAL, or GLO SVID > 24:
  3051 rows.
- Remaining: 109747 rows, exactly Phase234 native selected_rows.
- Remaining Cn0DbHz: 109747 positive, zero missing/nonpositive.
- Duplicate excess for (UTC, constellation, SVID, frequency band): zero.
  Bands here use <1.3 GHz for L5, otherwise L1; observed frequencies
  consist of GPS/GAL L1 and L5, BDS B1I, and GLO L1 channels.

An initial key using literal SignalType reported two apparent duplicates.
This key was invalid for the audit because empty tokens can occur on
different bands. The frequency-band key removes that ambiguity; do not
report these as duplicate observations.

Since the counted surviving rows equal native selected_rows and the
native loader only removes rows, the aggregate evidence leaves no extra
H removal attributable to the invalid-raw-pseudorange boundary identified
in Phase239. No population correction is justified by this H audit.
This is not a general proof for other routes, nor an exact independent
reimplementation of every native frequency tolerance or source converter.

Next investigate the remaining TDCP residual-model difference, keeping
metre sigma and k=4 as the current best H reference. Do not alter loader
admission merely to pursue a hypothesis contradicted by these counts.
