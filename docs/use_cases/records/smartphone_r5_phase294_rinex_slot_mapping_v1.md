# Phase294 constellation-dependent source slot mapping

Starting HEAD `8d1b348`; root-only work.

Read pinned [MALIB rtkcmn.c](https://raw.githubusercontent.com/JAXA-SNU/MALIB/159e150d4a54e6b7b15d81128289b8559523ca81/src/rtkcmn.c)
through an in-memory HTTP request. SHA-256:
`4176a2f24d184323d0dd5eb44dd2345dabf9b79b524847633f7901451a648fa6`.
code2idx dispatches to constellation-specific band functions (633-753).
Ported their band-to-index mapping as slotForRinexBand. Galileo E5b maps
to index 1; BeiDou bands 1 and 2 both map to index 0. Slot labels therefore
must not be treated as universal physical frequencies.

The API requires a validated RINEX band digit, not a native enum cast. It
returns no value for unsupported system/band. This does not validate tracking
letters, implement tracking priority, or certify GLONASS frequency channels.
Those are distinct input-admission responsibilities.

Added a 7-system by 9-band synthetic table test, including unsupported cells,
plus out-of-range bands and UNKNOWN system. Existing duplicate-slot rejection
is retained; colliding codes cannot silently win by arrival order.

Production FGO remains unchanged. Next integration must carry the native
observation code provenance and resolve tracking priority before grouping;
do not infer an unambiguous code from a many-to-one SignalType.
No real raw, candidate, truth, MAT, station-table or Kaggle/token access.
The only external read was algorithm source. No score was computed.

Validation: gnss_run_tests target build succeeded; all 22 base-compensation
tests passed (including synthetic fixture I/O). No full CTest or raw parity run.
