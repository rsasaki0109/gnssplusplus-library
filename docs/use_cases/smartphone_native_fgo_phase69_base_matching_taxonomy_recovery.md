# Phase69 base-matching taxonomy recovery

Phase69 is a separately frozen, truth-free recovery of the Phase68
evaluator-integrity failure. Phase68 read the first route's raw GNSS and base
RINEX once, then failed because its RINEX3 parser assumed one 80-column
continuation line per five observation types. The pinned base members contain
long single-line satellite records, so that assumption consumed the next `>`
epoch marker as a satellite record.

The Phase69 parser uses physical record framing: a first satellite line longer
than 80 columns is consumed alone; a standard line at most 80 columns consumes
the declared `ceil(observation_type_count/5)` lines and rejects a premature
epoch marker. Focused fixtures cover both forms and continuation observation
types. The diagnostic taxonomy remains exact SignalType, canonical
same-frequency variant, out-of-domain time, missing frequency, missing
satellite, and report-only duplicate canonical-frequency observations.

All four pinned raw CSV and base RINEX members are read exactly once in one
process after the evaluator manifest is sealed. No truth, MAT, navigation,
IMU, solver, validation, archive, or prior partial output is read. Raw
adopted and base selected populations are explicitly diagnostic proxies; they
are not claimed equal to native adopted FGO factors. The Phase67 coverage gate
and all correction policy remain unchanged.

See the [Phase69 freeze](records/smartphone_r5_phase69_base_matching_taxonomy_recovery_freeze_v1.json)
for immutable hashes and the [Phase68 failure record](records/smartphone_r5_phase68_base_matching_taxonomy_failure_v1.json)
for the preserved first-attempt accounting.
