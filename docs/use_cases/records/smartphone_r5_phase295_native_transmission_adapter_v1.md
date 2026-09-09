# Phase295 native observation transmission adapter

Starting HEAD `d07e728`; root-only. Added selectNativeEpoch accepting
already tracking-selected, quality-masked Observation rows for one epoch.
It uses preserved pseudorange_observation_type, not SignalType ordinal, to
map band to source slot and then group by satellite. Missing/malformed code
provenance, unsupported bands and duplicate slots reject. Invalid or
has_pseudorange=false rows are skipped. No code priority is invented.

Read-only findings: rinex.cpp selects candidates using native band/tracking
priority before emitting observations. android_raw_gnss.cpp:915-927 sets
code provenance, and its late duplicate handling is satellite/signal based.
These operations are not proof of source RTKLIB tracking-priority parity.
Discarded alternatives cannot be recovered by this adapter. Source tracking
priority remains unresolved; the adapter requires preselected input.

Tests exercise native Galileo C7Q versus C5Q selection despite reversed
arrival order, C8Q ambiguity retained in the same native SignalType, absent
code rejection, invalid-row skip and duplicate C7Q/C7I rejection. The code
syntax check does not certify every tracking letter/system combination or
SignalType consistency. Those remain upstream admission responsibilities.

Production FGO is not wired yet. Next state adapter work must preserve
selected code provenance through ephemeris/clock evaluation and prove the
caller supplies the correct masked epoch. No raw, truth, candidate, MAT,
station-table, network or Kaggle/token payload was read. No score computed.

Validation: target build succeeded; all 23 base-compensation tests passed.
Existing tests include synthetic RINEX fixture I/O. Not full CTest or
end-to-end raw-data parity.
