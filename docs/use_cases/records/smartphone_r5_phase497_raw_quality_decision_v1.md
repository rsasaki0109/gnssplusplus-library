# Phase497 — extra raw quality fields cannot discriminate H rows

Pinned H device_gnss.csv metadata audit completed exit 0 over 112833 rows.
MultipathIndicator is zero for every row. AgcDb, BasebandCn0DbHz,
FullInterSignalBiasNanos/Uncertainty and SatelliteInterSignalBiasNanos/
Uncertainty contain no finite values. This applies to the complete raw H
file, not a separately rematched retained-factor subset.

Existing loader already masks MultipathIndicator==1 for P/D/carrier. Adding
that same mask again provides no new selection on this route. All-zero
indicator is not proof of multipath-free reception. Missing AGC/baseband/bias
fields cannot serve as independent error witnesses, and must not be filled
with truth-derived values or treated as measured zero biases.

CSV includes derived satellite/WLS-position columns; the audit accesses only
the seven explicit raw quality metadata fields and does not use those derived
columns. No inference, correction, truth evaluation or measurement-default
change. Next evidence must come from available raw temporal/signal redundancy
or another physically justified model, not absent quality metadata. Goal unmet.
