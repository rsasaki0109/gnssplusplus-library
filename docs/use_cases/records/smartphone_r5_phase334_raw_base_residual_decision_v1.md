# Phase334: simultaneous un-smoothed residual structure persists

Added raw-paired mode to the diagnostic: take finite simultaneous GPS L1/L5
observations, one shared satellite state, native source Sagnac range and
native Klobuchar/Saastamoinen terms. Do not build a smoothed correction model.
Both frequencies use zero explicit group delay and the same query-time rows.
Standalone build and synthetic algebra/invalid-input tests passed before the
freeze. One verified raw base/nav execution completed, exit 0, no stderr.

All 3500 epoch fits passed on 23564 common rows. Median modeled L1 ionosphere
was 3.4380462586061737 m and troposphere 3.3072850750580076 m. These are model
delays, not observed errors. Adding back the modeled dispersive terms changed
the neutral combination by at most 7.105427357601002e-15 m. This is an internal
algebra/numerical cancellation check, not independent model accuracy proof.

Median apparent displacement norms remained 2.9202065119665894 m for L1,
3.329359866543525 m for L5 and 5.405936422322276 m for the neutral combination.
The latter's median post-fit RMS was 1.8357061715795677 m. Thus distinct
smoothing histories are not required to produce this residual structure.
Do not interpret the neutral norm as a surveyed reference error or assume
the algebraic dispersive term contains only physical ionosphere.

Also inspected local MatRTKLIB `+gt/Gobs.m` residuals: resPc is P minus
(range - satellite-clock + ionosphere + troposphere), consistent with the
native base residual's algebra. `+gt/Gsat.m` calls RTKLIB ionmodel/tropmodel.
This rules out an obvious sign discrepancy in these expressions but does not
verify full orbital, timing, antenna-reference or atmospheric parity. The
native troposphere has low-elevation clipping; the diagnostic's >=15-degree
cutoff is above that clipping region. A full time-convention audit remains.

No truth, phone, saved trajectory, station table, MAT payload or inference
feedback was used; no score/submission. Next independently check raw receive
times, station-reference metadata and native/source satellite-clock/orbit
conventions before proposing a correction-value change. Keep the established
base-OFF operational recipe and reject fitted coordinate shifts from this
diagnostic. The 0.782-class and leaderboard goals remain unachieved.
