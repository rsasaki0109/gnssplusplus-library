# Phase512 — GPS L5 needs ISC provenance, not a guessed TGD multiplier

Specification check: [IS-GPS-705J](https://www.gps.gov/sites/default/files/2025-07/IS-GPS-705J.pdf),
section 20.3.3.3.1.2.1, printed page 78 (PDF page index 90), gives L5 satellite
clock correction as delta_t_SV - TGD + ISC_L5I5 or ISC_L5Q5. These signal-specific
ISC values are supplied in message type 30. The frequency-squared multiplier
in the following ionospheric section is not a replacement for ISC.

Thus native P + c*clock - c*TGD has the sign/form of the TGD portion;
using the same TGD for GPS L1/L5 is not alone proof of a frequency-scaling bug.
Missing ISC is an unmodelled contribution, not evidence that its true value is
zero. This does not certify LNAV/CNAV clock interchangeability or full L1 parity.

Local input/parser evidence:

- All three Phase37 H/U/LAX brdc.nav headers are RINEX 3.04.
- rinex.cpp legacy GPS record parsing retains the primary TGD and IODC; it
  sets secondary TGD to zero for GPS. Ephemeris has no GPS L5 ISC field.
  The isc_l3ocp field found by search belongs to GLONASS and is unrelated.
- Android signal mapping accepts GPS_L5_Q but stores SignalType::GPS_L5.
  The loader does not consume the named inter-signal bias columns.

New scripts/audit_raw_gps_l5_bias.py was run successfully on the three Phase37
raw CSVs (same SHA256 pins as Phase511). It reads only raw constellation,
frequency, signal/code identifiers and four bias/uncertainty fields; hashing
reads bytes without interpreting derived columns. No position, truth or MAT.

| Route | Raw GPS L5 rows | SignalType | CodeType | Finite values in each of four bias fields |
|---|---:|---|---|---:|
| H | 19962 | GPS_L5_Q throughout | empty throughout | 0 |
| U | 3111 | GPS_L5_Q throughout | empty throughout | 0 |
| LAX | 4857 | GPS_L5_Q throughout | empty throughout | 0 |

Fields: FullInterSignalBiasNanos, FullInterSignalBiasUncertaintyNanos,
SatelliteInterSignalBiasNanos, SatelliteInterSignalBiasUncertaintyNanos.
All columns exist; none provides a finite correction for these raw L5 rows.
Counts are not final factor counts and do not imply sensor accuracy.

Decision: reject a frequency-ratio TGD adjustment or truth-fitted ISC table as
the next experiment. A true ISC implementation first needs contemporaneous
raw CNAV message-30 provenance and compatible clock/signal semantics. Do not
substitute today's calibration table or silently widen inputs to precise products.
No production correction, default or score changed. Next inspect available
raw navigation-message sources/metadata for that provenance; if unavailable,
continue another observable-model hypothesis rather than inventing bias values.
