# Phase302 selection-time boundary test

Starting HEAD d8c1e60a; root-only. Production code unchanged.

Added actual NavigationData + selected-state synthetic test: two SBAS
messages with toe 100/102 s and differing positions; reception 101.05 s
selects the newer message, while transmission at 100.95 s selects the older.
Passing the reception-selected object to stateFromSelectedEphemeris retains
it throughout both propagation evaluations. The synthetic 100 m difference
is deliberately constructed, not evidence of a real H boundary error.

Current NavigationData::getEphemeris uses native isValid/getAge, first
strictly best age, and optional Galileo environment filtering. isValid
uses 14400 s for Keplerian systems and 1800/7200 s for GLONASS/SBAS.
Therefore supplying reception time alone does not make it equivalent to the
pinned source selector. Do not wrap it and label it source-complete.

Next implementation: a separate explicit source selection policy over the
existing broadcast records, with pinned constellation age limits, tie order,
Galileo message/toe rules and no hidden environment dependency. Keep the
shared legacy selector unchanged. Then join selection to the already-tested
transmission/state adapter. This is necessary integration, not an accuracy
claim or a reason to sweep route thresholds.

No real raw, navigation payload, truth, candidate, MAT, station table,
network or Kaggle/token input; synthetic fixtures only. No score.

Validation: gnss_run_tests build succeeded; all 29 base-compensation tests
passed. Not full CTest, source-selector parity or real-data validation.
