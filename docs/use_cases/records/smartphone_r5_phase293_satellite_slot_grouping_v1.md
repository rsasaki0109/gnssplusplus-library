# Phase293 satellite slot grouping and mapping caveat

Starting HEAD `0200dae`, root-only. Production FGO remains unchanged.

Local obs2obs.c:232-247 copies obsd_t array indices directly into named
L1/L2/L5/etc fields. Those labels are storage slots: a physical RINEX band
digit alone does not prove the correct RTKLIB index for every constellation.
Native signal_policy.hpp also maps multiple RINEX bands to some SignalTypes.
Therefore Phase292's slot names must not be interpreted as an established
universal SignalType mapping. Source code-to-index and tracking selection
must be resolved before connecting native observations.

Implemented selectBySatellite for explicitly slotted, single-epoch rows.
It groups by full SatelliteId (constellation and PRN), then applies the
existing source slot selector. Duplicate slots, invalid slot indices and
PRN zero reject rather than silently picking an arrival order. Output is
returned only after the entire input is processed. Caller data is not
mutated. Constellation membership and cross-epoch identity remain caller
responsibilities; this API alone does not authenticate those properties.

Synthetic test covers constellation separation at equal PRN, reversed row
order, duplicate rejection, invalid slot and all-missing satellite. No
native SignalType mapping, solver integration or real-data parity is claimed.
Next: resolve the pinned RTKLIB code-to-index table and source raw converter
mapping before populating these rows in base and rover builders.

No real raw, truth, candidate, MAT, station-table or network/Kaggle reads.
Tests use synthetic data and existing synthetic RINEX fixtures only.

Validation: gnss_run_tests built successfully; all 21 base-compensation
tests passed. No full CTest or accuracy evaluation was performed.
