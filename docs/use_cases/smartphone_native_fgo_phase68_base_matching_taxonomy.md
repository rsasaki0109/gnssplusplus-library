# Phase68 base-matching taxonomy

Phase68 is a bounded, truth-free diagnosis of the Phase67 base-RINEX
coverage failure. It reads each pinned raw Android GNSS file and each pinned
base RINEX member once in one process; it does not run a solver and does not
read truth, MAT, navigation, IMU, validation, or prior phase output.

For each adopted raw pseudorange proxy row, the evaluator records whether the
native exact `(system, SVID, SignalType)` stream exists, whether a canonical
same-frequency stream exists for that satellite, whether either stream is
outside its finite time domain, and whether the satellite/frequency is absent
from the base. It separately counts duplicate finite base code observations at
the same satellite/frequency/epoch. Classification follows the source
contract and never chooses a signal variant from the observed result.

The Phase67 coverage gate is unchanged. This phase only identifies whether
the 15,198 misses are due to unsupported constellation/satellite, signal-code
identity, or base time-domain coverage. A future implementation phase would
require a new freeze and source-supported duplicate handling; no correction is
authorized here.

See the [Phase68 freeze](records/smartphone_r5_phase68_base_matching_taxonomy_freeze_v1.json)
for all input hashes, source pins, and read accounting.
