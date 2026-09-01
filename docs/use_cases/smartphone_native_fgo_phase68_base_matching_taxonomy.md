# Phase68 base-matching taxonomy

Phase68 was intended as a bounded, truth-free diagnosis of the Phase67
base-RINEX coverage failure. Its first one-shot execution was fail-closed by
an evaluator-integrity error before a route report was produced: the pinned
RINEX3 members use long single-line satellite records, while the evaluator
assumed one 80-column continuation line per five observation types and then
treated the next `>` epoch marker as a satellite record. It read one raw and
one base member for the first route, read no truth/MAT/navigation/IMU, and
ran no solver. No physical matching finding is claimed.

The immutable failure accounting is recorded in the
[Phase68 failure record](records/smartphone_r5_phase68_base_matching_taxonomy_failure_v1.json).
Phase69 is a separately frozen recovery with a new output root and fixture
coverage for both long records and standard continuation records; it must not
reuse this partial output or reread Phase68 inputs.

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
