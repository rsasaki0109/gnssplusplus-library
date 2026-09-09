# Phase401 — admitted-factor pairing utility

Implemented `tdcp_frequency_pairing.hpp`, operating read-only on native TDCP
factor objects. Returns original L1/L5 factor indices, keyed by exact previous
and current epoch indices, constellation and PRN. GPS L1CA/L5 and Galileo
E1/E5a are recognized; other signals and unmatched rows are not modified.
No raw observation can be recovered or synthesized by this interface.

Candidate pairs require equal finite positive dt, adjacent epochs, dt<=1.5 s,
and neither endpoint marked as a clock jump. Duplicate recognized band keys,
invalid indices/measurement metadata or unequal endpoint durations throw.
Results are deterministic by endpoint/system/PRN, not input arrival order.

Fresh standalone tests passed 5/5: reversed band order and unmatched-row
preservation; different satellite/interval separation; duplicate/dt mismatch
failure; clock reset/bridged-epoch exclusion; invalid index failure. Registered
in the existing CMake test list. Full CTest not run.

The utility is not yet called by the backend. Epoch identities and clock
flags must originate in the same native problem. It cannot validate raw
cross-band TimeOffsetNanos after those fields are discarded, actual epoch-time
versus dt consistency, or that rows were admitted by a particular selector.
Those are integration/provenance responsibilities, not inferred guarantees.

Next connect the pairing and residual-state wrapper through a default-off
native backend branch, with one prior per pair and original unmatched factors
retained. Include disabled-output and modified-residual diagnostics controls.
No raw solve, truth, MAT, saved trajectory input or accuracy score this phase.
The full performance goal remains unmet.
