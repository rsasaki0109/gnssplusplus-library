# Phase345: GLONASS propagation closes the base-constellation orbit check

Added explicit `glonass` mode to the standalone broadcast oracle comparison.
Map native raw-parsed position/velocity/acceleration, toe, taun and gamn into
independent MADOCA geph2pos. Compare all valid messages for PRNs 1..27 at
toe +/-1800 seconds and toe (GLONASS admission interval), with the same
fixed 0.001 m position/clock tolerance. Compile C objects and C++ consumer
consistently with both DENAGAL and DENAGLO; fail if satellite numbering does
not resolve to the requested constellation. No receiver solutions/truth/MAT.

One raw H navigation pass exited 0: 3456 comparisons (1152 messages), maximum
position difference 7.4505805969238281e-09 m, maximum clock difference 0 m.
Executable: `/dev/shm/gnss-test-build-recovery.BJvQt9/glo_oracle_audit`.
The source revision and raw navigation provenance remain those of Phase336.

Together with Phase336 GPS and Phase344 Galileo, all constellations present
in this H raw base file now have sub-millimetre sampled propagation agreement
with independent MADOCA equations for matched elements. This does NOT prove
independent parsing, transmission-time/message-selection, tracking-code,
antenna-reference, or full source pipeline parity. It does lower the priority
of a metre-scale native orbit-propagation bug as the correction regression's
cause. Stop extending this sampled propagation audit without new evidence;
further progress needs correction-reference/measurement-level experiments.
No production change or new accuracy score.
