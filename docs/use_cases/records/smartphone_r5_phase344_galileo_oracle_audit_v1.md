# Phase344: Galileo broadcast propagation comparison

Extended the Phase336 standalone oracle diagnostic with explicit `galileo`
selection, PRNs 1..36, and a fail-closed check that RTKLIB satellite numbering
actually resolves to the requested constellation. Recompiled BOTH external
C objects and the diagnostic with `-DENAGAL` to keep header/layout and
constellation configuration consistent. Same MADOCA source revision/hashes
and raw H navigation input as Phase336; no saved positioning input or truth.

Before the run, retained the same three toe-relative times (-3600,0,+3600 s)
and fixed 0.001 m position/clock tolerance. All valid native-parsed Galileo
messages are compared, not only source-selected I/NAV messages. Same mapped
elements are passed to independent eph2pos, so parser and message selection
parity remain outside scope.

One raw-navigation run exited 0: 31953 comparisons (10651 messages).
Maximum position difference: 3.0048686647604551e-08 m.
Maximum clock difference expressed in metres: 8.4499513838148278e-06 m.
Neither exhibits the metre-scale discrepancy needed to explain the observed
base-compensation degradation. No satellite coordinate/clock series exported.
No production change or new positioning accuracy result.

Build follows Phase336 native link with separate gal_ephemeris.o and
gal_rtkcmn.o; executable under
`/dev/shm/gnss-test-build-recovery.BJvQt9/gal_oracle_audit`.
GPS default invocation remains available. This comparison is independent
MADOCA propagation, not exact pinned MALIB/full-pipeline equivalence.
