# Phase523 — H initial residual alignment verified

Phase522 native PID 3856424 / session 40665 completed exit 0 in
411.38760067301337 seconds. Output SHA256 remains
4a0c4ef8822c72aa9134368cd57fe769817895e63b5dbe925783a9c8d091fa8e.
No live job remains from this run.

Added verify_phase522_ionosphere_monitor.py; passed current source/binary/raw
pins, output identity, whole graph/GNSS-first summary equality versus Phase505,
both termination contracts and complete IRLS/residual marker validation.
Synthetic parser controls: valid fixture plus six invalid cases passed.
An initial generated-script syntax error was repaired before running the
verifier; it never affected the native process or frozen inference sources.

Results: supported 3140, unsupported 0; signed normalized coefficient/residual
inner-product median 0.0652986616768. Adjacent eligible pairs 3139, same-sign
2191. IRLS information marker remains exactly the Phase519 printed values:
101916 rows, 98767 downweighted, nominal median 0.895282078029 and IRLS median
0.0802815904466. No corrections applied, no truth or accuracy evaluation.

These aggregate values do not identify a physical ionosphere residual.
Signed median hides possible opposing effects; same-sign persistence is not
statistical significance without the marginal sign distribution/serial model.
Multipath, clock transitions or correlated initial errors can produce it.
Do not fit a correction or tune Huber/prior thresholds from this H-only audit.

Next candidate action: replay the same frozen diagnostics on U/LAX using their
existing raw recipes and floor-only invariance baselines, before deciding on
a new factor family. Both are already development routes, not heldout tests.
The .782/LB goal remains active and unmet; this phase verifies diagnostics,
not improved positioning performance.
