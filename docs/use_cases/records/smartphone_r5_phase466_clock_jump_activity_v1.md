# Phase466 — clock-jump branch activity

Read existing summary metadata only: Phase438 H, Phase249 U, Phase455 LAX-T.
For each, clock-jump, gap and invalid-dt skip counters are zero. Main C0/D
factor counts are respectively 3139, 1101 and 1465 (one per adjacent epoch).
Thus the source/native clock-jump branch difference identified in Phase465
is inactive in these recorded baseline runs. Changing that branch cannot
explain or improve their current outputs. No new inference or scoring.

The original source fgo_gnss.m adds [Inf; zeros(6,1)] clock noise at a jump;
native nativeSourceClockC0DEdgeDecision skips the entire factor. Preserve this
as a known source-parity difference, not a demonstrated current-route defect.

A separate diagnostic concern surfaced: reported indeterminate linear solve
counts H=37, U=74, LAX=91, versus accepted outer iterations 41, 78, 95. All
three ultimately converged. These are trace-derived counts, not independently
observed exception logs: finalize() counts parsed attempts whose
system_solved_successfully is false. Verify parseAttempts and the linked
GTSAM trace format before treating them as numerical failure evidence or
changing solver settings. Do not conflate convergence with position accuracy.

Next useful check is trace-parser correctness and actual linear-solve status;
no additional clock-jump experiment or threshold sweep is warranted.
No production mutation in this phase; original goal remains active/unmet.
