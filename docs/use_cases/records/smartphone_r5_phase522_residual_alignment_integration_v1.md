# Phase522 — actual-factor residual alignment diagnostic

Retained the actual initial code factor residual in same-run diagnostic rows.
Added IRLS-weighted joint nuisance projection of coefficient and residual,
reporting median normalized inner product and adjacent supported sign counts.
No fitted ionosphere correction or graph feedback. Temporal pairs require
consecutive problem indices and 0 < dt <= 1.5 seconds; no clock-discontinuity
classification is claimed by these counts. Numerical support requires positive
projected information/energy and >1e-12 of each unprojected quantity. This is
a numerical-null guard, not physical signal admission or a tuned threshold.

Signed median can hide opposing structure; sign persistence alone can also
arise from multipath, systematic errors or correlated seeds. Neither proves
an ionosphere cause or justifies integrating a physical state. The diagnostic
is initial-state only, not an optimized trajectory covariance or posterior.

Build session 91101 exited 0. Runner scripts/run_phase522_ionosphere_monitor.py
passes syntax checking; targeted diff check passed. Prelaunch source pin audit
found exactly backend, Phase521 helper and its tests changed from Phase519.
Runner freezes source/input/binary hashes, checks baseline output identity and
existing IRLS marker. New residual marker needs additional validation after
completion. Raw run launched in this turn; consult live runner handle and
output/smartphone-r5/phase522-h-ionosphere-monitor-v1/mtv-h/started.json.
Do not restart due to observation timeout. At creation completion/invariance
remain UNPROVEN. No truth, MAT or saved positioning input. Goal active.
