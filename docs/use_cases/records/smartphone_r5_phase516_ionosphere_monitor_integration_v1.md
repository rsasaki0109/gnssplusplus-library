# Phase516 — main-stage read-only code ionosphere information monitor

Integrated Phase515 kernel after final main code selection, before optimize.
Phase171 only. Uses stored SPP-derived ionosphere coefficients, satellite
positions already in factors, current same-run handoff receiver positions,
and existing sigma. Nuisance columns: ECEF range Jacobian plus exact C7 basis
(base slot and selected slot assigned one, GPS L1 not doubled).

Reports epochs, invalid count, median projected/total information fraction,
and median projected information per square metre. This is local Gaussian
code-only information, not robust or full-graph observability, nor an error
bound. Empty aggregates report zero and must be interpreted with epoch count.
No factors, weights, seeds or output schema intentionally changed.

Build target gnss_fgo_imu_no_base completed successfully (session 47507).
Added scripts/run_phase516_ionosphere_monitor.py. It freezes raw input/source/
binary hashes, removes Phase507 readmission flag, and requires exact output
SHA256 equality to the Phase479 floor-only baseline plus one valid monitor.
Prelaunch pin audit found exactly the app and the Phase515 helper/test changes;
those are the explicit allowed changes. No unrelated pins waived.

Raw replay launched with native PID 3836972, runner session 98914. At this record
creation the process was live; completion and output invariance remain UNPROVEN.
Artifacts: output/smartphone-r5/phase516-h-ionosphere-monitor-v1/mtv-h/.
Poll the same session/process; do not restart based on elapsed observation time.
No truth or saved positioning read for inference. No accuracy claim or goal
completion. Interpret aggregate results only after terminal status and output
invariance checks; verify graph/stage counts as well.
