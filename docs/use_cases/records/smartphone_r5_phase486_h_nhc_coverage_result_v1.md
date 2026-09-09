# Phase486 — H raw NHC coverage and diagnostic invariance

Build of gnss_fgo_imu_no_base completed successfully. Single raw H replay via
scripts/run_phase486_nhc_monitor.py completed exit 0 in 176.04756045492832 s.
The attempted `python` invocation failed at shell lookup (no native process);
`python3` launched exactly one native process, PID 3750328, now terminal.

Artifacts: output/smartphone-r5/phase486-h-nhc-monitor-v1/mtv-h/
manifest.json, started.json, completed.json, stdout.log, stderr.log,
opaque_solution_output.csv, native_summary.json. Manifest was written before
launch, pinning raw inputs, current binary and source files. Historical source
pins were checked with only the two monitor implementation TUs allowed to
change; new helper/test/launcher pins were added. No truth or scoring was read.

Aggregate monitor: 3139 intervals, 3138 IMU-supported, 1627 admitted under
speed >=2 m/s, peak bias-corrected full angular speed <=0.2 rad/s and every
endpoint/interior IMU gap <=0.05 s. First epoch is excluded. No factors added.
This measures gate coverage, not physical validity of the mounting assumption.

Output SHA256:
4a0c4ef8822c72aa9134368cd57fe769817895e63b5dbe925783a9c8d091fa8e
matches the Phase479 lambda-floor output byte-for-byte. Thus this diagnostic
did not change H output. Neither route-transfer invariance nor an NHC accuracy
improvement is established. No saved positioning input was used: the speed
handoff is produced by GNSS-first in this invocation. Stored output hash was
used only after termination to verify invariance.

Next implementation step: add an explicit default-off batch NHC candidate
using this gate and the existing NonHolonomicFactor, with frame-correct pose
and velocity states and reported inserted-factor counts. Freeze its settings
before any scoring. Approximately 52% admission is not a reason to tune the
gate against truth. Overall .782 / leaderboard objective remains unmet.
