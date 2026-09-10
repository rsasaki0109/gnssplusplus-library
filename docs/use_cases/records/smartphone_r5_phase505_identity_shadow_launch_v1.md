# Phase505 — identity intersection raw run launched

Full dependent build session 5222 completed exit 0, including rebuilt
Observation consumers, both GTSAM backends and main CLI. Launched exactly
one Phase505 native diagnostic via run_phase505_code_monitor.py:
PID 3801727, exec session 44009. At recording time it is running.

Manifest under output/smartphone-r5/phase505-h-code-monitor-v1/mtv-h/ pins
current binary, source/header/tests and raw input hashes before launch.
Baseline argv retains lambda floor, disables NHC and both code floors.
Candidate labels/shadow inspection are diagnostic-only. No truth scoring.

Next poll existing handle, then run verify_phase505_identity_shadow.py.
That verifier checks exact baseline output hash, graph and GNSS-first
aggregate equality, stage termination contracts, one raw-candidate marker,
one shadow marker and nested candidate counts. The marker parser's synthetic
test passed earlier; verification on this live run is not complete yet.
Do not infer 426-to-807 intersection before actual output arrives.
