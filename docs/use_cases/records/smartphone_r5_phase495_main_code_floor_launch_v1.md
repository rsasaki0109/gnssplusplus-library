# Phase495 — main-only code floor raw H experiment launched

Build completed successfully. scripts/run_phase495_code_monitor.py pins the
current sources/binary and the Phase493 raw inputs before a single native
invocation. It derives the Phase493 baseline argv and adds only
--native-main-code-uncertainty-floor (plus new output destinations).
NHC and builder uncertainty floor remain off; numerical lambda floor stays on.

Two launcher syntax failures were detected before Python executed any code;
they created no native process or experiment output directory. After repair,
native PID 3770993 launched once, exec session 82771. At this record's writing
the same handle remains running; do not restart based on this record alone.
Poll that handle or verify process state first.

Artifacts under output/smartphone-r5/phase495-h-main-code-floor-v1/mtv-h/:
manifest.json and started.json; completed.json will be written on termination.
Expected main sigma changes 101916. No candidate result or convergence claim
yet. Added scripts/verify_phase495_main_floor.py to check output hash, current
pins, GNSS-first equality, graph aggregate counts and solver-stage contracts
after completion. Not yet run against a completed candidate. No truth access.
