# Phase488 — H batch NHC raw run completed

Main build succeeded. scripts/run_phase488_batch_nhc.py launched one native
process (PID 3755964), now terminal, exit 0 in 171.67615837603807 seconds.
Manifest frozen before launch in
output/smartphone-r5/phase488-h-batch-nhc-v1/mtv-h/manifest.json; same directory
contains started/completed metadata, logs, native summary and output CSV.

Current scripts/verify_phase488_batch_nhc.py passed: both stages converged,
finite costs and complete termination traces, expected stage solvers and
lambda floor preserved. GNSS-first summary equals Phase479 exactly. Main
graph adds exactly 1627 factors (254011 total), retains all prior non-cost,
non-iteration graph aggregate fields, and takes 40 iterations. Gate counts
3139 intervals / 3138 supported / 1627 admitted match monitor-only Phase486.

Frozen candidate SHA256:
0bbc0d1b4bdc18ee29972e74ca9b5d9666b766b361b9cf8c600b762e584098eb

No accuracy evaluation or truth read yet. Factor counts and convergence do
not establish improved positioning. Next: separately freeze an evaluation
manifest for this exact candidate and use the existing exact-key H development
metric once. No threshold sweep, no heldout or leaderboard claim. Candidate
remains default-off; overall goal unmet.
