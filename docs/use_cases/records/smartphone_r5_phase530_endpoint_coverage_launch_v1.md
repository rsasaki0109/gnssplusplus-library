# Phase530 — raw endpoint coverage and invariance replay

Phase529 dependency rebuild session 9037 completed exit 0. During that build,
before app compilation, added a read-only app monitor after final main row
selection: count TDCP edges with two positive finite coefficients, invalid
edges, coefficient extrema, and valid edges lacking at least one exact
(retained epoch, satellite, signal) code key. No time-nearest lookup or model
fallback. Missing-code counts do not label bad carriers.

Added run_phase530_ionosphere_monitor.py based on Phase522 raw H argv/pins.
Explicitly accounts for app, FGO metadata/builder, helper and prior test changes;
additionally pins fgo_internal.hpp and residual_ionosphere_contract.hpp (the
former was not in the inherited pin dictionary). Syntax/diff checks passed.
Old TDCP vector member spelling was corrected before app compilation.

Raw H launched native PID 3878588, runner session 47812, confirmed live.
Directory output/smartphone-r5/phase530-h-ionosphere-monitor-v1/mtv-h/.
Runner freezes input/source/binary hashes and checks unchanged output hash,
one endpoint monitor and zero invalid. It does not yet validate full endpoint
count/graph contracts: do that after completion. At record creation raw-run
completion and invariance are UNPROVEN. Poll the same process/session.

This adds metadata only; code/TDCP residual-ionosphere wrappers are still not
enabled. No truth/MAT/saved positioning input or accuracy evaluation. Goal
remains active and unmet. Disk is tight; avoid unrelated downloads/debug builds.
