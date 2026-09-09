# Phase571: raw-only H/A temporal residual monitor replays

Phase570 build session 26554 completed exit 0. No solver factors or weights
changed; only app post-fit diagnostic accumulation/logging added. Two focused
tests passed previously. Full CTest not run.

Runner `scripts/run_phase571_adjacent_monitor.py` freezes each native argv,
current binary/source/raw hashes and historical output/summary identity before
launch. Reads old solution bytes solely to hash for post-run comparison, never
parses or sends them to inference. Allowed changed preexisting source pins
are app and test CMake list only; new diagnostic header/test/runner also pinned.
No truth or new accuracy read. H and A are existing development routes.

For each completed run require exact solution-byte identity and exact
epochs/graph/GNSS-first/TDCP summary equality. Require one monitor line and
unchanged pins. These gates do not establish covariance-model correctness;
only that logging preserved this replay and produced an aggregate statistic.

H launched: session 67031, PID 3996373. A launched separately with same
binary. At record creation both completion results are pending; consult live
tool sessions/processes and per-route started/completed metadata before any
restart. No frozen candidate/noise model is being optimized here.
