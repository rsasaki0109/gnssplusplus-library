# Phase563: frozen raw-only rotation-rate experiment

Phase562 native build session 52901 completed successfully (exit 0).
Negative CLI smoke using the historical H raw command with Phase213 removed
and rotation-rate enabled returned 2 with the expected scope error, before
processing data. Standalone algebra/factor tests remain 5/5; full CTest unrun.

`scripts/run_phase563_rotation_rate.py freeze` completed. Manifest:
`output/smartphone-r5/phase563-rotation-rate-v1/manifest.json`.
Pins native src/include/app C++ files, build-list files, runner, binary and
three raw inputs. Baseline/candidate commands fixed together before scoring.
No truth reads. Historical manifest is command/input metadata only; saved
position output is not provided to inference. Baseline output SHA is used
only for post-run invariance checking. Candidate differs only in output
destinations and the rotation-rate flag; no tunable parameters.

Baseline launched: tool session 81892, native PID 3975438. Completion is not
yet verified. Poll that session/process rather than launching another run.
Candidate is not started, and runner rejects it unless baseline completes
successfully with the historical byte-identical output and unchanged pins.
After both runs, verify graph/row counts and actual enabled summary metadata
before any separately frozen development-accuracy evaluation.

This is an H development replay, not held-out or leaderboard evidence.

## Baseline completed; candidate launched

Baseline session 81892 completed exit 0 in 194.5893999011023 seconds.
Output SHA matched `4a0c4ef8822c72aa9134368cd57fe769817895e63b5dbe925783a9c8d091fa8e`;
all frozen pins revalidated. `verify_phase563_rotation_rate.py baseline`
passed exact historical epochs/graph/GNSS-first summary equality, with
66,685 Doppler factors in both stages and no epoch-counter changes.
This establishes disabled-path invariance for H, not general regression
coverage. Verifier was added after inference freeze and does not modify
the pinned solver/runner; pin it in the separate accuracy freeze if used.

Fixed candidate launched only after that gate: session 10547, native PID
3978452. Candidate completion/structural verification remain pending.
No truth or accuracy reads performed in this phase.
