# Phase357 — default-off baseline output identity

- Manifest: `smartphone_r5_phase357_h_baseline_replay_manifest_v1.json`
- Manifest SHA256: `0b2d5a749857ad1b004deff6ab84c450645349cb19850857a3efbe8a5c1c4146`
- Execution: primary agent, one native invocation, return code 0.
- Started: 2026-09-08T14:54:31.152363+00:00.
- Completed: 2026-09-08T14:59:08.703321+00:00 (277.55100333795417 seconds).
- Output: `output/smartphone-r5/phase357-h-baseline-replay-v1/mtv-h/opaque_solution_output.csv`.
- Output SHA256: `e1e148c41fc8b87e444bd845b4a96eb8e7a56137ec91851b5c0b5d6cba64af29`.
- Output size: 251174 bytes; 3139 data rows.

The output hash, byte count and row count exactly match the frozen Phase234
baseline identity from Phase235 metadata. No historical candidate payload or
ground-truth payload was read for this comparison. No accuracy evaluation,
Kaggle submission or retry was performed.

This confirms byte-identical output for this H-route baseline recipe with the
new diagnostic selectors disabled. It is not an all-route regression test,
the PPC real-data gate, a fresh held-out result, or evidence that the 0.782 / LB
target has been achieved. Experimental alternatives remain unpromoted.
