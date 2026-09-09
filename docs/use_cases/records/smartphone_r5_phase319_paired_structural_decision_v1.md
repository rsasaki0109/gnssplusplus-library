# Phase319: full paired GNSS/IMU native run

The frozen Phase319 run completed once (exit 0, 263.255 seconds). Both
GNSS-first and main converged with finite costs, complete termination traces,
valid configurations, and no fallback. Main used 38 outer iterations and
GNSS-first 45. The in-memory metre-clock handoff aligned all 3140 states.

The base model read 3500 epochs and built 38 correction streams. Of 101847
adopted P rows, 81394 were corrected exactly once; 19163 lacked streams and
1290 failed correction lookup. Count conservation passed. Main graph has
231862 factors and 15700 values, including 3139 IMU intervals, 3139 motion
factors, 66685 main Doppler factors, and 69270 built TDCP factors. The summary's
internal 3140 output states are distinct from published candidate coverage:
3139 exact UTC rows, no interpolation, edge hold, or unresolved epochs.

Candidate frozen in the structural result: 251174 bytes, SHA-256
`f74ef1987a7df192e296acc8b5934087100d5dc9a46fedf45e63cc7564538954`.
The candidate was read for hash and line count only, not reused for inference.
No truth was opened and no accuracy score has been computed. Successful
optimization does not establish better accuracy or source numerical parity.

Provenance caveat: the reused base report field `phase126_source_complete`
describes its equation mode, not historical Phase126 CLI admission. The
historical compound selector remains off; `native_paired_epoch_states` is on.
The raw header station reference and native orbit/rotation limitations remain.

Next freeze a one-shot development evaluator against this exact candidate and
the existing sealed H truth metadata, using the established exact-UTC metric.
Do not rerun the solver, tune against its score, or label H a heldout/LB test.
The operational baseline remains 1.0769392017393964 m until evidence supports
a change. The 0.782-class and leaderboard objectives remain unachieved.
