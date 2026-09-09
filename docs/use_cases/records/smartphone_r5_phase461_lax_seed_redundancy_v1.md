# Phase461 — actual LAX raw-P seed redundancy

Phase459 failed CLI validation (exit 2) before inference: the first-native-
output-epoch switch requires Phase171 FGO and is not a prep-stage switch.
Its output directory and failure metadata are retained. Phase460 is a new
explicitly corrected invocation removing only that switch, not an automatic
retry. Both scripts pin the existing binary and raw inputs before launch.

Phase460 used the existing native Phase149 preparatory mode with Phase157
bootstrap, all epochs, Android raw-clock-only/UTC keys and broadcast nav.
The seed config matches the Phase165 app seed solve (zero elevation/SNR masks,
same bootstrap and SPP defaults). No collect-all independent-solve mode was
enabled. Prep emits structural metadata, not coordinate/velocity series;
there is no output solution CSV, no FGO or IMU inference, and no truth access.
Manifest, logs and terminal metadata are in
`output/smartphone-r5/phase460-lax-seed-audit-v1/`.

Native exit 0; all 1466 input epochs reported. Selected indices are checked
against their raw source indices rather than relying on FGO output rows.

| Input/raw epoch | Used P rows | Degrees of freedom | GDOP | Residual RMS m | Max absolute residual m |
|---|---:|---:|---:|---:|---:|
| 850 | 11 | 6 | 1.5408 | 13.7613 | 25.6342 |
| 851 | 10 | 4 | 1.5004 | 13.5950 | 35.0020 |
| 852 | 12 | 6 | 2.2570 | 16.5314 | 33.9898 |
| 853 | 15 | 9 | 1.3540 | 26.0105 | 54.8600 |

All four seeds are accepted. In particular, epochs 851/852 have four/six
residual degrees of freedom, not zero. The exactly determined synthetic
failure from Phase458 is therefore not an explanation for these raw seed
epochs. Their downstream final P counts (1/6) must not be confused with the
initial raw-P solver's row counts (10/12).

Residual RMS is not position accuracy, and GDOP does not establish unbiased
measurements. No promotion, robust-weight switch or threshold change follows.
Next distinguish the seed solver's system-clock residual model from the
builder's scalar seed clock plus whole-route system/band median. A mismatch
there could affect mask admission even with redundant seed geometry; it is
an untested hypothesis, not an established fault. Inspect existing clock-group
metadata and exact equations before adding another experiment.

The native raw-only performance and leaderboard goals remain unmet.
