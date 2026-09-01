# Phase 64: base preflight policy recovery

Phase64 re-evaluated the sealed Phase63 base-RINEX evidence without reopening
the archive or any base file.  The source contract is the official
`taroz/gsdc2023` repository at commit
`29923f9f370f09ebc00f96d8cca375007a18e7d5`: `settings_train.csv` `Base1`
selects the declared `*_rnx2.obs` member, while `correct_pseudorange.m`
selects its 151-sample (1 Hz) or 11-sample (15 s) path from observed
`obsb.dt`, not a header `INTERVAL`.  `MARKER NAME` is informational only.

The v4 scorer adapted the sealed Phase63 record's top-level
`routes[route]` summaries to the gate helper's nested shape.  It pinned the
expected settings SHA-256
`3e6ae65388b2809088b16732b87744e673f860c24a1fe0f709ef903a87397f39`, the
archive SHA-256, all four materialized base hashes/bytes, finite
`APPROX POSITION XYZ`, and observed route intervals (1, 1, 15, 1 seconds).
All four route gates and aggregate gates passed.  The blank station IDs,
missing header intervals, and payload RINEX version `3.03` versus the legacy
settings `V2` remain informational under this source-defined policy; no
identity or interval was inferred from those fields.

This is a preflight authorization only.  No accuracy truth, solver, raw
handset, navigation, MAT, WLS/Sv coordinate, validation/holdout, or
Kaggle/token input was read.  Phase64 read the sealed Phase63 result once in
the v4 process; including the disclosed v1/v2/v3 failed attempts, the
cumulative result-JSON read count is four.  Archive opens and base-file
rereads are zero.  The Phase63 and Phase62 records remain immutable.

The v1/v2/v3 evaluator failures are retained rather than hidden: v1 used an
obsolete freeze key, v2 expected nested route summaries, and v3 used an
absent settings-digest key.  They did not invalidate the physical evidence;
v4 is the sealed schema/policy erratum.  The `0.782` target was not
evaluated.  A separate implementation freeze is required before any native
correction or accuracy read.

See the [freeze](records/smartphone_r5_phase64_base_preflight_policy_recovery_freeze_v1.json),
[v4 manifest](records/smartphone_r5_phase64_base_preflight_policy_recovery_manifest_v4.json),
[sealed result](records/smartphone_r5_phase64_base_preflight_policy_recovery_result_v4.json),
and [v4 evaluator](../../apps/commands/benchmarks/gnss_smartphone_phase64_base_preflight_policy_recovery_v4.py).
