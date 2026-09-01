# Phase73 source-exact pseudorange miss mask

Phase73 freezes one source-supported, non-overlapping candidate before any new
raw read. The official `taroz/gsdc2023` implementation first computes
`pc=interp1(...,"linear")`; MATLAB returns `NaN` outside the base stream's
domain. It subtracts `pc` from rover `resPc`, and `fgo_gnss_imu.m` adds a
pseudorange factor only under `~isnan(resPc)`. Doppler and TDCP are guarded by
separate conditions.

The native base-compensation path currently leaves an adopted pseudorange
factor unchanged when `correctionAt()` cannot produce an exact finite,
in-domain correction. Phase73 therefore freezes a new opt-in
`--native-base-pseudorange-source-miss-mask`, requiring the existing base
compensation flag and exact Phase64 base member. It retains and subtracts a
correction only for finite `pc`; missing-stream, out-of-domain, and nonfinite
correction rows are omitted from the undifferenced pseudorange factor set.
No new rows, extrapolation, endpoint hold, nearest-time fill, cross-signal
matching, TDCP/Doppler/IMU change, SPP change, or additional-frequency-band
option is included.

The frozen telemetry must reconcile original adopted pseudorange rows with
retained finite-`pc` rows and mutually exclusive drop reasons. It must report
`retained_finite_pc_fraction=1.0`, retained/original separately, and prove that
the retained count equals inserted undifferenced pseudorange factors. Exact
raw/base/navigation hashes, prediction-domain key coverage, candidate repeat
identity, and Phase43 flag-off identity remain hard structural gates. The
retained/original fraction is descriptive; it is not relabeled as prediction
coverage or truth-row coverage, and no earlier coverage gate is relaxed.

The source pins are `correct_pseudorange.m` SHA-256
`b0536ccff478b0aff253448ffb7a203c715b8064dd8dc85898e38f1f05d0441e` and
`fgo_gnss_imu.m` SHA-256
`c70090ccb8b27fc8ac7fd2929e2f995a14cfe7f089bb1fef067370e22051c3e3`, both at
official commit `29923f9f370f09ebc00f96d8cca375007a18e7d5`. Phase72's valid
structural no-go remains immutable: exact matched/all was `0.7804352754477443`
and finite/matched was `0.965992322855068`; the additional-band option is not
combined here. Phase43 remains champion, and this source-only freeze makes no
accuracy or `0.782` claim.

The machine-readable freeze is
[`smartphone_r5_phase73_source_exact_pseudorange_miss_mask_freeze_v1.json`](records/smartphone_r5_phase73_source_exact_pseudorange_miss_mask_freeze_v1.json).
At freeze time all raw, base, navigation, truth, MAT, archive, and solver
read counts are zero.
