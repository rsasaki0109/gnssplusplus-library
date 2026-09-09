# Phase224: source TDCP robustness versus current ECEF-D lane

Primary agent; source and historical metadata inspection only. No raw solve,
truth/candidate payload reads, MAT access, or accuracy calculation.

Current source inspection confirms parameters.m lines 120-125 select TDCP
Huber k=0.2 for Street/Mix and 0.5 otherwise. fgo_gnss_imu.m lines 307-316
uses resL differences, the earlier epoch observation sigma and this kernel;
Pixel5 selects XXCC rather than the listed Samsung XXDD branches.

This mismatch is not newly discovered: the Phase184 parity record already
documents and implements that mapping. However, the current CLI at
apps/native/gnss_fgo_imu_no_base.cpp lines 835-839 explicitly rejects Phase184
with Phase171 ECEF-D staging. Phase222 therefore still uses k=4.0, as its
frozen manifest states. Backend lines 55-58 resolve the selected threshold
and the ordinary TDCP insertion at line 2137 consumes it.

Historical Phase185/186 tested source k in the older no-D staging lane,
not the current combined main Doppler/motion graph. Its reported P50
0.7415623472454844 m and P95 2.078006117385388 m do not establish the effect
in the current lane. Do not repeat or relabel that historical evaluation.

Next bounded hypothesis: permit the already-implemented source Type mapping
with ECEF-D staging and the current main graph, after testing the combination.
The existing Phase184 selector applies to both staging and main; consequently
this experiment intentionally changes initialization as well as main TDCP
robustness. It must not claim unchanged GNSS-first estimates or seeds.
Use the source-derived k=0.2, not a truth-guided sweep. Preserve raw inputs,
noise sigma, correction equations and timing. First inspect all backend/CLI
guards and Phase184 tests for this combination; do not simply delete a guard
and infer compatibility. No production change was made in this audit.

The remaining resL correction and observation-sigma parity questions are
unresolved by this limited audit. Avoid claiming complete measurement-model
parity. Current H best remains Phase222/223 1.2680574850266653 m; H is
repeated development data and the 0.782/leaderboard objective remains unmet.
