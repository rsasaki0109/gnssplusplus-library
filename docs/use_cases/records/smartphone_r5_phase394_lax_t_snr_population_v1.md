# Phase394 — LAX-T raw SNR population and source likelihood

Read cached `.m` source text only (`functions/obserrmodel.m` and
`fgo_gnss_imu.m:300-324`), current C++ weighting code, historical Phase238–240
audit records, and raw LAX-T GNSS timing/constellation/frequency/CN0 columns.
No MAT payload, truth, saved positioning input, solver execution or score.

Source passes `obserr.L(i,j)` directly to the TDCP difference's noise model.
It does not quadrature-combine the two endpoint sigmas. Native source-metre
mode likewise uses previous-endpoint SNR without wavelength multiplication.
Thus adding sqrt(2) or combining endpoints is a new statistical-model
hypothesis, not a demonstrated source-port correction. Temporal correlation
of carrier errors matters; no such model change is justified by this audit.

Raw LAX-T audit finds 51243 Raw rows, all surviving the source timing,
bias-uncertainty and constellation/GLONASS-SVID filters used in Phase240.
All C/N0 values are positive and finite. No duplicate excess for
(UTC, constellation, SVID, frequency band). Band classification here is
<1.3 GHz=L5, otherwise L1, not a full independent frequency mapper.
L1 has 34856 rows; L5 has 16387. Their linearly interpolated 85th percentiles
are both 39.3 dB-Hz, matching the native observable-quality summary.
Native selected rows equal 51243, leaving no extra row removal to attribute
to the invalid-raw-pseudorange boundary in this route.

This eliminates a concrete candidate explanation for LAX-T's larger L1/E1
tail: no aggregate SNR-population loss is demonstrated. It does not establish
independent `.m` converter equivalence, per-signal exact graph sigma identity,
or a calibrated error model. Preserve the current weighting settings.

Next focus on the physical residual model (especially common clock versus
frequency-dependent changes) using raw simultaneous signals; do not keep
rerunning unchanged accuracy or tuning a global sigma from development truth.
The 0.782-class/LB objective remains unmet.
