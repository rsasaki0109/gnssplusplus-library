# Phase231: source metre-domain TDCP sigma helper

Added sourceTdcpSigmaMeters(signal, SNR, band percentiles), with no wavelength
argument. It returns the existing source SNR/type L formula directly in the
metre-valued resL residual domain and rejects nonpositive/nonfinite SNR,
band reference, signal factor or result. The old Phase117 helper is unchanged
for explicit migration; its historical source-unit explanation is superseded
by Phase230, not validated by preserving its regression tests.

Added controlled GPS L1 reference-SNR and +/-20 dB cases, GLO L1, GAL E1,
GPS L5, NaN/zero SNR and missing reference cases; contrasted the historical
wavelength-scaled helper. These establish arithmetic/unit behavior, not full
source percentile-population parity. Unsupported signals and all remaining
edge cases are not exhaustively tested by the new case.

Build session 19030 completed exit 0 for native app and gnss_run_tests.
UpstreamSourceTdcpMetersTest plus UpstreamObservablePreprocessingTest passed
10/10 on the fresh binary. git diff --check passed. No full CTest claim.

The helper is not yet connected to production factor construction. No raw
solve, truth/candidate payload read, MAT input or evaluation occurred. Current
H best remains Phase222/223 1.2680574850266653 m. Next integrate via a separate
default-off selector with previous-endpoint sigma, diagnostics and synthetic
factor-construction tests, preserving fixed-sigma and historical paths.
