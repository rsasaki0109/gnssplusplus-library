# Phase230: resolved source TDCP sigma-unit mismatch

Primary agent; local source inspection only, no truth/candidate payload,
MAT access, raw solve or score.

Evidence chain:

1. MatRTKLIB/+gt/Gobs.m:1155 constructs resL as L*lam-(rng-dts)
   and explicitly documents carrier phase residuals in metres.
2. gsdc2023/fgo_gnss_imu.m:307-316 differences resL, passes
   obserr.(f).L(i,j) directly to noiseModel.Diagonal.Sigmas and uses XXCC
   for Pixel5. obserrmodel.m has no wavelength operation in its sigma formula.
3. gtsam_gnss/src/TDCPFactor_XXCC.h documents the TDCP measurement as metres
   and subtracts it from LOS position displacement plus clock displacement;
   evaluateError contains no wavelength conversion.
4. Native observable_upstream_preprocessing.hpp:212 officialTdcpSigmaMeters
   instead labels the SNR-derived result sigma_cycles and multiplies by
   wavelength_m. fgo_problems.cpp:1614 uses that result for Phase117 TDCP sigma.

Conclusion: the inspected source TDCP likelihood consumes the SNR-derived
sigma directly in its metre-valued residual domain. The native Phase117
extra wavelength multiplication is not source parity. Its justification
confuses raw L storage in cycles with the later resL residual in metres.
Historical tests asserting the multiplication cannot establish upstream
parity. This finding does not imply the current fixed-sigma best is affected:
Phase222 has Phase117 OFF and fixed sigma 0.03 m.

At SNR equal to its band reference, GPS L1 source sigma is
(1/400)*0.8 = 0.002 m, whereas the existing helper returns 0.002*wavelength_m.
This is a deterministic unit mismatch, not a result of H truth inspection.

Next implement a clearly distinguished metre-domain source sigma helper with
controlled tests (reference SNR, +/-20 dB, signal types, invalid inputs),
then integrate it through an explicit default-off lane compatible with the
current main graph. Do not silently reinterpret frozen Phase117 artifacts;
preserve historical records and label the correction. SNR population parity
and resL atmospheric correction remain separate unresolved questions.
