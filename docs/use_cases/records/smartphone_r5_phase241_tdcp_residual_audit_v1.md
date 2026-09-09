# Phase241: TDCP residual atmosphere difference

Source/native code inspection only; no payload scoring or native rerun.

Source `MatRTKLIB/+gt/Gobs.m:1155` defines resL = L*lambda - (range -
satellite_clock). Its separate resLc includes atmosphere. The Pixel5
graph uses differences of resL (`gsdc2023/fgo_gnss_imu.m:307`), not resLc.

Native `src/algorithms/fgo_problems.cpp:740` currently builds corrected
carrier = raw_carrier + satellite_clock - troposphere + ionosphere.
With Phase120 off this is also the TDCP observable. The source-clock
factor's error is delta_range + delta_C[0] - delta_carrier
(`fgo_gtsam_internal.hpp`, TimeDifferencedCarrierFactorSourceClockArm).
Thus the current native observation includes a temporal atmospheric term
absent from source resL. It is a real equation difference, not evidence
that removing it necessarily improves accuracy.

An existing `ordinaryTdcpCarrierMeters(..., true)` helper supplies raw
carrier + satellite_clock. Historical Phase120 exposes this but is tied
to Phase118 and an old sealed recipe; source metre sigma explicitly
rejects that composition. Do not reinterpret frozen historical recipes.

## Next implementation contract

Add an independent default-off source-resL observable selector for the
current raw Pixel5 Phase171 lane, compatible with source metre sigma,
main Doppler and motion. Preserve historical selectors and reject mixed
old/new atmosphere switches. Do not change P/D, sigma, robust k, raw
admission, geometric factor type, time mapping, or stop conditions.

Reuse the pure existing helper. Keep code-phase jump admission on its
existing corrected-carrier quantity so this experiment isolates the
observable rather than changing pair selection. Test nonzero temporal
iono/trop changes, exact expected delta difference, preserved factor
keys/counts, and both GNSS-first/main propagation. Add explicit summary
telemetry and CLI invalid-combination tests before a frozen raw H run.

Source uses previous-LOS affine geometry while current native uses
nonlinear endpoint ranges; that is a separate difference. Removing the
atmosphere term alone must not be described as full source factor parity.
Keep k=4 for this isolated comparison against Phase234/235. No truth-based
threshold sweep; broader route verification remains necessary.
