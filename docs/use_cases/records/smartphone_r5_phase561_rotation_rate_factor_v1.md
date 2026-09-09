# Phase561: rotation-rate-only ECEF Doppler research factor

Status: implemented and standalone-tested; not connected to inference.
No raw route run, accuracy evaluation, leaderboard submission, or full CTest.

The existing raw-P ECEF Doppler transfer checks LOS norm within 1e-6 of
one. The new geometric velocity Jacobian is -u/(1-k), not a unit LOS.
Passing it through existing LOS contracts is therefore inappropriate.

Added `doppler_rotation_rate_factor.hpp`, a distinct ECEF velocity/scalar
clock-drift GTSAM factor. The model is:

    residual = (-u/(1-k)).dot(receiver_velocity) + receiver_clock_drift
               - (observed_rate - satellite_projection/(1-k) + satellite_clock_drift)

All rates and clock drifts above are m/s. Original noise is retained;
receiver clock Jacobian remains one. No normalization or denominator
rescaling of the measurement/noise. This is only the rotation-rate model
from Phase560, not full implicit transmit-time or receiver-clock-tag physics.
Constructor rejects invalid geometry, observations, noise dimension, or
colliding keys; evaluation rejects invalid state dimension/nonfinite states.

Verification: `/tmp/phase561_rotation_factor`, built from
`test_doppler_rotation_rate_factor.cpp` and `test_doppler_rotation_derivative.cpp`,
5/5 passed. The new test checks finite-difference velocity Jacobians,
non-unit coefficient preservation, satellite/receiver clock sign for three
receiver drifts, original noise pointer, whitened factor cost, invalid clock
dimension. Registered in the GTSAM CMake test block. Initial standalone build
needed an explicit NoiseModelFactorN include and local-library rpath-link;
both resolved before the passing run.

Next: explicit research selector and same-run graph integration, retaining
unit-LOS provenance separately. Main ENU velocity requires rotation of the
coefficient without normalization; this ECEF factor must not receive ENU
states. Freeze a raw-only experiment before accuracy reads. No evidence yet
that this small physical correction improves smartphone positioning.
