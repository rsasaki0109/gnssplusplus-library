# Phase557 — implicit light-time control and Android observable contract

Extended test_doppler_rotation_derivative.cpp with nine synthetic constant
satellite/receiver velocity combinations. Solve rho=|R(omega*rho/c)*s(t-rho/c)-r(t)|
iteratively. With u the rotated LOS, A=u dot R*vs and B=u dot dR/dtau*s,
rho_dot=(A-u dot vr)/(1+(A-B)/c). Central differences agree within 2e-6 m/s;
maximum difference from simple rotated relative velocity projection is
0.0379953 m/s in this grid. /tmp/phase557_derivative passed 2/2 tests.
No atmospheric/clock terms in this control; not an Android correction proof.

Checked official Android GnssMeasurement API:
https://developer.android.com/reference/android/location/GnssMeasurement.html#getPseudorangeRateMetersPerSecond()
Rate is at the measurement timestamp, uncorrected for receiver/satellite clock
frequency errors, with positive values for increasing separation and negative
proportionality to Doppler shift. This supports native rate/Doppler sign and
explicit clock handling. It does not fully specify vendor light-time modelling
or establish that this toy derivative should replace the active native rows.

Next derive the clock-inclusive observable and compare with a pinned physical
measurement reference before proposing a behavior-changing correction. Do not
apply the grid maximum as a constant bias or tune it against development truth.
No runtime modification, raw rerun or truth read in this step.
