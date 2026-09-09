# Phase556 — active Doppler scope and derivative control

App Phase171 main config sets generic corrected Doppler OFF, but this does not
disable Phase213's separate main family. GNSS-first ECEF staging sets corrected
Doppler ON; remapDopplerFactors copies its rows into main Phase213. Backend
rotates their LOS into the IMU ENU frame and inserts receiver velocity/drift
factors. Thus both stages consume the staged corrected rows, despite the main
generic flag. Distinguish stored receiver-minus-satellite LOS from the helper's
receiver-to-satellite LOS when checking the factor's positive dot product.

Added/registered test_doppler_rotation_derivative.cpp. Standalone
/tmp/phase556_derivative passes: for one synthetic moving satellite/receiver,
the derivative of helper-produced rotated range differs from rotated velocity
dot LOS by -0.00206414 m/s. Analytic rotation-angle derivative reconciles it
with central differences to 7.34e-8 m/s. This is a consistency test of that
explicit geometric travel-time rotation, not the full implicit transmit-time
ephemeris/clock equation. No runtime patch or accuracy claim follows from it.

Before changing active D rows, derive the complete intended transmit-time
model including receiver velocity dependence and verify on a geometric grid.
Do not inject the single synthetic residual as a constant correction, or
assume it explains meter-level positioning error. No new truth read.
