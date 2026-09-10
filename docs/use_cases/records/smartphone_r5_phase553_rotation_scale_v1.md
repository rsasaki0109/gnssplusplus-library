# Phase553 — frozen rotation scale control

Registered test_frozen_earth_rotation.cpp invokes the actual native
earthRotationCorrected helper. Standalone /tmp/phase553_rotation passes.
Across 24 azimuths, 8 elevations, 3 seed-offset axes and both signs at a
synthetic equatorial receiver with 22000 km slant range, maximum range errors:
1 m offset -> 1.51247e-6 m; 10 m -> 1.51023e-5 m;
100 m -> 0.000151034 m; 1000 m -> 0.00151034 m.

Each case checks the analytical bound horizontal_satellite_radius * omega/c
* seed_offset plus floating-point allowance. This compares frozen versus
recomputed geometric-travel-time rotations, not a full light-time solution.
No claim that real seed offsets are bounded by these synthetic inputs or that
measurement error maps directly to position error. This scale does not justify
prioritizing another costly candidate solely for frozen-angle correction.
No runtime behavior changed; full CTest remains unrun.
