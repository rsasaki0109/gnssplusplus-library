# Phase555 — Doppler signs and frame boundary inspection

Inspected fgo_problems.cpp preparation, doppler_contract.hpp and backend
undifferenced Doppler insertion. Raw RINEX Doppler is converted to range rate;
receiverOnlyResidual subtracts modeled satellite range rate and adds c times
satellite clock drift. Receiver prediction is -LOS dot velocity + receiver
clock drift. The Phase171 ECEF GNSS-first branch passes ECEF LOS directly;
the Pose3/IMU branch converts it at the ENU velocity-frame boundary.

The optional rotated-state helper rotates satellite position and velocity
and adds no explicit Sagnac term. The legacy branch instead uses unrotated
LOS plus the first-order satellite Sagnac-rate addend. These are separate
branches, not both additions on one observation. Exact light-time derivative
consistency and actual recipe branch selection need explicit checking before
any proposed change; do not infer active options merely from helper presence.

No evident sign reversal in the inspected receiver/satellite clock algebra.
No runtime edit, performance claim, raw rerun or truth read in this step.
