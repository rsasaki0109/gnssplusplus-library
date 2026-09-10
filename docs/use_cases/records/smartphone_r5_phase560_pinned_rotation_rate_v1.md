# Phase560 — immutable Google reference and isolated rate iteration

Resolved google/gps-measurement-tools master with git ls-remote to
ab1aebb3bd68b13e29b897a7f1e261b6c14a5098 and fetched the source at that revision:
https://raw.githubusercontent.com/google/gps-measurement-tools/ab1aebb3bd68b13e29b897a7f1e261b6c14a5098/GNSSLogger/pseudorange/src/main/java/com/google/location/lbs/gnss/gps/pseudorange/SatellitePositionCalculator.java
Confirmed the ascending-node rate includes the range-rate/c term there.

Added an isolated synthetic rate-iteration control: d=A+k*d gives d=A/(1-k),
where k=(omega/c)*LOS dot (rotated_y,-rotated_x,0). Five iterations match
closed form to 1e-10 m/s for stationary and +/-30 m/s receiver cases. The
stationary case matches the reference wrapper assumption; moving cases are
an algebraic extension. No claim of full Java ephemeris parity.

This differs from Phase557's complete implicit light-time denominator, which
also contains the satellite transmit-time derivative. Keep these hypotheses
separate. No inference selector, correction or new accuracy run introduced.
