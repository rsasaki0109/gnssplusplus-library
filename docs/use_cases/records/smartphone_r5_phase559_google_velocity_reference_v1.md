# Phase559 — independent Google velocity implementation

Inspected Google gps-measurement-tools SatellitePositionCalculator.java:
https://raw.githubusercontent.com/google/gps-measurement-tools/master/GNSSLogger/pseudorange/src/main/java/com/google/location/lbs/gnss/gps/pseudorange/SatellitePositionCalculator.java

The implementation iterates range/range-rate and includes propagation range
in ascending-node angle. Its ascending-node rate also depends on range-rate/c
(lines 218–222). Therefore it supplies independent implementation evidence
for differentiating the propagation rotation, beyond rotating velocity alone.
The wrapper sets receiver velocity to zero, and this observation does not
validate our full implicit transmit-time or receiver-clock-tag denominators.
The reviewed URL tracks master, not an immutable experiment dependency.

Next pin an immutable revision and derive a small equivalent rotation-rate
helper with a stationary-receiver reference check, then moving-receiver
synthetic checks. Do not claim Google parity or replace the native model until
those scopes and clock conventions match. No downloaded positioning data,
MAT access, runtime change, truth read or accuracy evaluation here.
