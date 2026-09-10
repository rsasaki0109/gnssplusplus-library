# Phase463 — held-out satellite clock control

Added a four-case actual-native raw-P synthetic test, using the existing
GPS/Galileo broadcast fixture (four satellites per constellation). Galileo
system bias is fixed at 17 or 57 m, and the held-out Galileo PRN 1 code has
either zero or +100 m additional error. Remove the entire target satellite
from the position and clock solve before prediction, avoiding direct reuse
of its code in the nuisance clock or geometry estimate. Seven other rows
remain, including three Galileo rows. RAIM and iterative outlier detection
stay enabled, Huber stays default OFF.

Native raw-P solves on the remaining observations recovered a prediction
whose held-out residual is 0 m for both clean cases and 100 m for both
contaminated cases (within the asserted 0.01 m tolerance). The existing 20 m
L1 centered-residual predicate accepts the former and rejects the latter.
Assertions test the behavior, not just finiteness. All 29 RawPSeedTest tests
passed after adding this control; `git diff --check` passed.

Scope limits: this is noiseless, known-start, GPS/Galileo-only synthetic
geometry with atmosphere disabled, all elevations admitted, equal weights,
100 iterations and an ideal zero residual center. Other observations are
clean. No second band is present, though all target satellite rows are removed
by the test. No FGO builder rule was changed. This does not prove performance
on real phones, reliable uncertainty, resistance to contamination among the
remaining satellites, or usability for groups with only one observation.

The control supports testing satellite-held-out prediction as a diagnostic
alternative to same-row fitted-clock subtraction. It does not justify
automatic re-admission. Before a candidate: require remaining-group support,
full-rank geometry and prediction uncertainty; test noise/remaining-row
contamination and absent-group cases, and compare with the actual native
correction model. Do not map constellation ISB directly into C7 frequency
slots or assume a zero-centered corrected residual on real data.

No real-data solve, truth read, MAT, stored-position inference input or
submission occurred. Production settings and the unmet goal remain unchanged.
