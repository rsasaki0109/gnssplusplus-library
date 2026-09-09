# Phase462 — seed clock model versus centered admission

Read-only equation and Phase460 metadata audit. No native solve, truth read,
saved positioning input, scoring or production change.

## Verified equation difference

Native SPP (`src/algorithms/spp.cpp`) predicts range + reference clock +
estimated non-reference system bias. `raw_p_seed.cpp` exports the reference
clock as an absolute range clock and other `ClockGroupBias.bias_m` values as
relative inter-system biases; these fields must not be treated alike.

The Phase165 app copies same-run seed position and the reference scalar clock
into builder inputs, disables another SPP solve, and only attaches the typed
retained seed vector after `buildPseudorangeProblem`. The adapter explicitly
does not map constellation-level SPP biases into frequency-specific C7 slots.
This is documented behavior, not evidence of a missing accidental assignment.

The builder's pre-mask residual is corrected P - seed range - scalar clock.
It then removes one whole-route median per system/band, not the epoch's SPP
inter-system bias. Thus a time-varying system offset can remain in a centered
mask residual even when SPP has fit that offset. Geometry, atmosphere, row
selection and frequency definitions also differ; equations alone do not
identify which contribution dominates the real rejected observations.

## Existing metadata evidence

Phase460 estimated relative Galileo biases in all 1466 epochs: minimum
-478.09860141795332 m, maximum -417.18553241144753 m. The **upper order-statistic
median** from the audit's jq calculation is -438.78557386795177 m. This is not
the average of the two central values and, critically, not the builder's
row-weighted system/band residual median.

At epoch 851 Galileo bias is the route minimum, about -39.31 m relative to
that descriptive median, with four used Galileo rows. At epoch 852 it is
-451.0427851886468 m (about -12.26 m), also with four Galileo rows. GLONASS at
851/852 has only one used row per epoch, so its free bias has no within-group
redundancy to validate that observation independently. Do not use these
numbers as direct correction candidates or assert they explain all rejected P.

## Decision boundary

This is a plausible mask/initializer interaction deserving an identity-matched
test, not a demonstrated bug or accuracy improvement. Blindly subtracting a
same-row fitted bias can hide multipath and guarantee a misleading residual
improvement, especially for single-row clock groups. Any prospective change
must protect against that self-fit effect (independent temporal or held-out
observation support), preserve C7 frequency semantics, and retain existing
quality masks. No blanket re-admission or same-route threshold sweep.

Next implement a synthetic multi-constellation control with known changing
system bias and a separate contaminated code row. Verify that a proposed
clock-aware admission rule can distinguish common bias motion from individual
code corruption without fitting and testing the same observation unchecked.
If it cannot, keep the existing rule. Existing development-route results and
the global performance/leaderboard goal remain unchanged and unmet.
