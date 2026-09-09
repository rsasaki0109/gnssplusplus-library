# Phase574: raw ADR uncertainties and endpoint covariance primitive

Inspected native Android loader: ADR metres are consumed; the optional
AccumulatedDeltaRangeUncertaintyMeters field is not currently retained in
Observation. ReceivedSvTimeUncertainty is a different existing field and
must not be reused as carrier uncertainty.

Read-only CSV inventory, raw fields only. Require ADR state bit1 set, neither
reset/slip bit2/4 set, finite nonzero ADR; count finite positive uncertainty.
These are NOT native final admitted factor counts or proof of calibration:

| Route | Raw ADR rows | Positive uncertainty rows | Median uncertainty m |
|---|---:|---:|---:|
| H | 78778 | 78778 | 0.0028421844273806814 |
| U | 20552 | 20552 | 0.0032108332879374254 |
| LAX | 29857 | 29857 | 0.002930522447223262 |
| A | 56156 | 56156 | 0.0025499350854412276 |

Inputs are existing Phase37 H/U/LAX and Phase25 A raw CSVs. No derived
positions, truth, navigation products, or accuracy evaluations read.

Implemented diagnostic/research `tdcp_endpoint_covariance.hpp` for one
continuous same-satellite/signal arc: C=D diag(sigma_endpoint^2) D'. Diagonal
is sum of endpoint variances, adjacent off-diagonal is negative shared
variance. Coefficients are not fitted from Phase571 post-fit correlations.
This assumes independent endpoint errors; metadata alone does not prove it.
No inference integration, existing factor sigma/loss unchanged.

Tests cover direct matrix construction and positive definiteness, telescoping
sum retaining only outer endpoint noise, equal-sigma correlation -1/2, and
missing/invalid/overflow/underflow uncertainty rejection. Dense matrix is a
small-arc reference primitive, not a scalable whole-route whitening design.

Next implementation requirements: retain optional raw ADR uncertainty without
changing admission; native loader tests for missing/zero/nonfinite values;
explicit arc/gap/slip identity; robust-loss semantics (correlated Gaussian
whitening is not automatically equivalent to current scalar Huber factors).
Freeze a candidate only after those contracts are implemented. Full CTest
unrun; .782/LB goal still unmet.
