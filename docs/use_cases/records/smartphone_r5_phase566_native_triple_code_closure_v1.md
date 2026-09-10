# Phase566: native raw triple-code closure

Implemented diagnostic-only `gps_triple_code_closure.hpp`. With
g2=(f1/f2)^2, g5=(f1/f5)^2, w2=(1-g5)/(g2-1), compute
w2*(P2-P1)+(P5-P1). Coefficients are fixed by carrier frequencies before
the raw read, not fitted. This cancels common geometry/clock/troposphere
and first-order inverse-frequency-squared code delay in the ideal model.
It retains differential code delays, multipath, noise and other departures;
it cannot identify receiver versus satellite hardware contributions.

Three standalone gtests passed: cancellation over twelve geometry/ionosphere
cases, response to separate frequency biases, invalid observations.
Registered in ordinary CMake gtest sources; full CTest not run.

Native diagnostic `scripts/native_base_triple_code_audit.cpp` uses the actual
RINEXReader with source header tracking and additional frequency bands.
No navigation, station coordinate, positioning estimate, carrier ambiguity,
MAT, truth, or solver is used. It emits aggregate stream-center/MAD values
only, not corrections. No minimum stream-count or residual threshold tuned.
One H raw-base pass completed successfully:

- 3,500 epochs; C1C/C2W/C5X triples: 17,500 rows, 5 satellites.
- Median across satellites of absolute temporal stream median: 2.228671036535534 m.
- Median across satellites of within-stream MAD: 0.22513222476768302 m.

The earlier inventory required carrier availability too (17,433 C2W rows),
whereas this code-only test does not. No C2X output: native source header
selection chooses one tracking code per frequency slot (rinex.cpp around
1683), excluding C2X here. Thus this is not a W-versus-X comparison.

These are uncorrected code combinations. Known satellite group/inter-signal
delays may contribute; this does not diagnose a solver defect, validate
station position, or justify subtracting 2.23 m from anything. The next
useful test must account for the available broadcast code-delay convention
before attributing residual structure to unknown receiver biases. Do not
repeat constant-state fitting or change baseline from this diagnostic alone.

Provenance: raw H base SHA remains Phase565's
4d3e37cbe0347fa56216db54ede9e0f30731885f337f1653ab5a86afb2bb2150.
Diagnostic source SHA 1e62ccbe2279b3e352a470917e3316927023da0c3b22c62a0435fff053e6f966;
header SHA c30f55ff6ebee167af8fb662a2e44f73a28f6a4a8c5103dae7c6862bbb0f9a33;
binary `/tmp/phase566_triple_audit` SHA
849183ca84388b81f325b35ead4e720d38518b700410f69d30790465f6017651.
Linked existing build native libraries. This is a diagnostic, not a separately
hash-frozen accuracy experiment. No new positioning score or submission.
