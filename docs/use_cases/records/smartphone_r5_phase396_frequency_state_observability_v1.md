# Phase396 — frequency-state design boundary

Inspected current native graph, not just CLI flags. Legacy residual ionosphere
is one vertical L1 residual state per epoch, used by pseudorange factors with
thin-shell/frequency coefficients and temporal priors. Ordinary TDCP has no
such keys. Epoch-local C7 parity explicitly rejects this legacy option.
Do not relax that guard as a purported TDCP correction: it would leave the
carrier likelihood unchanged and does not establish a coherent joint model.

Added four standalone Eigen/gtest algebra controls in
`tests/test_dual_frequency_tdcp_observability.cpp`, also registered in the
existing test source list. Fresh standalone binary passes 4/4. Full CTest
was not run. No production factors/weights or candidate outputs changed.

For the explicit research model y1=g-dI and y5=g-alpha*dI,
alpha=(1575.42/1176.45)^2, the controls establish:

1. Two distinct frequencies recover common range/clock change and a slant
   L1 ionosphere-change variable in this noiseless two-variable model.
2. A single band or identical frequencies cannot separate these variables.
3. Adding an unconstrained independent band-bias variable makes the model
   underdetermined (two rows, three unknowns, explicit null direction).
4. Eliminating ionosphere amplifies independent equal-variance noise in the
   recovered common component by (alpha^2+1)/(alpha-1)^2 in variance;
   the two recovered components are correlated.

These controls do not validate the physical hypothesis, phone noise
independence, satellite-state parity, or whole-graph observability. The raw
dual-frequency diagnostic cannot distinguish ionosphere from tracking error.
Do not add transformed dual-frequency factors alongside their original
observations as independent measurements; that double-counts information.

A viable experiment needs a separate default-off TDCP slant-change model,
retaining original frequency rows with joint nuisance elimination or explicit
states and honest covariance; single-frequency intervals need a justified
prior/dynamics. It must not reuse the legacy P-only switch, saved trajectory
seeds, or fitted truth corrections. Existing C7 P biases and corrected-carrier
atmospheric conventions must be traced before integration. Goal remains unmet.
