# Phase400 — Pose3 derivative and paired native optimization controls

Extended `test_tdcp_residual_state_factor.cpp`, fresh standalone GTSAM build
and run passed 5/5 tests. No production graph change, raw solve, truth,
saved position input, MAT or new accuracy evaluation.

New Pose3 control wraps the actual native C7/lever-arm TDCP factor with a
nonzero lever arm and rotated endpoint poses. Zero residual-state cost equals
the original factor. At nonzero residual state, all 12 endpoint pose tangent
derivatives agree with central finite differences (1e-8 tolerance).

New paired graph uses two actual native C7 point TDCP factors with separate
scalar Huber noise models, one shared residual-slant-change key, four tight
geometry/clock fixture priors and one nuisance prior (seven factors total).
LM recovers injected .03 m residual change within 1e-6 m and lowers cost.
Synthetic prior scales are not proposed phone-model parameters.

This establishes mechanics with strongly anchored geometry/clock, not full
smartphone observability, outlier separation or positioning improvement.
Native main and staging integration, exact satellite/endpoint pairing,
clock resets, unmatched-row preservation, single prior per pair, diagnostics
of the modified residual, and explicit prior-scale selection remain undone.
Next implement pairing on already-admitted TDCP factors without reading
saved trajectories or selecting rows from truth. Goal remains unmet.
