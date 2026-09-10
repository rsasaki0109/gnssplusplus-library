# Phase167 dedicated raw-P no-Doppler 1000-iteration termination lane

Phase167 adds one default-off selector,
`--native-phase167-raw-p-no-doppler-lm-termination-budget`, which is accepted
only together with the Phase165 same-run raw-P GNSS-only graph.  It sets that
dedicated graph's configured LM budget to exactly 1000 and enables the existing
native Phase143 termination trace.  It does not alter graph factors, Values,
initialization, weights, tolerances, clock gauges, or any legacy/normal-D path.
The old Phase143 main-graph CLI flag remains separate and unchanged.

The cached source evidence is
`output/reproducibility-cache/gsdc2023/fgo_gnss.m`, SHA-256
`5368e4056f70f448b728792c5e4c124b7f2afc1e00653492b807083bd3ccf0c3`, lines
203--207: GTSAM LM parameters use `setMaxIterations(1000)`.  Native backend
admission requires GTSAM, Point3/no-IMU velocity states, the Phase165 graph,
and configured `max_iterations == 1000`; otherwise it fails closed.

The result serializer labels Phase167 separately and publishes the native
termination branch, configured/effective limits, attempted/accepted/rejected
outer iterations, lambda counts, costs, solver/elimination/order, trace
completeness, `tolerance_proven`, and `effective_cap_reached`.  A cap or
lambda branch is never relabeled as tolerance convergence.  Historical
Phase165 output fields remain unchanged when the selector is off.

Validation performed before any Phase168 raw execution:

* actual `gnss_fgo_imu_no_base` target built successfully;
* focused selector/default/gate/Phase143/Phase164/Phase165/Phase167 filter:
  15/15 passed;
* Phase167 synthetic trace-on versus trace-off comparison preserved graph
  factor/value counts, initial/final costs, and solution positions;
* no raw GNSS, nav, truth, MAT, or solver route execution was performed by
  Phase167 implementation validation.

Implementation commit is recorded by the parent task's subsequent Phase168
manifest before the one-shot H execution.
