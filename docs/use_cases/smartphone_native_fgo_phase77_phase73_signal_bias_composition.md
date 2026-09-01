# Phase77: Phase73 finite-pc miss mask plus signal-bias composition

Phase77 freezes one source-supported raw-only structural candidate after the
Phase76 accuracy no-go. The candidate keeps the Phase73 finite in-domain base
pseudorange correction exactly unchanged and adds the existing opt-in
`--native-signal-bias-states` path. The native signal-bias contract attaches a
static meter-valued state to each eligible secondary `(system, SignalType)`
undifferenced pseudorange factor, with a fixed zero-mean 1000 m gauge prior.
No IFLC, frequency canonicalization, base model change, truth coordinate, or
post-freeze tuning is allowed.

The choice is based on source/code inventory: the official `fgo_gnss_imu.m`
passes each signal type to `PseudorangeFactor_XC`, while the Phase73/76 native
summaries show zero receiver signal-bias states. Phase46 receiver timing was a
common-mode no-go, Phase60 Full/Satellite ISB data were nonfinite, and the
Phase20 stop candidate is a different already-evaluated raw-IMU component.
Phase11 is pinned only as historical source/structural provenance, not as a
route selector or scoring input.

The structural matrix is four Pixel5 routes, two candidate repetitions and one
Phase73 no-bias control per route (12 native invocations). Each invocation
reads the pinned raw GNSS/IMU/nav/base inputs once. The candidate must expose
finite signal-bias states/factors, exact output keys, deterministic repeats,
and preserve the exact Phase73 miss-mask population by identity: the candidate
must reproduce the sealed Phase73 control's original/retained/dropped counts,
fractions, and correction p50/p95/max for every route. The historical matching
and finite-among-matched fractions remain reported diagnostics, but are not
re-applied as new thresholds here because the already-sealed Phase73
population itself is the control contract. No truth is read in this stage; a
separate accuracy freeze is required only if every structural gate passes.

See the [machine-readable freeze](records/smartphone_r5_phase77_phase73_signal_bias_composition_structural_freeze_v1.json)
for source hashes, flags, telemetry, matrix accounting, and fail-closed rules.
