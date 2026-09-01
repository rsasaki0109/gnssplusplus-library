# Phase 39 GNSS-first velocity-only handoff

Phase 39 evaluates a native, explicit opt-in handoff for the Android raw
GNSS/IMU path.  It addresses the Phase 38 MTV-h diagnosis without treating a
GNSS-first position as an output or as an initialization coordinate:

```text
--native-gnss-first-velocity-only-handoff
```

The GNSS-first P+D optimizer is retained only as an independent source of
Doppler velocity states and heading seeds.  A raw-Doppler WLS solve may seed
that optimizer, but it is only an initializer: the handed-off values are the
final GNSS-first optimizer result, never WLS values or a position-difference
proxy.  The candidate converts those final ECEF velocity states to ENU at the
first original raw SPP position.  It requires one finite state for every raw
observation epoch and a velocity norm no greater than 70 m/s.  WLS edge hold
is disabled and recorded as zero; any bounded interpolation remains an
initializer diagnostic and is never handed off directly.  The original raw SPP
positions and receiver clocks remain in the IMU problem; zero GNSS-first
position/clock copies is an explicit diagnostic.
GNSS-first position-invalid counts (including out-of-Earth states) are shown
in the candidate summary and do not get hidden by a coordinate substitution.

The production/default path and the Phase 31 champion remain unchanged when
the flag is absent.  Candidate failure is fail-closed: no partial velocity
handoff, precomputed coordinate, alternate trajectory, edge hold, truth input,
or MATLAB artifact is allowed.  The ordinary TDCP contract and exact raw UTC
key projection remain structural gates; TDCP factors must be built and
inserted with a positive equal count.

The pre-implementation route/input/flag contract is frozen in the
[Phase 39 freeze record](records/smartphone_r5_phase39_gnss_first_velocity_only_handoff_freeze_v1.json).
The truth-free runner is
`apps/commands/benchmarks/gnss_smartphone_phase39_velocity_only_handoff.py`.
After the source and focused-test commit is pushed, run:

```bash
python3 apps/commands/benchmarks/gnss_smartphone_phase39_velocity_only_handoff.py \
  run-matrix
```

The runner executes the fixed MTV-h target, Phase 31 train3, and Phase 37
safe2 routes twice with the candidate and checks flag-off identity on the same
cohort.  It publishes a structural seal only when every route, repeat,
convergence, finite-output, <=70 m/s, raw-key, TDCP, and flag-off gate passes;
truth remains sealed in either outcome.
