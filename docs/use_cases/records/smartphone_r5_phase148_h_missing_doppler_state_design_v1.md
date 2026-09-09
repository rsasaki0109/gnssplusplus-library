# Phase148 H missing-Doppler velocity/clock-state design

Status: design-only, source and sealed-metadata audit.  No Phase148 raw,
solver, truth, MAT, Kaggle, or accuracy execution was performed.  The pinned
historical input is the Phase147 H diagnostic and its sealed report/result;
those artifacts are preserved and are not reclassified here.

## Finding

The H failure is an admission/contract failure, not evidence of a numerical
optimizer regression.

1. `src/algorithms/fgo_gtsam_backend.cpp:64-66` defines
   `use_gnss_velocity_states` as `config.use_velocity_states` **and** a
   nonempty retained undifferenced-D family.  The Phase143 admission at
   `:97-105` consequently aborts GNSS-first H before graph/state construction.
2. Independently, the Phase135 transaction at `:629-633` requires nonempty
   pseudorange, Doppler, and TDCP families.  Broadening the first predicate
   would therefore still not make the pinned Phase135 recipe a valid no-D
   recipe.
3. `include/libgnss++/algorithms/doppler_velocity_wls.hpp:89-165` requires at
   least four D rows and rank four.  The raw-D initializer in
   `include/libgnss++/algorithms/source_clock_c0d_initializer.hpp:31-70`
   only validates/copies finite `EpochSeed.receiver_clock_drift_mps`; it does
   not create velocity states.  There is no current native GNSS-only,
   raw-P-only finite-difference velocity initializer, and no permission to
   synthesize D by zeroing, holding, or interpolating it.

The Phase147 H stderr therefore identifies the earliest blocker as
`main-optimizer-admission`; its absent native summary must not be interpreted
as a factor or convergence result.

## Source-backed topology and observability

The official source cache shows the intended topology is independent of the
presence of individual D observations:

- `output/reproducibility-cache/gsdc2023/fgo_gnss.m:89-121` creates X, V, C,
  and D keys for every epoch.  Its initialization uses `posbl` and `obs.dclk`;
  it does not invent missing measurements.
- `output/reproducibility-cache/gsdc2023/fgo_gnss.m:154-201` adds
  `MotionFactor_XXVV` and `ClockFactor_CCDD` on eligible adjacent epochs even
  when the conditional D-factor loop has no rows.
- `output/reproducibility-cache/gtsam_gnss/src/MotionFactor_XXVV.h:16-63`
  constrains position change by the two endpoint velocities.  CCDD in
  `ClockFactor_CCDD.h:16-64` constrains C[0] change by endpoint D and leaves
  C[1..6] as the documented clock-vector components.
- The native GTSAM backend currently materializes V/D only in the
  Doppler-dependent block (`src/algorithms/fgo_gtsam_backend.cpp:750-806`),
  has position-only GNSS motion (`:1877-1903`), and adds broad zero V/D
  priors only as a sparse/degenerate gauge guard (`:2175-2200`).  Those priors
  are not a physical velocity or clock-drift observation and cannot replace
  XXVV/CCDD observability.

Thus a correct no-D path must establish X observability from same-run raw P,
V observability through the official motion relation, and D/C observability
through a finite, source-backed clock sequence plus CCDD.  The H raw field
coverage and finite/gap pattern for those seeds were not read in this design
task and remain unresolved.

## Proposed staged implementation (unimplemented)

1. Add a raw-P-only, same-run seed stage before graph admission.  It must
   retain provenance for every accepted epoch, require finite P-derived
   positions and strictly valid time intervals, and reject gaps or missing
   endpoints rather than using a precomputed trajectory, MAT result, truth, or
   solution payload.
2. Derive velocity only by the explicitly specified finite-difference policy
   over those valid intervals, with source epoch/time and gap checks recorded.
   This is the native analogue of the official `velini=posbl.gradient(obs.dt)`
   source operation; it is not implemented by this record.
3. Use the existing raw-D initializer only when exact finite
   `receiver_clock_drift_mps` coverage is certified for the graph epochs.  If
   H has no such source values, fail closed; do not create D values.
4. Implement a dedicated, default-off no-D GNSS-first graph mode.  In that
   mode only, materialize V/D for all admitted epochs, add the source-backed
   XXVV and existing CCDD-equivalent edges with current dt/phone/jump guards,
   preserve C7/D and all current numerical settings, and export state counts
   explicitly.  This must be a new contract, not a bypass of the Phase143 or
   Phase135 guards; the Phase135 affine family remains invalid when its D
   measurement family is empty unless a separately reviewed contract changes
   that rule.

The priority is the raw-P seed stage before changing graph admission.  No code
change is justified until the source seed coverage and synthetic graph
observability are proven.

## Acceptance tests for a future implementation

- A synthetic multi-epoch P-only problem with finite positions/times is
  accepted only under the explicit default-off no-D mode; every epoch has X,
  V, C, and D keys, XXVV/CCDD edge counts match eligible intervals, and no D
  observation factor is silently fabricated.
- Synthetic missing/nonfinite P, duplicate/nonmonotonic time, and excessive
  gap cases are rejected before admission.  Missing/nonfinite raw D is
  rejected rather than replaced by zero, hold, WLS, or interpolation.
- A synthetic ordinary-D problem follows the existing state initialization,
  factor counts, Phase143 iteration/termination semantics, and exports
  unchanged.
- Default-off behavior preserves the current H admission failure, while the
  Phase135 nonempty-family transaction remains explicit and test-covered.
- Structural export tests assert finite/full epoch alignment and distinguish
  configured versus effective iteration limits; no accuracy or coordinate
  claim is made by this design.

## Unresolved evidence and boundary

The following require a separately authorized future raw-only preflight/run:
H raw-P finite position coverage and valid interval pattern; exact finite raw
D coverage; CCDD clock observability under H's C7/D setup; and any resulting
native convergence.  No H/U/A/LAX numerical result, truth comparison, or
accuracy/leaderboard conclusion follows from this record.

Next action and priority are sealed in
`smartphone_r5_phase148_next_action_v1.json`.
