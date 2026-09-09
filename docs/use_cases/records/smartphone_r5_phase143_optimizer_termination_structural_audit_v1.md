# Phase143 structural launch-free qualification audit

Execution label: Luna Max  
Phase: 143  

This audit defines the launch-free structural qualification boundary after
the Phase143 implementation.  It reads tracked source and sealed metadata
only.  It does not materialize or read raw phone GNSS/IMU, broadcast
navigation, or base RINEX payloads; it does not read solution coordinates or
truth/MAT/PDC/precomputed artifacts; and it does not start the native solver,
accuracy evaluator, or Kaggle client.

## Pinned implementation and design

The source candidate is the already-frozen Phase143 design
`03073a14590f0c6316e7a0294f6b575b796b7c11` and its implementation
`f07a80bb7e26fb0502454b3cb8dc291c53545714`.  The target application is
`build/apps/gnss_fgo_imu_no_base` with SHA-256
`15ab7b401f16476928643bf41485e168a4a8be28bdb6fd1b2f8f7e26e166ef20`.
The implementation adds only the opt-in main LM termination budget and its
scalar native-authoritative report.  It does not alter graph construction,
factor equations, values, initialization, noise, filter, C7/D/CCDD state,
raw-base correction, TDCP/IMU behavior, QR branch, ordering, lambda policy,
damping, output, or legacy defaults.

The earlier design audit and freeze remain immutable evidence:

* `smartphone_r5_phase143_optimizer_termination_parity_audit_v1.md`,
  commit `8a345be4c267ba2be269c1e679f04a356c5ba188`, SHA-256
  `6a7f1775474578cc3c7d8048976d0e55027a97af9918d08d64cd311e4f93a8b6`;
* `smartphone_r5_phase143_optimizer_termination_parity_freeze_v1.json`,
  commit `03073a14590f0c6316e7a0294f6b575b796b7c11`, SHA-256
  `9d579969bdd2002f251db16c0624d03da6fd2d5d33e751f0d1e0ac5d40b3a33f`.

## Structural recipe

The future raw lane is exactly two routes in order, one solver invocation per
route: MTV-A (`2021-03-16-18-59-us-ca-mtv-a/pixel5`) followed by LAX-T
(`2022-04-01-18-22-us-ca-lax-t/pixel5`).  Its recipe is:

* Phase135 official affine P/D/ordinary-TDCP family: on;
* Phase138 TDCP initial-range constant: on;
* Phase118 official Highway TDCP Huber mapping: on, fixed sigma 0.03 m;
* Phase107 native raw-base compensation and source miss mask: on;
* Phase143 official main LM termination budget: on;
* Phase117, Phase120, Phase126 through Phase134, and additional-frequency
  selectors: off.

The raw lane may use only raw phone GNSS/IMU, broadcast navigation, and the
sealed raw-base RINEX already admitted by the preceding raw-base contract.
All materialization occurs only after a new independent authorization.  This
launch-free audit has no authorization to read those inputs.

## Termination contract

The native summary must contain exactly one
`smartphone-r5-native-fgo-phase143-termination.v1` object for each active
stage.  The object is copied from the active `FGOResult`/GNSS-first report;
the wrapper may validate it but may not infer or overwrite it.  Required
fields are:

* selector state, stage, configured and effective max iterations;
* attempted, accepted, rejected outer iterations and total inner lambda
  attempts;
* finite initial/final costs and strict cost-decrease marker;
* the native termination branch;
* relative/absolute/error tolerances;
* initial/final/maximum lambda, lambda factor and bounds, model-fidelity
  threshold, diagonal damping and fixed-lambda-factor state;
* linear solver, elimination, ordering and explicit-ordering presence; and
* no-fallback and complete-trace markers.

The main stage must report effective max iterations exactly 1000 when the
selector is enabled.  Its configured value is the frozen 12→1000 boundary;
GNSS-first remains configured/effective 1000.  A
`maximum_outer_iterations` branch is valid only when accepted outer
iterations equal the effective cap.  Other valid branches are the native
convergence, small-cost, maximum-lambda, no-inner-iteration, exception, and
explicit no-progress enum values.  Missing, duplicate, renamed, inferred,
non-finite, conflicting, or generic-iteration-derived telemetry fails
closed.  A raw structural result must additionally require finite costs and
strictly decreasing final versus initial cost for both stages, with no retry
or fallback.  No artificial wall-clock timeout shorter than the contract is
allowed in a future authorized runner.

## Existing structural gates retained

The Phase135/138 contract remains the authority for all non-termination
gates.  The future result must show all admitted P/D/TDCP rows using the
affine branch and zero legacy counts, Phase138 range-anchor count and
exactly-once transactional adjustment, fixed LOS and one Sagnac evaluation,
raw-base exactly-once and miss conservation, exact finite C7/D meter-unit
handoff, C0/D state alignment, MULTIFRONTAL_QR/EliminateQR main solving,
positive accepted progress, finite earth-valid expected output coverage, and
the Pixel5 final offset exactly once.  An opaque solution hash/row-count may
be sealed, but structural validation never opens or interprets its rows.

The wrapper must preserve the native summary bytes/hash separately from any
normalized structural view.  It may not fabricate stage values from a return
code, generic iteration field, output row count, or historical Phase141/Phase
112 result.  Historical artifacts remain immutable and are comparison-only.

## Qualification and authorization boundary

Launch-free qualification consists of source/freeze/manifest pin checks,
synthetic in-memory summaries covering convergence and cap branches,
rejection/non-finite/missing telemetry cases, selector isolation, and the
existing focused/full C++ qualification.  It records zero raw payload reads,
zero solver invocations, zero solution-coordinate reads, zero truth/MAT/PDC/
precomputed reads, and zero accuracy/Kaggle activity.

After a passing launch-free manifest and pre-raw seal, a new independent raw
authorization is required.  That authorization must pin the full audit,
freeze, implementation, runner/manifest, pre-raw, and target-binary hashes;
materialize MTV-A then LAX-T once; fail closed without retry/fallback when any
termination or existing structural gate is missing; and commit the opaque
raw result separately.  Truth/accuracy remains a separate later
authorization.  This audit, freeze, runner, and pre-raw seal do not authorize
raw or solver execution.

## Source witnesses and hashes

The implementation witness files at this audit point are:

* `apps/native/gnss_fgo_imu_no_base.cpp` — SHA-256
  `085cedec22d58158a760a5d06acf63271f82c9d98025a81ed3b93618966bef2a`;
* `include/libgnss++/algorithms/fgo.hpp` — SHA-256
  `b24ea504dea3a68583659939f74fa4766e10dc5c96f727c1bb3a57d593263d00`;
* `include/libgnss++/algorithms/fgo_config.hpp` — SHA-256
  `f982cb0468bab27222634a76953cda00a7ece6a360bc38ed4e25308c45e1ca12`;
* `src/algorithms/fgo_gtsam_backend.cpp` — SHA-256
  `1d1e7819fe20c163d336ac4acc8b0fb6c7fdb3385096962330e894be56b22399`;
* `src/algorithms/fgo_gtsam_internal.hpp` — SHA-256
  `be61a643c181206c3260549c05407e81b6a7932c81357a82e4de379b5bf560d5`;
* target binary — SHA-256
  `15ab7b401f16476928643bf41485e168a4a8be28bdb6fd1b2f8f7e26e166ef20`.

These are static source/binary pins only.  No payload hash is introduced by
this audit.
