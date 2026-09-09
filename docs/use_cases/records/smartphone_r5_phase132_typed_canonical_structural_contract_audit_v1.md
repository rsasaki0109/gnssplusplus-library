# Smartphone R5 Phase132 typed canonical preflight structural audit

Status: sealed launch-free contract design (Luna Max)

This audit covers the Phase132 implementation committed after the Phase132
zero-support forensic freeze.  It reads source and sealed metadata only.  It
does not open phone GNSS/IMU, broadcast navigation, base RINEX, solution,
truth, MAT, PDC, precomputed-coordinate, or Kaggle payloads, and it does not
launch the native solver.

## Starting state and evidence

The forensic freeze `3b3c785f5a641bc3f2f49ff4e704f80b41a7ea06` identified the
earliest Phase131 failure: its authorized runner delegated inventory building
to the Phase130 literal descriptor gate.  Valid rover rows therefore stopped
before query-time GLONASS certification, support lookup, Phase131 Python
canonicalization, native command construction, and the C++ resolver.  The
route-level key sets were not sealed, so the route data cause remains
undetermined.  Phase130 counters remain comparison-only evidence.

The implementation under audit is
`4f546734771ac56d8342aadb88d6c36f454540fa`.  It replaces only that runner
delegation.  It reuses the existing Phase128 parsers and Phase130 finite
support classifier, and applies the existing Phase131 typed family policy to
both sides of the preflight.  No C++ source, graph, factor, equation, unit,
FCN, sigma, filter, Huber, QR, IMU, TDCP, LM, or default setting is changed.

## Frozen preflight boundary

For every already-parsed row, the runner records the source identity and maps
the correction join to

```text
(GNSSSystem, SatelliteId.prn, PhysicalFrequencyFamily,
 certified_GLONASS_FCN when the system is GLONASS)
```

`L1` and `L5` are the only physical families.  Android aliases and RINEX
tracking-code aliases may collapse only when they map to the same physical
family.  Literal signal/tracking text remains provenance and is not a join
field.  A GLONASS FCN must come from the existing exact query-time
header/geph certification, must be in `[-7, 6]`, and is never inferred from
Android carrier frequency or an external table.  Unknown family, missing or
invalid FCN, conflict, tie, or unsupported identity is an explicit miss or a
fail-closed route error according to the existing Phase126-131 policy.

Each retained rover factor must have one finite exact endpoint, exact interior
sample, or adjacent finite in-domain two-point bracket in the canonical base
stream.  Every row is classified as certified or an explicit miss; unused
base samples are retained in local accounting.  Row index, equal counts,
equal cadence, nearest/hold, extrapolation, zero correction, raw uncorrected
fallback, partial remapping, and cross-family/cross-satellite fill are
forbidden.  Empty usable support fails before native launch.

## Execution evidence boundary

The runner records, separately and monotonically, that the Python typed
preflight ran, that the Phase131 selector was forwarded exactly once, that an
argv was constructed, that native invocation was attempted, and that native
canonicalizer rows/rejections reached the summary.  A preflight failure has
solver invocation count zero and cannot claim selector or resolver reach.
The native resolver count is explicitly the native summary's canonical rows
plus canonical rejected rows; zero means no canonicalizer row reached, not
that non-canonical observations were counted.

The old Phase130 literal counters are comparison-only and do not participate
in Phase132 admission.  The old Phase131 authorization is not reusable.

## Qualification and future authorization

Launch-free synthetic tests cover same-family aliases, distinct physical
bands, invalid/missing FCN, canonical key conservation, selector isolation,
preflight failure with solver zero, and native evidence accounting.  A
future raw authorization must independently pin this implementation, the
new validator/manifest/pre-raw artifacts, and the target binary, then permit
exactly MTV-A followed by LAX-T once each.  Only raw phone GNSS/IMU, broadcast
navigation, and sealed raw base RINEX may be materialized.  Truth, MAT, PDC,
precomputed coordinates, accuracy, solution-row inspection, Kaggle,
rerun/fallback/repair/sweep remain forbidden.  Structural failure is
fail-closed.

Read accounting for this audit:

```text
raw phone GNSS reads                         0
raw phone IMU reads                          0
broadcast navigation reads                   0
raw base RINEX/header reads                  0
solver invocations                           0
solution/truth coordinate-row reads          0
MAT/PDC/precomputed-coordinate reads         0
Kaggle/token access                           0
reruns/fallbacks/repairs                      0
```
