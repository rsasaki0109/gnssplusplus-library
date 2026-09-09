# Phase139 wrapper path-schema audit

Status: read-only audit of the Phase138 authorization boundary.  No raw
GNSS/IMU/navigation/base payload, solver, truth, MAT, PDC, precomputed
coordinate, accuracy, or Kaggle artifact was opened.

## Evidence

The Phase138 authorization schema is
`smartphone-r5-phase138-affine-tdcp-structural-raw-authorization.v1`.  In
each route, `raw_inputs` is a flat mapping: the keys are
`device_gnss.csv`, `device_imu.csv`, and `brdc.nav`, and each value is the
metadata object containing `path`, `bytes`, `sha256`, `source`,
`read_before_authorization`, and `copy_or_transform`.  `base.obs` is a
separate flat `base_input` metadata object.  This is visible in the sealed
authorization at lines 80--123 and 126--165.

The authorized Phase138 wrapper does not implement that representation.  At
`apps/commands/benchmarks/gnss_smartphone_phase138_affine_tdcp_structural_authorized_execute.py:294--297`,
it iterates the flat `(name, metadata)` pair and then calls
`_mapping(metadata, name, "raw input")`.  That call requires a second nested
`metadata[name]` mapping.  The first route therefore fails with the sealed
message `raw input/device_gnss.csv: missing`, before `digest_file()` at
lines 84--92 can open a payload.  The sealed result
`356c93bb9eb5c741ec1e15cc19de5c58428d0fcf` records one wrapper invocation,
zero payload reads, zero solver invocations, and LAX-T not entered.

There are two adjacent boundary gaps.  `verify_authorization()` at lines
163--167 checks phase and status but not the authorization schema version;
`actual_payload_paths()` also relies on incidental mapping behavior rather
than validating route, field, value, and path types before resolution.  An
unexpected nested object, missing field, boolean-as-integer, absolute path,
or `..` path could otherwise be accepted or fail with an ambiguous error.

## Phase139 correction boundary

Implement only a parser/contract correction in the authorized wrapper:

1. Require the exact Phase138 authorization schema version, route-list shape,
   flat `raw_inputs` key set, separate `base_input`, and mapping types.
2. Read each raw metadata object directly (no nested filename lookup), require
   the sealed key sets and strict scalar types, and reject missing/unknown
   keys, malformed SHA-256/size/provenance values, and invalid booleans.
3. Accept only non-empty repository-relative paths with the expected basename;
   reject absolute paths, `.`/`..` traversal, NULs, forbidden categories, and
   paths that lexically escape the repository.  Preserve the existing
   post-authorization hash and exact-once policy.
4. Keep command construction, selectors, solver, factors, recipe, output
   policy, and all Phase135/138 algorithm code unchanged.

This is a wrapper-only fix.  The old Phase138 authorization and failed result
remain historical and immutable.  Because the wrapper hash and parser
behavior change, any future raw attempt requires a new independent
authorization, a new runner/manifest pin, and a new pre-raw zero-read seal;
the old authorization must not be reused.

## Synthetic qualification plan

The focused tests will exercise flat-valid metadata, nested-invalid metadata,
missing key, wrong type, absolute/traversal path, duplicate/unknown key, and
selector-isolation cases.  Tests call the parser only with in-memory route
objects and do not call `digest_file`, native subprocesses, or any payload
path.  Full C++ tests are permitted as build/regression qualification only;
raw, solver, truth, MAT/PDC, accuracy, and Kaggle read accounting must remain
zero.

Audit evidence pins: Phase138 authorization JSON SHA-256
`3a2b97ddfbd3290b081cf7501532f2e0d08b9bae22bf263e9d786a2f27d0f0653`,
Phase138 failed-result JSON SHA-256
`c65a66297bd962d412371398050bbc00e8ae7ae2c40c542365613e19bebe3c04`, and
pre-fix wrapper SHA-256
`7edfa6aacf7af6e7fd14d0bd13dc22f6547628bf4e02c62ffecc41eb53742663`.
