# Smartphone R5 Phase129 structural-contract audit

- Execution label: `Luna Max`
- Audit date: `2026-09-04`
- Repository: `/home/sasaki/workspace/rtklib_v2_ws/gnssplusplus-library`
- Scope: launch-free contract qualification for the already implemented
  Phase129 candidate.
- Implementation pin: `05e57d5008320734de93c763ff84ccd31f755805`
- Candidate freeze pin: `0645f7297c766e8190b4d3560a0dd34af012d631`

This audit reads source text, existing sealed contract metadata, and the
target binary identity only.  It does not open raw phone GNSS/IMU, broadcast
navigation, raw-base RINEX, truth, MAT, precomputed coordinates/corrections,
solution rows, PDC, Kaggle resources, or tokens.  It does not launch a native
solver.  All pre-raw read counters are therefore zero.

## Contract decision

The single structural candidate is the Phase129 local GLONASS miss overlay,
enabled only with the complete Phase126/127/128 source-complete recipe and
Phase118 official TDCP Huber selector.  The route matrix is exactly:

1. `2021-03-16-18-59-us-ca-mtv-a/pixel5` (MTV-A)
2. `2022-04-01-18-22-us-ca-lax-t/pixel5` (LAX-T)

Each route has one future run, in this order.  The runner remains launch-free
until a separate independent raw authorization is committed.  Before a solver
could start, each route must pass one inventory stage.  The inventory is not
allowed to mutate or copy payloads and must classify every GLONASS row as
exactly one of certified or explicit local miss.  Non-GLONASS rows remain
retained under the existing source-complete path.

## Inventory gate

The stage-1 ledger must prove all of the following for rover and base:

- exact typed satellite/signal keys and the existing native GPST query time;
- header-primary or selected time-valid `geph.frq` provenance, with no fixed
  channel, carrier-frequency inference, external table, or extrapolation;
- per-record missing/invalid/conflict/tie/time/range/wavelength reasons;
- `input = certified + explicit_local_miss` for GLONASS, and the shared base
  and rover ledgers are identical for the rows they share;
- certified rows have finite positive wavelength, while uncertified rows are
  explicit misses and are absent from both the base correction stream and
  shared rover factor vector;
- all-miss/empty usable routes fail closed with zero solver invocations.

The Phase129 implementation keeps Phase126 global source-complete A/B/C
failures global.  It changes only the earlier unaccepted-GLONASS provenance
outcome to a local miss when the selector is on.  It does not add factors or
states, change equations, units, sigma, filters, LM schedule, QR selection,
TDCP, IMU, C7/D/CCDD handoff, Pixel5 offset, or output alignment.  The
Phase111 exact-key finite correction mask and conservation remain shared by
the base and factor ledgers.

## Stage-2 structural gate

Only an inventory-passing route may invoke the pinned native binary once.  A
sealed summary must contain the Phase129 row/stream/factor conservation and
exactly-once/no-fallback markers, GNSS-first and main accepted iterations,
finite strict cost decrease, C7/D/CCDD exact/full/finite handoff, finite
earth-valid output coverage, and the fixed Phase126 A/B/C transaction.  The
summary may expose opaque output row/hash metadata only; no solution
coordinate rows are read or published by this contract.

## Fail-closed and authorization boundary

The launch-free validator checks the source pins, binary hash, command
selectors, forbidden lineage, route order, and zero pre-raw accounting.  It
also validates an in-memory inventory record for synthetic tests.  It never
materializes a route input and never calls a subprocess.  A future
independent authorization must explicitly permit the two inventory reads per
route and, only after a passing inventory, at most one solver invocation per
route.  A failed inventory invokes zero solvers; no retry, fallback, repair,
or rerun is permitted.  Truth/accuracy authorization is a separate later
boundary and is not part of this artifact.

## Evidence used

The preceding Phase129 candidate audit/freeze (`0ed5798afdea0738e059623a2f5b5470a5dcd2f1`,
`0645f7297c766e8190b4d3560a0dd34af012d631`) establishes the local certified-or-
miss semantics.  Phase128 inventory-first contract/result records establish
the route order, raw-only lineage, and source-complete structural gates.
Implementation source markers and the target binary are pinned by the next
freeze/manifest; no historical result is modified or reread as payload.

## Outcome

Freeze exactly one launch-free structural contract.  The contract is
qualified by synthetic inventory records only.  It is not raw execution
authorization and makes no accuracy claim.
