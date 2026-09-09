# Smartphone R5 Phase133 runner/native selector boundary audit

- Execution label: `Luna Max`
- Audit date: `2026-09-04`
- Repository: `/home/sasaki/workspace/rtklib_v2_ws/gnssplusplus-library`
- Scope: read-only audit of the sealed Phase132 structural result, its
  qualification note, and the native CLI source.  This audit does not open a
  raw phone file, broadcast-navigation payload, raw-base RINEX payload,
  solution row, truth row, MAT/PDC/precomputed-coordinate artifact, or
  Kaggle/token resource.  It does not launch the native process or rerun a
  route.

## Disposition

Exactly one Phase133 candidate is selected:

`phase133-runner-native-selector-boundary-v1`

The candidate removes the Phase130 token from the native argv assembled by a
new Phase133 runner boundary.  Phase130 was a contract/runner comparison
selector, not a native CLI selector.  The change is limited to argv
composition and launch-free validation.  It does not change the native
binary, graph, factors, equations, units, correction values, FCN policy,
sigma, robust kernel, QR selection, IMU/TDCP processing, LM schedule,
initialization, output, or legacy default.

The Phase132 native command must not be reused: it contains the unsupported
Phase130 token and both sealed route attempts failed closed before native
resolver/graph entry.  A future raw authorization must pin the Phase133
runner, manifest, and a new target-binary hash check, then materialize only
the authorized raw inputs after authorization.

## Authoritative evidence

| item | value |
|---|---|
| Phase132 sealed structural result commit | `d28b6a17c85504fb26c5f7b3ecd6982968169643` |
| Phase132 qualification note commit | `e42d61f23a335562657b926e866e058f69e2c702` |
| Phase132 implementation commit | `4f546734771ac56d8342aadb88d6c36f454540fa` |
| Phase132 forensic freeze used by the contract | `3b3c785f5a641bc3f2f49ff4e704f80b41a7ea06` |
| native source | `apps/native/gnss_fgo_imu_no_base.cpp` |
| native source SHA-256 at audit | `6fcee581af70535b8af09a674a234352abaaf83a34b722d4e91577abaf352170` |
| target binary SHA-256 (historical, hash-only) | `ed24b57b29f679123c3a52dbb04fad9b979092564abc4a35905bdb2abfaa0c8e` |

The sealed result records the same native failure for MTV-A and LAX-T:

`Unknown argument: --native-phase130-shared-ledger-key-local-support`

Both routes had one inventory pass and one attempted native invocation,
return code `2`, zero native resolver calls, zero main accepted iterations,
and no cost transition.  The historical Phase132 read accounting records
zero solver/truth/solution-row/MAT/PDC/precomputed-coordinate/Kaggle reads
before and outside the authorized raw attempt.  No Phase133 payload read is
made here.

## Source and command boundary

The native `usage()` text and its `main` argument parser contain these
Phase118+native selectors:

| ownership | selector | required Phase133 argv count |
|---|---|---:|
| native | `--native-phase118-official-tdcp-huber-k` | 1 |
| native | `--native-phase126-raw-base-source-complete` | 1 |
| native | `--native-phase127-glonass-channel-provenance` | 1 |
| native | `--native-phase128-glonass-provenance-parser-admission` | 1 |
| native | `--native-phase129-glonass-local-miss-mask` | 1 |
| native | `--native-phase131-canonical-correction-band-key` | 1 |
| runner/contract only | `--native-phase130-shared-ledger-key-local-support` | 0 |

The following selectors remain explicitly off and absent from the future
Phase133 argv:

`--native-phase117-tdcp-snr-type-sigma`,
`--native-phase120-official-tdcp-resl-atmosphere-cancellation`, and
`--native-base-pseudorange-preserve-additional-frequency-bands`.

The sealed Phase132 command snapshot had the seven native/runner tokens in
this order:

```text
phase118 phase126 phase127 phase128 phase129 phase130 phase131
```

The Phase133 snapshot is the same recipe with exactly one deletion:

```text
phase118 phase126 phase127 phase128 phase129 phase131
```

All other tokens and placeholders remain byte-for-byte equivalent to the
Phase132 launch-free recipe.  This is an argv ownership correction, not a
solver configuration change.

## Native help cross-check policy

The native source is checked read-only for the usage/parser ownership table.
The Phase133 focused tests also feed a synthetic fake-binary `--help` text to
the same cross-check and feed a synthetic argv to a fake binary that rejects
unknown options.  A real `--help` binary launch is intentionally not made in
this launch-free qualification.  The cross-check must prove that every
required native selector is advertised/parsed, Phase130 is not advertised or
parsed, and no forbidden selector is forwarded.

## Required evidence and call counts

Phase133 launch-free evidence must distinguish these counters; copied
Phase130 comparison counters are never admission predicates:

* typed Phase132/131 preflight call count: positive before native command
  construction;
* old literal Phase130 preflight call count: exactly zero;
* native command construction and selector forwarding: exactly one per
  future passing route;
* native resolver/graph reachability: measured only from a future native
  summary, never inferred from Python preflight;
* on any preflight failure: command construction, native invocation, and
  solver invocation are all zero.

The launch-free Phase133 qualification records all raw phone GNSS/IMU,
broadcast-navigation, raw-base, native, solution-coordinate, truth,
accuracy, MAT/PDC/precomputed-coordinate, and Kaggle reads as zero.  It
records no route result and no solution rows.

## Future authorization boundary

No raw materialization, native solver invocation, truth evaluation, accuracy
calculation, solution publication, rerun, fallback, repair, or sweep is
authorized by this audit.  A later independent authorization may run exactly
MTV-A then LAX-T once each, using only raw phone GNSS/IMU, broadcast nav, and
the separately sealed raw-base RINEX.  It must pin the Phase133 freeze,
runner/validator/manifest, pre-raw accounting, and target binary hash before
any payload read.  Structural failure remains fail-closed; truth and
accuracy remain separate authorizations.
