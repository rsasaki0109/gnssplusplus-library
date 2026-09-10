# Phase95 raw-input path availability audit v1

## Decision

This is a read-only Luna Max audit. It did not open, hash, or parse any raw
GNSS, IMU, or broadcast-navigation file, and it did not launch the native
solver. The result is:

`DATA_PRESENT_PHASE94_RUNNER_PATH_REGRESSION`

The four Phase93 sealed runs prove that the required raw artifacts were
available at their inherited Phase91 paths. A metadata-only `stat` check in
this audit confirms that all twelve of those files are still regular files.
The four Phase94 sealed runs instead passed twelve different, root-relative
`raw/phase93/...` paths; all twelve are absent, and `raw/` is absent at the
Phase94 repository root. Therefore the Phase94 no-go is a runner path
resolution failure, not missing raw data.

One and only one correction candidate is frozen for a future, separately
authorized change:

`phase95_phase94_wrapper_inherit_phase91_raw_paths_and_materialize_exact_route_inputs`

The candidate is a wrapper resolver/materialization correction only. It must
reuse the pinned Phase91 route-to-file mapping, perform the same safe
repo-root-relative existence/stat checks, substitute the three actual paths
into the already sealed Phase94 command, and preserve every native flag and
the Phase94 evaluator contract. It does not authorize raw execution in this
phase.

## Audit boundary and accounting

| item | value |
| --- | --- |
| execution label | Luna Max |
| audit mode | read-only path/stat/source/manifest/log audit |
| raw file byte reads in this audit | 0 |
| raw file hashes computed in this audit | 0 |
| native solver invocations in this audit | 0 |
| truth/MAT/precomputed-coordinate/base/Kaggle/token reads | 0 |
| accuracy calculations or submission release | 0 |
| mutations to raw input locations | 0 |

The SHA values below are sealed metadata from Phase91/93, not hashes computed
by this audit. File availability was checked with `Path.stat()` only.

## Pinned evidence

| evidence | commit or artifact | SHA-256 where applicable |
| --- | --- | --- |
| Phase93 runner | `b2a5d47a847e1a1a5b63ac2d2a910bad313d0096` | `96f841081788de1c3b7aea1b7983ff5f77bbf04a6cc69e63264e822791d9a920` |
| Phase93 execution manifest | `docs/use_cases/records/smartphone_r5_phase93_source_staging_clock_state_handoff_execution_manifest_v1.json` | `f5ef7cc946e1ef875a30546f32cee9d04a2aa82471f92767ab4935652cec4277` |
| Phase93 sealed result | `docs/use_cases/records/smartphone_r5_phase93_source_staging_clock_state_handoff_structural_result_v1.json` | `cc5eb2a56ee4239a9e25eca04d4573fd8fcfc0cc071f0cb3a507f33ba7f1ca59` |
| Phase94 pre-raw matrix | `4c663870ae9defc4400b64e48a29365e0d3888fc` | manifest SHA `4a36a4871c305f66c419db6a80ca4c2a932a37e0522cf8dc688fd2792df7d3ed` |
| Phase94 path-handling pin | `42f17dc8a10db5afe2ac43539ca87551272d515f` | see Phase94 manifest/auth pins |
| Phase94 unavailable-stage telemetry pin | `89fc48f4e998f892f01d06ca9be4e2fdc8750752` | wrapper SHA `fa124b79f9119c20b7396977c3feecefa3e0410d64dfa4b0ec5212ac5d55194d` |
| Phase94 sealed result | `6f9fd4d209a7b4280f4d12f0aacc1c988e627a89` | `c02afcd0129072209ae8bfc14765f1bfce87defc40358974e7f3b37535e07ba9` |
| Phase94 raw authorization | `docs/use_cases/records/smartphone_r5_phase94_source_clock_c0d_stage_diagnostics_raw_execution_authorization_v1.json` | `11773c8c471a790cdea6cfc756eed3e008f034ed57ef6a6e2cffd6eaeb5f8382` |

The Phase94 result was regenerated/sealed at `6f9fd4d`; the execution itself
is not repeated by this audit.

## Repository-root and data-root resolution

Both pinned wrappers define the repository root as:

```text
ROOT = Path(__file__).resolve().parents[3]
```

For both files, this resolves to:

```text
/home/sasaki/workspace/rtklib_v2_ws/gnssplusplus-library
```

Neither wrapper resolves a separate external data root. All relative native
arguments are interpreted with `cwd=ROOT`.

### Phase93 working path

The Phase93 wrapper additionally pins and reads only the JSON structure of:

```text
ROOT/docs/use_cases/records/smartphone_r5_phase91_source_clock_c0d_gnss_first_raw_drift_d_initializer_execution_manifest_v1.json
```

`load_raw_inputs()` validates the inherited route order, safe relative paths,
the sealed SHA fields, and `ROOT / path_text`; it calls `is_file()` and
`stat().st_size` but deliberately does not open or hash raw bytes. Its
`materialize_command()` then replaces each Phase93 role placeholder after
checking the placeholder contract. The native process is launched with
`cwd=ROOT` and the substituted Phase91 path.

### Phase94 working path

The Phase94 wrapper has no inherited-manifest/data-root resolver. Its
`_raw_input_metadata()` computes `ROOT / Path(record["raw_inputs"][name]["path"])`
and records only `is_file()`. Its `materialize_command()` validates the
manifest command and returns it unchanged. `execute_matrix()` passes that
unchanged list to `subprocess.run(..., cwd=ROOT, ...)`. Consequently
`raw/phase93/...` means:

```text
/home/sasaki/workspace/rtklib_v2_ws/gnssplusplus-library/raw/phase93/...
```

That directory does not exist. The existing raw directories are instead
under `output/smartphone-r5/phase25-raw-clock-eval-v1/raw` and
`output/smartphone-r5/phase37-pixel5-repeatability-v1/routes/.../inputs`.

## Per-route and per-file path audit

The Phase93 actual paths and their sealed byte/SHA metadata are from the
Phase91 inherited manifest and the Phase93 route metadata. `stat` in this
audit confirmed the actual size and regular-file status shown below. Every
Phase94 expected path was checked with `stat`; every one was missing.

| route / file | Phase94 command path (stat) | Phase93/Phase91 actual path (stat) | sealed bytes / SHA-256 |
| --- | --- | --- | --- |
| `2021-03-16-18-59-us-ca-mtv-a/pixel5` / `device_gnss.csv` | `raw/phase93/2021-03-16-18-59-us-ca-mtv-a/pixel5/device_gnss.csv` — missing | `output/smartphone-r5/phase25-raw-clock-eval-v1/raw/2021-03-16-18-59-us-ca-mtv-a/pixel5/device_gnss.csv` — regular file, 57,715,495 bytes | 57,715,495 / `c7d50d5127d16586adc6c79d724758e298b385496da22c5e5dfd6ec522cbc863` |
| `2021-03-16-18-59-us-ca-mtv-a/pixel5` / `device_imu.csv` | `raw/phase93/2021-03-16-18-59-us-ca-mtv-a/pixel5/device_imu.csv` — missing | `output/smartphone-r5/phase25-raw-clock-eval-v1/raw/2021-03-16-18-59-us-ca-mtv-a/pixel5/device_imu.csv` — regular file, 34,393,802 bytes | 34,393,802 / `afc540e7c4ce2ca66b442a1afbcd604e9f6b3d2cc4d3733183739901b5b97bd6` |
| `2021-03-16-18-59-us-ca-mtv-a/pixel5` / `brdc.nav` | `raw/phase93/2021-03-16-18-59-us-ca-mtv-a/pixel5/brdc.nav` — missing | `output/smartphone-r5/phase25-raw-clock-eval-v1/raw/2021-03-16-18-59-us-ca-mtv-a/pixel5/brdc.nav` — regular file, 9,955,040 bytes | 9,955,040 / `6adfaf7fe4452a4faeb94a7b607c15e05f578c46a028a030b94aa6f79de194cd` |
| `2021-08-24-20-32-us-ca-mtv-h/pixel5` / `device_gnss.csv` | `raw/phase93/2021-08-24-20-32-us-ca-mtv-h/pixel5/device_gnss.csv` — missing | `output/smartphone-r5/phase37-pixel5-repeatability-v1/routes/2021-08-24-20-32-us-ca-mtv-h/pixel5/inputs/device_gnss.csv` — regular file, 74,299,283 bytes | 74,299,283 / `46482b82db0992c1f063dbd9cf697268605234d3e38bcbd23525fd4b60bc17a7` |
| `2021-08-24-20-32-us-ca-mtv-h/pixel5` / `device_imu.csv` | `raw/phase93/2021-08-24-20-32-us-ca-mtv-h/pixel5/device_imu.csv` — missing | `output/smartphone-r5/phase37-pixel5-repeatability-v1/routes/2021-08-24-20-32-us-ca-mtv-h/pixel5/inputs/device_imu.csv` — regular file, 54,271,475 bytes | 54,271,475 / `fa3f17d07570fdbb8030f307130f33ca3613e8125475c2a907c48f7db3455480` |
| `2021-08-24-20-32-us-ca-mtv-h/pixel5` / `brdc.nav` | `raw/phase93/2021-08-24-20-32-us-ca-mtv-h/pixel5/brdc.nav` — missing | `output/smartphone-r5/phase37-pixel5-repeatability-v1/routes/2021-08-24-20-32-us-ca-mtv-h/pixel5/inputs/brdc.nav` — regular file, 10,805,490 bytes | 10,805,490 / `147d948f0eba3bf09e295e7f67fbe8db60c25e236bc3c7d958dc933483f10909` |
| `2022-04-01-18-22-us-ca-lax-t/pixel5` / `device_gnss.csv` | `raw/phase93/2022-04-01-18-22-us-ca-lax-t/pixel5/device_gnss.csv` — missing | `output/smartphone-r5/phase37-pixel5-repeatability-v1/routes/2022-04-01-18-22-us-ca-lax-t/pixel5/inputs/device_gnss.csv` — regular file, 33,837,317 bytes | 33,837,317 / `50362c01bff3e0bb7088e54021164591cd750227ed97c2fd7d95d763a08798f1` |
| `2022-04-01-18-22-us-ca-lax-t/pixel5` / `device_imu.csv` | `raw/phase93/2022-04-01-18-22-us-ca-lax-t/pixel5/device_imu.csv` — missing | `output/smartphone-r5/phase37-pixel5-repeatability-v1/routes/2022-04-01-18-22-us-ca-lax-t/pixel5/inputs/device_imu.csv` — regular file, 23,649,244 bytes | 23,649,244 / `2e39a3e9f294c64b8ecfd452d0960025d1013b97f2d7497e6e48a2a1997b38c5` |
| `2022-04-01-18-22-us-ca-lax-t/pixel5` / `brdc.nav` | `raw/phase93/2022-04-01-18-22-us-ca-lax-t/pixel5/brdc.nav` — missing | `output/smartphone-r5/phase37-pixel5-repeatability-v1/routes/2022-04-01-18-22-us-ca-lax-t/pixel5/inputs/brdc.nav` — regular file, 10,635,773 bytes | 10,635,773 / `443d3d5a73f4895b83e576e24a568f4658f869e79a480856de7f0717763dcbe6` |
| `2023-03-08-21-34-us-ca-mtv-u/pixel5` / `device_gnss.csv` | `raw/phase93/2023-03-08-21-34-us-ca-mtv-u/pixel5/device_gnss.csv` — missing | `output/smartphone-r5/phase37-pixel5-repeatability-v1/routes/2023-03-08-21-34-us-ca-mtv-u/pixel5/inputs/device_gnss.csv` — regular file, 23,385,520 bytes | 23,385,520 / `a0fc8e71bdfc03be61b99efcd7d41fbba8ffec126df78b55243f681fd211f204` |
| `2023-03-08-21-34-us-ca-mtv-u/pixel5` / `device_imu.csv` | `raw/phase93/2023-03-08-21-34-us-ca-mtv-u/pixel5/device_imu.csv` — missing | `output/smartphone-r5/phase37-pixel5-repeatability-v1/routes/2023-03-08-21-34-us-ca-mtv-u/pixel5/inputs/device_imu.csv` — regular file, 17,453,338 bytes | 17,453,338 / `c7d726e1cc0dacc7a569bd8be9bc2333765bbb3b3010203a4aea2e49778101ab` |
| `2023-03-08-21-34-us-ca-mtv-u/pixel5` / `brdc.nav` | `raw/phase93/2023-03-08-21-34-us-ca-mtv-u/pixel5/brdc.nav` — missing | `output/smartphone-r5/phase37-pixel5-repeatability-v1/routes/2023-03-08-21-34-us-ca-mtv-u/pixel5/inputs/brdc.nav` — regular file, 11,506,619 bytes | 11,506,619 / `8893ef62fecdd9986f6b2cf1b7b980defa6430da5801aafcab4ecf4c99a03b92` |

The three actual files for each route are the exact three names required by
the Phase94 raw-input contract: Android raw GNSS, Android raw IMU, and
broadcast navigation. There is no evidence of a missing data body in the
filesystem. Content identity is not independently recomputed here because
that would read raw bytes; the sealed Phase91 SHA pins and the successful
Phase93 native raw-read accounting are retained as provenance.

## Manifest schema and argv expansion

### Phase93

The Phase93 manifest has schema
`smartphone-r5-phase93-source-staging-clock-state-handoff-execution-manifest.v1`.
Its four route records deliberately carry role-only
`raw/phase93/<route>/pixel5/{device_gnss.csv,device_imu.csv,brdc.nav}`
placeholders with `sha256: null` and `read_at_manifest_creation: false`.
This alone does not identify the runtime path. At execution, the pinned
Phase93 runner loads the Phase91 manifest, whose schema is
`smartphone-r5-phase91-source-clock-c0d-gnss-first-raw-drift-d-initializer-execution-manifest.v1`
and whose route records contain the actual `output/...` paths and sealed SHA
pins. For each route, the runner checks the manifest placeholder and then
performs these exact substitutions:

```text
--android-gnss <Phase91 device_gnss.csv path>
--android-imu  <Phase91 device_imu.csv path>
--nav         <Phase91 brdc.nav path>
```

The Phase93 sealed route metadata records those substituted command paths,
not the placeholders.

### Phase94

The Phase94 manifest has schema
`smartphone-r5-phase94-source-clock-c0d-stage-diagnostics-execution-manifest.v1`.
It also carries role-only placeholders with `sha256: null` and
`read_at_manifest_creation: false`, but its wrapper does not load the Phase91
manifest. The evaluator's `_validate_command()` explicitly checks each
command argument against the Phase94 record's placeholder, and the wrapper
returns the unchanged command. Thus all four native argv lists contain:

```text
--android-gnss raw/phase93/<route>/pixel5/device_gnss.csv
--android-imu  raw/phase93/<route>/pixel5/device_imu.csv
--nav         raw/phase93/<route>/pixel5/brdc.nav
```

The result route metadata confirms these exact arguments for all four routes.
The Phase94 wrapper then launches from `cwd=ROOT`, where those paths are
absent.

## Sealed log/result cross-check

Phase93's sealed result reports four native invocations, four raw GNSS reads,
four raw IMU reads, four broadcast-navigation reads, zero evaluator raw reads,
zero truth/MAT/base/precomputed-coordinate/Kaggle reads, and zero accuracy
calculations. Its no-go status is due to structural solver gates, not input
conversion. The four Phase93 stderr files contain post-input structural
diagnostics (or one native C0/D exception), with no missing-file error.

Phase94's sealed result reports, for each route, return code `1`, no summary,
four GNSS route arguments, four IMU route arguments, four navigation route
arguments, zero runner raw-byte reads, zero forbidden-lane reads, and one
native invocation. Each preserved stderr is the same fail-closed pattern:

```text
failed to convert raw Android GNSS: failed to open raw Android GNSS CSV: raw/phase93/<route>/pixel5/device_gnss.csv
```

The error occurs before problem construction, GNSS-first staging, or main
graph telemetry. It is therefore consistent with the metadata-only absence
checks and the unchanged placeholder argv.

## Candidate comparison and freeze choice

| candidate | assessment |
| --- | --- |
| Inherit the pinned Phase91 route map in the Phase94 wrapper and substitute exact actual paths after the existing placeholder validation | **Selected; unique minimal correction.** It matches the already successful Phase93 resolver, preserves the Phase94 command flags/evaluator contract, and changes no native algorithm. |
| Create `raw/phase93` symlinks/copies to the existing output files | Rejected. It mutates/stages the data namespace, does not fix the pinned wrapper contract, and would make a path regression invisible. |
| Replace each Phase94 manifest placeholder with a hardcoded `output/...` path | Rejected. It duplicates the sealed Phase91 route mapping, changes the Phase94 manifest/evaluator pins, and is a broader contract rewrite than reusing the proven resolver. |

The selected candidate remains default-off and is not an execution
authorization. A future implementation must pin the inherited Phase91
manifest, corrected wrapper, tests, manifest, authorization, and a fresh
output root before any new raw run. Native solver/filter/LM flags, C0/D
equations, units, sigma, fail-closed behavior, and no-fallback policy remain
unchanged.

## Existing Phase94 authorization: cannot be reused

Reuse is **NO-GO** for both hash and policy reasons:

1. The authorization's `commands_source` is the exact command list in the
   pinned Phase94 manifest; those commands contain the missing placeholders.
2. The authorization pins the Phase94 wrapper SHA
   `fa124b79f9119c20b7396977c3feecefa3e0410d64dfa4b0ec5212ac5d55194d` and
   the manifest SHA
   `4a36a4871c305f66c419db6a80ca4c2a932a37e0522cf8dc688fd2792df7d3ed`.
   A corrected wrapper or command materialization would not be those pins.
3. The policy says `no_rerun: true`, one invocation per route, and stop after
   four routes. All four invocations have already been consumed and their
   fail-closed logs/result are sealed.

Accordingly, no existing authorization is extended, amended, or replayed by
this audit. The separate freeze artifact records the only candidate and
keeps `raw_execution_authorized: false`.
