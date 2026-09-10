# Phase119 official route-Type provenance audit

Status: sealed read-only audit.  The scope is the official GSDC2023 MATLAB
call graph, its settings/list provenance, the tracked native Phase118 route
crosswalk, and sealed metadata only.  No new raw phone GNSS/IMU/navigation or
base read, truth/MAT read, solver invocation, accuracy evaluation, solution
read, precomputed-coordinate read, PDC access, Kaggle access, or k sweep was
performed.

## Decision

The route-specific official Type is established for both Phase118 primary
routes:

| exact official dataset row | Course | Phone | official `setting.Type` | TDCP Huber k |
| --- | --- | --- | --- | ---: |
| MTV-A | `2021-03-16-18-59-us-ca-mtv-a` | `pixel5` | `Highway` | `0.5` |
| LAX-T | `2022-04-01-18-22-us-ca-lax-t` | `pixel5` | `Highway` | `0.5` |

The mapping is not inferred from the route name, a truth score, or a native
result.  The authoritative row provenance is the sealed Phase80
`public_settings_huber.by_route` record.  It identifies its source as the
pinned public `dataset_2023/settings_train.csv`, pins the settings member
SHA-256 `3e6ae65388b2809088b16732b87744e673f860c24a1fe0f709ef903a87397f39`,
and records `route_mapping_fixed_before_read=true`, `route_score_selector=false`,
and `route_score_or_truth_selection=false`.  The original settings payload is
not reopened in Phase119; the official cache intentionally excludes the
`dataset_2023` directory.  The prior sealed source-metadata read is therefore
the admissible list evidence, and its read accounting is retained below.

Accordingly, the Phase118 assumption of `Highway` for both MTV-A and LAX-T is
consistent with the official route list and with the official code path.  The
official `parameters.m` branch maps any Type other than `Street`/`Mix` to
`L_robust_prm=0.5`; the native Phase118 resolver maps the exact string
`Highway` to `0.5`.  No Phase119 numerical change is justified.

## Exact official call path

The official source is row-driven rather than route-name-driven:

1. `output/reproducibility-cache/gsdc2023/run_fgo.m:12-20` chooses the
   `train` or `test` dataset root and reads
   `./dataset_2023/settings_"+dataset+".csv`.  Its checked-in default is
   `dataset="test"`; the same row contract applies to the training settings
   list that contains the two audited routes.
2. `run_fgo.m:29-32` selects `setting=settings(i,:)` and constructs the
   trip path as `datapath+setting.Course+"/"+setting.Phone+"/"`.  Thus the
   exact route key is the `Course/Phone` pair shown in the table above; the
   `Type` field is carried by that same row.
3. `run_fgo.m:34-55` passes that row to the optional GNSS-first
   `fgo_gnss(datapath,setting,true)`, the first `fgo_gnss_imu` pass, and the
   final `fgo_gnss_imu(datapath,setting,false)`.  There is no separate
   route-name or score lookup in this driver.
4. `fgo_gnss.m:18-32` and `fgo_gnss_imu.m:18-38` read `setting.Course` and
   `setting.Phone`, then call `parameters(setting,initflag)` with the
   unchanged row.  Their graph loops use `FTYPE=["L1","L5"]`.
5. `parameters.m:12-20` copies `setting.Type` into `prm.Type`.  Its
   `parameters.m:77-83`, `:99-108`, and `:119-125` branches use the same
   `setting.Type` to create the P, Doppler, and carrier/TDCP Huber kernels.
   In particular, `Street` or `Mix` selects `0.2` for `prm.L_robust_prm`,
   and all other official Types, including `Highway`, select `0.5`.
6. `fgo_gnss.m:132-148` and `fgo_gnss_imu.m:202-221` attach `prm.P_kernel`
   and `prm.D_kernel` to the Pseudorange/Doppler factors.  Their ordinary
   TDCP blocks at `fgo_gnss.m:179-200` and `fgo_gnss_imu.m:301-321` attach
   the single run-level `prm.L_kernel` to every retained adjacent pair.

The official cache's source Git tree contains the driver and functions but
not `dataset_2023/settings_train.csv` or `settings_test.csv`: `.gitignore`
contains `/dataset_2023`.  Therefore the source text alone cannot enumerate
the target rows.  The sealed Phase80 settings-list provenance is what closes
that otherwise missing list edge; without that pinned row record, this audit
would be `unknown` and fail closed.

## Cross-check against Phase118 native admission

The native route table is not used as the primary official evidence.  It is
checked only for exact agreement with the sealed settings-list result:

* `apps/native/gnss_fgo_imu_no_base.cpp:375-397` matches exact
  `Course/Phone` dataset IDs and stores the environment string.
  Both audited IDs map to `Highway`; no route score or truth field is read.
* `apps/native/gnss_fgo_imu_no_base.cpp:7716-7730` passes the matched
  environment to the Phase118 option and explicitly retains fixed
  `tdcp_sigma_m=0.03` while disabling the Phase117 dynamic sigma path.
* `include/libgnss++/algorithms/fgo_config.hpp:1824-1853` accepts only
  `Street`, `Mix`, and `Highway`: the first two resolve to `0.2`, `Highway`
  resolves to `0.5`, and empty/unrecognised Type fails closed.
* `src/algorithms/fgo.cpp:120-126` and
  `src/algorithms/fgo_gtsam_backend.cpp:46-52,1224-1226` reject an unknown
  Type and pass the resolved scalar only to ordinary TDCP noise construction.

The sealed Phase118 structural manifest/result also records `Highway` and
`0.5` for both routes.  That is execution metadata confirming the selected
configuration, not independent evidence for the official Type; the independent
evidence is the pinned settings-list record above.

## Candidate freeze boundary

Exactly one default-off candidate is frozen for any future implementation or
execution contract:

* Candidate ID: `phase119-official-route-type-provenance-gated-tdcp-k-v1`.
* Change one thing only: source `official_tdcp_setting_type` from the pinned
  authoritative settings row keyed by exact `Course/Phone`, and admit the
  Phase118 TDCP-k resolver only after one unique row has been proven.  For
  MTV-A and LAX-T this produces `Highway` and `k=0.5`, so the expected numeric
  Phase118 recipe is unchanged; this is a provenance/gating correction, not a
  truth-tuned numerical correction.
* Preserve fixed ordinary TDCP sigma `0.03 m`, official mapping
  `Street/Mix=0.2` and `Highway/other official Type=0.5`, ordinary TDCP
  residual/equation/units/pair order, all reject predicates, C7/D/C0D,
  raw-base, Pixel5 offset, QR, IMU, filter, initialization, and LM settings.
* Require exactly one matching authoritative row.  Missing, duplicate,
  conflicting, malformed, or unrecognised Type is `unknown` and aborts before
  solver construction.  No route-name heuristic, score/truth lookup, default
  Highway fallback, Type/k sweep, or retry is permitted.
* The candidate remains opt-in/default-off, raw-only, solution-withholding,
  and is not implementation or execution authorization.  Phase119 authorizes
  no raw, base, solver, truth, accuracy, or Kaggle action.

## Next official source-backed parity priority

Priority 1 (audit only; not frozen as a second candidate): **ordinary TDCP
source-domain equation/normalization parity**.

The official representation is explicit: `functions/gnsslog2obs.m:182-188`
stores accumulated delta range as cycles (`ADR_m/lambda`) and Doppler as
`-pseudorange_rate_mps/lambda`; `functions/exobs_residuals.m:89-99` forms the
wavelength-scaled carrier/Doppler difference; and both official FGO functions
form `resL(i+1)-resL(i)` before calling the custom TDCP factor.  Native
`src/algorithms/fgo_problems.cpp:1446-1452` constructs a corrected
metre-valued carrier delta, while
`src/algorithms/fgo_gtsam_internal.hpp:3403-3407,3497-3500` evaluates the
range/clock difference against that metre delta.  The high-level adjacent
family agrees, but the official custom-factor source and native cycles-to-
metres/sign/endpoint normalization have not been proven line-by-line in this
audit.  This is the next source-backed parity question; it does not authorize
changing the equation, sigma, admission, or robust kernel.

## Evidence pins

| item | SHA-256 or commit | role |
| --- | --- | --- |
| Official `run_fgo.m` | `02150849653b2e5c80609e971dbef41b010569de5dcc304782235b7fc96b9065` | settings read and FGO calls |
| Official `fgo_gnss.m` | `5368e4056f70f448b728792c5e4c124b7f2afc1e00653492b807083bd3ccf0c3` | GNSS graph and TDCP call |
| Official `fgo_gnss_imu.m` | `c70090ccb8b27fc8ac7fd2929e2f995a14cfe7f089bb1fef067370e22051c3e3` | GNSS/IMU graph and TDCP call |
| Official `parameters.m` | `518925e9c75c7a14fceb5cd99432fe883311e0d64c72b94396744ac765120f52` | `Type` propagation and Huber map |
| Official `gnsslog2obs.m` | `665ce43d1fc3c5a9c3b3a6fd76f81539126e3ab897fbba484ab0ccd18964acff` | cycles/metres source representation |
| Official `exobs_residuals.m` | `50c954a825edbdeaf5c9884486aff56c058f51d47610a06722d4f42b33095324` | source TDCP normalization gate |
| Phase80 source/list freeze | commit `57cf769`; SHA `9ebdde21fe7fd955029243d3bdbba3a9f9d3832388d67480760394177c8f685e` | sealed settings-list route/type provenance |
| Phase118 source-parity freeze | commit `5fcc06ab64189dd5dfb8001664bdb8496f85224f`; SHA `c6f4fda2abe170417610d4fd1a4ae8a1a618d07096ff0432669081ab06a1cf77` | fixed sigma/k candidate anchor |
| Phase118 structural result | commit `b4ea80d`; SHA `88e8799050fd396eeb14e83d4f5923339279f46d0219a385e303d3cb50731538` | sealed configuration metadata only |
| Native route admission | `apps/native/gnss_fgo_imu_no_base.cpp` SHA `43e9fbbe8a0043f648b52b02e63be36b8960e412a989d21cfd345c8b8a9312d5` | independent exact cross-check |
| Native Type resolver | `include/libgnss++/algorithms/fgo_config.hpp` SHA `38ae28a5bdb398b0a764f9f09f8456107ddc0168654b1d1791879377cca24748` | fail-closed k mapping |

## Read accounting

| activity | Phase119 count/status |
| --- | ---: |
| Official MATLAB source/list text or sealed metadata reads | source/metadata only |
| New raw phone GNSS/IMU/navigation payload reads | `0` |
| New raw-base bytes, headers, or hashes | `0` |
| Truth payload reads | `0` |
| MAT reads or generated MAT | `0` |
| Precomputed phone coordinates/trajectories | `0` |
| Solver/native process invocations | `0` |
| Accuracy calculations or score reads | `0` |
| Solution rows opened/published | `0` |
| PDC, Kaggle, or token access | `0` |
| Type/k sweep, rerun, fallback, tuning | `0` |

The audit is source/list provenance only.  The companion freeze must retain
the exact audit hash and keep every execution/accuracy/publication authority
false.
