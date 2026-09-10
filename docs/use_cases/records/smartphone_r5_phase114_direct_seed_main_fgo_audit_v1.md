# Phase114 direct-seed main FGO read-only audit

- Execution label: `Luna Max`
- Audit date: `2026-09-03`
- Status: `sealed-read-only-audit-one-candidate-detailed-freeze`
- Scope: sealed Phase80/82/85/92/99/112/113 records, the official
  `fgo_gnss.m`/`fgo_gnss_imu.m` source, and the native direct-WLS/FGO source.
- Current source tip: `1f64b248b2e998859479587250e8efdb2dea2eed`.

This is a source and sealed-aggregate audit.  It did not open raw phone GNSS,
raw phone IMU, broadcast-navigation, or base-RINEX payloads; it did not read a
truth or MAT payload, invoke the native solver or an evaluator, inspect a
solution row, access PDC/precomputed coordinates/Kaggle, or rerun a route.
Phase82 scalar scores below are copied sealed aggregates; no truth read or
metric recalculation was performed here.

## Decision

Exactly one default-off candidate is frozen in the companion JSON:

`phase114-main-direct-wls-ephemeral-c7d-seed-v1`.

The candidate is source-motivated but **not implemented and not execution
authorized**.  The existing native direct-WLS path is a velocity/clock-rate
initializer only.  It deliberately bypasses GNSS-first, leaves position and
receiver-clock copying at zero, and supplies only an ENU velocity sequence to
IMU construction.  It has no seven-component epoch-local C initializer and it
does not perform the Phase101 raw-D key alignment.  The current C7 mapping
therefore remains an explicit implementation boundary, not an inferred
zero-fill.

The single future candidate would, in one process and from the same raw
observations, bypass the GNSS-first optimizer and construct an ephemeral main
seed consisting of:

1. same-run raw SPP position and scalar receiver-clock seeds;
2. the existing direct Doppler WLS velocity and clock-rate result where its
   exact retained epoch has a valid solve;
3. an explicit official seven-component epoch-local C vector with exact
   constellation/frequency mapping; and
4. the retained raw `EpochSeed.receiver_clock_drift_mps` D sequence, aligned
   by raw identity rather than by vector position.

It would then hand that seed directly to the existing meter-state C0D
Pose3+IMU graph, retaining Phase99 main `MULTIFRONTAL_QR`, Phase107/109 raw
base code correction, and the Phase112 final Pixel5 position offset.  The
candidate changes no factor, equation, unit, sigma, filter, LM schedule,
ordering, or fallback policy.  Legacy/default and the GNSS-first path remain
unchanged.  A missing C7 mapping, missing retained key, missing WLS row,
nonfinite value, or unsupported route must fail closed.

## Sealed evidence

| Authority | MTV-A | LAX-T | Relevant sealed observation |
| --- | ---: | ---: | --- |
| Phase80 comparator, as sealed by the Phase99 audit | main `12` accepted; cost `155976340.16237149 -> 28128.450524519692` | main `12` accepted; cost `190528071.24200463 -> 22206.857308543094` | no-C0D/legacy-era main graph progressed |
| Phase82 direct-quality aggregate | score `1.1139384500152307`, prediction `2158/2159` | score `0.9389644134001871`, prediction `1465/1465` | stored scalar aggregate only; not recomputed in this audit |
| Phase85 source-C0D manifest | pre-QR C0D recipe; default off | pre-QR C0D recipe; default off | CCDD contract was still the seconds-valued form |
| Phase92 structural result | all route gates false | all route gates false | no-go before a usable sealed C/D/main result |
| Phase99 QR structural result | GNSS-first `167` accepted, cost `5618000.786274777 -> 21298.114523521894`; main QR `12` accepted, cost `79358354.39651252 -> 34843.513194108804` | GNSS-first `262` accepted, cost `6419784.856579111 -> 12818.288538454106`; main QR `12` accepted, cost `166205998.11910567 -> 20384.75538283932` | full finite D and exact handoff; main QR structural GO |
| Phase112 offset structural result | stage `218` accepted, cost `4516236.0222890135 -> 16097.714277915162`; main QR `12`, cost `130723525.58263575 -> 29729.5954122218`; output `2159` | stage `234` accepted, cost `6412859.293244721 -> 12298.593335217001`; main QR `12`, cost `162184507.52202186 -> 19851.9725625441`; output `1466` | raw-base/QR/offset exactly-once structural GO |

The Phase99 and Phase112 main results show that A/LAX already have a finite,
progressing same-run GNSS-first-to-main path.  A direct seed could avoid a
future stage-to-main regression, but no direct-seed result exists in the
sealed records, so improvement is only a hypothesis.

### H and U boundary

The sealed Phase113/Phase95 diagnostics classify the remaining routes before
any direct-seed claim can be made:

| Route | Retained epochs | Raw D | Undifferenced Doppler factors | Earliest boundary |
| --- | ---: | ---: | ---: | --- |
| MTV-H | `1247` | `1247/1247` finite | `0` | C0D admission guard; no WLS row set, no native velocity/D state path |
| MTV-U | `720` | `720/720` finite | `4845`; `671` C0D factors | GNSS-first progressed (`1000` accepted, `23108268994.98668 -> 82026870.12520209`), but main accepted `0`; `564/720` positions were Earth-valid |

MTV-H cannot use the existing direct WLS: the WLS builder consumes only
undifferenced-Doppler factors and the solver requires at least four rows per
epoch.  Removing the C0D guard would not create velocity or D states.  The
official MATLAB first pass can form velocity from `posbl.gradient(obs.dt)`,
but deriving that native replacement would be a new state-initialization path,
not existing direct WLS and not a guard-only change.

MTV-U has Doppler support, but full direct-WLS coverage is not established.
The direct validator rejects any invalid estimate and requires one valid
estimate for every already-retained `problem.epochs` entry.  Phase82's
`1101` prediction rows are a sealed output-domain count, not proof that the
current Phase101 C0D problem retains `1101` seed epochs; the Phase113 U
diagnostic observed only `720` retained/problem epochs against an expected
`1102` output domain.  The candidate must therefore require exact full
retained-key coverage and cannot pad, truncate, interpolate, hold, or infer
the missing epochs.

## Source audit

### Official two-stage state contract

The official `fgo_gnss.m` initializes/reloads position, velocity, `clk`, and
`dclk` (`:34-48`), inserts `x`, `v`, seven-component `c`, and `d`
(`:89-121`), adds CCDD (`:170-176`), and exports both `clkest` and `dclkest`
with position/velocity (`:220-229`).  `fgo_gnss_imu.m` loads that GNSS result
(`:40-61`), inserts Pose3/position/velocity/clock/drift/bias values
(`:165-187`), adds CCDD (`:270-276`), and exports `dclkest` again.  This
supports the concept of a same-run direct seed, but the MATLAB source does
not define the native direct-WLS C7 adapter or explicitly request QR.

### What native direct WLS actually produces

`doppler_velocity_wls::ObservationRow` contains only a LOS, a Doppler
residual, and sigma.  Its design row is `[los_x, los_y, los_z, 1]`, so the
four solved unknowns are ECEF velocity and one scalar clock rate.  The
`Estimate` type has `velocity_ecef_mps`, `clock_rate_mps`, covariance and
quality gates, but no position, clock-bias, or C[0..6] field
(`include/libgnss++/algorithms/doppler_velocity_wls.hpp:24-75,88-105`).

`fgo_problems.cpp:1109-1154` builds those estimates exclusively from
`problem.undifferenced_doppler_factors`.  Its bounded completion operates on
that estimate vector; it does not create an epoch-local clock vector.

The app's `validateDirectDopplerWlsHandoff` checks the already-existing raw SPP
positions, validates the WLS estimates, converts valid velocity states to ENU,
and explicitly sets `direct_doppler_wls_positions_clocks_copied = 0`
(`apps/native/gnss_fgo_imu_no_base.cpp:2361-2383,2482-2519`).  The direct branch
returns before GNSS-first (`:7054-7070`), and later `velocity_handoff` points
only at the direct WLS velocity vector (`:7443-7450`).  Thus the current path
can leave same-run raw SPP position/scalar-clock entries in `problem.epochs`,
but it does not generate a position/clock/C7 result from WLS and does not
handoff raw D by the Phase91 exact-key contract.

The WLS `clock_rate_mps` is also not the raw
`EpochSeed.receiver_clock_drift_mps`: the backend uses it only when the
generic `use_doppler_velocity_wls_initialization` branch is selected
(`src/algorithms/fgo_gtsam_backend.cpp:484-500`).  The Phase101 raw-D vector
must instead be populated from retained raw epochs and validated separately.

### C7 and D are currently GNSS-first-only handoff products

`EpochSeed` contains a scalar `receiver_clock_bias_m`, a raw
`receiver_clock_drift_mps`, and raw identity fields, but no seven-component C
array (`include/libgnss++/algorithms/fgo.hpp:40-64`).  `FGOResult` exports
`epoch_clock_drift_mps` and `epoch_clock_bias_components_m` only after the
source-meter graph has optimized its D/C keys
(`include/libgnss++/algorithms/fgo.hpp:1650-1680`; backend
`src/algorithms/fgo_gtsam_backend.cpp:1786-1840`).

The existing main backend accepts a C vector only when the handoff vector has
exact size and finite values; otherwise the active Phase101 path returns
without a scalar/global-ISB fallback (`src/algorithms/fgo_gtsam_backend.cpp:
147-177`).  `ensureBaseClock` uses the handoff vector for all seven entries;
only the non-handoff path initializes component zero from the scalar clock and
the remaining components to zero (`:730-761`).  The
`sourceClockComponentFor` helper (`src/algorithms/fgo_gtsam_internal.hpp:
1302-1329`) maps factor rows to C7 components, but is not an initializer or a
raw WLS-to-C7 conversion.

Therefore “C7 raw initialization mapping” is the missing, source-visible
boundary.  Reusing the non-handoff zero values would silently weaken the
Phase101 topology and is not admitted by this freeze.

### QR, base, offset, and legacy separation

Phase99 selects QR only for the existing Pose3+IMU meter-state graph when
`use_imu`, C0D meter parity, the GNSS-first meter handoff, and the non-raw-D
main handoff predicates all hold (`src/algorithms/fgo_gtsam_backend.cpp:44-60`).
The selector changes only the existing LM parameter's supported
`MULTIFRONTAL_QR` enum (`src/algorithms/fgo_gtsam_internal.hpp:119-130`; backend
`:1621-1652`).  GNSS-first remains on Cholesky.  Phase85 and Phase92 records
contain no Phase99 QR selector, so their pre-QR/zero-step behavior cannot be
used as a direct-seed QR comparison; Phase99 is the first sealed A/LAX main QR
structural GO.

The CLI contract currently rejects direct WLS together with the C0D handoff
(`apps/native/gnss_fgo_imu_no_base.cpp:790-802,826-832,886-893`).  The exact
Phase107/109 base admissions are nested under the Phase101 selector recipe
(`:895-928`), and Phase112's Pixel5 offset admission is also tied to that
recipe.  Consequently the requested composition is not an existing command
combination: it requires one new, explicitly scoped direct-seed selector and
the C7/raw-D adapter described above.  No such implementation is added by
this audit.

### A/LAX stage-regression hypothesis

Facts:

- Phase99/112 show finite GNSS-first progress, full C7/D handoff, QR main
  progress, and exact output coverage for A/LAX.
- The existing direct-WLS path bypasses GNSS-first and leaves position/clock
  copy count zero while handing only velocity to IMU construction.
- No sealed run combines direct WLS with C7, raw D, Phase109 base, QR, and the
  final Pixel5 offset.

Inference, not an observed result: bypassing the GNSS-first output could avoid
a contaminated stage-to-main position/clock seed and might reduce a stage
regression.  The same bypass also replaces a proven full C7/D handoff with an
unimplemented C7 mapping and an unproven retained-epoch set.  A/LAX therefore
qualify the candidate for a future structural experiment, not for promotion.

## Single candidate boundary

The companion JSON freezes exactly one default-off candidate with these
requirements:

1. Same-process ephemeral inputs only: raw phone GNSS/IMU and broadcast nav;
   if a later raw-base authorization composes Phase109, use only its sealed
   raw-base RINEX/station metadata path.  Never read a saved result, PDC,
   MAT, truth, precomputed coordinate, or external seed.
2. Do not invoke the GNSS-first optimizer for this candidate.  Preserve the
   raw SPP position/scalar-clock values only as same-run `EpochSeed` state,
   and use direct WLS only for its finite velocity/clock-rate output.
3. Add an explicit raw-D adapter from
   `EpochSeed.receiver_clock_drift_mps`, keyed by exact raw source identity and
   retained order.  Do not substitute the WLS clock rate for raw D.
4. Add and validate an official C7 initializer in exact component order
   `[base_gps_l1, glo_l1, gal_l1, bds_l1, gps_l5, gal_l5, bds_l5]` with no
   global ISB double state.  Every retained epoch must have seven finite
   components; missing/unsupported mapping fails closed.
5. Hand position, velocity, scalar clock, C7, and D into the existing
   meter-state C0D Pose3+IMU graph with Phase99 QR, Phase109 correction, and
   Phase112 final-only Pixel5 offset.  Keep official CCDD
   `(C2-C1)-((D1+D2)*dt/2)`, C/ISB metres, D metres/second, `dt` seconds,
   sigma `0.1 m`, filters, priors, initialization conventions outside this
   seed, LM lambda/iteration/ordering, and no-fallback policy unchanged.
6. Structural qualification must record exact retained-key coverage, finite
   C7/D/position/velocity, QR selection, accepted main iterations, strict
   cost decrease, finite Earth-valid expected output, exactly-once base and
   offset behavior, and no fallback.  H is an expected diagnostic rejection
   until a separately justified non-Doppler velocity source exists; U must
   prove full retained seed coverage rather than equating Phase82's 1101
   output rows with the current 720-epoch C0D problem.

Rejected as separate candidates: removing only the H C0D guard, deriving
velocity from a position gradient, zero-filling C7, using WLS clock rate as
raw D, changing retained-epoch filtering, sequential QR/ordering changes,
Cholesky fallback, LM/sigma tuning, and any PDC or saved-coordinate bridge.
Those are either unsupported by the sealed evidence or materially different
algorithms.

## Read accounting

| Activity in this audit | Count |
| --- | ---: |
| Sealed record/source text reads | nonzero; aggregate/source text only |
| Raw phone GNSS/IMU/nav payload reads | `0` |
| Raw-base payload/header reads | `0` |
| Truth payload reads | `0` |
| MAT binaries read or generated | `0` |
| Native solver/evaluator invocations | `0` |
| Accuracy recalculation or route scoring | `0` |
| PDC/precomputed-coordinate reads | `0` |
| Kaggle/token access | `0` |
| Route reruns/fallbacks | `0` |
| New solution/large artifacts | `0` |

## Pinned authorities

| Authority | Commit or SHA-256 |
| --- | --- |
| Phase80 freeze | `57cf7698c36256a535ac388cbc45dd93afecc9bb`; `9ebdde21fe7fd955029243d3bdbba3a9f9d3832388d67480760394177c8f685e` |
| Phase80 structural manifest | `6d20541560b726ed2409cd05902e5105874ac72e113af0c3812f029130df310f` |
| Phase82 sealed aggregate result | commit `0b2b210f371cd1c8246b25ad7d30b86cd9d37478`; `39c5a38b7e3e61fda0306582fffb961dad3e6fb0bd49fcfe70e66674e2c90873` |
| Phase85 C0D manifest | `08112a33b9aeee5361aa0dfe547ffdb970197200`; `20adc2e3388b92d045580739d025106c18aa068fcf26f8d117d48d3d2f5f7058` |
| Phase92 sealed result | commit `3008b299565346132c76867410a3a2fb6c8d65c6`; `8488306b3ec61d0360418de73fa1596271597b6971d77de676fcd279d7e1b01c` |
| Phase99 QR audit/freeze/result | audit `0e10a62d1fbec6d3cab8d2faed51d02c9b538f8726ba177bb8a5fece50e8e498`; freeze `4e9819d172c4c39842f04d6202c815c9631041320f0b785e2cb2d6a67edf4c27`; result `b5a786e87084bc83eed8a2181f2f31c1199e4c833d30449e14f1e42422b741c9` |
| Phase112 sealed structural result | commit `dfa9c3a186c9d8633dc572192b513b584f18800a`; `087b1bf51b4e584b86a7b558560df4ceb78e19b01e572f2d60aeef4c3d759ba0` |
| Phase113 sealed result | commit `1f64b248b2e998859479587250e8efdb2dea2eed`; `fa80a3480162b73faa4a8f778fe2aa4dac28510ee2d3b1d4e8351c34faba14d1` |
| Official `fgo_gnss.m` | `5368e4056f70f448b728792c5e4c124b7f2afc1e00653492b807083bd3ccf0c3` |
| Official `fgo_gnss_imu.m` | `c70090ccb8b27fc8ac7fd2929e2f995a14cfe7f089bb1fef067370e22051c3e3` |
| Native app | `268bd030496bb677a42d3400e20ec37bc8e36589827fb28fbf57e3e398a1b1be` |
| Native GTSAM backend | `781d65e34826ec08e04d1dfdac0ab10b2ed9409726078465f3020b57afac4baa` |
| Native GTSAM internal header | `cc6327e36800afa8217e5a834260adc81c0a74a5aefbfc956bb4679c8d86105f` |
| `FGOResult`/`EpochSeed` header | `f13d743225184d5ed4b229f473fb69b290f61aa83398753bdb2d6a95ceeb2e05` |
| Native WLS problem builder | `e7607ce8f0fe27fd382a01b5b6f147b027b7bf51453e920eae458c86ddac0c17` |
| Native WLS contract header | `ef888b3e6c2f15d0dd035c0905bafdee463e346c549f81cf892ec30c1c4dcde0` |

This record freezes a design boundary only.  It authorizes no implementation,
raw execution, truth evaluation, accuracy promotion, or submission.
