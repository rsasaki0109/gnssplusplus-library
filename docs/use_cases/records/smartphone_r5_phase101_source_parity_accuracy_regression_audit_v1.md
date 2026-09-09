# Phase101 source-parity and accuracy-regression audit

- Execution label: `Luna Max`
- Audit date: `2026-09-03`
- Status: `sealed-read-only-audit-one-candidate-qualified`
- Scope: the official GSDC2023 MATLAB source and the native Phase99/100
  implementation/records.  Accuracy evidence is limited to sealed aggregate
  metadata from Phase82 and Phase100; the Phase100 solution and truth files
  were not reopened.
- Candidate scope: one future raw-only, same-run native candidate; no MAT,
  precomputed coordinate, PDC, base, or Kaggle input; production default
  remains unchanged.

This audit compares source text and sealed metadata only.  It did not execute
the solver, read raw device or navigation bytes, read a MAT payload, read a
base or precomputed-coordinate artifact, reopen a Phase100 solution/truth
file, calculate a new score, or access Kaggle/tokens.  The companion freeze
JSON is a design boundary, not an implementation or execution authorization.

## Decision

The most explanatory *allowed* unported/misported source contract is the
receiver clock/ISB state representation:

> The official graph has a 7-component `C_i` vector at every retained epoch
> (base receiver clock plus the source's system/frequency clock components),
> while native has a scalar per-epoch `c_i` and time-invariant global
> constellation `i` states.

This is a direct line-level state, key, Jacobian, and unit mismatch.  It is
stronger source-parity evidence than the native `f` option: official `f` is
only the loop variable over `FTYPE = ["L1","L5"]`; the native `'f'` key is a
separate optional static secondary-signal code-bias family and has no official
graph-state counterpart.  Re-enabling that option would also change the
secondary observation population, so it is not the source-parity candidate.

Base pseudorange compensation is an important historical difference and may
explain part of the accuracy regression, but it is explicitly outside the
raw-only/no-base boundary.  It is therefore not frozen.  The sole candidate
freeze is the official per-epoch `C`/ISB parity described in the companion
JSON.

The decision is a hypothesis, not a causal proof.  Phase82 and Phase100
changed several coupled contracts, including base compensation, signal-bias
states, quality-anchor settings, meter C0/D, and the main QR branch.  The
sealed evidence cannot attribute the LAX-T change to one factor without a new
raw-only ablation.

## Sealed accuracy comparison

The values below are copied from sealed aggregate/route records; no solution
or truth payload was opened for this audit.

| Route | Phase82 sealed direct-quality/Phase80 score | Phase100 sealed raw-only QR score | Change | Phase100 status |
|---|---:|---:|---:|---|
| MTV-A | `1.1139384500152307 m` | `1.147436714982201 m` | `+0.0334982649669703 m` | route gates passed |
| LAX-T | `0.9389644134001871 m` | `3.3204375254720286 m` | `+2.3814731120718413 m` | absolute `<=3 m` gate failed |

Phase100's two-route aggregate is `2.2339371202271145 m` versus its pinned
Phase43 control `3.3158312396716876 m`; it is not a Phase82 four-route
comparison.  LAX-T still improved against its same-route Phase43 control
(`4.511656202485746 m`) by `1.1912186770137172 m`, but failed the absolute
route threshold.  These facts establish a regression relative to the sealed
Phase82 LAX-T recipe, not a controlled one-variable experiment.

The sealed Phase100 structural handoff remained healthy on both in-scope
routes: GNSS-first C0D accepted `167` (MTV-A) and `262` (LAX-T) outer steps,
with costs `5618000.786274777 -> 21298.114523521894` and
`6419784.856579111 -> 12818.288538454106`; optimized D was finite for all
`2159/2159` and `1466/1466` retained epochs.  Phase100 selected
`MULTIFRONTAL_QR` only for the main graph.  The sealed accuracy record
withheld main diagnostic fields, so it does not establish a main factor-family
failure or prove that QR caused either route's score.

## Authority and read accounting

| Item | Pinned authority |
|---|---|
| Phase82 sealed accuracy aggregate | `docs/use_cases/records/smartphone_r5_phase82_phase80_direct_quality_accuracy_result_v1.json`, SHA-256 `39c5a38b7e3e61fda0306582fffb961dad3e6fb0bd49fcfe70e66674e2c90873` |
| Phase82 metric/freeze contract | `docs/use_cases/records/smartphone_r5_phase82_phase80_direct_quality_accuracy_freeze_v1.json`, SHA-256 `33bbfc4051af20cd3f39fbf8620ea4d277c4acc5dbe5d8d1621452a657eebf6b` |
| Phase100 sealed accuracy aggregate | `docs/use_cases/records/smartphone_r5_phase100_phase99_qr_accuracy_result_v1.json`, SHA-256 `7acd77e948f17de96f8dd24a7c137e708f9a7f5329dbef52c3a78d8cca40b2b7` |
| Phase100 raw-only command manifest | `docs/use_cases/records/smartphone_r5_phase100_phase99_qr_accuracy_manifest_v1.json`, SHA-256 `e31e959347aa4ca8d4f3bce3b5d66955c2abf8336fa3c965b59da199e7eb7729` |
| Official GNSS source | `output/reproducibility-cache/gsdc2023/fgo_gnss.m`, SHA-256 `5368e4056f70f448b728792c5e4c124b7f2afc1e00653492b807083bd3ccf0c3` |
| Official GNSS+IMU source | `output/reproducibility-cache/gsdc2023/fgo_gnss_imu.m`, SHA-256 `c70090ccb8b27fc8ac7fd2929e2f995a14cfe7f089bb1fef067370e22051c3e3` |
| Official parameters | `output/reproducibility-cache/gsdc2023/parameters.m`, SHA-256 `518925e9c75c7a14fceb5cd99432fe883311e0d64c72b94396744ac765120f52` |
| Official signal mapping | `output/reproducibility-cache/gsdc2023/functions/sysfreq2sigtype.m`, SHA-256 `5ec1d299e04b45f604921cbae3b1fac8f78f3f76a0152496d86011a32f1d0079` |
| Official CCDD factor | `output/reproducibility-cache/gtsam_gnss/src/ClockFactor_CCDD.h`, SHA-256 `7c174217218aa0059155e1e5a7182c69d56a3cd389d0ebd95c242d6d8c65b9fc` |
| Official pseudorange factor | `output/reproducibility-cache/gtsam_gnss/src/PseudorangeFactor_XC.h`, SHA-256 `7f467698f2239818724eba4485b830fbc543586bbee06a21b8b2c03ae5b56415` |
| Native graph backend | `src/algorithms/fgo_gtsam_backend.cpp`, SHA-256 `38c555b82785aee3d1d5fd8a81ff3bde8f63fd0a9910d5f1786a7c68152d7f3f` |
| Native key helpers | `src/algorithms/fgo_gtsam_internal.hpp`, SHA-256 `fb9cf4796ab639c8e563ea7ea1bdcbca0b00fec928bb411410168987fd4a8356` |
| Native FGO result contract | `include/libgnss++/algorithms/fgo.hpp`, SHA-256 `97a3fe0788e1c82810647c51ae3213bb7341d328498c0c55c98fa57ce8bb6b6c` |
| Native raw entry point | `apps/native/gnss_fgo_imu_no_base.cpp`, SHA-256 `a2f87bae4b39a0d0027cb4df93883214a805ab5fc199611f7241c86ed6720741` |
| Native problem builder | `src/algorithms/fgo_problems.cpp`, SHA-256 `e7607ce8f0fe27fd382a01b5b6f147b027b7bf51453e920eae458c86ddac0c17` |

Read accounting for this audit:

| Activity | Count |
|---|---:|
| Native solver/raw/nav invocations | `0` |
| Raw GNSS/IMU/navigation bytes read | `0` |
| Phase100 solution or truth payloads reopened | `0` |
| New truth/accuracy calculations | `0` |
| MAT/base/precomputed-coordinate reads | `0` |
| Kaggle/token access | `0` |
| Route reruns or tuning | `0` |
| New large artifacts | `0` |

The official `.m` files contain source-level `load(...)` calls, but the audit
read code only; no MATLAB code was executed and no `.mat` payload was opened.

## Line-level source comparison

### States and key families, especially `f`/ISB/`C`/`D`

| Contract | Official MATLAB source | Native Phase100 source/recipe | Parity finding |
|---|---|---|---|
| Position/velocity | `fgo_gnss.m:89-93,107-120`; `fgo_gnss_imu.m:102-106,169-186` inserts per-epoch `x`, `v` | `fgo_gtsam_internal.hpp:948-975`; `x` and `v` are per-epoch; IMU adds `p`/`b` | Same broad families; native IMU adds Pose3/bias as expected. |
| Base clock `C` | `fgo_gnss.m:92,113,119`; `fgo_gnss_imu.m:105,176,184`: `c_i` is a 7-vector at every epoch | `fgo_gtsam_internal.hpp:948-956` and `fgo_gtsam_backend.cpp:647-674`: scalar per-epoch `c_i` | **Mismatch:** native scalarizes the official vector. |
| ISB | Official `PseudorangeFactor_XC.h:52-64` forms a 7-entry selector `hc`, with `C_i[0]` plus `C_i[sysidx]`; CCDD acts on all vector components | `fgo_gtsam_backend.cpp:675-683` creates one global `i` key per non-GPS clock group; P factors use scalar `c_i` plus that global `i` | **Mismatch:** time-varying per-epoch vector components became global constellation states; frequency/system indexing is not source-exact. |
| `f` | Official `FTYPE = ["L1","L5"]` and `for f=FTYPE` at `fgo_gnss.m:30,134` and `fgo_gnss_imu.m:36,204`; no `f` graph key | Native `'f'` is `signalBiasKey` at `fgo_gtsam_internal.hpp:957-960`, inserted only by `use_receiver_signal_bias_states` | **Not a source counterpart:** native `f` means static secondary code bias, not official frequency-loop state. Phase100 did not enable it. |
| Drift `D` | `fgo_gnss.m:93,114,120,224-229`; one per-epoch scalar `d_i` | `fgo.hpp:1638-1643` exports `epoch_clock_drift_mps`; backend `1600-1622` exports optimized D; Phase100 seeds/hand-offs retained raw D | Broadly aligned after Phase93; finite/full handoff is sealed for both routes. |
| IMU states | `fgo_gnss_imu.m:145-186` adds Pose3 `p`, `x`, `v`, `c`, `d`, bias `b` | backend IMU construction `469-596`; native key families `p/x/v/c/d/b` | Broad family alignment; clock/ISB representation remains the primary mismatch. |

### Priors, measurements, IMU, noise, and timing

| Area | Official source lines and behavior | Native Phase99/100 behavior | Finding |
|---|---|---|---|
| Initial priors | GNSS: infinite Vector priors for `x`, `v`, 7-vector `c`, scalar `d` at `fgo_gnss.m:107-120`; IMU adds infinite Pose3 and bias priors at `fgo_gnss_imu.m:169-187` | backend inserts current scalar `c`, global `i`, D, Pose3/velocity/bias and configured priors; C0D uses the source-meter `.1 m` row | State dimension and prior incidence differ for C/ISB even where the broad infinite-seed intent matches. |
| GNSS code | Official base correction at `fgo_gnss.m:72-79`, then P factors at `:123-142`; `PseudorangeFactor_XC` uses `hc` against 7-vector C | Phase100 raw-only forbids base/PDC and uses native undifferenced P at backend `743-891`; Phase100 direct-quality flag uses source SNR model, but no signal-bias flag | Base correction is a known coupled difference and prohibited candidate; C/ISB selector difference remains allowable. |
| Doppler | Official `DopplerFactor_VD` at `fgo_gnss.m:143-148` uses `v,d`; filters from `exobs_residuals` and parameters | Native `v,d` factors at backend `893-922`; raw corrected undifferenced D; Phase93 retained raw D initializer and optimized handoff | D equation/unit and staged handoff are aligned structurally; no D fallback is permitted. |
| Carrier/TDCP | Official L1/L5 loops and TDCP at `fgo_gnss.m:179-200` and `fgo_gnss_imu.m:301-322`; uses XXDD or XXCC by phone | Phase100 config explicitly has carrier and ordinary TDCP disabled; no carrier-family parity is claimed | Deliberate Phase100 scope difference, not selected as candidate because the requested candidate must stay within raw-only same-run contract and no extra factor-family change. |
| IMU preintegration | `fgo_gnss_imu.m:129-143,280-299`: gravity `[0,0,-g]`, zero coriolis, synchronized acc/gyro covariances, `RzRyRx(mountingAngle)`, Combined IMU and bias between factors | native `buildImuInput`/backend: `RzRyRx(-85,178,-94)`, gravity `9.80665`, preintegration, bias random walk, Pose3/velocity/bias chain | Broad source settings are present; Phase100 selector is main QR only, not an IMU retune. |
| Sigmas/robust loss | `parameters.m:22-44,62-128,130-142,176-203`: SNR85, D `/12`, L `/400`, source P/D/L masks, Huber P/D/L, clock `.1`, motion by route, IMU noise/bias values | native upstream preprocessing and direct-quality configuration implement SNR85/D `/12`; `makeNoise` adds configured Huber; C0D sigma is official `.1 m` | No sigma or robust-loss tuning is justified/frozen. The candidate must preserve all current values. |
| Clock CCDD equation | Official `ClockFactor_CCDD.h:39-61`: `C2-C1`, subtract `(D1+D2)dt/2` only component 0; Jacobian `[-I,+I,-dt/2,-dt/2]` over vector C | native C0D source factor at backend `1150-1248` preserves scalar meter C0/D equation and `.1 m` sigma | Equation on component 0 is aligned, but native cannot carry the official seven component C residual/Jacobian. |
| GNSS-first/main handoff | `fgo_gnss.m:220-230` exports `clkest,dclkest`; `fgo_gnss_imu.m:40-48` consumes position/velocity/C/D and derives attitude from velocity | entry point copies same-run position/clock and optimized D; Phase100 seal reports exact finite D coverage and main QR selection | Position/velocity/D staging is structurally aligned; C vector/ISB handoff is the unported portion. |
| Epoch timing/output | MATLAB uses `is:ie`, adjacent `obs.utcms` dt at `fgo_gnss.m:154-166`, no explicit raw UTC output contract | native raw UTC key alignment excludes one leading warm-up: Phase100 expected problem/prediction rows `2159/2158` MTV-A and `1466/1465` LAX-T; exact-key evaluator contract | Counts are explainable alignment policy, not the LAX-T accuracy cause; candidate must preserve exact retained-key alignment. |
| Optimizer | MATLAB sets LM max `1000` at `fgo_gnss.m:203-214` and `fgo_gnss_imu.m:325-340` | Phase100 GNSS-first max is `1000`, main is pinned Phase99 QR with max `12`; no LM tuning is proposed | Stage-specific max-iteration difference is intentional and outside this candidate. |

### Signal indexing detail

The official `sysfreq2sigtype.m:5-17` maps L1 to GPS/GLO/GAL/CMP entries
`0..3` and L5 to GPS/GAL/CMP entries `4..6`, with `7` for other signals.
That index selects a component of each epoch's C vector.  Native's
`clockBiasGroup()` groups GPS/QZSS together and creates one global ISB per
other constellation (`fgo_gtsam_internal.hpp:929-956`), which is a different
state topology even when the same raw satellites are present.

## Fact versus inference

### Facts

1. The official source inserts and exports a 7-vector C at each epoch and a
   scalar D, and its CCDD factor accepts vector C (`fgo_gnss.m:107-120,220-229;
   ClockFactor_CCDD.h:16-63`).
2. Official code iterates a variable named `f` over L1/L5; it does not create a
   graph key named `f`.
3. Native's `'f'` key is a static receiver secondary-signal code-bias key,
   and native's `'i'` key is global/time-constant per constellation.
4. Phase82 LAX-T is `0.9389644134001871 m`; Phase100 LAX-T is
   `3.3204375254720286 m`; both records state finite exact prediction-domain
   coverage, while Phase100 fails only its absolute LAX-T route threshold in
   the listed route gates.
5. Phase100 GNSS-first C0D progress and full finite D handoff are sealed for
   MTV-A and LAX-T.  Phase100 main diagnostic fields are null in the accuracy
   result because that run withheld them.

### Inferences and limits

1. The per-epoch C/ISB topology is the most direct source-parity explanation
   that is still legal to test raw-only.  It is not proven to explain all
   `+2.3814731120718413 m` LAX-T change.
2. Base pseudorange compensation is likely relevant because it is explicit in
   the official source and present in the Phase82 recipe, but testing it would
   violate the current no-base boundary.
3. Enabling native `f` signal-bias states may recover a historical Phase82
   behavior, but it is not official `f` parity and changes the observation
   population; it is not selected.
4. The structural Phase100 QR result cannot distinguish C/ISB topology,
   factor weighting, or initialization as the numerical cause.  No tuning or
   ablation is authorized by this audit.

## Candidate comparison and freeze choice

| Candidate | Evidence | Raw-only allowed? | Why selected/rejected |
|---|---|---:|---|
| Base pseudorange compensation | Explicit official `correct_pseudorange` at `fgo_gnss.m:72-79`; present in Phase82 recipe and absent from Phase100 | No | Rejected by the current no-base contract, despite strong historical relevance. |
| Native static `f` signal-bias states | Phase82 recipe enabled `--native-signal-bias-states`; native key family exists | Yes | Rejected: official `f` is a loop variable, not a graph state; enabling it also admits broad secondary raw bands and is not a line-faithful port. |
| Official per-epoch 7-vector C/ISB parity | Official `PseudorangeFactor_XC`/CCDD are vector-valued; native scalar `c` + global `i` is explicit in current source | Yes | **Selected and frozen:** direct state/key/unit/Jacobian parity gap, with no base/MAT/PDC dependency. |

## Frozen implementation boundary

The companion JSON freezes exactly one opt-in candidate:
`phase101-raw-only-native-source-parity-epoch-c-isb-v1`.

The future implementation must:

1. Add a default-off selector scoped to both the GNSS-first Point3/velocity
   staging graph and its same-run Pose3/IMU main graph.  Legacy/default and
   selector-off behavior must remain unchanged.
2. Represent each retained epoch's C as the official 7-vector.  Preserve the
   official component/index mapping (`C[0]` base clock and `C[1..6]` source
   signal/system components) and the native retained raw `D_i` initializer in
   metres/second.  Do not introduce native `'f'` signal-bias states as part of
   this candidate.
3. Use the official vector CCDD residual/Jacobian and its existing official
   ordinary clock sigma (`0.1 m` on the active component, with the source
   vector noise construction); do not tune sigma, LM, robust loss, filters,
   equations, units, or factor families.
4. Build raw-only P/D factors against the exact C vector, hand off optimized
   C and D by the retained epoch key together with the existing same-run
   position and velocity, and reject missing, nonfinite, short, duplicate, or
   misaligned state vectors.  No raw/zero/WLS/interpolated fallback is allowed.
5. Keep the Phase100 source-meter C0D/QR and IMU settings otherwise fixed for
   qualification; the candidate is a state-topology parity test, not a solver,
   base, PDC, carrier, or noise experiment.
6. Keep output/truth separation and fail-closed structural/accuracy gates.  A
   future raw qualification must use a new manifest and independent
   authorization; this audit authorizes no run.

The candidate does not promise that the official vector model will converge or
improve LAX-T.  It is the only candidate permitted to cross the next
implementation boundary from this audit.

## Next boundary

The audit and candidate freeze are intentionally separate commits.  The next
work may implement the frozen default-off C/ISB candidate and add focused
synthetic/regression coverage.  Before any raw execution, a separate
qualification, manifest, and one-shot authorization must prove exact raw-only
inputs, C/D full finite alignment, unchanged legacy behavior, and no
solution/truth leakage.  No accuracy promotion or Kaggle submission is
authorized here.
