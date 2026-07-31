# GNSS-only MultiSD fixed-lag FGO design

## Scope and acceptance contract

This path uses PPC rover/base GNSS observations only. IMU, LiDAR, camera, map,
and external-route data are excluded from both estimation and validation.
The deployment targets are Tokyo correct FIX >= 70%, Nagoya correct FIX >= 80%,
false/FIX <= 0.1%, zero >1 m false fixes, and p95 <= 100 ms/epoch. The feature
is opt-in and must remain shadow-only until all integrity gates pass nested,
route-blocked PPC validation.

## Evidence used

- Li et al., ION GNSS+ 2024, "Factor Graph Optimization Based Multi Epoch
  Ambiguity Resolution for GNSS RTK": MultiSD uses ambiguity correlation over
  multiple epochs and reports improved urban-canyon fixation and positioning.
  <https://doi.org/10.33012/2024.19805>
- Wen and Hsu, GraphGNSSLib: a sliding historical/current factor graph with DD
  pseudorange, carrier phase, and Doppler, followed by LAMBDA. It is used only
  as an architectural comparison; no source is copied because the repository
  does not publish a recognized license.
  <https://github.com/weisongwen/GraphGNSSLib>
- Chi et al., GICI-LIB: its differential AR path retains SD ambiguities and
  forms between-satellite differences at AR time, and its graph checks the
  post-fix solution. GICI is GPL-3.0, so only the published concepts are used;
  this implementation is independent.
  <https://github.com/chichengcn/gici-open>
- RTKLIB-demo5: operational comparison for staged subset AR, satellite
  admission/lock lifecycle, and fix-and-hold. No source is copied.
  <https://github.com/rtklibexplorer/RTKLIB>
- Teunissen and Verhagen's integer-aperture/fixed-failure-rate work: a fixed
  ratio is not a correctness test and its threshold should depend on model
  strength and an explicit failure-rate contract. The implementation keeps
  ratio/ADOP/BSSR as candidate-quality gates, not independent evidence.
  <https://doi.org/10.1007/s10291-012-0299-z>
- Teunissen (1998), bootstrapped ambiguity success probability: BSSR is a
  conservative covariance-domain bound, but cannot expose unmodelled urban
  bias. <https://doi.org/10.1007/s001900050199>

## Existing gap

`RTKProcessor` already stores reference-independent, per-satellite SD
ambiguities and converts them to DD immediately before LAMBDA. The FGO problem
builder historically stores one ambiguity per `(satellite, reference, signal,
arc)`. A harmless DD reference change therefore starts a new FGO ambiguity and
discards useful temporal information. The production `gnss_fuse` RTK path also
does not run the FGO path, so existing FGO experiments cannot improve its FIX
rate.

## Target architecture

1. Build FGO ambiguity states per `(satellite, signal, rover carrier arc)`.
   Each DD carrier factor connects two SD states as `N_sat - N_ref`; reference
   changes do not alter either key. Only a detected slip, loss of lock, or
   excessive observation gap starts a new satellite arc.
2. Optimize a causal fixed-lag graph containing DD pseudorange/carrier,
   rover-only between-satellite Doppler, and TDCP factors. Marginalize expired
   epoch states while retaining active SD ambiguity information.
3. At AR time only, create a full-rank between-satellite-difference projection
   per constellation/signal/arc component. Project both the SD float vector and
   its full covariance, including position cross-covariance. Never integer-fix
   an individual gauge-dependent SD state.
4. Rank decorrelated ambiguity subsets using a conservative bootstrapped
   success-rate/ADOP criterion, then run bounded top-K LAMBDA. Re-optimize each
   candidate with tight BSD constraints and reject it if graph cost worsens.
5. Give FIX authority only after a disjoint causal validator passes. Validation
   satellites/factors may not have contributed to the candidate, subset
   selection, robust weights, or parameter tuning.
6. Batch independent top-K/subset covariance projections on GPU only after the
   CPU reference is bitwise/within-tolerance correct. Keep a CPU fallback and
   enforce the p95 budget in CI.

## Implementation state

`FGOConfig::use_multisd_ambiguities` is default-off. When enabled, the problem
builder now creates reference-independent SD ambiguity states, initializes them
from rover-base carrier-minus-code, and connects DD carrier factors to the
target and reference states. The legacy DD topology is unchanged when disabled.
At AR time, the latest active DD edge set now projects the SD float state and
its full covariance into gauge-free BSD combinations. Bounded top-K LAMBDA,
bootstrapped success-rate telemetry/gating, ADOP telemetry/gating, and
difference constraints for integer re-optimization are implemented. Individual
SD states are never labelled fixed. Unit coverage verifies that a CMC-driven
DD reference change reuses the same satellite SD state, that BSD integers are
recovered without fixing the SD gauge, and that the legacy ambiguity path does
not regress. The disjoint causal validator and a `gnss_solve` rolling-window
shadow are now wired default-off. They have no production RTK authority, so
MultiSD remains opt-in and experimental pending the full promotion matrix.

## Validation sequence

1. Structural tests: reference-switch invariance, per-satellite slip reset,
   covariance projection symmetry/PSD, rank, and permutation invariance.
2. Synthetic integer tests: known SD gauge, BSD truth recovery, top-K ordering,
   and wrong-candidate post-fit rejection.
3. PPC shadow telemetry: candidate supply, oracle-correct supply, ratio,
   bootstrapped success rate, ADOP, graph cost delta, validator residuals, and
   latency. No threshold selection may use the held-out route block.
4. Nested route/time-blocked cross-validation plus cycle-slip, NLOS,
   satellite-loss, and outage injection. The final policy is adopted only if
   both cities meet the FIX/integrity contract without regressions.

## Initial PPC shadow evidence (2026-07-31)

Five non-overlapping 100-epoch run1 blocks were evaluated with the Eigen batch
backend, MultiSD, top-4 LAMBDA, and no IMU or external sensor. Tokyo blocks
0-100, 100-200, and 500-600 produced 300/300 correct FIX and zero false fixes;
Nagoya 500-600 produced 100/100 correct FIX. Nagoya 0-100 exposed the required
negative control: ratio 1.532 passed the experimental 1.5 aperture and produced
100/100 fixes at about 0.65 m 3D error. Bootstrapped success rate was 1.0 and
ADOP was 0.029 cycles, so covariance-only quality metrics did not detect this
model-bias failure. Raising the ratio to 2.0 rejected the entire bad block
(0 FIX), but threshold tightening alone is not the intended availability
solution. This result makes a disjoint measurement-domain causal validator a
hard promotion requirement. It also confirms the validator must detect common
model bias rather than reusing the same ambiguity covariance evidence.

## Disjoint top-K validator checkpoint (2026-07-31)

The Eigen MultiSD path now deterministically reserves satellites before the
float solve and removes every DD carrier/code factor involving them from the
entire window. The lowest-elevation surplus satellites are reserved so the
strongest satellite-PAR geometry remains in ILS. Each bounded LAMBDA hypothesis
is independently constrained and re-optimized. A hypothesis is checked at the
latest causal edge using only reserved DD carrier integer distance and DD code
RMS. FIX authority requires sufficient evidence and exactly one passing top-K
hypothesis; otherwise the result reverts to the untouched float solution.
Individual hypothesis position and residual diagnostics are emitted to JSON.

Synthetic coverage exercises clean acceptance, four-satellite independent
carrier corruption rejection, and insufficient-evidence fail-closed behavior.
Four holdout satellites are the safe default because two carrier equations do
not overdetermine a 3-D candidate position. The feature and all thresholds
remain default-off at the MultiSD authority level.

On exploratory 100-epoch PPC windows, four holdouts, offset 2, top-K=4 and
require-all carrier agreement produced 100/100 correct FIX with zero false
fixes on Tokyo 0--100, Tokyo 500--600, and Nagoya 0--100. In particular, the
Nagoya block that previously fixed 100/100 at about 0.65 m was changed by the
disjoint graph to a correct 0.09--0.11 m solution. Nagoya 500--600 abstained,
so this is a safety/candidate-quality breakthrough but not yet the availability
target. The offset is exploratory and may not be promoted without nested
route/time-block validation.

A 3-of-4 carrier-majority ablation recovered Nagoya 500--600 availability but
fixed all 100 epochs about 1.65 m wrong (100 >1 m false fixes). It is therefore
rejected; require-all remains the default. Increasing top-K from 4 to 16 also
made the Nagoya 0--100 window non-unique (two passing hypotheses about 4 cm
apart), while K=4 retained a unique correct hypothesis. Position-domain
hypothesis clustering is recorded as future research, not granted FIX
authority. Candidate re-optimization currently costs roughly 0.3--0.8 s per
100-epoch batch in these samples; this is still an offline batch measurement,
not proof of causal online p95 latency.

The original global LAMBDA path also skipped all large sparse-normal problems.
MultiSD now retains the sparse normal matrix whenever global LAMBDA is enabled
and solves only the ambiguity columns through sparse LDLT; it never forms the
full state covariance. An 80-epoch synthetic smoke test forces this sparse path
and verifies BSD/top-K fixing. The 100-epoch Tokyo sample generated 16 BSD
candidates and four hypotheses. Total batch time ranged from about 61 to
193 ms per 100-epoch batch on the development host; this is not yet an online
per-epoch fixed-lag latency measurement.

## Causal `gnss_solve` shadow checkpoint (2026-07-31)

`gnss_solve` now has an explicit default-off GNSS-only shadow output. When
`--multisd-fgo-shadow-csv` is supplied, it keeps a rolling observation window
that ends at the current RTK epoch, builds the MultiSD DD code/carrier,
single-difference Doppler, and TDCP graph, and records only the solution whose
timestamp equals that current epoch. No future observation can enter the
window. The RTK solution is used only as a geometry seed; the shadow never
changes RTK state, status, or output. The path takes no IMU, LiDAR, or camera
input.

The first causal smoke used top-K=4, four disjoint holdout satellites, offset
2, and require-all carrier agreement. These are development samples, not a
promotion or full-run FIX-rate claim:

| PPC block | window | evaluated | correct FIX | >1 m false | max FIX error | wall p95 |
|---|---:|---:|---:|---:|---:|---:|
| Tokyo run1 0--30 | 25 | 21 | 20 | 0 | 0.067 m | 1315 ms |
| Tokyo run1 0--30 | 10 | 21 | 21 | 0 | 0.066 m | 201 ms |
| Tokyo run1 500--530 | 10 | 21 | 19 | 0 | 0.034 m | 145 ms |
| Nagoya run2 0--30 | 10 | 21 | 20 | 0 | 0.105 m | 124 ms |
| Nagoya run2 500--530 | 10 | 21 | 8 | 0 | 0.249 m | 118 ms |

The 10-epoch window preserved the safe decisions in these samples and reduced
wall time by roughly 4x versus rebuilding 25 epochs, but it does not yet meet
the 100 ms p95 contract. Independent fixed top-K re-solves can now run as a
deterministic parallel batch behind `parallelize_lambda_hypotheses`; a smoke
test requires the sequential and parallel FIX decision, selected rank, and
latest position to match. This is the CPU fallback and intended seam for a
future CUDA batched normal-equation/factorization backend. The remaining large
cost is rebuilding and solving the common float graph each epoch, so GPU alone
must not substitute for fixed-lag reuse/marginalization. Promotion still
requires PPC run/time-block nested validation, injected fault tests, and the
full latency distribution.
