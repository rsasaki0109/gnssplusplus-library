# libgnss++

Modern C++20 GNSS toolkit for non-GUI positioning.

Native `SPP`, `RTK`, `PPP`, `CLAS/MADOCA`, `RTCM`, `UBX`, and direct `QZSS L6`
handling without an external RTKLIB runtime.

![UrbanNav Odaiba social card](docs/driving_odaiba_social_card.png)

## What You Get

- Solvers: `gnss spp`, `gnss solve`, `gnss ppp`
- Inputs/tools: RINEX, RTCM, UBX, SBF, NMEA, BINEX, QZSS L6
- Products: `SP3`, `CLK`, `IONEX`, `DCB`
- Extras: benchmarks, web dashboard, Python bindings, Docker, ROS 2 playback

![Feature overview](docs/libgnsspp_feature_overview.png)

## Results And Validation Status

| Area | Public comparison | Evidence / status |
|---|---|---|
| RTK | PPC Tokyo/Nagoya vs RTKLIB `demo5` | +17.0 pp positioning, +28.1 pp official score, -11.96 m P95 H delta |
| GNSS/IMU FGO | PPC Tokyo vs `tightly-coupled-gnss-imu-fgo` | Higher <50 cm fraction (avg +5.6 pp) and fix-rate (avg +8.2 pp) on all 3 runs; fixed-only RMS also wins 2 of 3 runs |
| CLAS PPP | Six PPC Tokyo/Nagoya runs vs MRTKLIB CLAS | 24.851% aggregate FIX, 0.377 m FIX RMS2D (lower than MRTKLIB on all six runs), 36.523 m all-solution RMS2D across 58,259 scored epochs; 19 FIX epochs (0.03%) exceed 3 m, all in one pre-existing 4 s Nagoya 2 burst |
| Urban RTK | UrbanNav Tokyo Odaiba vs RTKLIB `demo5` | More fixes, lower Hp95/Vp95; `--preset odaiba` closes Hmed |
| SPP | PPC SPP adaptive robust + policy gate | No P95 regression with <=1 pp positioning drop |

### PPC 2024 goal matrix vs Kaiyodai and gici-open

The audited KF/FGO selected profile clears the distance-weighted PPC public
target at **78.8455%** (published target: **78.7%**). It also exceeds the
Tokyo 1 public FIX rate (**80.861%** vs **80.8%**). A separate FIX-target
profile clears Nagoya 1 by the narrow measured margin **85.100974%** vs
**85.1%**, with 0.913% Wrong FIX/FIX and 1.460 m P95 horizontal error. The
public FIX targets come from the [Kaiyodai RTK paper](https://www.denshi.e.kaiyodai.ac.jp/wp-content/uploads/pdf/content/2024okada,sasaki,ando.pdf),
and the PPC score target from the [Turing tight-coupling slides](https://www.denshi.e.kaiyodai.ac.jp/wp-content/uploads/2025/01/Turing-Inc.-Tight-coupling-Factor-Graph-%E4%BA%95%E4%B8%8A%E6%A7%98-%E5%9C%A7%E7%B8%AE.pdf).

| Run | libgnss++ FIX | gici-open FIX | Wrong FIX/FIX | libgnss++ correct FIX/ref | gici-open correct FIX/ref | 50 cm/ref | libgnss++ official | gici-open official | P95 H |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| Tokyo 1 | **80.861%** | 46.472% | 2.855% | **71.467%** | 43.528% | 80.286% | 79.458% | **80.263%** | 2.082 m |
| Tokyo 2 | **82.340%** | 76.938% | 0.463% | **79.827%** | 74.462% | 88.395% | 88.696% | **90.652%** | 1.604 m |
| Tokyo 3 | **78.461%** | 73.347% | 1.366% | **75.962%** | 71.923% | 86.295% | **85.969%** | 83.787% | 1.671 m |
| Nagoya 1 | **83.659%** | 67.812% | 0.547% | **78.460%** | 60.005% | 85.845% | 65.201% | **70.851%** | 1.332 m |
| Nagoya 2 | **55.553%** | 39.988% | 0.541% | **48.587%** | 35.330% | 60.787% | **55.529%** | 39.847% | 18.144 m |
| Nagoya 3 | **44.761%** | 21.399% | 3.007% | **43.415%** | 18.285% | 61.354% | **72.336%** | 33.495% | 1.908 m |
| **Macro mean** | **70.939%** | 54.326% | **1.463%** | **66.287%** | 50.589% | 77.160% | **74.532%** | 66.483% | 4.457 m |

![PPC libgnss++ and gici-open comparison](docs/ppc_libgnss_gici_comparison.png)

![PPC public targets](docs/ppc_public_targets.png)

The audited runtime profile uses candidate telemetry only; reference truth is
used after output generation for scoring. It reaches an official score of
**78.845491%** while reducing aggregate wrong FIX from 869 to **574**, errors
above 5 m from 96 to **42**, and errors above 10 m from 59 to **5**.

![PPC selected XY trajectories by FIX status](docs/ppc_kf_fgo_fix_status_xy.png)

`gici-open` was reproduced from commit
`e7666110a88d22e08aad24345a253564af9b8024` on its `forppc2024` branch and
evaluated with the same six references and metric code. The libgnss++ FIX macro
is **+16.613 pp** higher and Wrong FIX/FIX is **1.025 pp** lower. These are
in-sample benchmark results, not a held-out generalization claim.

See the [goal audit](docs/ppc_goal_completion_audit.md),
[FIX integrity audit](docs/ppc_fix_integrity_audit.md),
[kinematic integrity LOO report](docs/ppc_kinematic_integrity_loo.md), and
[external residual-integrity holdout](docs/ppc_residual_integrity_external_audit.md).
The [Nagoya 3 root-cause analysis](docs/ppc_nagoya3_wrong_fix_root_cause.md)
documents the catastrophic float-KF wrong basin. See the
[reproduction commands](docs/ppc_reproduction.md) for the gate design,
external replay, event ledger, machine-readable metrics, and licensing details.

### GNSS/IMU Tightly-Coupled FGO vs tightly-coupled-gnss-imu-fgo

GTSAM-based fixed-lag factor-graph backend (`FGOBackend::GTSAM`) with
tightly-coupled IMU, multi-frequency DD RTK, per-epoch partial LAMBDA,
fix-and-hold, and an urban-robustness stack (CMC multipath screening,
CP-hold recovery, DDPR-anchored resets, FDE, elevation-dependent sigma,
and independent surplus-satellite validation).
Public PPC Tokyo moving-RTK replays with the dataset's tactical-grade IMU,
versus [inuex35/tightly-coupled-gnss-imu-fgo](https://github.com/inuex35/tightly-coupled-gnss-imu-fgo)
(Python + GTSAM) on the same rover/base/IMU data:

| Run | libgnss++ <50cm | Reference <50cm | libgnss++ fix | Reference fix | libgnss++ fixed RMS | Reference fixed RMS |
|---|---:|---:|---:|---:|---:|---:|
| Tokyo run1 | 54.9% | **56.7%** | **53.8%** | 49.5% | 1.180 m | **0.815 m** |
| Tokyo run2 | **85.7%** | 69.9% | **78.6%** | 60.8% | **0.109 m** | 0.277 m |
| Tokyo run3 | **77.5%** | 67.9% | **69.3%** | 59.4% | **0.125 m** | 0.211 m |

![Tokyo run1 GNSS/IMU FGO](docs/gnss_imu_fgo_tokyo_run1.png)
![Tokyo run2 GNSS/IMU FGO](docs/gnss_imu_fgo_tokyo_run2.png)
![Tokyo run3 GNSS/IMU FGO](docs/gnss_imu_fgo_tokyo_run3.png)

With the geometry-free reset below, libgnss++ exceeds the reference's raw FIX
rate on all three runs, and its <50 cm fraction and fixed-only horizontal RMS
on two of three. Raw FIX rate alone can hide wrong integer solutions, so the
same clean full-length replays were also scored with the PPC official segment
metric (0.11 s matching tolerance, 0.5 m correctness threshold):

| Run | Correct FIX distance, baseline -> GF reset | Wrong FIX distance, baseline -> GF reset | Official score, baseline -> GF reset | GF guard demotions |
|---|---:|---:|---:|---:|
| Tokyo run1 | 25.656% -> **33.322%** | 23.765% -> **21.221%** | 30.823% -> **39.375%** | 14 |
| Tokyo run2 | 60.304% -> **74.599%** | 12.573% -> **1.386%** | 65.264% -> **81.615%** | 0 |
| Tokyo run3 | 59.175% -> **65.099%** | 7.918% -> **1.818%** | 64.081% -> **71.207%** | 17 |
| Distance-weighted aggregate | 49.181% -> **57.409%** | 13.741% -> **7.650%** | 54.178% -> **63.692%** | 31 |

Wrong FIX/FIX falls from 21.839% to 11.759% in the aggregate, while matched
positioning distance is unchanged at 99.682%. Fixed-only horizontal
RMS/P95 improves from 1.267/0.892 m to 1.180/0.877 m on run1, from
0.225/0.600 m to 0.109/0.155 m on run2, and from 0.257/0.539 m to
0.125/0.079 m on run3. Vertical absolute P95 also improves on every run
(1.478 -> 1.463 m, 0.878 -> 0.141 m, and 1.152 -> 0.253 m). Cached-input
solver wall times on the validation host were 463.5, 584.6, and 844.9 s;
these are workload reports, not controlled microbenchmarks.

Every new behavior is opt-in and non-GTSAM builds are unchanged. Reproduce
with `gnss_fgo_parity` (requires a GTSAM build) and the evaluated preset:

```
--imu <run>/imu.csv --fixed-lag 5 --multi-freq --partial-ar --hold \
--elev-mask 25 --snr-mask 30 --cmc --cmc-level 0.75 --cp-hold \
--exc-recovery --ddpr-anchor --fde --varerr --fix-demote \
--fix-demote-res 25 --fix-demote-posthold 5 \
--fix-demote-surplus-crosscheck --fix-demote-surplus-anchor-reprieve \
--fix-demote-spp-model-reprieve --surplus-validation \
--surplus-validation-min-n 3 \
--surplus-validation-aperture-lt1 0.15 --surplus-validation-aperture-1to2 0.3 \
--surplus-validation-aperture-gt2 0.45 --anchor-gated-unfix-reset \
--imu-ratio-relaxed 1.5 --gf-slip-reset
```

With `--dump-csv <path>`, the epoch table includes the ambiguity-candidate
funnel (`amb_after_hold`, `amb_final`, and `amb_excl_*`) and the last
successfully searched LAMBDA position candidate (`lambda_candidate_*`), even
when the ratio or a later integrity decision leaves the epoch FLOAT. A
normalized satellite/signal trace is written beside it as
`<path>.ar_candidates.csv`.

`--clock-resilient-temporal-shadow` additionally audits receiver-clock-free
satellite-single-difference temporal carrier residuals. It is monitor-only and
does not add graph factors or change FIX/FLOAT decisions. With `--dump-csv`, a
per-factor classification trace is written to
`<path>.clock_resilient_tdcp_factors.csv` in addition to the epoch counters.
For large holdouts,
`--clock-resilient-shadow-truth-replay --ref reference.csv --dump-csv <path>`
recomputes the trace at the reference trajectory without rerunning the solver.
The formulation, research basis, and Tokyo validation are documented in
[`docs/fgo_clock_resilient_temporal_shadow.md`](docs/fgo_clock_resilient_temporal_shadow.md).

`--predicted-ddpr-quality-shadow` audits each temporally continuous DD
pseudorange pair against two causal witnesses: trapezoid-integrated
double-difference Doppler and the one-step IMU-predicted geometry.  The two
tests remain receiver-clock-free because rover and base clock terms cancel in
the satellite double difference.  This option is monitor-only: it neither
changes factor weights nor participates in ambiguity resolution.  With
`--dump-csv <path>`, its per-factor trace is written to
`<path>.predicted_ddpr_quality.csv`.

For offline threshold development, combine it with
`--clock-resilient-shadow-truth-replay --ref reference.csv`.  The resulting
DDPR sidecar adds the previous and current truth-position DDPR residuals, so a
causal runtime score can be joined to an independently labelled observation.
Thresholds must be selected on the development run and then held fixed for
validation/holdout runs; see
[`docs/fgo_fix_rate_observation_quality_plan.md`](docs/fgo_fix_rate_observation_quality_plan.md).

`--predicted-ddpr-bias-state-shadow` separately tests whether those causal
predicted-geometry residuals contain a persistent pair-specific bias.  A
bounded scalar random walk uses only earlier rows to predict the current row;
the current residual is assimilated after its diagnostic is emitted.  The
option remains solver-independent and can be enabled without retaining the
quality CSV.  With `--dump-csv <path>`, it writes
`<path>.predicted_ddpr_bias_state.csv`.  Its causal burn-in, reset rules,
research rationale, and staged activation gates are documented in
[`docs/fgo_ddpr_bias_state_shadow_plan.md`](docs/fgo_ddpr_bias_state_shadow_plan.md).

The normalized trace's `disposition` values are:

| Value | Meaning |
|---:|---|
| 0 | Reached the final per-epoch LAMBDA candidate set |
| 1 | Excluded while building the problem (currently sustained CMC level) |
| 2 | Carrier factor suppressed by full CP hold |
| 3 | Carrier retained as float but quarantined from AR during CP recovery |
| 4 | Removed by one-band-per-satellite selection |
| 5 | Removed by constellation exclusion |
| 6 | Removed by the previous epoch's per-satellite residual gate |
| 7 | Removed by carrier FDE |
| 8 | Removed because its ambiguity key left the active smoother |
| 9 | Candidate present, but the whole epoch failed the quality gate |
| 10 | Candidate present, but ambiguity resolution was disabled |

These fields only report decisions already made by the solver; enabling the
CSV dump does not alter candidate selection or FIX/FLOAT decisions.

The same dump also includes `gf_slip_events`, `gf_slip_max_jump_m`, and
`gf_slip_tainted_ambiguities`. This geometry-free shadow follows the 0.05 m
single-difference carrier jump used by
[GICI-LIB](https://github.com/chichengcn/gici-open/blob/master/src/gnss/ambiguity_common.cpp)
and the multi-frequency arc checks in
[RTKLIB](https://github.com/tomojitakasu/RTKLIB/blob/master/src/rtkpos.c).
It ignores gaps and arcs already restarted by the existing LLI/CMC logic,
then reports how many unchanged DD ambiguity arcs remain exposed after a
new jump. Without `--gf-slip-reset` it never restarts an arc or changes the
factor graph.

`--gf-slip-reset` turns the shadow into a conservative arc-integrity action.
A detected jump is held for 1.0 s and cancelled if a receiver-side LLI, gap,
or dropout naturally replaces the arc first. If the same arc survives,
both bands' DD ambiguities are restarted: the geometry-free combination proves
that one integer changed but cannot safely identify which one. During the
resulting low-redundancy reacquisition, a FIX is neither held nor reported when
all independent code checks agree it is grossly implausible (at most six fixed
ambiguities, ratio at most 10, DDPR RMS at least 10 m, and a fresh SPP position
more than 25 m away). This guard is intrinsic to the GF reset even when the
broader `--fix-demote` feature is disabled.

The same shadow also compares each rover/base single-difference phase change
with trapezoid-integrated Doppler at the existing RTK detector's 0.20 m
threshold. `doppler_slip_signals` reports per-signal innovations and
`gf_doppler_isolated_pairs` counts GF events for which Doppler identifies
exactly one of the two bands. This remains diagnostic-only. In particular,
the Doppler result is not used alone because RTKLIB disables its analogous
detector over receiver clock-jump sensitivity; requiring the clock-free GF
event first lets the trace evaluate Doppler only as a band-isolation witness.

Each provisional LAMBDA candidate also reports temporal integer-consensus
fields.  They compare only ambiguity indices shared with the immediately
preceding epoch, so an LLI/CMC/FDE arc change cannot count as continuity.
`lambda_candidate_integer_overlap` and `_agreements` expose the raw counts;
`_agreement_fraction` and `_consensus_streak` summarize consecutive epochs
with at least four exactly matching integers.  This is a diagnostic analogue
of the consecutive-fix evidence used before ambiguity holding in
[RTKLIB](https://github.com/tomojitakasu/RTKLIB/blob/master/src/rtkpos.c), not
an acceptance rule.  Tokyo run1 confirmed why: long-lived wrong integer basins
also produce exact temporal agreement, so the fields never change FIX/FLOAT,
hold, or graph state.

`--ratio-impact-monitor` adds a counterfactual partial-AR audit for epochs
that remain ratio-rejected. It removes each target satellite in turn, reruns
LAMBDA, and writes the best ratio, subset size, candidate ECEF position, and
fixed/float and fixed/IMU separations as `ratio_impact_*`. The audit is
diagnostic-only: it cannot change the selected subset, graph, hold state, or
reported FIX/FLOAT status. A normalized row-per-exclusion trace is also
written beside the epoch CSV as `<dump>.ratio_impact.csv`; it includes the
excluded satellite, its ambiguity variance/fractional-cycle/DDPR-residual
proxies, and the resulting counterfactual candidate.

The epoch CSV also records whether its builder position came from a fresh
current-epoch SPP solution (`spp_seed_fresh`) and that seed's ECEF position.
These diagnostic-only fields allow integer candidates to be audited against
an independent undifferenced-code/RAIM witness; they do not feed the graph or
change FIX/FLOAT decisions.

#### Surplus-satellite rescue integrity

LAMBDA candidates that fall short of the ratio gate can be rescued by an
independent integrity test: DD carrier observations that were excluded from
the fix (FDE quarantine, CMC exclusion, partial-AR drops) are re-differenced
against an alternate reference satellite at the candidate fixed position and
checked against a PDOP-scaled nearest-integer aperture, with a
GQEBR→GQEB→GQER→GQE→GQB→GQ constellation fallback. A rescue additionally
requires at least 10 observed satellites, global DD-code RMS at most 5 m,
fixed-hypothesis carrier post-fit RMS at most 0.05 m, and the strongest GQEBR
surplus pool. The weaker fallback pools remain available for monitoring and
established-fix veto experiments, but cannot create a relaxed-ratio fix.
Low-count ambiguity rescue remains off.

The final fixed-lag-5 preset was replayed over all epochs of all three Tokyo
runs. “Correct” and “wrong” below classify FIXED epochs by 3D error below or
above 0.5 m:

| Run | Correct FIX | Wrong FIX | Fixed horizontal RMS | All epochs 3D <50 cm |
|---|---:|---:|---:|---:|
| Tokyo run1 | 4759 → **4955** | 1088 → **978** | 0.6866 → **0.6616 m** | 5722 → **5932** |
| Tokyo run2 | 6336 → **6339** | **314 → 314** | 0.21668 → **0.21663 m** | **7130 → 7130** |
| Tokyo run3 | 9961 → **9963** | **1002 → 1002** | 0.25744 → **0.25742 m** | **10407 → 10407** |

`--fix-demote-surplus-anchor-reprieve` is an additional opt-in, fail-closed
recovery for otherwise demoted fixes. It requires the surplus test to pass,
at least 12 observed satellites, fixed/float separation at most 1 m, carrier
post-fit RMS at most 0.1 m, and a trusted independently solved DD-code anchor
within 8 m of the fixed candidate. The anchor is used only to validate the
reprieve; this switch does not enable additional anchor-based demotions.
Full-run A/B replays added only correct fixes:

| Run | Correct FIX | Wrong FIX |
|---|---:|---:|
| Tokyo run1 | 4955 → **5144** | **978 → 978** |
| Tokyo run2 | 6339 → **6357** | **314 → 314** |
| Tokyo run3 | 9963 → **9969** | **1002 → 1002** |

`--anchor-gated-unfix-reset` adds a fail-closed ambiguity-reacquisition
policy for sustained fresh-AR outages. The existing satellite-count, GDOP,
and FDE-fraction gates still apply; at each surviving trigger, a current-epoch
DD-code anchor must also have enough factors, pass its residual gate, remain
within 20 m of the IMU prediction, and disagree with the optimized antenna
position by at least 1 m. Only then are live ambiguity generations renewed.
The reset does not engage CP hold, break the IMU chain, or label its trigger
epoch FIXED. Full Tokyo replays combined it with the anchor reprieve above:

| Run | Anchor-only correct/wrong FIX | Anchor-gated reset correct/wrong FIX | Resets allowed / skipped |
|---|---:|---:|---:|
| Tokyo run1 | 5144 / 978 | **5469 / 649** | 4 / 5 |
| Tokyo run2 | **6357 / 314** | **6357 / 314** | 0 / 3 |
| Tokyo run3 | **9969 / 1002** | **9969 / 1002** | 0 / 4 |

`--fix-demote-spp-model-reprieve` is a separate opt-in for the narrow case
where the absolute DD-code residual is the only demotion reason. It keeps the
FIXED label only for a fresh LAMBDA candidate with at least 10 fixed
ambiguities, at most 2 cm separation from the IMU-predicted pose, and at most
8 m separation from the current epoch's fresh standalone SPP/RAIM solution.
A coasted or header-derived SPP seed fails closed. The original 5 m gate was
expanded to 8 m after a Tokyo run1 development audit, then validated unchanged
on run2 and applied once to held-out run3. Full implementation replays rescued
243/311/9 correct epochs and zero wrong epochs on runs 1/2/3 respectively;
because most were stationary, the aggregate correct-FIX distance gain was
102.776 m (+0.304 pp), so this is a safe incremental guard rather than the
complete FIX-rate target. Combined with the two preceding reprieves,
correct-FIX distance is now approximately 58.2776% (+1.2045 pp from 57.0731%);
about 0.7955 pp remains to the +2 pp goal.

The FGO epoch CSV also reports monitor-only LAMBDA covariance diagnostics for
each provisional candidate: the bootstrapped success-rate lower bound, the
same bound after 2/4/8/16x covariance inflation, and the fixed-failure-rate
ratio threshold and verdict for `Pf_tol=0.001`. These values come from the
same two-candidate search used by the existing ratio test, add no CLI option,
and do not change candidate selection or FIX/FLOAT decisions. The FFRT
coefficient table follows [Hou, Verhagen, and Wu (2016)](https://doi.org/10.3390/s16070945)
and unsupported ambiguity dimensions fail closed in telemetry.

`--conditional-mf-shadow` adds a separate, diagnostic-only multi-frequency AR
experiment. It resolves one primary band per satellite, conditions the
remaining float ambiguities and covariance on those primary integers, then
runs a second LAMBDA search. The CSV records both stage ratios, bootstrapped
success rates, ambiguity counts, and the counterfactual position. The shadow
does not promote FLOAT epochs, add integer priors, alter holding, or change the
reported position. A current-preset Tokyo run1 500-epoch A/B replay produced
499 correct FIX and zero wrong FIX in both arms, with identical status,
position, ratio, and AR-outcome rows; the shadow produced 197 candidates.
Historical full-run candidate auditing found that ratio/BSR passage alone was
not a safe promotion rule, so activation remains deliberately unavailable.
See the [multi-band AR audit](docs/fgo_multiband_ar_fix_rate_audit.md).

`--multiepoch-ar-shadow` adds an independent temporal partial-AR experiment.
Only integer candidates that remain identical for at least three consecutive
epochs on the same uninterrupted DD ambiguity arc enter a reduced LAMBDA
search (minimum four ambiguities). The CSV reports the persistent subset size,
history depth, ratio, bootstrapped success rate, history agreement, candidate
position, and float/IMU separations as `multiepoch_ar_*`. It also tests each
candidate against carrier rows whose ambiguities were not used by the reduced
search. `multiepoch_ar_surplus_eval`, `_pass`, `_level`, and `_used` expose
that held-out alternate-reference witness; lack of a sufficiently independent
pool produces no verdict. The `multiepoch_ar_graph_cost_*` columns independently
impose the persistent integers as tight priors, batch-refine a copy of the
active fixed-lag graph, and score the result against the original prior-free
graph. They report evaluation/pass state, active factor count, before/after
cost, and delta. Neither witness changes the graph, hold state, solution,
ratio, or FIX/FLOAT label. A held-out-carrier gate failed on Tokyo run2; a
later combined carrier/IMU/graph-cost gate survived run2 but failed on the
previously unused run3. Both therefore remain monitor-only.
See the
[multi-epoch AR audit](docs/fgo_multiepoch_ar_fix_rate_audit.md).

Across the three courses, distance-weighted correct FIX improved from
57.0731% at the pre-reprieve baseline to approximately **58.2153%**, while
distance-weighted wrong FIX fell from 7.7043% to **7.0089%**. The policy is
opt-in and leaves the default library behavior unchanged.

The rescue remains opt-in at the library API level; the command above is the
validated shipping preset. Separately, counterfactual auditing of the
fixed-lag-QR configuration moved its `--fix-demote-res` from 25 to 40 (25
demoted mostly sub-0.5 m fixes on run1/run3 while run2's genuine wrong-basin
cluster sits far above 40). Its Tokyo PPC full-run results are:

```
--imu <run>/imu.csv --fixed-lag 1 --fixed-lag-qr --hold \
--imu-ratio-aperture --fixed-history-dr --cmc --cmc-level 0.75 \
--fde --fde-cp-quarantine --varerr --integ-cov 1e-6 \
--fix-demote --fix-demote-dist 5 --fix-demote-res 40 --fix-demote-posthold 5 \
--multi-freq --partial-ar --elev-mask 25 --snr-mask 30 \
--imu-preset-tactical --exc-recovery --ddpr-anchor \
--surplus-validation --surplus-validation-min-n 3 \
--surplus-validation-aperture-lt1 0.15 --surplus-validation-aperture-1to2 0.3 \
--surplus-validation-aperture-gt2 0.45
```

| Run | Fix-rate | Fixed-only RMS | Rescue contribution |
|---|---:|---:|---:|
| tokyo/run1 | 67.23% | 0.797 m | +2.99 pp, RMS improved |
| tokyo/run2 | 77.71% | 0.742 m | RMS 6.07 m → 0.74 m with res 40 |
| tokyo/run3 | 75.14% | 1.054 m | RMS 1.96 m → 1.05 m, >5 m fixes 38 → 13 |

The rescued-only population on run1 has 0.565 m RMS (n=693). The companion
`--surplus-validation-veto` mode (re-testing established fixes) is implemented
but counterfactually false-alarm dominated — leave it off. For repeated runs
on identical inputs, `--problem-cache <file>` caches the parsed observations
and built DD problem (~2× faster validation loops with an /O2 build).

### Moving CLAS PPP vs MRTKLIB

The current moving-data gate replays all six public PPC Tokyo/Nagoya runs at
5 Hz from QZSS L6 corrections with kinematic dynamics enabled. Scoring matches
solutions to the PPC reference, discards the first 60 matched epochs per run,
and defines TTFF as the first run of at least 30 consecutive FIX epochs.

MRTKLIB columns are the published v0.4.2 results from the
[CLAS benchmark article](https://zenn.dev/hatognss/articles/7a54dd82606faf).
The native results below are the complete-run outputs after the
hold-continuation carve-out landed in #349 and the outage-counter parity
fix landed in #351; each run has 100% interval coverage and at least
99.92% epoch coverage.

| Run | libgnss++ FIX | MRTKLIB FIX | FIX RMS2D* | MRTKLIB RMS2D† | All RMS2D* | FLOAT RMS2D* | SINGLE RMS2D* | max FIX* | >3 m FIX* |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| Tokyo 1 | **10.704%** | 4.900% | **0.352 m** | 0.747 m | 41.862 m | 16.800 m | 80.343 m | 1.961 m | 0 |
| Tokyo 2 | 21.507% | **21.700%** | **0.322 m** | 0.514 m | 25.882 m | 18.287 m | 45.148 m | 1.013 m | 0 |
| Tokyo 3 | **37.951%** | 7.400% | **0.192 m** | 0.801 m | 35.276 m | 19.531 m | 88.519 m | 2.986 m | 0 |
| Nagoya 1 | **36.737%** | 17.000% | **0.450 m** | 1.105 m | 57.163 m | 7.948 m | 119.111 m | 1.043 m | 0 |
| Nagoya 2 | **23.959%** | 23.400% | **0.625 m** | 1.119 m | 25.829 m | 16.230 m | 40.405 m | 3.200 m | 19 |
| Nagoya 3 | **8.776%** | 6.300% | **0.304 m** | 0.318 m | 13.724 m | 14.360 m | 14.380 m | 0.587 m | 0 |
| **Six-run aggregate** | **24.851%** | — | **0.377 m** | — | **36.523 m** | **16.843 m** | **70.337 m** | **3.200 m** | **19** |

\* libgnss++ precision uses the raw PPC reference point (already
antenna-positioned; no lever-arm transform is applied — an earlier revision
of this table double-applied a vehicle→antenna lever arm on top of an
already-antenna-positioned reference, inflating FIX RMS2D by ~0.3–0.9 m and
incidentally masking the Nagoya 2 tail below the 3 m line). † The published
MRTKLIB precision uses the same raw PPC reference, so the FIX RMS2D and p68
columns are directly comparable, not merely contextual — libgnss++ FIX
RMS2D is now lower than MRTKLIB's on all six runs (bolded above).

![PPC six-run moving CLAS metric comparison](docs/ppc_clas_full_comparison.png)

Across 58,259 scored epochs, native CLAS produced 14,478 FIX epochs. A finite
SPP candidate rejected by the chi-square/redundancy validation still remains
excluded from ordinary filter admission, cold starts, and AR. For catastrophic
FLOAT/SPP disagreement above 250 m only, it can continue the counted MRTKLIB
`maxdiffp` recovery path. On Tokyo 2 this moves the bad-seed recovery from TOW
177750.0 to 177747.4 (311.6 m to 5.4 m), 0.8 s before the MRTKLIB recovery at
177748.2. The six-run all-solution RMS2D is 36.523 m; FLOAT and SINGLE RMS2D are
16.843 m and 70.337 m respectively.

Of the 14,478 FIX epochs, 19 (0.03%) exceed 3 m horizontal error; all 19 fall
in a single contiguous 4-second burst on Nagoya 2 (TOW 556406.4–556410.4,
errors 3.17–3.20 m, max 3.200 m), inside the known seed-geometry `maxdiffp`
reset zone. The identical 19 epochs, at matching TOWs and errors, are present
in the pre-#349 baseline run, so this is a pre-existing wrong-fix tail, not a
regression from the hold-continuation carve-out or the #351 outage-counter
parity fix — it was previously invisible because the lever-arm
double-correction happened to shift it under the 3 m line (old Nagoya 2 max
was 2.486 m). The other five runs still have zero FIX epochs above 3 m.

| Complete trajectories | Horizontal error and FIX epochs |
|---|---|
| ![PPC six-run CLAS trajectories](docs/ppc_clas_full_trajectories.png) | ![PPC six-run CLAS errors](docs/ppc_clas_full_errors.png) |

See the [complete table](docs/ppc_clas_full_table.md),
[machine-readable metrics](docs/ppc_clas_full_metrics.json), and
[PPC CLAS validation note](docs/ppc_clas_validation.md) for definitions and
reproduction details.

## Quick Start

Choose the entrypoint that matches your job:

- [Robotics quick start](docs/robotics_quickstart.md): RTK replay, local web
  inspection, ROS2 receiver launch, and rosbag capture.
- [Research quick start](docs/research_quickstart.md): repeatable sign-off
  runs, profile comparisons, Python inspection, and artifact layout.
- [Dataset gallery](docs/dataset_gallery.md): current public dataset lanes and
  the adapter contract for adding more.

Build:

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
python3 apps/gnss.py doctor
python3 apps/gnss.py ros2-doctor --device /dev/ttyUSB0
python3 apps/gnss.py ros2-bag-doctor --bag /path/to/rosbag --summary-json output/ros2_bag_doctor_summary.json
python3 apps/gnss.py field-report --out output/field_report.md
python3 apps/gnss.py robotics-smoke --profile realtime
```

`ros2-bag-doctor` reads sqlite ROS2 bags for message-level rates/gaps. For MCAP
bags it uses the optional Python `mcap` package when available, and otherwise
falls back to MCAP `metadata.yaml` for topic presence, counts, and duration.
`gnss web` auto-discovers `output/field_report*.json` and shows the report
links, Markdown preview, setup/ROS2/bag/smoke status, and next debug commands.

Run a solution:

```bash
python3 apps/gnss.py spp \
  --obs data/rover_static.obs \
  --nav data/navigation_static.nav \
  --out output/spp_solution.pos
```

RTK example:

```bash
python3 apps/gnss.py solve \
  --rover data/rover_kinematic.obs \
  --base data/base_kinematic.obs \
  --nav data/navigation_kinematic.nav \
  --mode kinematic \
  --out output/rtk_solution.pos
```

Run the web UI:

```bash
python3 apps/gnss.py web --port 8085
```

Then open `http://127.0.0.1:8085`.

List commands:

```bash
python3 apps/gnss.py commands
python3 apps/gnss.py commands --json
python3 apps/gnss.py commands --query ppp --limit 10
```

## Docker

```bash
docker build -t libgnsspp:latest .
docker run --rm -it -p 8085:8085 -v "$PWD:/workspace" \
  libgnsspp:latest web --host 0.0.0.0 --port 8085 --root /workspace
```

## Benchmarks

- [Benchmarks](docs/benchmarks.md)
- [Validation](docs/validation.md)
- [PPC reproduction commands](docs/ppc_reproduction.md)
- [SPP accuracy notes](docs/references/spp-accuracy-improvement.md)

## Docs

- <https://rsasaki0109.github.io/gnssplusplus-library/>
- [Documentation index](docs/index.md)
- [Quick start](docs/quickstart.md)
- [Robotics quick start](docs/robotics_quickstart.md)
- [Research quick start](docs/research_quickstart.md)
- [Dataset gallery](docs/dataset_gallery.md)
- [Interfaces](docs/interfaces.md)
- [Architecture](docs/architecture.md)
- [Reference analyses](docs/references/index.md)
- [Contributing](CONTRIBUTING.md)

## Install

```bash
cmake --install build --prefix /opt/libgnsspp
/opt/libgnsspp/bin/gnss --help
```

## Tests

```bash
ctest --test-dir build --output-on-failure
```

## License

MIT License. See [LICENSE](LICENSE). Permissive third-party attributions and
the separate GPL-only competitor-benchmark boundary are documented in
[THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md).
