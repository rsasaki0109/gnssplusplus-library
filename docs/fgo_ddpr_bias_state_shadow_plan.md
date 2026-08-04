# DD pseudorange bias-state shadow plan

## Goal

Improve the float state presented to ambiguity resolution without buying FIX
rate by accepting more wrong integers.  This experiment asks a narrower
question first: does a target/reference/signal DD pseudorange pair contain a
causally predictable, persistent bias that is not already explained by the
one-step IMU geometry prediction?

TDCP is not the default answer.  Carrier, ambiguity acceptance, and the active
graph remain unchanged while this question is measured.

## Research decision

The candidate families considered were:

- switchable constraints, which add a latent switch, a prior favouring active
  measurements, and optionally a temporal switch transition;
- graduated non-convexity and robust losses, which reduce the influence of
  large residuals during optimization;
- explicit per-satellite pseudorange multipath/error states; and
- a causal random-walk bias predictor evaluated outside the optimizer.

Switchable constraints and GNC are credible robust-estimation techniques, but
their first-order effect is still to weaken suspect observations.  The prior
run1 DDPR downweight experiment increased wrong-FIX distance, so they are not
the first activation candidate here.  A free additive bias state inside the
graph is also unsafe as a first experiment: the current observation can help
estimate the variable that explains itself, and pair bias remains entangled
with position error unless persistence is demonstrated independently.

The selected first step is therefore a monitor-only scalar random walk for
each `(target satellite, reference satellite, signal)` key.  It consumes the
existing causal predicted-DDPR residual rows.  At epoch `k`, only the posterior
through `k-1` may predict the bias at `k`; the current residual updates the
state only after the diagnostic row has been emitted.

Research references:

- Suenderhauf et al., GNSS multipath mitigation with switchable constraints:
  <https://nikosuenderhauf.github.io/assets/papers/IV12-multipathMitigation.pdf>
- Suenderhauf and Protzel, Switchable Constraints for Robust Pose Graph SLAM:
  <https://nikosuenderhauf.github.io/assets/papers/IROS12-switchableConstraints.pdf>
- Wen et al., FGO-GNC for GNSS outlier mitigation:
  <https://arxiv.org/abs/2109.00667>
- GTSAM GNC optimizer documentation:
  <https://borglab.github.io/gtsam/gncoptimizer/>
- GraphGNSSLib, an open DD GNSS factor-graph implementation:
  <https://github.com/weisongwen/GraphGNSSLib>
- GICI, an open GNSS/INS estimator with robust pseudorange handling:
  <https://github.com/chichengcn/gici-open>

## Shadow model

For every continuous pair, propagate

`P(k|k-1) = P(k-1|k-1) + q^2 dt`

and emit the causal prediction `b(k|k-1)` before reading the current residual
into the state.  The exported corrected residual is

`r_corrected(k) = r_predicted_ddpr(k) - b(k|k-1)`.

The update is a scalar Kalman step.  Its innovation is clipped to a configured
multiple of the innovation sigma so that a one-epoch impulse cannot become a
long-lived bias.  The DD row sigma is conservatively approximated from the
existing two-epoch IMU-innovation sigma.  A minimum sigma prevents numerical
overconfidence.

The prediction is unavailable during burn-in.  Missing/non-finite IMU
geometry, discontinuous epochs, a gap, or a reference/key change starts a new
state.  No diagnostic field is consumed by the graph, covariance, LAMBDA,
hold logic, or exported solution.

Initial development defaults are `q=0.25 m/sqrt(s)`, initial sigma `5 m`,
minimum measurement sigma `0.5 m`, robust update limit `3 sigma`, and two
prior updates before a prediction is declared usable.  These are shadow
parameters, not solver tuning.

## Precommitted gates

### Gate 0: exact non-interference

- monitor off versus on has identical status, ECEF position, ratio, fixed
  ambiguity count, and AR outcome at every epoch;
- fixed-lag and batch paths remain deterministic; and
- no shadow result can reach the active graph.

### Gate 1: run1 causal predictiveness

- every usable prediction is formed before the current residual update;
- among gross truth-labelled rows, the median absolute residual must improve
  by at least 25%;
- clean-row absolute-residual P95 may regress by at most 5%;
- at least 100 gross rows and 100 clean rows must be usable; and
- report support, burn-in, resets, signed/absolute error distributions, and
  results by pair age rather than only the best aggregate threshold.

Truth is offline scoring only.  It cannot be read by the runtime monitor.

### Gate 2: identifiability and short activation

Before adding any graph state, show that improvement is pair-specific rather
than a cross-pair coherent position error.  Compare the pair prediction with
the same-epoch median across other pairs and require the pair-specific model
to retain a material advantage.

Only then may a separate default-off graph experiment be designed.  On a
representative 500-epoch run1 slice it must have zero additional wrong FIX,
no lost correct FIX, no new failure/non-finite epoch, and at most 10% runtime
overhead.

### Gate 3: frozen validation

Run full run1 before freezing the activation.  Run2 is then a one-shot
validation and run3 remains sealed until run2 passes.  Each holdout must have:

- no increase in wrong-FIX distance;
- no decrease in correct-FIX distance;
- no regression in matched distance or solver failures; and
- fixed RMS and P95 regression no greater than 5%.

Failure retains useful telemetry but rejects activation.  It does not trigger
post-hoc TDCP, ratio-threshold, or holdout tuning.

## Execution sequence

1. Add the independent, default-off shadow and normalized CSV.
2. Unit-test persistence, impulse bounding, continuity reset, and causal
   burn-in.
3. Prove exact solver non-interference in both backend modes.
4. Run and score run1 against the gates above.
5. Continue to activation/run2/run3 only if every preceding gate passes.

After producing a runtime bias sidecar and a truth-replay quality sidecar,
score the frozen gates with:

```bash
python scripts/analyze_ddpr_bias_state_shadow.py \
  --bias-csv <runtime.csv.predicted_ddpr_bias_state.csv> \
  --truth-quality-csv <truth.csv.predicted_ddpr_quality.csv> \
  --json <gate1-summary.json>
```

## Development result

The default-off shadow passed exact non-interference in unit tests and in a
50-epoch Tokyo run1 A/B.  The complete epoch CSV SHA-256 was identical with
the monitor off and on.  The monitor produced 1,960 rows, of which 1,880 were
usable after burn-in.

Full run1 then passed the causal usefulness gate:

| Metric | Raw | Bias-corrected shadow |
|---|---:|---:|
| Gross truth-labelled rows | 11,522 | 11,522 |
| Gross absolute residual median | 10.266 m | 1.716 m |
| Clean truth-labelled rows | 149,789 | 149,789 |
| Clean absolute residual P95 | 1.150 m | 0.524 m |

The gross median improvement was 83.29%.  A same-epoch common-bias control
left a 9.967 m gross median, versus 1.716 m for the pair-specific prediction
(ratio 0.172), so the gain is not primarily a coherent pose/common-mode term.
The run contained 189,036 usable/joined rows out of 197,868 bias rows.

A separate causal activation was then tested with the same frozen state
parameters on the first 500 run1 epochs.  It subtracted the prior pair bias
from DD code only after two earlier updates, clipped updates at 3 sigma, and
capped the applied correction at 20 m.  It failed the precommitted short-run
gate:

- baseline: 476 correct FIX, 0 wrong FIX, fixed RMS 0.03684 m;
- activation: 472 correct FIX, 0 wrong FIX, fixed RMS 0.04502 m;
- transitions: 13 lost correct FIX and 9 added correct FIX; and
- both runs had zero NONE/non-finite epochs.

The activation therefore loses correct fixes and regresses fixed RMS by about
22.2%.  Its switch and graph changes were removed.  Only the non-interfering
shadow telemetry and scorer are retained.  Full activation, run2, and sealed
run3 were not executed.
