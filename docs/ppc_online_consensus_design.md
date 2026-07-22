# Online KF/FGO integrity consensus design

This design turns the offline Nagoya 2/3 trajectory selectors into a causal,
truth-free runtime integrity manager. PPC reference trajectories remain offline
labels and never enter the manager.

## Evidence and design constraints

Nagoya 3 exposes three distinct failure modes:

1. a permanent last-FIX anchor rejects every later candidate;
2. time-based anchor expiry reacquires the still-wrong KF basin; and
3. Doppler-only dead reckoning rejects the wrong basin but drifts too far to
   authorize later recovery.

The manager must therefore separate **detection** from **recovery authority**.
High prefit residuals, suppression storms, covariance collapse, and Doppler
disagreement may quarantine the primary KF. None of those signals alone may
promote a new FIX anchor. Recovery requires agreement with an independently
initialized estimator carrying its own uncertainty and absolute/IMU updates.

## Runtime inputs

All inputs are available without reference truth:

- primary KF position, covariance, status, AR ratio, and ambiguity count;
- DD prefit/postfit residuals, NIS, and suppressed-observation count;
- reference-satellite/subset membership and change counters;
- independent FGO/KF position, covariance, age, reset generation, and health;
- Doppler/IMU continuity innovation; and
- time since the last jointly trusted anchor.

The independent estimator must not share the primary KF position, ambiguity
state, robust-suppression decisions, or accepted integer constraints when it is
initialized. Shared rover/base observations are unavoidable, so estimator
agreement is necessary but not by itself sufficient.

## State machine

| State | FIX output | Entry | Exit |
|---|---|---|---|
| `NORMAL` | Primary policy | No active integrity evidence | Suspect score persists for N epochs or hard disagreement fires |
| `SUSPECT` | FLOAT | One or more correlated failure signals | Return to `NORMAL` after short clean streak, or enter `QUARANTINE` |
| `QUARANTINE` | FLOAT/SPP | Hard wrong-basin evidence or repeated suspect epochs | Independent estimator is healthy and agreement persists |
| `RECOVERY` | FLOAT, or provisional FIX after the separate health gate | Primary and independent estimates agree inside uncertainty aperture | Promote jointly trusted anchor after M consecutive epochs |

A missing or over-uncertain independent estimate cannot authorize recovery.
It also must not silently clear `QUARANTINE`; the solver continues with FLOAT or
SPP until evidence is available.

The provisional FIX output used in replay does not shorten the recovery streak
or promote an anchor. It additionally requires ratio >=3, estimator separation
<=6 m, prefit RMS <=5 m, and at most 20 suppressed outliers. This restored five
correct Nagoya 3 labels while restoring no wrong label.

Nagoya 2 also established a negative result: its FGO graph can remain in a
wrong basin until chunk reinitialization, and a standalone DDPR-LS anchor may
collapse to four factors with a misleadingly small residual. The FGO trajectory
and four-factor anchor therefore cannot authorize recovery. Detection evidence
and recovery authority remain separate interfaces.

## Multi-shadow position recovery

A single long-running FGO shadow is not position authority. On Tokyo 1 it would
recover 87 primary epochs but make 3363 unsafe replacements. Position recovery
therefore requires a unique largest cluster from at least two distinct,
independently restarted FIX shadows. The current truth-free gates are:

- cluster diameter at most 0.25 m;
- separation from the primary of at least 0.5 m;
- at least one cluster member younger than 1000 emitted epochs; and
- at most 2 m error against a causal two-epoch constant-velocity prediction.

Equal-size competing clusters, duplicate source IDs, missing prediction
history, stale-only clusters, and non-FIX/unhealthy shadows fail closed. The
selected cluster position is its coordinate-wise ECEF median. Primary status,
epoch grid, and telemetry are preserved.

The offline Python replay implements the same decision boundary in
`scripts/apply_ppc_fgo_position_consensus.py`. The library-side
`MultiShadowPositionConsensus` exposes it without a GTSAM dependency. Tokyo 1
replay replaced 37 positions for 34 wrong-to-correct and 2 correct-to-wrong
changes. The thresholds were then frozen; an unseen Tokyo 2 replay replaced
five positions, all wrong-to-correct. Across the selected six-run probe this
reduces total wrong FIX from 668 to 631 and macro Wrong FIX/FIX from 1.736789%
to 1.664773%, while preserving 78.845491% weighted official score, 80.860848%
Tokyo 1 FIX, and 66.304140% correct FIX/reference. This remains a staged probe
until more held-out and external-city shadows are available.

## Apertures and uncertainty

The agreement aperture is covariance-aware:

```text
d = ||p_primary - p_independent||
sigma2 = trace(P_primary_xyz + P_independent_xyz)
agree = d <= clamp(min_m, k_sigma * sqrt(sigma2), max_m)
```

The aperture is paired with absolute health ceilings for both estimators so a
pair of highly uncertain estimates cannot agree by using an arbitrarily large
bound. A reset-generation mismatch requires a fresh recovery streak.

## Initial rollout

1. Add a pure `IntegrityConsensusManager` with deterministic state-machine unit
   tests and no solver dependencies.
2. Extend native epoch telemetry with state, reason bits, disagreement, aperture,
   independent age/covariance, and recovery streak.
3. Feed it two causally produced trajectories in the replay harness before
   wiring an in-process shadow estimator.
4. Validate thresholds with leave-one-run-out: tune on five PPC runs and report
   the held-out run without retuning.
5. Require six-run gates for official score, correct FIX/ref, wrong FIX, >5 m,
   >10 m, and runtime before enabling any preset.
6. Repeat on a non-PPC urban dataset and an IMU-absent fallback configuration.

## Acceptance gates

The first runtime candidate must keep the current six-run official score at or
above 78.7%, correct FIX/ref at or above 66.230%, and Tokyo 1 FIX at or above
80.8%, while reducing >10 m wrong FIX from 59 to at most 30 and total wrong FIX
from 869 to at most 700. The final target is >10 m at most 10 and macro wrong
FIX/FIX below 1.5%.

## Current staged result

The causal N3 replay, staged kinematic output guard, and two-window Tokyo 3 FGO
consensus replay keep the official score at 78.716143%, correct FIX/ref at
66.250407%, and Tokyo 1 FIX at 80.860848%. They reduce total wrong FIX to 668,
>5 m wrong FIX to 42, >10 m wrong FIX to 5, and macro Wrong FIX/FIX to
1.736789%. This clears the final >10 m and total-wrong gates; macro <1.5%
remains open. The Tokyo 3 layer uses 5 m separation, FGO GDOP <=4, DDPR RMS
<=40 m, and nsat >=8. Two overlapping independent initializations remove 142
wrong and 5 correct FIX labels and finish in NORMAL state without replacing a
position. Six-fold LOO selects the bounded kinematic plateau extension
consistently, while secondary-trigger thresholds and held-out benefit vary;
that component remains exploratory until external validation.

The later multi-shadow position probe plus a bounded-latency residual policy
reaches **1.463246% macro Wrong FIX/FIX** on the six public runs. The residual
policy buffers at most seven epochs, demotes a confirmed eight-epoch sequence
with prefit RMS >40 m, ratio <=15, at least twelve suppressed outliers, and a
suppressed-outlier/RTK-observation fraction of at least 0.5, and
also rejects a single-epoch spike when prefit RMS is at least 40 m with at most
14 satellites. Together the residual gates catch 57 wrong FIX and harm ten
correct FIX epochs, all on Nagoya 2. Total wrong FIX is 574; weighted official
score remains 78.845491%, correct FIX/reference is 66.286505%, and Tokyo 1 FIX
remains 80.860848%. This became the README staged result only after the external
audit described below. A 3000-epoch UrbanNav Odaiba holdout rejected the original 15 m floor:
it selected 27 otherwise-consistent FIX epochs whose 3D errors were tightly
clustered at 0.67--0.69 m (consistent with the dataset's reference/antenna
offset). A 40 m floor removes those selections without changing any PPC
selection. Full Shinjuku then exposed 22 false demotions in a sequence with
intermittent 11-outlier epochs. Raising the streak floor to twelve outliers
and requiring an outlier/`RTKObs` fraction of at least 0.5 breaks that sequence
without changing any PPC selection. The u-blox UrbanNav
runs are therefore development evidence; a separately frozen receiver/dataset
holdout was still required. The subsequently frozen Trimble rover full-run
holdout selected 22 of 973 matched FIX epochs, caught 22 FIX errors above 2 m,
and harmed zero correct FIX, yielding
`safe_no_false_demotions` in
[`ppc_residual_integrity_external_audit.md`](ppc_residual_integrity_external_audit.md).
Because Shinjuku had only 51 FIX epochs (0.29%), this is active but limited
receiver-diversity evidence rather than a broad multi-city generalization.

## License boundary

The manager, state machine, telemetry schema, tests, and libgnss++ estimator
interfaces remain MIT-licensed. External GPL reference solvers and datasets are
invoked only by benchmark adapters and are never copied, linked, or vendored
into the library. Optional GTSAM use follows its permissive upstream license;
the consensus API must also work without GTSAM.
