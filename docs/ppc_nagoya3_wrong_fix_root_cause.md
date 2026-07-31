# Nagoya 3 catastrophic wrong-FIX root cause

This investigation uses PPC reference truth only after each solver run to label
errors. Every proposed runtime discriminator is available in the native POS or
`--debug-epoch-log` stream.

## Reproduction

The historical `goal_kf_nagoya3_r2_min8_rescue29_8` artifact contains a
106.130 m, nine-epoch wrong-FIX event at GPS TOW 554493.600-554495.200. The
current selected six-run profile retains that historical position source but
demotes some other events through truth-free integrity selectors.

Replaying the same settings on the current solver gives two deliberately
contrasting results:

| Position jump policy | FIX | Wrong FIX | >10 m FIX | Correct FIX | Interpretation |
|---|---:|---:|---:|---:|---|
| Current default, 5 m | 206 | 0 | 0 | 206 | Catastrophic candidates are rejected, but the stale trusted anchor prevents FIX recovery. |
| `--max-pos-jump 0` | 2,182 | 870 | 297 | 1,312 | Coverage recovers, but the filter promotes a collapsed wrong basin. |
| Historical artifact | 2,116 | 362 | 277 | 1,754 | Older behavior used by the selected trajectory source. |
| Anchor age 5 s + overconfidence reset | 1,905 | 664 | 110 | 1,241 | Two resets fire, but the anchor expires while the KF is still wrong and permits early wrong-basin reacquisition. |
| Anchor age 155 s + overconfidence reset | 745 | 357 | 0 | 388 | Catastrophic errors disappear, but periodic expiry destroys useful FIX coverage. |
| Anchor age 25 s + conditional 10 m Doppler consensus | 689 | 9 | 0 | 680 | Independent Doppler continuity rejects the wrong basin, but its long-gap drift rejects too many later correct candidates. |

The no-jump replay reproduces the same failure family as one contiguous
114-epoch wrong-FIX event at TOW 554472.600-554495.200 with 94.697 m maximum
3D error. Its position differs from the older artifact, but the time interval,
residual signature, and wrong-basin behavior coincide.

## Runtime evidence

For the reproduced 114-epoch event:

| Signal | Event value | Meaning |
|---|---:|---|
| Float position covariance trace, median | 0.000809 m2 | The KF is extremely confident in its wrong position. |
| DD prefit residual RMS, median | 11.942 m | Raw observations strongly disagree with the state. |
| DD post-suppression RMS, median | 0.688 m | Robust suppression makes the remaining observations appear acceptable. |
| Suppressed observations, POS median | 39 | A large part of the measurement set is discarded. |
| AR ratio, median | 4.5 | The integer test can still pass after covariance collapse. |
| Prior held integers | 0 | The event is not inherited from a previous hold state. |
| Selected references | 5 distinct sets | Reference membership changes repeatedly inside the basin. |
| Subset AR | active | Four- to twelve-pair subsets repeatedly produce accepted candidates. |

The event therefore starts as a **float KF wrong basin**, not as a fresh
held-integer failure. LAMBDA and hold FIX subsequently certify positions that
are only centimeters from the already-wrong float state. A fixed/float jump
gate cannot detect this condition because both solutions share the same basin.

## Required fix

A safe recovery policy must combine:

1. high DD prefit residual RMS;
2. sustained observation suppression;
3. implausibly small position covariance;
4. reference/subset instability; and
5. an independent continuity source such as Doppler/IMU FGO.

The opt-in overconfidence reset now clears the collapsed KF position/covariance,
ambiguity state, and stale trusted/fixed anchors. Offline threshold screening on
the no-jump replay found that prefit RMS >10 m, at least 35 cumulative suppressed
observations, and covariance trace <=0.01 m2 selected 123/297 >10 m wrong FIX
epochs and zero correct FIX epochs. In a causal replay, however, a five-second
anchor expiry reacquired the still-wrong basin and a 155-second expiry removed
too much coverage. Time expiry alone is therefore rejected as a sign-off policy.

The next implementation step is an online independent-KF/FGO consensus permit:
an expired anchor may be replaced only after a separately initialized estimator
agrees for consecutive epochs. Reference truth remains offline scoring data only.
The Doppler-only prototype validates the consensus concept but also shows why a
fixed-distance dead-reckoning gate is insufficient: after the last accepted FIX
at TOW 554006.8, its uncorrected track drifts by tens to hundreds of meters.
The next estimator must carry uncertainty and obtain independent absolute or IMU
corrections instead of integrating Doppler indefinitely.
The implementation contract and rollout gates are specified in
[`ppc_online_consensus_design.md`](ppc_online_consensus_design.md).

## Artifacts

- [`ppc_wrong_fix_event_ledger.json`](ppc_wrong_fix_event_ledger.json): final
  six-run event inventory.
- `output/nagoya3_rootcause_base_debug.csv`: current default-jump replay.
- `output/nagoya3_rootcause_nojump_debug.csv`: no-jump reproduction with
  reference-satellite and integrity-state telemetry.
- `output/nagoya3_rootcause_nojump_ledger.json`: detailed reproduced event.
- `output/nagoya3_anchor_overconf_r10_o35_c001_full_summary.json`: five-second
  anchor-expiry causal replay.
- `output/nagoya3_anchor155_overconf_r10_o35_c001_full_summary.json`: 155-second
  anchor-expiry causal replay.
- `output/nagoya3_conditional_doppler10_anchor25_summary.json`: conditional
  Doppler-consensus causal replay.

The `output/` diagnostics are local benchmark artifacts and are not required at
runtime.
