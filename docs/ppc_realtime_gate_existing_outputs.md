# PPC existing real-time gate outputs

This note compares existing full six-run deployable RTK gate outputs. PPC
reference truth is used only after the run for scoring and wrong-FIX labeling.

Wrong FIX is `status == FIXED` with post-run 3D reference error above 0.50 m.

## Current recommendation

The final deployable status profile is the sigma-profile replay with
`--demote-fixed-status-nis-per-obs 2`. It minimizes the measured Wrong/FIX rate
among the current real-time-only sweep:

| profile | official | FIX epochs | Wrong FIX | Wrong/FIX | min realtime | decision |
|---|---:|---:|---:|---:|---:|---|
| current_sigma | 64.750% | 41608 | 3078 | 7.398% | 6.883x | control |
| current_sigma_demote_nis2 | 64.707% | 29705 | 563 | 1.895% | 7.046x | recommended when minimizing Wrong/FIX |

The tradeoff is explicit: `nis2` drops the weighted official score by 0.043 pp
and emits 11903 fewer FIX epochs, but reduces Wrong FIX by 2515 epochs. The
sections below are chronological rejected/diagnostic probes unless explicitly
marked as the current recommendation.

## Full six-run outputs

| profile | deployable gate | official | delta vs baseline | FIX epochs | Wrong FIX | Wrong/FIX | decision |
|---|---|---:|---:|---:|---:|---:|---|
| baseline_sigma001 | none beyond coverage profile | 64.001% | +0.000 pp | 42762 | 5333 | 12.471% | baseline |
| fixed_nis10 | `--max-fixed-update-nis-per-obs 10` | 62.512% | -1.489 pp | 38954 | 4758 | 12.214% | rejects fewer wrong FIX but loses too much score |
| fixed_post6 | `--max-fixed-update-post-rms 6` | 62.606% | -1.395 pp | 42397 | 5739 | 13.536% | reject |
| nis10_ratio6 | `--max-fixed-update-nis-per-obs 10 --max-fixed-update-gate-ratio 6` | 63.373% | -0.628 pp | 41350 | 4962 | 12.000% | safer but still lower score |
| nis8_ratio6 | `--max-fixed-update-nis-per-obs 8 --max-fixed-update-gate-ratio 6` | 22.318% | -41.683 pp | 3752 | 33 | 0.880% | reject, collapses FIX/score |
| snr_nis8_window | SNR weighting plus NIS gate in 7000-9000 m baseline window | 63.929% | -0.071 pp | 44393 | 7214 | 16.250% | reject, more wrong FIX |
| secondary_window | SNR weighting plus primary/secondary fixed-update windows | 64.326% | +0.325 pp | 44355 | 7106 | 16.021% | reject, more wrong FIX |
| secondary_window_reset12 | secondary_window plus non-FIX reset12 | 63.315% | -0.685 pp | 45089 | 9491 | 21.049% | reject |

The current no-worse-Wrong-FIX policy rejects every profile that improves
official score. The stricter `nis8_ratio6` experiment confirms that lowering the
NIS threshold can almost eliminate wrong FIX, but it does so by collapsing fixed
coverage and official score. The least-bad safe existing full output remains
`nis10_ratio6`, but it still gives up 0.628 pp official score.

## Partial post4 / nis20 probes

Only three runs exist locally for the stricter POS-replay candidates, so they
are not a six-run claim. They explain why a global gate is risky:

| run | profile | official | FIX epochs | Wrong FIX | Wrong/FIX |
|---|---|---:|---:|---:|---:|
| tokyo_run1 | baseline | 63.376% | 9018 | 1319 | 14.626% |
| tokyo_run1 | fixed_post4 | 58.959% | 8544 | 1878 | 21.980% |
| tokyo_run1 | fixed_nis20 | 60.330% | 8410 | 1877 | 22.319% |
| nagoya_run2 | baseline | 29.958% | 5431 | 1919 | 35.334% |
| nagoya_run2 | fixed_post4 | 32.866% | 5142 | 380 | 7.390% |
| nagoya_run2 | fixed_nis20 | 27.654% | 5071 | 945 | 18.635% |
| nagoya_run3 | baseline | 40.861% | 2741 | 881 | 32.142% |
| nagoya_run3 | fixed_post4 | 36.619% | 1978 | 684 | 34.580% |
| nagoya_run3 | fixed_nis20 | 44.932% | 2275 | 423 | 18.593% |

`post4` is excellent for Nagoya run2 but bad for Tokyo run1 and Nagoya run3.
`nis20` is good for Nagoya run3 but bad for Tokyo run1 and lower-score on
Nagoya run2. The next deployable path should therefore be a conditional gate
using real-time diagnostics such as baseline length, ratio, NIS, post residual,
and possibly speed; a global fixed-update threshold is not enough.

## Narrow baseline-window probe

POS-only analysis suggested that the 8000-8500 m baseline bucket is high-risk,
especially for Nagoya run2. Direct solver replays show that simply adding this
window to the low-ratio NIS gate is still too aggressive because early fixed
candidate rejection feeds back into later ambiguity reacquisition:

| run | profile | official | FIX epochs | Wrong FIX | Wrong/FIX |
|---|---|---:|---:|---:|---:|
| nagoya_run2 | baseline | 29.958% | 5431 | 1919 | 35.334% |
| nagoya_run2 | nis10_ratio6 | 33.240% | 5341 | 621 | 11.627% |
| nagoya_run2 | nis8_ratio6 | 10.431% | 758 | 30 | 3.958% |
| nagoya_run2 | nis8_ratio6_bl8000_8500 | 10.431% | 758 | 30 | 3.958% |
| nagoya_run2 | nis10_ratio6_bl8000_8500 | 10.431% | 758 | 30 | 3.958% |

This rejects the simple baseline-window version. The safer existing result for
Nagoya run2 remains `nis10_ratio6`: it improves official score and cuts Wrong
FIX substantially on that run, but the full six-run aggregate still loses
0.628 pp official score.

## Status-only demotion probe

The fixed-update reject gates above feed back into ambiguity reacquisition. A
gentler deployable pattern is to keep the solver trajectory untouched, but
demote only the emitted status from FIXED to FLOAT when real-time diagnostics
flag a risky fixed epoch. `scripts/apply_ppc_status_demotion.py` reproduces that
POS-only status demotion without using reference truth.

On the baseline `sigma_0p001` six-run output, `--max-ratio 6
--max-nis-per-obs 10` demotes 3185 of 42762 FIXED epochs:

| profile | official | FIX epochs | Wrong FIX | Wrong/FIX |
|---|---:|---:|---:|---:|
| baseline_sigma001 | 64.001% | 42762 | 5333 | 12.471% |
| status_demote_nis10_ratio6 | 64.001% | 39577 | 3273 | 8.270% |

This keeps official score because positions are unchanged, while reducing
truth-labeled wrong FIX by 2060 epochs. It should be treated as the next
implementation candidate: move the same real-time status demotion into the
solver/app output path so the internal fixed-state recovery is not disrupted.

## Runtime status-demotion replay

The runtime implementation was replayed with the same deployable rule,
`--demote-fixed-status-nis-per-obs 10 --demote-fixed-status-gate-ratio 6`.
It preserves the internal fixed solution for feedback and receiver-position
seeding, but the emitted status is demoted before PPC scoring.

Full six-run output:

| profile | official | FIX epochs | Wrong FIX | Wrong/FIX | min realtime | decision |
|---|---:|---:|---:|---:|---:|---|
| baseline_sigma001 | 64.001% | 42762 | 5333 | 12.471% | n/a | baseline |
| pos_status_demote_nis10_ratio6 | 64.001% | 39577 | 3273 | 8.270% | n/a | POS-only diagnostic |
| current_build_no_demote | 22.318% | 4195 | 36 | 0.858% | 6.467x | dirty-build control |
| runtime_status_demote_nis10_ratio6 | 22.318% | 4137 | 34 | 0.822% | 6.570x | neutral vs current control |

Per-run runtime output:

| run | positioning | FIX | official |
|---|---:|---:|---:|
| tokyo_run1 | 89.8% | 7.2% | 9.2% |
| tokyo_run2 | 94.5% | 9.8% | 21.7% |
| tokyo_run3 | 96.8% | 7.1% | 38.3% |
| nagoya_run1 | 86.0% | 10.6% | 20.9% |
| nagoya_run2 | 83.2% | 9.6% | 10.4% |
| nagoya_run3 | 94.0% | 0.1% | 4.9% |

The runtime replay was initially compared against the older `sigma_0p001`
artifact, but a fair no-demotion control from the same command surface also
lands at 22.318% official with only 4195 FIX epochs. Therefore this run is not a
valid demotion-vs-baseline rejection. Apples-to-apples, runtime status demotion
is nearly neutral: it changes 4195 FIX / 36 Wrong FIX to 4137 FIX / 34 Wrong FIX
at the same official score.

The gap is configuration, not the demotion implementation. The historical
`sigma_0p001` artifact was generated by the direct solver sweep with additional
RTK tuning flags including `--carrier-phase-sigma 0.001`, `--ratio 2.8`,
`--max-postfix-rms 0.2`, `--max-consec-float-reset 10`,
`--max-subset-ar-drop-steps 18`, adaptive dynamic slip thresholds, and position
jump guards. Replaying Tokyo run1 with those flags gives 8747 FIX epochs
(84.08% fix rate) and 67.043% official score, versus 777 FIX epochs and 9.195%
official from the plain `--ratio 2.4` matrix command. The next full runtime
demotion check should use the sigma-profile command surface.

## Sigma-profile runtime demotion

The sigma-profile command surface was then replayed through
`ppc-coverage-matrix` with and without runtime status demotion:

```text
--ratio 2.8
--carrier-phase-sigma 0.001
--max-postfix-rms 0.2
--max-consec-float-reset 10
--max-subset-ar-drop-steps 18
--adaptive-dynamic-slip-thresholds
--adaptive-dynamic-slip-nonfix-count 25
--max-pos-jump 5.0
--max-pos-jump-min 5.0
--max-pos-jump-rate 25.0
```

Full six-run current-build comparison:

| profile | official | FIX epochs | Wrong FIX | Wrong/FIX | min realtime | decision |
|---|---:|---:|---:|---:|---:|---|
| current_sigma | 64.750% | 41608 | 3078 | 7.398% | 6.883x | control |
| current_sigma_demote_nis10_ratio6 | 64.750% | 39138 | 1699 | 4.341% | 6.413x | candidate |

Per-run Wrong FIX:

| run | current_sigma | current_sigma_demote |
|---|---:|---:|
| tokyo_run1 | 936 | 629 |
| tokyo_run2 | 160 | 103 |
| tokyo_run3 | 870 | 385 |
| nagoya_run1 | 124 | 91 |
| nagoya_run2 | 537 | 290 |
| nagoya_run3 | 451 | 201 |

This is the first runtime status-demotion result that satisfies the current
mis-fix policy on the sigma-profile surface: weighted official score is
unchanged while 1379 Wrong FIX epochs are removed. FIX output drops by 2470
epochs, so the next check should decide whether that status conservatism is
acceptable for the headline profile or should be narrowed.

## Narrowed sigma-profile demotion

Because runtime demotion is output-only, the no-demotion sigma-profile POS files
can be replayed offline to narrow the thresholds before another full solver run.
Among the threshold sweep, `--demote-fixed-status-nis-per-obs 20
--demote-fixed-status-gate-ratio 6 --max-demote-fixed-status-baseline 9500`
kept Wrong/FIX below 5% while recovering most of the FIX epochs lost by the
stricter `nis10_ratio6` candidate.

Full runtime replay:

| profile | official | FIX epochs | Wrong FIX | Wrong/FIX | FIX drop | Wrong drop | min realtime | decision |
|---|---:|---:|---:|---:|---:|---:|---:|---|
| current_sigma | 64.750% | 41608 | 3078 | 7.398% | 0 | 0 | 6.883x | control |
| current_sigma_demote_nis10_ratio6 | 64.750% | 39138 | 1699 | 4.341% | 2470 | 1379 | 6.413x | safer, more conservative |
| current_sigma_demote_nis20_ratio6_maxbl9500 | 64.750% | 39924 | 1928 | 4.829% | 1684 | 1150 | 7.259x | narrower candidate |

Per-run Wrong FIX for the narrower candidate:

| run | current_sigma | nis10_ratio6 | nis20_ratio6_maxbl9500 |
|---|---:|---:|---:|
| tokyo_run1 | 936 | 629 | 647 |
| tokyo_run2 | 160 | 103 | 126 |
| tokyo_run3 | 870 | 385 | 436 |
| nagoya_run1 | 124 | 91 | 112 |
| nagoya_run2 | 537 | 290 | 307 |
| nagoya_run3 | 451 | 201 | 300 |

The narrowed candidate is a better headline tradeoff if the policy target is
Wrong/FIX below 5% rather than minimizing Wrong FIX at all costs. It recovers 786
FIX epochs compared with `nis10_ratio6`, keeps the same 64.750% official score,
and still removes 1150 Wrong FIX epochs versus the no-demotion sigma control.

## Aggressive sigma-profile demotion

The 5% target still leaves too many Wrong FIX epochs. A stricter deployable
runtime replay with `--demote-fixed-status-nis-per-obs 3
--demote-fixed-status-gate-ratio 15` pushes the full-matrix Wrong FIX count below
1000 while keeping the weighted official score unchanged.

Full runtime replay:

| profile | official | FIX epochs | Wrong FIX | Wrong/FIX | FIX drop | Wrong drop | min realtime | decision |
|---|---:|---:|---:|---:|---:|---:|---:|---|
| current_sigma | 64.750% | 41608 | 3078 | 7.398% | 0 | 0 | 6.883x | control |
| current_sigma_demote_nis20_ratio6_maxbl9500 | 64.750% | 39924 | 1928 | 4.829% | 1684 | 1150 | 7.259x | too many Wrong FIX |
| current_sigma_demote_nis3_ratio15 | 64.750% | 35382 | 980 | 2.770% | 6226 | 2098 | 6.274x | aggressive candidate |

Per-run Wrong FIX for the aggressive candidate:

| run | current_sigma | nis20_ratio6_maxbl9500 | nis3_ratio15 |
|---|---:|---:|---:|
| tokyo_run1 | 936 | 647 | 442 |
| tokyo_run2 | 160 | 126 | 75 |
| tokyo_run3 | 870 | 436 | 218 |
| nagoya_run1 | 124 | 112 | 27 |
| nagoya_run2 | 537 | 307 | 157 |
| nagoya_run3 | 451 | 300 | 61 |

This is the better headline profile when status correctness matters more than
maximizing output FIX count. It removes 2098 Wrong FIX epochs versus the
no-demotion sigma control, and 948 more than the 5% narrowed candidate, at the
cost of 4542 additional output FIX epochs.

## Ultra-conservative sigma-profile demotion

Pushing Wrong/FIX lower requires dropping the ratio gate entirely and demoting
every output FIX with `--demote-fixed-status-nis-per-obs 2`. This is the lowest
Wrong/FIX runtime result in the current deployable sweep, but unlike the milder
runtime-demotion profiles it gives up a small amount of official score.

Full runtime replay:

| profile | official | FIX epochs | Wrong FIX | Wrong/FIX | FIX drop | Wrong drop | min realtime | decision |
|---|---:|---:|---:|---:|---:|---:|---:|---|
| current_sigma | 64.750% | 41608 | 3078 | 7.398% | 0 | 0 | 6.883x | control |
| current_sigma_demote_nis3_ratio15 | 64.750% | 35382 | 980 | 2.770% | 6226 | 2098 | 6.274x | sub-1000 Wrong FIX |
| current_sigma_demote_nis2 | 64.707% | 29705 | 563 | 1.895% | 11903 | 2515 | 7.046x | lowest Wrong/FIX |

Per-run Wrong FIX for `nis2`:

| run | current_sigma | nis3_ratio15 | nis2 |
|---|---:|---:|---:|
| tokyo_run1 | 936 | 442 | 271 |
| tokyo_run2 | 160 | 75 | 65 |
| tokyo_run3 | 870 | 218 | 123 |
| nagoya_run1 | 124 | 27 | 6 |
| nagoya_run2 | 537 | 157 | 69 |
| nagoya_run3 | 451 | 61 | 29 |

`nis2` is the right headline status profile if the goal is to minimize Wrong/FIX
as much as possible with deployable real-time fields. It reduces Wrong/FIX to
1.895%, but the cost is high: 5677 fewer FIX epochs than `nis3_ratio15`, and a
weighted official score drop of 0.043 pp.
