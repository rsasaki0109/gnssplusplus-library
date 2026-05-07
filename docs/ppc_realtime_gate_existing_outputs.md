# PPC existing real-time gate outputs

This note compares existing full six-run deployable RTK gate outputs. PPC
reference truth is used only after the run for scoring and wrong-FIX labeling.

Wrong FIX is `status == FIXED` with post-run 3D reference error above 0.50 m.

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
| runtime_status_demote_nis10_ratio6 | 22.318% | 4137 | 34 | 0.822% | 6.570x | reject, collapses FIX/score |

Per-run runtime output:

| run | positioning | FIX | official |
|---|---:|---:|---:|
| tokyo_run1 | 89.8% | 7.2% | 9.2% |
| tokyo_run2 | 94.5% | 9.8% | 21.7% |
| tokyo_run3 | 96.8% | 7.1% | 38.3% |
| nagoya_run1 | 86.0% | 10.6% | 20.9% |
| nagoya_run2 | 83.2% | 9.6% | 10.4% |
| nagoya_run3 | 94.0% | 0.1% | 4.9% |

This closes the status-only path as a performance candidate in its global
`NIS/obs > 10, ratio <= 6` form. The POS-only replay was useful for isolating
wrong-FIX labels, but the deployable PPC score depends on the emitted FIX
status. Demoting too many FIX epochs makes the official score collapse even
when positions remain available.
