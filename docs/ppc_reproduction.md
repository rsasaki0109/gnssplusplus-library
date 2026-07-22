# PPC Reproduction Commands

`PPC-Dataset` can be verified directly from an extracted dataset tree:

```bash
python3 apps/gnss.py ppc-demo \
  --dataset-root /datasets/PPC-Dataset \
  --city tokyo \
  --run run1 \
  --solver rtk \
  --require-realtime-factor-min 1.0 \
  --summary-json output/ppc_tokyo_run1_rtk_summary.json
```

## Current Sign-Off Profile

Use `ppc-rtk-signoff` for the deployable status profile:

```bash
python3 apps/gnss.py ppc-rtk-signoff \
  --dataset-root /datasets/PPC-Dataset \
  --city tokyo \
  --realtime-profile sigma-demote \
  --rtklib-bin /path/to/rnx2rtkp \
  --summary-json output/ppc_tokyo_run1_rtk_signoff.json
```

Use `ppc-coverage-matrix` to replay the full six-run `sigma-demote nis2-ratio4`
profile:

```bash
python3 apps/gnss.py ppc-coverage-matrix \
  --config-toml configs/benchmarks/ppc_sigma_demote_nis2_ratio4.toml
```

Override `--dataset-root` after `--config-toml` when your PPC checkout is not
under `data/PPC-Dataset`.

## Local MADOCALIB RTKLIB Smoke

If a standard RTKLIB `rnx2rtkp` is not installed but `external/madocalib` is
available locally, build its console app and use the PPC RTK config for the
fork-specific option enum:

```bash
make -C external/madocalib/app/consapp/rnx2rtkp/gcc

python3 apps/gnss.py ppc-coverage-matrix \
  --config-toml configs/benchmarks/ppc_sigma_demote_nis2_ratio4.toml \
  --max-epochs 20 \
  --rtklib-bin external/madocalib/app/consapp/rnx2rtkp/gcc/rnx2rtkp \
  --rtklib-config scripts/madocalib_ppc_rtk.conf \
  --output-dir output/ppc_rtklib_baseline_smoke \
  --summary-json output/ppc_rtklib_baseline_smoke/summary.json \
  --markdown-output output/ppc_rtklib_baseline_smoke/table.md
```

`ppc-demo` passes `-p 2` to `rnx2rtkp` for RTK comparisons. The MADOCALIB
config intentionally omits `pos1-posmode` because that fork's config enum does
not accept relative `kinematic`, while the CLI option does.

## Coverage Matrix

Use this for the README RTKLIB `demo5` comparison table:

```bash
python3 apps/gnss.py ppc-coverage-matrix \
  --dataset-root /datasets/PPC-Dataset \
  --rtklib-root output/benchmark \
  --ratio 2.4 \
  --summary-json output/ppc_coverage_matrix/summary.json \
  --markdown-output output/ppc_coverage_matrix/table.md

python3 scripts/update_ppc_coverage_readme.py \
  --summary-json output/ppc_coverage_matrix/summary.json
```

## Full Moving CLAS Comparison

Replay the complete available interval of all six Tokyo/Nagoya runs with the
same kinematic CLAS parity profile. The first command downloads/decodes QZSS L6
corrections, expands the dense SSR input, and runs `gnss_ppp`; allow substantial
disk space and runtime for the six full histories.

```bash
python3 scripts/generate_ppc_clas_scorecard.py \
  --dataset-root data/PPC-Dataset \
  --work-dir output/ppc_clas_full \
  --l6-cache output/ppc_clas_full/l6_cache \
  --configs parity \
  --report output/ppc_clas_full/scorecard.md

python3 scripts/generate_ppc_clas_full_comparison.py \
  --dataset-root data/PPC-Dataset \
  --results-dir output/ppc_clas_full \
  --metrics docs/ppc_clas_full_metrics.json \
  --markdown docs/ppc_clas_full_table.md \
  --trajectory-figure docs/ppc_clas_full_trajectories.png \
  --error-figure docs/ppc_clas_full_errors.png \
  --metric-figure docs/ppc_clas_full_comparison.png
```

Scoring rotates the dataset's city-specific body-frame lever arm through each
PPC attitude sample to compare at the antenna phase center: Tokyo uses
`[0.31, 0.0, -0.55]` m and Nagoya uses
`[0.593, -0.670, -1.216]` m in the PPC vehicle FRD convention (x forward,
y right, z down). The first 60
matched epochs are discarded independently per run, status 6 is FIX, and TTFF
starts at the first 30 consecutive FIX epochs. MRTKLIB's published v0.4.2
figures use the unmodified reference point, so README comparisons label that
reference-definition difference explicitly.

## Audited KF/FGO Goal Matrix And gici-open

The README goal matrix uses all six public Tokyo/Nagoya runs. The selected POS
paths and complete per-run metrics are recorded in
[`ppc_kf_fgo_goal_metrics.json`](ppc_kf_fgo_goal_metrics.json). Its final
Tokyo 1 tier is a position-only, reference-free choice between the preceding
selected trajectory and an independently generated tightly-coupled RTK
trajectory:

```bash
python3 scripts/select_pos_candidate_quality.py \
  --baseline-pos output/tokyo1_selected_quality_rtkbaseline_tier2_truthfree.pos \
  --candidate-pos output/tc_m3_full_t1_on/rtk.pos \
  --output-pos output/tokyo1_selected_quality_tcm3_tier3_truthfree.pos \
  --summary-json output/tokyo1_selected_quality_tcm3_tier3_truthfree.json \
  --candidate-status 4 --candidate-min-ratio 2 \
  --candidate-min-satellites 8 \
  --candidate-max-post-rms-m 0 \
  --candidate-max-nis-per-observation 0 \
  --min-position-separation-m 2
```

This selects 141 candidate positions. It preserves the baseline epoch grid,
status labels, and telemetry; it neither reads nor accepts a reference path.
The preceding tiers use the same invariant. Before the final confidence gate,
apply the Nagoya 2 wrong-basin escape. It replaces a baseline FIX position and
status with an independently generated tightly-coupled FGO FLOAT candidate only
when the baseline prefit RMS exceeds 8 m and at least 45 observations were
suppressed. The selector accepts no reference path:

```bash
python3 scripts/select_pos_candidate_quality.py \
  --baseline-pos output/nagoya2_selected_quality_rtkbaseline_tier2_truthfree.pos \
  --candidate-pos output/probe_fuse_nagoya2_full_tc_m4.pos \
  --output-pos output/nagoya2_wrong_basin_fgo_escape_truthfree.pos \
  --summary-json output/nagoya2_wrong_basin_fgo_escape_truthfree.json \
  --candidate-status 3 --candidate-min-ratio 0 \
  --candidate-min-satellites 0 \
  --candidate-max-post-rms-m 0 \
  --candidate-max-nis-per-observation 0 \
  --baseline-min-prefit-rms-m 8 \
  --baseline-min-outliers 45 \
  --replace-status \
  --min-position-separation-m 0
```

This selects 35 epochs. Apply the final truth-free confidence gate to all six
selected paths after applying the Nagoya 3 consensus escape:

```bash
python3 scripts/select_pos_candidate_quality.py \
  --baseline-pos output/nagoya3_selected_quality_rtkbaseline_truthfree.pos \
  --candidate-pos output/goal_kf_nagoya3_r2_min8_rescue29_8_cmcref/solution.pos \
  --output-pos output/nagoya3_consensus_escape_truthfree.pos \
  --summary-json output/nagoya3_consensus_escape_truthfree.json \
  --candidate-status 4 \
  --candidate-min-ratio 2 --candidate-max-ratio 2 \
  --candidate-min-satellites 12 \
  --candidate-max-post-rms-m 0 \
  --candidate-max-nis-per-observation 0 \
  --baseline-min-satellites 19 \
  --replacement-status 3 \
  --min-position-separation-m 5
```

This selects 27 epochs without a reference input. The alternative position is
emitted as FLOAT because the two filters disagree by at least 5 m while the
candidate AR ratio is at its acceptance boundary. Apply the final confidence
gate, then score the resulting outputs with the common metric implementation:

```bash
python3 scripts/apply_ppc_status_demotion.py \
  --pos tokyo_run1=output/tokyo1_selected_quality_tcm3_tier3_truthfree.pos \
  --pos tokyo_run2=output/tokyo2_selected_quality_rtkbaseline_tier2_truthfree.pos \
  --pos tokyo_run3=output/tokyo3_selected_quality_rtkbaseline_tier2_truthfree.pos \
  --pos nagoya_run1=output/hybrid_nagoya1_multistage_m4_fixedpos_bridge05_vertical025_veld_vertical10_truthfree.pos \
  --pos nagoya_run2=output/nagoya2_wrong_basin_fgo_escape_truthfree.pos \
  --pos nagoya_run3=output/nagoya3_consensus_escape_truthfree.pos \
  --output-dir output/ppc_kf_fgo_consensus_escape \
  --min-satellites 8 \
  --low-satellite-ceiling 11 \
  --low-satellite-max-ratio 15

python3 scripts/summarize_fgo_ppc_matrix.py \
  --dataset-root data/PPC-Dataset \
  --pos tokyo/run1=output/ppc_kf_fgo_consensus_escape/tokyo_run1.pos \
  --pos tokyo/run2=output/ppc_kf_fgo_consensus_escape/tokyo_run2.pos \
  --pos tokyo/run3=output/ppc_kf_fgo_consensus_escape/tokyo_run3.pos \
  --pos nagoya/run1=output/ppc_kf_fgo_consensus_escape/nagoya_run1.pos \
  --pos nagoya/run2=output/ppc_kf_fgo_consensus_escape/nagoya_run2.pos \
  --pos nagoya/run3=output/ppc_kf_fgo_consensus_escape/nagoya_run3.pos \
  --output-json output/kf_fgo_consensus_escape_ppc_matrix.json \
  --markdown-output output/kf_fgo_consensus_escape_ppc_matrix.md
```

The resulting distance-weighted official score is **78.716143%**. Tokyo 1 is
**80.952819% FIX**. The wrong-basin escape reduces Nagoya 2 wrong FIX above
10 m from 36 to 4 and above 5 m from 42 to 10. The consensus escape reduces
Nagoya 3 wrong FIX from 123 to 98 and wrong FIX above 10 m from 52 to 27. The
subsequent integrity gate
demotes 954 emitted FIX labels without changing any position. Across all six
runs the combined pipeline reduces wrong FIX from 1,385 to 869 and wrong FIX
above 5 m from 249 under the previous minimum-9 gate to 96, while retaining
the Tokyo 1 public 80.8% FIX target. The separate Nagoya 1 FIX-target profile
is generated by `gnss ppc-demo` with the
`goal_kf_current_r2_min8_rate20_rescue29_8` settings recorded in its summary
JSON; it measures **85.100974% FIX**, **0.912712% Wrong FIX/FIX**, and
**1.459572 m P95 H**. It is intentionally not substituted into the six-run
official-score matrix.

### Causal consensus and LOO kinematic integrity layer

The current first-stage integrity result replaces the offline Nagoya 3
position substitution above with a causal KF/FGO status-only replay. It never
reads a reference trajectory and never replaces the primary position:

```bash
python3 scripts/apply_ppc_integrity_consensus.py \
  --primary-pos output/nagoya3_selected_quality_rtkbaseline_truthfree.pos \
  --shadow-csv output/fgo_partial_noreset_ddpranchor_nagoya3_first2100_ecef.csv \
  --shadow-csv output/fgo_partial_noreset_ddpranchor_nagoya3_start2000_3000_ecef.csv \
  --output-pos output/nagoya3_selected_online_consensus_a11_recovery.pos \
  --summary-json output/nagoya3_selected_online_consensus_a11_recovery.json \
  --ledger-csv output/nagoya3_selected_online_consensus_a11_recovery_ledger.csv \
  --agreement-aperture-m 11 \
  --allow-recovery-fixed-output
```

Apply the satellite gate, its strong-telemetry exoneration, and the staged
causal kinematic quarantine in one pass:

```bash
python3 scripts/apply_ppc_status_demotion.py \
  --pos tokyo_run1=output/tokyo1_selected_quality_tcm3_tier3_truthfree.pos \
  --pos tokyo_run2=output/tokyo2_selected_quality_rtkbaseline_tier2_truthfree.pos \
  --pos tokyo_run3=output/tokyo3_selected_quality_rtkbaseline_tier2_truthfree.pos \
  --pos nagoya_run1=output/hybrid_nagoya1_multistage_m4_fixedpos_bridge05_vertical025_veld_vertical10_truthfree.pos \
  --pos nagoya_run2=output/nagoya2_wrong_basin_fgo_escape_truthfree.pos \
  --pos nagoya_run3=output/nagoya3_selected_online_consensus_a11_recovery.pos \
  --output-dir output/ppc_kf_fgo_online_consensus_kinematic_advanced \
  --min-satellites 8 \
  --low-satellite-ceiling 11 \
  --low-satellite-max-ratio 15 \
  --exonerate-min-satellites 11 \
  --exonerate-max-prefit-rms-m 0.5 \
  --exonerate-max-nis-per-obs 0.2 \
  --kinematic-max-jump-m 12 \
  --kinematic-min-acceleration-mps2 200 \
  --kinematic-hold-epochs 3 \
  --kinematic-plateau-max-jump-m 0.1 \
  --kinematic-max-hold-epochs 8 \
  --kinematic-secondary-min-jump-m 5 \
  --kinematic-secondary-min-acceleration-mps2 100 \
  --kinematic-secondary-min-prefit-rms-m 5 \
  --kinematic-secondary-max-ratio 10 \
  --kinematic-secondary-min-outliers 10 \
  --kinematic-secondary-max-satellites 13
```

Score that directory with `summarize_fgo_ppc_matrix.py` as above. The result is
78.716143% distance-weighted official score, 80.860848% Tokyo 1 FIX,
66.255853% macro correct FIX/ref, 1.932347% macro Wrong FIX/FIX, 810 total wrong
FIX, 42 wrong FIX above 5 m, and 5 above 10 m. The status-only layers increase
correct FIX by 9 epochs relative to the previous 869-wrong-FIX matrix.

`evaluate_ppc_kinematic_integrity_loo.py` reproduces the 5-run training / 1-run
held-out grid over jump `{8,10,12,15,20}` m, acceleration
`{50,100,200,400}` m/s2, and hold `{1,2,3,5}` epochs. Its checked-in output is
[`ppc_kinematic_integrity_loo.md`](ppc_kinematic_integrity_loo.md). Five folds
select 12 m / 200 m/s2 / 3 epochs and the Tokyo 3 held-out fold selects the same
thresholds with a five-epoch hold. Extension LOO fixes that primary policy and
selects plateau and secondary settings from the other five runs. All folds
choose an eight-epoch maximum plateau; exact secondary thresholds vary and
held-out incremental catches occur on only two runs. Reference truth is used
only inside this offline LOO scorer.

Generate the two independently initialized Tokyo 3 FGO shadow windows with the
shipping preset (repeat for `START=10950` and `START=11150`):

```bash
START=10950
gnss_fgo_parity \
  --rover data/PPC-Dataset/tokyo/run3/rover.obs \
  --base data/PPC-Dataset/tokyo/run3/base.obs \
  --nav data/PPC-Dataset/tokyo/run3/base.nav \
  --imu data/PPC-Dataset/tokyo/run3/imu.csv \
  --ref data/PPC-Dataset/tokyo/run3/reference.csv \
  --start-epoch "$START" --max-epochs 400 --fixed-lag 5 \
  --multi-freq --partial-ar --hold --elev-mask 25 --snr-mask 30 \
  --imu-preset-tactical --cmc --cmc-level 0.75 \
  --cp-hold --cp-hold-res 2.0 --exc-recovery --ddpr-anchor --fde --varerr \
  --fix-demote --fix-demote-dist 5 --fix-demote-res 25 \
  --fix-demote-posthold 5 \
  --dump-csv "output/fgo_shipping_tokyo3_start${START}_400_ecef.csv"
```

`--ref` is read only by the parity harness CSV writer for offline error columns;
the FGO solve receives rover/base/nav/IMU data only. Replay both absolute-ECEF
shadows through the truth-free status-only manager:

```bash
python3 scripts/apply_ppc_integrity_consensus.py \
  --primary-pos output/ppc_kf_fgo_online_consensus_kinematic_advanced/tokyo_run3.pos \
  --shadow-csv output/fgo_shipping_tokyo3_start10950_400_ecef.csv \
  --shadow-csv output/fgo_shipping_tokyo3_start11150_400_ecef.csv \
  --output-pos output/tokyo3_advanced_online_consensus_fgo_a5_gdop4_two_window.pos \
  --summary-json output/tokyo3_advanced_online_consensus_fgo_a5_gdop4_two_window_summary.json \
  --ledger-csv output/tokyo3_advanced_online_consensus_fgo_a5_gdop4_two_window_ledger.csv \
  --agreement-aperture-m 5 --shadow-max-gdop 4 \
  --allow-recovery-fixed-output
```

Use that Tokyo 3 POS with the other five advanced kinematic POS files. The
final matrix is 78.716143% official score, 80.860848% Tokyo 1 FIX,
66.250407% macro correct FIX/ref, 1.736789% macro Wrong FIX/FIX, 668 total
wrong FIX, 42 above 5 m, and 5 above 10 m. The status-only layers retain four
more correct FIX epochs than the original 869-wrong-FIX baseline.

For the competitor reproduction, check out
[`inuex35/gici-open`](https://github.com/inuex35/gici-open) at
`e7666110a88d22e08aad24345a253564af9b8024` (`forppc2024`), run its three
`option/tcN.yaml` configurations against the corresponding Tokyo and Nagoya
inputs, and export each NMEA trajectory. GICI remains a separate GPL-3.0
program; only its NMEA output crosses the benchmark boundary. Convert and score
each output with:

```bash
python3 scripts/convert_gici_nmea_to_pos.py \
  ../gici-open/reproduction_outputs/tokyo1.nmea \
  output/gici/tokyo1.pos --libgnss-status
```

Repeat the conversion for all six runs, then pass the six POS paths to
`summarize_fgo_ppc_matrix.py` with the same `--dataset-root` and `--pos`
layout shown above and `--match-tolerance-s 0.11`. The reproduced GICI macro FIX
rate is **54.325998%**, versus **71.073429%** for the selected libgnss++
matrix. GICI's local Windows/WSL replay required portability and RINEX-reader
fixes in its own GPL worktree; those changes are not copied into this MIT
repository. This is therefore a same-machine reproduction, not a claim that
the pristine upstream checkout runs the PPC data without adaptation.

Regenerate the machine-readable audit and both README figures from the scored
artifacts:

```bash
MPLBACKEND=Agg python3 scripts/generate_ppc_goal_scorecard.py \
  --lib-matrix output/kf_fgo_staged_integrity_full_matrix.json \
  --gici-matrix output/gici_reproduction_ppc_matrix.json \
  --nagoya1-public-summary output/goal_kf_current_r2_min8_rate20_rescue29_8/summary.json \
  --summary-json docs/ppc_kf_fgo_goal_metrics.json \
  --comparison-png docs/ppc_libgnss_gici_comparison.png \
  --targets-png docs/ppc_public_targets.png \
  --status-demotion-min-satellites 8 \
  --status-demotion-low-satellite-ceiling 11 \
  --status-demotion-low-satellite-max-ratio 15 \
  --status-exonerate-min-satellites 11 \
  --status-exonerate-max-prefit-rms-m 0.5 \
  --status-exonerate-max-nis-per-obs 0.2 \
  --kinematic-max-jump-m 12 \
  --kinematic-min-acceleration-mps2 200 \
  --kinematic-hold-epochs 3 \
  --kinematic-plateau-max-jump-m 0.1 \
  --kinematic-max-hold-epochs 8 \
  --kinematic-secondary-min-jump-m 5 \
  --kinematic-secondary-min-acceleration-mps2 100 \
  --kinematic-secondary-min-prefit-rms-m 5 \
  --kinematic-secondary-max-ratio 10 \
  --kinematic-secondary-min-outliers 10 \
  --kinematic-secondary-max-satellites 13 \
  --online-consensus-summary output/tokyo3_advanced_online_consensus_fgo_a5_gdop4_two_window_summary.json \
  --staged-integrity-audit docs/ppc_residual_integrity_external_audit.json

MPLBACKEND=Agg python3 scripts/plot_ppc_status_trajectories.py \
  --dataset-root data/PPC-Dataset \
  --metrics-json docs/ppc_kf_fgo_goal_metrics.json \
  --output docs/ppc_kf_fgo_fix_status_xy.png \
  --title "PPC 2024 selected trajectories by solution status" \
  --subtitle "Staged integrity: correct FIX, wrong FIX (>0.5 m 3D), FLOAT, and other states"
```

Regenerate the wrong-FIX severity and leave-one-run-out audit with
`scripts/analyze_ppc_wrong_fix_residuals.py`. Pass the previous `min_sat9` and
`integrity_gate` and `wrong_basin_escape` directories and the new
`consensus_escape` directory as `--profile`, plus the six pre-gate selected POS
paths as `--loo-pos`; the
tracked outputs are
[`ppc_fix_integrity_audit.md`](ppc_fix_integrity_audit.md) and
[`ppc_fix_integrity_audit.json`](ppc_fix_integrity_audit.json). The LOO sweep
uses only NumSat and AR ratio at runtime, caps training correct-FIX harm at 2%,
and selects by >5 m wrong FIX, total wrong FIX, then retained FIX count.

Build the contiguous wrong-FIX event ledger from the final six POS files with:

```bash
python3 scripts/build_ppc_wrong_fix_event_ledger.py \
  --dataset-root data/PPC-Dataset \
  --pos tokyo_run1=output/ppc_kf_fgo_staged_integrity_full/tokyo_run1.pos \
  --pos tokyo_run2=output/ppc_kf_fgo_staged_integrity_full/tokyo_run2.pos \
  --pos tokyo_run3=output/ppc_kf_fgo_staged_integrity_full/tokyo_run3.pos \
  --pos nagoya_run1=output/ppc_kf_fgo_staged_integrity_full/nagoya_run1.pos \
  --pos nagoya_run2=output/ppc_kf_fgo_staged_integrity_full/nagoya_run2.pos \
  --pos nagoya_run3=output/ppc_kf_fgo_staged_integrity_full/nagoya_run3.pos \
  --output-json docs/ppc_wrong_fix_event_ledger.json \
  --markdown-output docs/ppc_wrong_fix_event_ledger.md
```

Reference truth is used only to label completed events. Fingerprints and the
optional `--debug-log RUN_KEY=CSV` context use runtime telemetry. The tracked
ledger contains 574 epochs in 188 events, including 4 events with at least one
error above 10 m (5 such epochs). Native `--debug-epoch-log` output includes the
selected reference-satellite set and prior held-integer, consecutive-FIX, and
tracked ambiguity counts needed to distinguish a fresh LAMBDA failure from
inherited hold state.

The selector thresholds were tuned on this public benchmark. The audit proves
that truth is not consumed during selection and that FIX labels are not
inflated; it does not turn the public runs into a held-out validation set.

The native solver also exposes an experimental prefit wrong-basin reset through
`--max-fixed-prefit-rms`, `--min-fixed-prefit-outliers`, and
`--fixed-prefit-reset-streak`. It emits the trigger epoch as FLOAT, clears held
integers, and restarts ambiguity acquisition from SPP. On the raw Nagoya 2
profile, a 12 m / 45-outlier / one-epoch setting reduced Wrong FIX/FIX from
8.759% to 2.552%, but the official score fell from 39.783% to 32.631% because
FIX coverage did not recover quickly enough. It is therefore retained as an
opt-in diagnostic and is not part of the selected goal matrix.

### Staged bounded-latency integrity policy

After applying the frozen Tokyo 1/Tokyo 2 multi-shadow position consensus,
apply the base confidence and residual policies in one pass. They consume only
emitted status and RTK telemetry; the eight-epoch prefix requires at most seven
epochs (1.4 seconds at 5 Hz) of output buffering:

```bash
python3 scripts/apply_ppc_status_demotion.py \
  --input-dir output/ppc_kf_fgo_online_consensus_fresh_kinematic_holdout \
  --output-dir output/ppc_kf_fgo_staged_integrity_full \
  --min-satellites 8 \
  --low-satellite-ceiling 11 \
  --low-satellite-max-ratio 15 \
  --exonerate-min-satellites 11 \
  --exonerate-max-prefit-rms-m 0.5 \
  --exonerate-max-nis-per-obs 0.2 \
  --residual-streak-min-prefit-rms-m 40 \
  --residual-streak-max-ratio 15 \
  --residual-streak-min-outliers 12 \
  --residual-streak-min-outlier-fraction 0.5 \
  --residual-streak-epochs 8 \
  --residual-streak-buffer-prefix \
  --residual-spike-min-prefit-rms-m 40 \
  --residual-spike-max-satellites 14
```

Audit runtime/reference separation and per-run harm with:

```bash
python3 scripts/evaluate_ppc_residual_integrity_policy.py \
  --dataset-root data/PPC-Dataset \
  --input-dir output/ppc_kf_fgo_online_consensus_fresh_kinematic_holdout \
  --summary-json output/ppc_residual_integrity_fixed_policy_audit.json \
  --markdown-output output/ppc_residual_integrity_fixed_policy_audit.md
```

The staged matrix is 78.845491% distance-weighted official, 80.860848% Tokyo 1
FIX, 66.286505% correct FIX/reference, and 1.463246% macro Wrong FIX/FIX, with
574 total wrong FIX. The policy catches 57 wrong and harms ten correct FIX,
all on Nagoya 2; the other five runs are 0/0. It therefore clears the public
stretch metric. The original 15 m streak floor selected 27 otherwise-consistent
FIX epochs in the first 3000 Odaiba rover epochs. Raising only that floor to
40 m removes the Odaiba false alarm while preserving all 67 PPC selections and
the matrix above. A subsequent full Shinjuku replay exposed 22 false demotions
from a high-prefit sequence containing intermittent 11-outlier epochs. Raising
the consecutive-streak floor from ten to twelve outliers breaks that sequence
without changing any PPC selection. Requiring suppressed outliers to comprise
at least half of `RTKObs` additionally normalizes the guard across receiver
observation counts and also preserves all 67 PPC selections. These UrbanNav
u-blox runs are now development evidence, not untouched validation; the policy
was held outside the README sign-off pending a separately frozen receiver
holdout. With the complete staged policy (including the existing low-satellite
base gate), those development runs demote 57 FIX epochs, catch eight errors
above 2 m including all six Shinjuku errors above 10 m, and harm 49 correct FIX
epochs.

UrbanNav's Applanix reference point and rover antenna position have a roughly
0.7 m fixed separation in the initial Odaiba window. External audit commands
therefore record an explicit `--wrong-fix-threshold-m 2.0`; this affects only
offline labels and is not available to the runtime policy. PPC audits retain
the 0.5 m competition threshold.

After solving Odaiba and Shinjuku with the same `low-cost` RTK preset, audit
both full-run outputs without changing the frozen runtime thresholds:

```bash
python3 scripts/evaluate_ppc_residual_integrity_policy.py \
  --run urban_odaiba=output/urbannav_odaiba_full.pos,/datasets/UrbanNav-TK-20181219/Odaiba/reference.csv \
  --run urban_shinjuku=output/urbannav_shinjuku_full.pos,/datasets/UrbanNav-TK-20181219/Shinjuku/reference.csv \
  --match-tolerance-s 0.25 \
  --wrong-fix-threshold-m 2.0 \
  --summary-json output/urbannav_residual_integrity_full.json \
  --markdown-output output/urbannav_residual_integrity_full.md
```

The frozen policy was then evaluated on the separately solved Trimble rover
streams. Odaiba produced 10,956 valid solutions / 922 FIX (8.42% FIX), and
Shinjuku produced 17,707 / 51 (0.29% FIX). Across the 973 matched FIX epochs,
the complete staged policy selected 22 Shinjuku epochs, caught 22 of 49 FIX
errors above 2 m, harmed zero correct FIX, and reported
`safe_no_false_demotions`; the audit is
[`ppc_residual_integrity_external_audit.md`](ppc_residual_integrity_external_audit.md).
This is active receiver-diversity safety/efficacy evidence, although the very
low Shinjuku FIX coverage limits how broadly it can be generalized. It is the
external gate used for the README staged-result promotion.

## Historical Diagnostics

Older selector, residual-reset, IMU bridge, and tail-cleanup sweeps are kept as
benchmark diagnostics rather than README sign-off commands. See
[Benchmarks](benchmarks.md) for the scorecards and reproduction commands, and
[PPC realtime gate outputs](ppc_realtime_gate_existing_outputs.md) for the
accepted/rejected runtime status-demotion profiles.

## Provenance

The PPC summary records `receiver_observation_provenance` for the bundled
survey-grade rover/base RINEX streams. Proprietary receiver-engine solutions are
not assumed to be part of the PPC benchmark target.

RTK ionosphere sweeps can be run through `ppc-demo`, `ppc-rtk-signoff`, or
`ppc-coverage-matrix` with `--iono auto|off|iflc|est`; PPC summaries record the
requested value as `rtk_iono`. Ambiguity-ratio sweeps use `--ratio <value>`;
PPC summaries record the requested value as `rtk_ratio_threshold`.

Dataset source: [taroz/PPC-Dataset](https://github.com/taroz/PPC-Dataset)

Verify the complete metric, artifact, external-validation, CI, and license
contract with:

```bash
python3 scripts/verify_ppc_goal_completion.py \
  --summary-json docs/ppc_goal_completion_audit.json \
  --markdown-output docs/ppc_goal_completion_audit.md
```
