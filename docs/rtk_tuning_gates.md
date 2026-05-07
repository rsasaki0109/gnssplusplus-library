# RTK Tuning Gates

The default RTK pipeline is the sign-off path for the PPC production runs.
Additional gates are default-off unless noted here and are intended for
diagnostics, profile sweeps, or dataset-specific opt-in profiles such as
Odaiba.

## Default Guards

The low-speed non-FIX drift guard is part of the default kinematic output path,
including the coverage profile. It rejects long FLOAT/SPP fallback drifts only
when the surrounding FIX anchors indicate near-stationary motion. Use
`--no-nonfix-drift-guard` to reproduce the raw unguarded fallback stream.

The SPP height-step guard is also default-on in the kinematic output path. It
rejects SPP-only vertical spikes above `--spp-height-step-min` /
`--spp-height-step-rate` while preserving FLOAT and FIXED epochs.

The FLOAT bridge-tail guard is default-on after six-run PPC sign-off. It rejects
FLOAT epochs in slow bounded FIX-to-FIX segments when they diverge from the
anchor bridge, and uses horizontal FIX-anchor speed for its motion gate. Use
`--no-float-bridge-tail-guard` to reproduce the pre-bridge-tail coverage stream.

## Opt-In Gates

| Flag | Purpose | Default |
|------|---------|---------|
| `--ar-policy {extended\|demo5-continuous}` | AR extras gate. `demo5-continuous` disables relaxed-hold-ratio / subset-fallback / hold-fix / Q-regularization for demo5-style continuous ambiguity tracking. | `extended` |
| `--max-subset-ar-drop-steps <N>` | Extend the progressive subset-AR search by dropping up to N worst-variance DD pairs. Diagnostic only: Nagoya run2 spot checks showed extra candidates are usually rejected by the jump gate unless paired with riskier validation changes. | `6` |
| `--max-hold-div <m>` | Reject fix if the hold-state diverges from float by more than N meters. | `0` disabled |
| `--max-pos-jump <m>` | Reject fix if the epoch-to-epoch position jump exceeds N meters. Truth-validation against PPC reference showed jumps cluster at <5 m for correct fixes or >10 m for wrong FIX. Pass `0` to disable. | `5` m |
| `--max-pos-jump-min <m>` + `--max-pos-jump-rate <m/s>` | Reject fix if the jump from the last fixed position exceeds `max(min, rate * dt)`, so vehicle gaps can be tested without a stale absolute distance clamp. | `0` / `0` disabled |
| `--max-float-spp-div <m>` | Reject FLOAT epochs that diverge from the same-epoch SPP solution by more than N meters, then fall back to SPP/no-solution. Diagnostic gate for PPC FLOAT high-error sweeps. | `0` disabled |
| `--max-float-prefit-rms <m>` + `--max-float-prefit-max <m>` + `--max-float-prefit-reset-streak <N>` | Reset ambiguity states for the next epoch after N consecutive otherwise accepted FLOAT epochs have high DD prefit residual RMS or max residual. The current FLOAT epoch is still reported. | `0` / `0` disabled / `3` |
| `--min-float-prefit-trusted-jump <m>` | Continuity selector for the residual gate. When > 0, high-residual FLOAT resets are allowed only if the FLOAT position has also diverged by at least N meters from the last trusted FIX/FLOAT position. | `0` disabled |
| `--max-update-nis-per-obs <v>` | Reject a whole RTK DD Kalman update before state/covariance mutation when normalized innovation squared divided by active observations exceeds N. | `0` disabled |
| `--max-fixed-update-nis-per-obs <v>` | Reject only FIXED ambiguity candidates when the preceding RTK DD update NIS divided by active observations exceeds N, while retaining the FLOAT solution for that epoch. | `0` disabled |
| `--max-fixed-update-post-rms <m>` | Reject only FIXED ambiguity candidates when the preceding RTK DD update post-suppression residual RMS exceeds N meters, while retaining the FLOAT solution for that epoch. | `0` disabled |
| `--max-fixed-update-gate-ratio <v>` | Apply the fixed-only NIS/post-RMS gates only when the AR ratio is finite and at or below N. | `0` unconditional when gates are enabled |
| `--min-fixed-update-gate-baseline <m>` + `--max-fixed-update-gate-baseline <m>` | Optional baseline-length window for the fixed-only NIS/post-RMS gates. | `0` / `0` disabled |
| `--min-fixed-update-gate-speed <m/s>` + `--max-fixed-update-gate-speed <m/s>` | Optional speed window for the fixed-only NIS/post-RMS gates. | `0` / `0` disabled |
| `--max-fixed-update-secondary-gate-ratio <v>` + baseline/speed secondary window flags | Optional second OR-window for fixed-only NIS/post-RMS gates. | `0` for all fields |
| `--demote-fixed-status-max-ratio <v>` | Keep the position solution but output FIX as FLOAT when the AR ratio is finite and at or below N. PPC `nis2-ratio4` uses this to reduce measured Wrong/FIX while preserving official-score coverage. | `0` disabled |
| `--rtk-snr-weighting` + `--rtk-snr-reference-dbhz <v>` + `--rtk-snr-max-variance-scale <v>` + `--rtk-snr-min-baseline <m>` | Low-cost observation model diagnostic. Uses the lower rover/base SNR for each SD link and inflates DD phase/code variance below the reference SNR, capped by the max scale. | `false` / `45` / `25` / `0` |
| `--cycle-slip-threshold <m>` + `--doppler-slip-threshold <m>` + `--code-slip-threshold <m>` + `--strict-dynamic-slip-thresholds` + `--adaptive-dynamic-slip-thresholds` | Cycle-slip sensitivity sweep. Dynamic RTK keeps protective minimum thresholds by default; strict/adaptive modes are opt-in urban reacquisition experiments. | `0.05` / `0.20` / `5.0` / `false` / `false` |
| `--adaptive-dynamic-slip-nonfix-count <N>` | Non-FIX epochs before adaptive dynamic slip thresholds activate. | `3` |
| `--adaptive-dynamic-slip-hold-epochs <N>` | Epochs to keep adaptive dynamic slip thresholds after activation. | `10` |
| `--nonfix-drift-max-residual <m>` + `--nonfix-drift-min-horizontal-residual <m>` | Tighten the default low-speed non-FIX drift guard for tail diagnostics while avoiding vertical-only fallback pruning. | `30` / `0` |
| `--fixed-bridge-burst-guard` + `--fixed-bridge-burst-max-residual <m>` | Reject isolated short FIX bursts when they diverge from the straight bridge between surrounding FIX anchors. | `false` / `20` |
| `--max-consec-float-reset <N>` | Auto-reset ambiguities after N consecutive float epochs. `10` is a historical PPC official-score probe; the current PPC sign-off path is `--realtime-profile sigma-demote`. | `0` disabled |
| `--max-consec-nonfix-reset <N>` | Auto-reset ambiguities after N consecutive FLOAT/SPP/no-solution epochs. | `0` disabled |
| `--max-postfix-rms <m>` | Reject fix if the L1 post-fix DD phase residual RMS exceeds N meters. | `0` disabled |
| `--enable-wide-lane-ar` + `--wide-lane-threshold <cycle>` | Pre-compute MW wide-lane integers and inject them as Kalman constraints into the LAMBDA search. Odaiba's opt-in preset uses this to beat demo5 Hmed while still beating demo5 Fix count and tails. | `false` / `0.25` |
| `--enable-wlnl-fallback` | Allow the experimental MW wide-lane / narrow-lane fallback after ordinary LAMBDA failure in non-IFLC runs. It reuses `--wide-lane-threshold` for both integer checks. | `false` |
| `--enable-bsr-decimation` + `--bsr-worst-axes <N>` + `--bsr-max-drops <N>` | Add BSR-guided subset AR decimation candidates alongside the variance-drop subset family. PPC high-wrong probes preserved FIX count but did not reduce Wrong/FIX, so this remains diagnostic. | `false` / `3` / `6` |

On PPC Tokyo and Nagoya, leave these off unless reproducing a specific sweep.
On Odaiba or other urban multipath datasets, use `--preset odaiba` for the
explicit demo5-beating tradeoff.
