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

The offline status replay also supports a truth-free kinematic integrity layer.
The six-fold LOO profile uses a 12 m jump, 200 m/s2 acceleration, and a
three-epoch hold. The advanced replay can extend a post-trigger plateau to an
eight-epoch maximum while jumps remain <=0.1 m. Its secondary trigger combines
jump/acceleration with prefit RMS, AR ratio, suppressed-outlier count, and
satellite count; LOO support for its exact thresholds is limited, so it remains
exploratory. Low-satellite exoneration requires nsat >=11, prefit RMS <=0.5 m,
and NIS/observation <=0.2; kinematic quarantine always overrides that
exoneration. These are `apply_ppc_status_demotion.py` replay options, not native
RTK defaults.

The online consensus manager does not let a soft primary-only telemetry warning
create a quarantine while its independent estimator is unavailable, because
such a state has no evidence source with which to recover. A hard primary
suspect remains fail-closed. `--allow-soft-suspect-without-shadow` restores the
legacy diagnostic behavior. For tight-aperture experiments,
`--require-primary-suspect-for-disagreement` requires joint primary telemetry
and estimator disagreement to advance the disagreement streak; it is opt-in.

## Opt-In Gates

| Flag | Purpose | Default |
|------|---------|---------|
| `--ar-policy {extended\|demo5-continuous}` | AR extras gate. `demo5-continuous` disables relaxed-hold-ratio / subset-fallback / hold-fix / Q-regularization for demo5-style continuous ambiguity tracking. | `extended` |
| `--max-subset-ar-drop-steps <N>` | Extend the progressive subset-AR search by dropping up to N worst-variance DD pairs. Diagnostic only: Nagoya run2 spot checks showed extra candidates are usually rejected by the jump gate unless paired with riskier validation changes. | `6` |
| `--max-hold-div <m>` | Reject fix if the hold-state diverges from float by more than N meters. | `0` disabled |
| `--hold-fix-unchanged-set` | Refuse the hold-fix fallback when a current DD pair has no held integer (a satellite joined or an ambiguity was reset), instead of rounding that pair. Fewer wrong FIX, fewer FIX: on PPC + UrbanNav Tokyo, `gnss solve --preset low-cost` went from 5.77 % to 3.10 % of FIXED epochs more than 0.5 m (3D) off the reference, with 7.5 % fewer FIXED epochs. Also in `gnss live` and `gnss replay` ([details](interfaces.md#rtk-hold-fix-fallback-known-weakness)). | `false` |
| `--hold-fix-min-matched-pairs <n>` | Refuse the hold-fix fallback when fewer than n current DD pairs carry a held integer. The default 4 is the floor it always had. With 12, `gnss solve --preset low-cost` on PPC + UrbanNav Tokyo went from 5.77 % to 2.85 % of FIXED epochs more than 0.5 m (3D) off the reference, keeping 101 % of the correct FIXED epochs. On held-out SmartPNT-POS and TEX-CUP urban runs it went from 1.69 % to 1.07 %, keeping 98.5 %. Also in `gnss live` and `gnss replay` ([details](interfaces.md#rtk-hold-fix-fallback-known-weakness)). | `4` |
| `--min-fix-satellites <k>` | Refuse an ambiguity-resolution FIX (full set, subset or WL/NL fallback) whose fixed set has fewer than k non-reference satellites. The epoch stays FLOAT unless the hold-fix fallback applies. With 4 added to `--preset low-cost --hold-fix-min-matched-pairs 12`, held-out GREAT-WHU, SmartPNT-POS and TEX-CUP urban runs went from 0.95 % to 0.37 % of FIXED epochs more than 0.5 m (3D) off the reference, keeping 97.5 % of the correct FIXED epochs. `gnss solve` on PPC + UrbanNav Tokyo went from 2.85 % to 1.22 %, keeping 97.6 %. Also in `gnss live` and `gnss replay` ([details](interfaces.md#rtk-fixes-from-few-satellites)). | `0` disabled |
| `--max-pos-jump <m>` | Reject fix if the epoch-to-epoch position jump exceeds N meters. Truth-validation against PPC reference showed jumps cluster at <5 m for correct fixes or >10 m for wrong FIX. Pass `0` to disable. | `5` m |
| `--max-fixed-anchor-age <s>` | Stop using the last FIX position as a jump reference after N seconds. This prevents permanent stale-anchor lockout, but Nagoya 3 causal replays showed that time expiry alone either reacquires a wrong basin or loses too much coverage; use only for diagnostics. | `0` disabled |
| `--max-fixed-doppler-consensus <m>` | When the FIX anchor is expired or the residual/covariance gate is suspect, reject FIX candidates farther than N meters from an independently propagated rover-Doppler position track. Accepted FIX only may re-anchor the track. Nagoya 3 at 10 m reduced wrong FIX from 664 to 9, but long-gap Doppler drift reduced correct FIX from 1,241 to 680, so this is an integrity experiment rather than a sign-off default. | `0` disabled |
| `--max-pos-jump-min <m>` + `--max-pos-jump-rate <m/s>` | Reject fix if the jump from the last fixed position exceeds `max(min, rate * dt)`, so vehicle gaps can be tested without a stale absolute distance clamp. | `0` / `0` disabled |
| `--max-float-spp-div <m>` | Reject FLOAT epochs that diverge from the same-epoch SPP solution by more than N meters, then fall back to SPP/no-solution. Diagnostic gate for PPC FLOAT high-error sweeps. | `0` disabled |
| `--max-float-prefit-rms <m>` + `--max-float-prefit-max <m>` + `--max-float-prefit-reset-streak <N>` | Reset ambiguity states for the next epoch after N consecutive otherwise accepted FLOAT epochs have high DD prefit residual RMS or max residual. The current FLOAT epoch is still reported. | `0` / `0` disabled / `3` |
| `--min-float-prefit-trusted-jump <m>` | Continuity selector for the residual gate. When > 0, high-residual FLOAT resets are allowed only if the FLOAT position has also diverged by at least N meters from the last trusted FIX/FLOAT position. | `0` disabled |
| `--max-fixed-prefit-rms <m>` + `--min-fixed-prefit-outliers <N>` + `--fixed-prefit-reset-streak <N>` | Detect an accepted FIX in a high-prefit-residual basin after suppression, emit the trigger epoch as FLOAT, clear held integers and stale anchors, and restart ambiguity acquisition from the same-epoch SPP position. Both residual thresholds are required. Nagoya 2/3 diagnostics reduced large wrong FIX but also reduced official-score coverage, so this remains experimental and is not used by the PPC sign-off profile. | `0` / `0` disabled / `1` |
| `--max-fixed-overconfidence-cov-trace <m2>` | Add an upper FLOAT position-covariance-trace condition to the fixed-prefit reset, targeting filters that are implausibly confident while raw DD residuals remain large. It has no effect unless the fixed-prefit residual gate is enabled. | `0` disabled |
| `--max-update-nis-per-obs <v>` | Reject a whole RTK DD Kalman update before state/covariance mutation when normalized innovation squared divided by active observations exceeds N. | `0` disabled |
| `--max-fixed-update-nis-per-obs <v>` | Reject only FIXED ambiguity candidates when the preceding RTK DD update NIS divided by active observations exceeds N, while retaining the FLOAT solution for that epoch. | `0` disabled |
| `--max-fixed-update-post-rms <m>` | Reject only FIXED ambiguity candidates when the preceding RTK DD update post-suppression residual RMS exceeds N meters, while retaining the FLOAT solution for that epoch. | `0` disabled |
| `--max-fixed-update-gate-ratio <v>` | Apply the fixed-only NIS/post-RMS gates only when the AR ratio is finite and at or below N. | `0` unconditional when gates are enabled |
| `--min-fixed-update-gate-baseline <m>` + `--max-fixed-update-gate-baseline <m>` | Optional baseline-length window for the fixed-only NIS/post-RMS gates. | `0` / `0` disabled |
| `--min-fixed-update-gate-speed <m/s>` + `--max-fixed-update-gate-speed <m/s>` | Optional speed window for the fixed-only NIS/post-RMS gates. | `0` / `0` disabled |
| `--max-fixed-update-secondary-gate-ratio <v>` + baseline/speed secondary window flags | Optional second OR-window for fixed-only NIS/post-RMS gates. | `0` for all fields |
| `--demote-fixed-status-max-ratio <v>` | Keep the position solution but output FIX as FLOAT when the AR ratio is finite and at or below N. PPC `nis2-ratio4` uses this to reduce measured Wrong/FIX while preserving official-score coverage. | `0` disabled |
| `--demote-fixed-status-min-satellites <N>` | Keep the position solution but output FIX as FLOAT when fewer than N satellites are used. The audited KF/FGO integrity matrix uses 8 together with the paired low-satellite ratio gate; it changes no trajectory positions and keeps Tokyo 1 above the public FIX target. | `0` disabled |
| `--demote-fixed-status-low-satellite-ceiling <N>` + `--demote-fixed-status-low-satellite-max-ratio <R>` | Keep the position solution but output FIX as FLOAT when at most N satellites are used and the AR ratio is at most R. Both options are required. The audited integrity profile combines N=11/R=15 with minimum 8 satellites. | `0` disabled |
| `--rtk-snr-weighting` + `--rtk-snr-reference-dbhz <v>` + `--rtk-snr-max-variance-scale <v>` + `--rtk-snr-min-baseline <m>` | Low-cost observation model diagnostic. Uses the lower rover/base SNR for each SD link and inflates DD phase/code variance below the reference SNR, capped by the max scale. | `false` / `45` / `25` / `0` |
| `--rtk-snr-elevation-aware` + `--rtk-snr-code-only` | Refinements of `--rtk-snr-weighting` (need it enabled). Elevation-aware measures the C/N0 deficit against an expected open-sky C/N0 of `33 + 15 sin(el)` dB-Hz (capped by the reference) instead of the constant reference, so low-elevation satellites are not penalised twice and zenith-side satellites blocked by an overpass (48 to 33 dB-Hz) are. Code-only applies the inflation to code rows only, so the phase weight and the fix rate are kept. On UrbanNav Odaiba (stopped under an overpass, 10-29 m vertical float error) the default profile Vp95 drops 11.5 to 5.8 m; PPC coverage-profile weighted official is +1.06 pp (elevation-aware) / +0.23 pp (plus code-only) but not uniform per run (nagoya_run3 -7.0 / -2.6 pp), so both stay opt-in. | `false` / `false` |
| `--rtk-canyon-carrier-prior` | Kinematic RTK. The per-epoch SPP re-seed discards the position/ambiguity cross-covariance, so under a blocked sky the biased code defines the three float combinations that a stationary geometry cannot observe. While a zenith-side (>= 45 deg) satellite's code disagrees by more than the 3 m outlier gate with the position advanced from the previous posterior by a validated time-differenced carrier displacement (code +4..+14 m, carrier intact), that propagation replaces the re-seed (cross-covariances kept; process noise 1 cm/s) and re-initialised ambiguities are seeded from the carrier at the propagated position instead of from code. It starts only from a posterior that was code/carrier consistent for 5 s, releases after 5 s of consistency, lasts at most 180 s per episode and suspends the preset float-prefit reacquisition reset while engaged. Development set UrbanNav Odaiba only when this row was written (PPC and Hong Kong not yet evaluated): stop under the overpass 273750-273860 Vp95 27.2 to 2.0 m (default profile) and 24.4 to 3.9 m (`--preset odaiba`); epochs outside both stops unchanged; the second stop (274230-274300) is not engaged. Stays opt-in. | `false` |
| `--cycle-slip-threshold <m>` + `--doppler-slip-threshold <m>` + `--code-slip-threshold <m>` + `--strict-dynamic-slip-thresholds` + `--adaptive-dynamic-slip-thresholds` | Cycle-slip sensitivity sweep. Dynamic RTK keeps protective minimum thresholds by default; strict/adaptive modes are opt-in urban reacquisition experiments. | `0.05` / `0.20` / `5.0` / `false` / `false` |
| `--adaptive-dynamic-slip-nonfix-count <N>` | Non-FIX epochs before adaptive dynamic slip thresholds activate. | `3` |
| `--adaptive-dynamic-slip-hold-epochs <N>` | Epochs to keep adaptive dynamic slip thresholds after activation. | `10` |
| `--nonfix-drift-max-residual <m>` + `--nonfix-drift-min-horizontal-residual <m>` | Tighten the default low-speed non-FIX drift guard for tail diagnostics while avoiding vertical-only fallback pruning. | `30` / `0` |
| `--fixed-bridge-burst-guard` + `--fixed-bridge-burst-max-residual <m>` | Reject isolated short FIX bursts when they diverge from the straight bridge between surrounding FIX anchors. | `false` / `20` |
| `--max-consec-float-reset <N>` | Auto-reset ambiguities after N consecutive float epochs. `10` is a historical PPC official-score probe; the current PPC sign-off path is `--realtime-profile sigma-demote`. | `0` disabled |
| `--max-consec-nonfix-reset <N>` | Auto-reset ambiguities after N consecutive FLOAT/SPP/no-solution epochs. | `0` disabled |
| `--max-postfix-rms <m>` | Reject fix if the L1 post-fix DD phase residual RMS exceeds N meters. | `0` disabled |
| `--enable-wide-lane-ar` + `--wide-lane-threshold <cycle>` | Pre-compute MW wide-lane integers and inject them as Kalman constraints into the LAMBDA search. | `false` / `0.25` |
| `--wide-lane-min-arc-samples <N>` | Average the MW wide-lane over each satellite's continuous arc and constrain only when both arcs hold at least N samples. `0` is the legacy single-epoch test, which admits near-random integers under urban code multipath. `--preset odaiba` uses `100`. | `0` |
| `--enable-wlnl-fallback` | Allow the experimental MW wide-lane / narrow-lane fallback after ordinary LAMBDA failure in non-IFLC runs. It reuses `--wide-lane-threshold` for both integer checks. | `false` |
| `--enable-bsr-decimation` + `--bsr-worst-axes <N>` + `--bsr-max-drops <N>` | Add BSR-guided subset AR decimation candidates alongside the variance-drop subset family. PPC high-wrong probes preserved FIX count but did not reduce Wrong/FIX, so this remains diagnostic. | `false` / `3` / `6` |

On PPC Tokyo and Nagoya, leave these off unless reproducing a specific sweep.
On Odaiba or other urban multipath datasets, `--preset odaiba` (low-cost +
arc-smoothed wide-lane AR) gives the most fixes and the lowest Hmed. Its
FLOAT/SPP epochs in two stationary deep-canyon stretches carry up to 28 m
vertical error, so its Hp95/Vp95 are above the default profile's on the
2026-10-11 Odaiba run, both over all epochs (5.28/13.89 m against 4.77/11.54 m)
and over the epochs both profiles publish (5.11/11.81 m against 4.57/10.52 m)
([benchmarks](benchmarks.md#urbannav-tokyo-odaiba-snapshot)).

For those stationary deep-canyon errors, see
[Urban canyon RTK options](use_cases/urban_canyon_rtk_options.md): it compares
`--rtk-canyon-carrier-prior` and the SNR options on Odaiba, Hong Kong and PPC,
and warns against combining the carrier prior with the SNR options under
`--preset odaiba` (wrong fixes 12 → 1,009 on Odaiba).
