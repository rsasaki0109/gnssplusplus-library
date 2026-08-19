# Clock-resilient temporal carrier shadow

## Purpose

Receiver clock jumps make undifferenced Doppler and TDCP continuity checks
unsafe when the jump is not modelled consistently. This diagnostic forms a
satellite single difference before taking the time difference:

```
((carrier_target - range_target) - (carrier_ref - range_ref))_k
  - ((carrier_target - range_target) - (carrier_ref - range_ref))_{k-1}
```

A carrier-domain term common to every satellite at epoch `k`, including a
receiver clock jump, cancels in the first satellite difference. This is a
diagnostic shadow only: it is never inserted into the Eigen or GTSAM graph and
cannot change the exported position, ambiguity decision, or FIX/FLOAT status.

## Continuity rules

- Observations are grouped by constellation and signal.
- The highest-elevation usable satellite starts as the causal reference.
- The reference remains fixed while it is usable. A reference change starts a
  new temporal arc; no factor bridges the change.
- Both target and reference must have carrier phase and no loss-of-lock flag.
- Only adjacent epochs within `max_tdcp_gap_s` are compared.
- `previous_los` comes from the previous epoch and `los` from the current
  epoch. The legacy Taroz-parity SD-TDCP builder remains unchanged.

Enable the audit in `gnss_fgo_parity` with
`--clock-resilient-temporal-shadow`. With `--dump-csv`, the following columns
are emitted:

- `clock_resilient_tdcp_n`
- `clock_resilient_tdcp_rms_m`
- `clock_resilient_tdcp_max_abs_m`
- `clock_resilient_tdcp_clean`
- `clock_resilient_tdcp_witnessed_outliers`
- `clock_resilient_tdcp_unexplained_outliers`

A normalized per-factor trace is also written automatically beside the epoch
table as `<path>.clock_resilient_tdcp_factors.csv`. Each row includes the
target/reference/signal arc, residual, normalized residual, raw-observation
Doppler innovation and uncertainty, normalized Doppler innovation,
geometry-free/hold/FDE witnesses, and final
`clean`/`witnessed_outlier`/`unexplained_outlier` classification. The frozen
diagnostic thresholds are 0.10 m for the temporal-carrier residual and 5 sigma
for the Doppler-integrated innovation. The latter combines temporal-carrier
sigma with the trapezoid-integrated single-difference Doppler sigmas. They are
deliberately not estimator gates.

## Tokyo validation

The evaluated production profile was first replayed for the first 500 run1
epochs with the shadow disabled and enabled. All 500 rows had identical ECEF
position, solution status, ambiguity ratio, fixed count, and AR outcome.

| Metric | Result |
|---|---:|
| Epochs with a temporal carrier diagnostic | 499 / 500 |
| Evaluated satellite-difference factors | 22,482 |
| Factor-weighted RMS | 0.0122 m |
| Per-epoch RMS median / P95 | 0.006 / 0.028 m |
| Maximum absolute residual | 0.511 m |
| Epochs with maximum residual above 0.1 m | 41 |
| Epochs with maximum residual above 1 m | 0 |
| FIX epochs, disabled / enabled | 499 / 499 |

The centimetre-scale bulk distribution is promising, but the 0.511 m tail is
too large for direct graph insertion at millimetre carrier sigma. The next
step is to classify the tail using loss-of-lock, geometry-free, and Doppler
consistency witnesses, then test robust gating on held-out runs before this
measurement is allowed to influence the estimator.

The same frozen profile and thresholds were then run over all three Tokyo
sequences. Run1 was the design sequence, run2 the fixed evaluation sequence,
and run3 the holdout. Each run was compared with its pre-existing baseline;
ECEF position, status, ratio, fixed count, and AR outcome matched on every
row.

| Run | Rows | Active epochs | Factors | Epoch RMS P50 / P95 | Max abs. | Epochs max >0.1 m | Covered by existing witness |
|---|---:|---:|---:|---:|---:|---:|---:|
| Tokyo run1 | 11,905 | 11,860 | 277,226 | 0.011 / 0.301 m | 480.248 m | 2,896 | 2,059 |
| Tokyo run2 | 9,147 | 9,069 | 292,607 | 0.008 / 0.061 m | 16.115 m | 952 | 469 |
| Tokyo run3 holdout | 15,294 | 15,095 | 513,824 | 0.007 / 0.190 m | 147.929 m | 2,743 | 1,972 |
| Total | 36,346 | 36,024 | 1,083,657 | - | 480.248 m | 6,591 | 4,500 |

An existing witness means at least one geometry-free event, Doppler event,
carrier-FDE exclusion, or active carrier hold occurred in the same epoch.
That coarse coverage still leaves 2,091 epochs above 0.1 m unexplained. The
factor-weighted RMS is 1.20 m because a small number of gross carrier-arc
errors dominate it, even though all three medians are 7--11 mm.

Decision: keep this path monitor-only. Receiver-clock cancellation works and
the bulk signal is useful, but epoch-level guards are not selective enough for
safe graph insertion. Per-factor innovations and target/reference arc
classifications are therefore required before any later graph experiment.
Only a frozen gate that survives run2 and run3 should be promoted to a graph
factor.

### Per-factor classifier checkpoint

The classifier was then replayed over the same first 500 run1 epochs. Raw
single-difference Doppler is carried with each shadow factor, so the witness is
available even when Doppler graph factors are disabled by the production
profile.

| Metric | Result |
|---|---:|
| Factors / residual outliers | 22,482 / 50 |
| Outliers with an independent witness | 10 (20%) |
| Unexplained outliers | 40 (80%) |
| Doppler-evaluated factors | 22,482 (100%) |
| Factors above the 5-sigma Doppler threshold | 5,570 (24.8%) |
| Factors with causal arc calibration | 21,188 (94.2%) |
| Factors above the calibrated 5-sigma threshold | 233 (1.10%) |
| Residual outliers above the calibrated threshold | 0 / 50 |
| Residual outliers with a geometry-free witness | 10 |
| Maximum temporal-carrier residual | 0.511 m |
| Baseline rows changed | 0 / 500 |

Replacing the original 0.20 m Doppler threshold with the propagated 5-sigma
test reduced its background flag rate only from 27.4% to 24.8%. Per-signal
quantiles also varied widely and would suppress genuine carrier tails if used
as fixed thresholds. The dominant effect was instead a persistent
target/reference arc bias.

A causal arc calibration was therefore added: after ten prior samples, the
current signed innovation is centred by the median of at most 30 prior samples
and scaled by the larger of propagated measurement sigma and `1.4826 * MAD`.
The current sample is appended only after classification, so the diagnostic
does not use future data or dilute its own event. This reduced the background
flag rate to 1.10%, but none of the 50 temporal-carrier residual tails was a
calibrated Doppler event at any tested threshold from 3 to 10 sigma. The
previous Doppler coverage was therefore caused by persistent bias rather than
independent transient evidence.

Decision: raw and propagated-sigma Doppler values remain in the trace, but no
longer have classification authority. Only a causally calibrated Doppler
event, geometry-free event, carrier hold, or carrier-FDE rejection can mark a
residual as witnessed. On the run1 design slice this leaves the ten
geometry-free-witnessed tails and correctly marks the other 40 as unexplained.
No Doppler-derived gate should be inserted into the estimator unless a future
holdout shows transient-event correlation that is absent here.

### Solver-independent truth replay and holdouts

`gnss_fgo_parity --clock-resilient-shadow-truth-replay --ref reference.csv
--dump-csv <path>` evaluates the same shadow and classifier at the high-precision
reference ECEF trajectory without running GTSAM. A cached 9,147-epoch run2
problem replays in seconds; run3 including RINEX parsing, problem construction,
cache creation, and 513,824-factor classification completed in under three
minutes on the validation machine.

The run1 500-epoch truth replay produced the same 50-outlier count as the full
solver evaluation and matched 49 of the 50 factor identities. This is stronger
than replaying the exported solution CSV: that table rounds ECEF to millimetres
and exports the selected FIXED position, while the original shadow is evaluated
at the internal float trajectory.

The frozen 0.10 m residual and causal 5-sigma witness rules were then applied
unchanged to run2 and run3:

| Run | Factors | Residual tails | Tail rate | Witnessed | Unexplained | Unexplained share | Calibrated Doppler flags | Max abs. |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| run1 design (500 epochs) | 22,482 | 50 | 0.22% | 10 | 40 | 80.0% | 233 / 21,188 (1.10%) | 0.511 m |
| run2 holdout | 292,607 | 1,222 | 0.42% | 209 | 1,013 | 82.9% | 7,915 / 268,009 (2.95%) | 2.219 m |
| run3 holdout | 513,824 | 2,309 | 0.45% | 314 | 1,995 | 86.4% | 15,300 / 475,744 (3.22%) | 2.397 m |
| Total | 828,913 | 3,581 | 0.43% | 533 | 3,048 | 85.1% | - | - |

Final decision: do not insert this temporal-carrier path into the graph and do
not use it as an ambiguity/FIX gate. Receiver-clock cancellation is correct,
but tail frequency roughly doubles on both holdouts and 85.1% of all tails
lack an independent witness. The monitor, normalized trace, and fast truth
replay remain useful research/diagnostic surfaces. FIX-rate development should
now move to independently validated ambiguity selection and integrity evidence
rather than further TDCP threshold tuning.

## References and implementation precedents

- Momoh, Bhattarai, and Ziebart (2019), receiver clock jump and cycle-slip
  correction using adaptive time differences and WLS:
  https://doi.org/10.1007/s10291-019-0832-4
- Guo and Zhang (2014), real-time receiver clock-jump classification and
  observable reconstruction for PPP:
  https://doi.org/10.1007/s10291-012-0307-3
- Freda et al. (2015), TDCP velocity estimation and quality control:
  https://doi.org/10.1007/s10291-014-0425-1
- Wang et al. (2020), TDCP constraints in tightly coupled INS/GNSS:
  https://doi.org/10.1109/TIM.2019.2957848
- Angrisano et al. (2022), smartphone TDCP velocity and reliability testing:
  https://doi.org/10.3390/s22218514
- RTKLIB disables a Doppler slip detector because of clock-jump sensitivity:
  https://github.com/tomojitakasu/RTKLIB/blob/master/src/rtkpos.c
- GICI-LIB records receiver clock-jump handling as an open issue in its
  relative-position cycle-slip path:
  https://github.com/chichengcn/gici-open/blob/master/src/gnss/ambiguity_common.cpp
