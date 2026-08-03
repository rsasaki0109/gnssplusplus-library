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
safe graph insertion. The next phase must emit per-factor innovations and
classify each target/reference arc using loss-of-lock, geometry-free,
Doppler, reference-change, and robust residual evidence. Only a frozen gate
that survives run2 and run3 should be promoted to a graph factor.

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
