# CD family on the PPC RTK profile: the Doppler consensus locks out, NO_GO

This note follows the [CDMH note](rtk_cdmh_ppc_evaluation.md), which ran PPC
with the bare Tokyo command line. Here the CD-family flags are put on top of
the repository's own PPC RTK profile. **No solver code and no default change.**
Record: [JSON](ppc_profile_cd_evaluation.json).

The reference trajectory is used only for labels and for analysis-only
residuals.

## Setup

**Profile P** is the RTK argv of `apps/commands/benchmarks/gnss_ppc_native_replay.py`:

```
gnss_solve --rover rover.obs --base base.obs --nav base.nav --mode kinematic --no-kml
           --preset low-cost --ratio 2.4 --max-subset-ar-drop-steps 18
           --rtk-snr-weighting --no-arfilter
```

The `low-cost` preset already includes:

- a 30 m/s motion-aware jump limit;
- a full-ratio floor for subset AR;
- a float-divergence reset.

P's fixed epochs are within 0.3% of the recorded baseline in
[ppc_fix_recovery_evaluation.json](ppc_fix_recovery_evaluation.json) on all
six runs.

**Recipes:** P, P+M, P+CDM and P+CDMH. CD, M and H are as in the
[CDMH note](rtk_cdmh_ppc_evaluation.md). The rule was frozen before the
replay and gave:

- the binary, develop `2e224ea`;
- three predictions (below);
- the gate of [fix_recovery_guard.md](fix_recovery_guard.md) on every run,
  against P;
- no retuning after the replay.

## Result

| Six PPC runs | FIX | Wrong >0.5 m | Wrong >2 m | Wrong >3 m | P-correct loss per run |
| --- | ---: | ---: | ---: | ---: | --- |
| P | 40075 | 3222 | 918 | 543 | – |
| P+M | 40075 | 3222 | 918 | 543 | 0 (POS byte-identical to P) |
| P+CDM | 6657 | 127 | 0 | 0 | 58–89% |
| P+CDMH | 6881 | 260 | 0 | 0 | 59–90% |

Predictions:

1. **P+M is inert: met.** Its POS is byte-identical to P on all six runs.
2. **P+CDM lowers wrong FIX at a large loss: met.**
   - Wrong >0.5 m and >2 m both decrease on all six runs.
   - Every wrong FIX beyond 2 m is gone.
   - The P-correct loss is 58–89% on every run, not just above 1%.
3. **P+CDMH has no more wrong FIX than P+CDM: not met.**
   - Summed wrong >0.5 m is 260 vs 127.
   - Most of it is one 35 s streak on nagoya/run1 (TOW 551489.2–551524.4,
     up to 1.32 m), where CDM has a single wrong epoch.
   - On this run the Kalman hold keeps the wrong basin that the legacy
     overwrite left.

**Decision: NO_GO for all three, and no profile or default change.**

- P+CDM and P+CDMH fail every run on loss and right-censoring. P+CDM also
  raises the horizontal P95 on 5 runs, missing epochs on 4 and the recovery
  P95 on 3.
- P+M passes the listed items only because it is identical to P, so the
  decrease clause fails.

## Why CD loses 80% of P's correct fixes

The debug log of P+CDM gives the reject reason at each P-correct epoch that
P+CDM loses. The POS of that replay is byte-identical to the scored one.

| Run | Lost | `fixed_doppler_consensus` | `too_few_pairs` | `lambda_not_fixed` | `low_ratio_weak_integer_system` |
| --- | ---: | ---: | ---: | ---: | ---: |
| tokyo/run1 | 6977 | 4009 | 1168 | 791 | 925 |
| tokyo/run2 | 6105 | 3523 | 661 | 1632 | 273 |
| nagoya/run1 | 3315 | 1310 | 1009 | 821 | 173 |

The biggest cause is the Doppler consensus check (D, 3 m), not the
10-ambiguity floor. In P, only 5–12% of the correct AR fixes have fewer than
10 fixed ambiguities, so the floor alone would already cost more than 1%.

`scripts/analysis/truth_dd/doppler_consensus_age.py`
splits the check's decisions by the age of its Doppler track, that is, the
time since the last accepted FIX re-anchored it. Over the three runs:

| Track age | Correct accepted / rejected | Wrong accepted / rejected | Track error p50 / p95 m |
| --- | ---: | ---: | ---: |
| < 2 s | 3889 / 0 | 2 / 0 | 0.03 / 0.24 |
| 2–15 s | 8 / 112 | 0 / 0 | 5.0–5.3 / 5.7–7.5 |
| 15–120 s | 3 / 1372 | 0 / 0 | 6–20 / 20–60 |
| ≥ 120 s | 0 / 8072 | 0 / 13 | 57–188 / 310–470 |

- **It mostly vetoes correct fixes.** The check rejected 9556 correct
  candidates, whose 3D error is 1–16 cm. It rejected 13 wrong ones.
- **It cannot recover by itself.** Only an accepted FIX re-anchors the
  track. Once the track drifts past 3 m, the check rejects every later
  candidate until a gap over 2 s drops the track. The longest lock-out is
  2807 epochs, about 9 min, on tokyo/run1 from TOW 187793.6.
- **The bare-argv CDM of the CDMH note has the same lock-out.** Its
  tokyo/run2 debug log has 5244 correct and 8 wrong candidates rejected by
  the check, and a 3513-epoch lock-out from TOW 177209.4. CD's FIX gain on
  bare PPC was therefore capped by this lock-out as well.

## Why the track drifts: PPC Doppler velocity

`scripts/analysis/truth_dd/doppler_residuals.cpp`
solves `spp_velocity::solveVelocityFromObservations()` at the reference
position and compares it with the reference velocity.
`scripts/analysis/truth_dd/doppler_velocity_stats.py`
summarizes the output. The reference velocity is consistent with the
reference positions: the finite-difference p95 is 0.05–0.17 m/s.

| Run | Velocity error p50 / p95 / p99 m/s | Same, robust variant | Track drift after 5 s p95 m, library / robust |
| --- | ---: | ---: | ---: |
| tokyo/run1 | 0.28 / 3.18 / 6.20 | 0.24 / 1.61 / 3.28 | 12.6 / 5.8 |
| tokyo/run2 | 0.16 / 1.99 / 3.78 | 0.15 / 1.18 / 2.12 | 7.1 / 4.6 |
| tokyo/run3 | 0.11 / 1.48 / 2.89 | 0.11 / 0.90 / 1.70 | 5.1 / 3.2 |
| nagoya/run1 | 0.09 / 3.51 / 8.08 | 0.09 / 1.81 / 4.99 | 13.1 / 6.4 |
| nagoya/run2 | 0.09 / 2.97 / 4.73 | 0.09 / 1.53 / 2.86 | 10.8 / 4.9 |
| nagoya/run3 | 0.30 / 3.00 / 4.95 | 0.28 / 2.08 / 3.46 | 10.1 / 6.9 |

- **The solve has no per-observation outlier rejection.** It is
  least squares over every Doppler row. Its only gate is χ²/dof ≤ 25 with
  σ = 0.5 m/s / sin(el).
- **Outliers are common in these streets.** Per-signal residuals have a
  core of 0.02–0.03 m/s, but 2–7% of rows exceed 1 m/s.
- **Third-frequency signals are much noisier.** GPS L5, Galileo E5b, BDS B1C
  and B2a, and QZSS L5 have a median of 0.11–0.21 m/s, and 11–18% of their
  rows exceed 1 m/s. They are rare rows here: 12–31 k each against 0.1–0.6 M
  for the main signals.
- **A robust re-solve helps but does not fix it.** The analysis-only variant
  drops the worst residual beyond 4σ and solves again (0.2–0.5 rows per
  epoch). It cuts the p95 error by 31–49% and the 5 s drift p95 by 32–55%.
  A 3 m consensus is still exceeded within 5 s at the p95.
- **Trimble Tokyo is much better.** There, the 1 s Doppler-integration p95
  was 0.10 m ([max jump note](rtk_max_position_jump_motion.md)), against
  1.4–3.4 m here.

## Open

These are two separate defects. Both need a reproduction test and a frozen
evaluation of their own.

- **The consensus reference has no age or uncertainty bound.** D therefore
  turns from a check into a lock-out wherever the Doppler is worse than on
  Trimble Tokyo. M and the fix-history motion compensation use the same
  velocity, but only over ≤ 2 s.
- **The Doppler velocity solve is not robust.** It is shared with the
  FLOAT continuity bridge in `gnss_solve` and with the fusion velocity
  update, so a change there touches default paths.

## Reproduce

1. Build develop `2e224ea`. Fetch PPC as in the
   [CDMH note](rtk_cdmh_ppc_evaluation.md).
2. Run P and each recipe with the argv above. Score with exact 1 ms
   matching.
3. Build `doppler_residuals.cpp` like `export_truth_dd.cpp`
   ([long-lock note](rtk_long_lock_overconfidence.md)). Then run:

```sh
doppler_residuals <run>/base.nav <run>/rover.obs <run>/reference.csv obs.csv epoch.csv
python3 scripts/analysis/truth_dd/doppler_velocity_stats.py <dir> tokyo-run1 ...
python3 scripts/analysis/truth_dd/doppler_consensus_age.py data/PPC-Dataset/tokyo run1 \
    <P+CDM debug.csv> <P+CDM pos>
```

The JSON records the binary, POS and scorer hashes.
