# Candidate integrity witness shadow

## Objective

Recover correct ratio-rejected RTK ambiguity candidates without treating a
larger LAMBDA ratio, a smaller robust cost, or agreement with the live float
graph as independent evidence. This milestone is monitor-only and default-off.
It never changes the smoother, ambiguity hold, FIX/FLOAT, or reported pose.

The preceding DDPR-only alternating GNC experiment failed: candidate FLOAT RMS
increased from 4.849 m to 7.370 m and runtime increased 236%. In particular,
one candidate's ratio rose from 1.243 to 3.483 while its position moved about
1.24 m away from an already-correct reported solution. GNC is therefore not an
acceptance witness in this experiment.

## Research basis

- Song et al., *Two stage GNSS outlier detection for factor graph optimization
  based GNSS-RTK/INS/odometer fusion* (2025), first checks pseudorange change
  with Doppler and then checks predicted DD pseudorange with short-term
  IMU/odometer propagation: <https://arxiv.org/abs/2510.00524>.
- Wang et al., *Solution Separation Based Integrity Monitoring for Ambiguity
  Resolution Enabled GNSS Positioning* (2023), evaluates measurement-fault and
  incorrect-fix hypotheses together rather than treating ambiguity validation
  as fault-free: <https://doi.org/10.33012/2023.18607>.
- Taylor and Gross (ION ITM 2026) compare solution-separation RAIM with robust
  FGO and publish their experiment code. Their result warns that exhaustive
  solution separation is computationally expensive in high-outlier settings:
  <https://doi.org/10.33012/2026.20495> and
  <https://github.com/cntaylor/FGO_gnss_integrity>.
- GraphGNSSLib confirms the relevant RTK graph structure: DD pseudorange,
  carrier phase, and Doppler form the float graph before LAMBDA ambiguity
  resolution: <https://github.com/weisongwen/GraphGNSSLib>.

## Frozen witness rule

For every provisional LAMBDA candidate, record four checks that already exist
in the backend but have not been combined as an authority-neutral audit:

1. **Code-only anchor:** the current-epoch DDPR-LS/FDE anchor is trusted under
   the shipping minimum-factor and residual gates, and lies within 1.0 m of
   the candidate.
2. **Short-term inertial prediction:** the candidate lies within 2.0 m of the
   IMU-propagated epoch seed.
3. **Doppler-only DR:** the independently propagated SD-Doppler track is aged
   1--30 epochs and candidate separation passes the fixed 3-D chi-square 99%
   gate (`mahalanobis2 <= 11.345`).
4. **Carrier observation consistency:** at least four candidate DD carrier
   rows are available and their post-fit RMS is at most 0.05 m.

The composite shadow passes only when all four checks pass and the candidate
ratio exceeds the existing relaxed floor of 1.0. Thresholds are frozen before
truth scoring. The code-only anchor and Doppler track do not consume the
candidate's integer constraint; the IMU seed and carrier check are supporting
consistency tests, not independently sufficient witnesses.

One CLI option enables the complete audit:

```text
--candidate-integrity-shadow
```

It may materialize private SD-Doppler rows for the external DR estimator, but
does not insert them into the FGO graph.

## Gates

### 500-epoch smoke: Tokyo run1 source epochs 5000--5499

- exact equality of every pre-existing solution field with monitor off/on;
- no NONE or non-finite increase;
- at least 20 otherwise-FLOAT candidates with a complete four-witness verdict;
- zero composite-passed candidate above 0.5 m 3-D truth error; and
- at least one composite-passed correct candidate.

Failure stops the experiment without threshold tuning.

### Full Tokyo run1 development gate

- at least 100 complete witness verdicts;
- at least 60 composite-passed correct FLOAT candidates;
- zero composite-passed wrong candidates at the 0.5 m 3-D aperture; and
- solver overhead at most 20%.

Only a full run1 pass permits a default-off activation A/B. Tokyo run2 and
run3 remain sealed until then. Reference truth is used only by the offline
scorer and never by the runtime shadow.

## Tokyo run1 500-epoch smoke result

The frozen shipping-profile replay used source epochs 5000--5499. Enabling
`--candidate-integrity-shadow` left every exported solution key exactly
unchanged relative to the preceding monitor-only replay: 333 FIX, 167 FLOAT,
zero NONE/non-finite epochs, and zero differences in time, status, ECEF,
ENU error, ratio, fixed count, or AR outcome.

The monitor produced 322 complete verdicts and 95 composite passes. All 95
passed candidates were correct within the frozen 0.5 m 3-D aperture (maximum
3-D error 0.092 m, RMS 0.058 m). This is a clean safety result, but it has no
incremental FIX yield:

| Metric | Result |
|---|---:|
| complete verdicts | 322 |
| composite passes | 95 |
| passed candidates above 0.5 m 3-D | 0 |
| passed candidates already reported FIX | 95 |
| passed candidates finally reported FLOAT | 0 |
| final FLOAT epochs | 167 |
| final FLOAT epochs with any provisional candidate | 2 |
| final FLOAT epochs with a complete verdict | 0 |

Among 331 ratio-above-1 provisional candidates, the trusted anchor was
available for all 331, IMU passed 330, carrier post-fit passed 326, and the
Doppler DR verdict was available and passed for 322. The anchor aperture was
the selective witness (98 passes). However, 329 of those 331 candidates were
already normal FIX. The two final-FLOAT candidates were not actionable: the
opening ratio-1.243 candidate preceded any independent DR reset and failed
the IMU aperture; the ratio-2.191 candidate had a 66.191 m anchor separation
and a DR age of 89 epochs, beyond the frozen 30-epoch limit.

Solver time rose from the preserved 15.753 s baseline to 34.722 s (+120.4%),
also exceeding the full-run 20% overhead ceiling. The smoke therefore fails
its otherwise-FLOAT coverage gate. Full run1 and sealed run2/run3 are not run.
Do not activate this composite as a FIX authority: it safely reconfirms
already-fixed candidates but cannot increase fix rate on this development
slice.

The next experiment should instrument and improve the pre-validation
ambiguity-candidate funnel (candidate availability, conditioning, and subset
formation) using run1-only diagnostics. It must count finally-FLOAT
opportunities, must not relax the integer ratio, and must not tune these
witness apertures.
