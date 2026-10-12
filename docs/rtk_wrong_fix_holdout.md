# Wrong-FIX status demotion: holdout result (No-Go)

This note records a frozen-contract check of the PPC-selected wrong-FIX
demotion profile on data it was not selected on, and a reference-point problem
in the Hong Kong UrbanNav truth that this check exposed.

## Candidate and contract

Candidate, unchanged from
[PPC existing real-time gate outputs](ppc_realtime_gate_existing_outputs.md):
`--demote-fixed-status-nis-per-obs 2 --demote-fixed-status-max-ratio 4`
(output a FIX epoch as FLOAT when the update NIS per observation exceeds 2 or
the AR ratio is at most 4). It was selected on the six PPC runs, so PPC is
training data here.

The contract was frozen on 2026-10-11 before any holdout run:

- **Labels.** Wrong FIX means FIXED with a 3D error above 0.50 m against
  `reference.csv`. Correct FIX means FIXED with a 3D error of 0.50 m or less.
  Epochs are matched to the nearest reference epoch within 0.25 s.
- **Holdout runs (6):**
  - UrbanNav Tokyo 2018-12-19: Shinjuku and Odaiba, each with the Trimble and
    the u-blox rover. The surveyed TUMSAT base is used for all four; the
    `base_trimble.obs` headers of both runs carry the same APPROX POSITION.
  - UrbanNav HK Deep-Urban-1 and Harsh-Urban-1.
- **Profiles.** P1 = `gnss_solve --mode kinematic --glonass-ar off`; P2 = P1
  plus `--preset low-cost`. Each profile was run with and without the
  candidate.
- **Gates.** These are pooled over the holdout, and both profiles must pass:
  - W0: only the status column changes.
  - W1: wrong FIX is reduced by at least 50 %. Fewer than 20 baseline wrong FIX
    counts as insufficient evidence.
  - W2: at least 80 % of correct FIX epochs are kept.
  - W3: no run with 50 or more FIX epochs gets a higher Wrong/FIX.

The binary was develop `bf15a3b9`.

## Holdout results

| Run | Profile | FIX base → cand | Correct base → cand | Wrong base → cand | Wrong/FIX base → cand |
|---|---|---|---|---|---|
| Shinjuku Trimble | P1 | 122 → 93 | 122 → 93 | 0 → 0 | 0 → 0 |
| Shinjuku Trimble | P2 | 15049 → 12073 | 13112 → 11527 | 1937 → 546 | 12.9 % → 4.5 % |
| Shinjuku u-blox | P1 | 1 → 1 | 0 → 0 | 1 → 1 | — |
| Shinjuku u-blox | P2 | 5482 → 5075 | 5143 → 4832 | 339 → 243 | 6.2 % → 4.8 % |
| Odaiba u-blox | P1 | 461 → 321 | 461 → 321 | 0 → 0 | 0 → 0 |
| Odaiba u-blox | P2 | 3693 → 3638 | 3657 → 3615 | 36 → 23 | 1.0 % → 0.6 % |
| Odaiba Trimble | P1 | 1298 → 816 | 1298 → 816 | 0 → 0 | 0 → 0 |
| Odaiba Trimble | P2 | 8022 → 7542 | 7908 → 7464 | 114 → 78 | 1.4 % → 1.0 % |
| HK Deep | P1 | 6 → 6 | 0 → 0 | 6 → 6 | see below |
| HK Deep | P2 | 251 → 235 | 6 → 6 | 245 → 229 | see below |
| HK Harsh | P1 | 7 → 6 | 0 → 0 | 7 → 6 | see below |
| HK Harsh | P2 | 53 → 41 | 0 → 0 | 53 → 41 | see below |

| Gate | P1 (default) | P2 (low-cost) |
|---|---|---|
| W0 status only | pass | **fail**: 3 runs emit a different epoch set (Shinjuku Trimble 16809 vs 16805 rows, Shinjuku u-blox 7701 vs 7583, HK Harsh 1103 vs 1106) |
| W1 wrong FIX −50 % | **insufficient**: 14 → 13 | pass: 2724 → 1160 (−57.4 %) |
| W2 correct FIX kept ≥ 80 % | **fail**: 65.4 % | pass: 92.0 % |
| W3 no run worse | pass | pass |

**Decision: No-Go.** The profile is not proposed as a default.

What the numbers do show:

- **Default profile (P1).** P1 already has almost no wrong fixes on this
  holdout, and the candidate removes a third of its correct fixes.
- **Low-cost preset (P2).** P2 is where wrong fixes are. On the four Tokyo
  runs alone (no HK), the candidate cuts wrong FIX from 2426 to 890 (−63 %)
  while keeping 92 % of correct FIX. The gain is concentrated on Shinjuku
  Trimble, where the baseline low-cost Wrong/FIX is 12.9 %.
- **Why W0 fails for P2.** Demoting the status changes which epochs the
  downstream `gnss_solve` post-filters emit. A follow-up candidate would have
  to demote without changing the emitted epoch set. Because the four Tokyo
  runs have now been seen, it would also need fresh holdout data.

On PPC (training data, P2), the candidate takes wrong FIX from 1280 to 688
(−46 %) and keeps 84.7 % of correct FIX. That is milder than the 7.4 % →
1.7 % Wrong/FIX in the gate note, which was measured on a different
(real-time sigma) pipeline. The current `gnss_solve` low-cost pipeline does
not reproduce it.

## Hong Kong truth is not at the antenna

Nearly every HK FIX epoch is labelled wrong in both arms: P2 Deep has 245 of
251. Most of those labels are not wrong fixes. Rotating the FIX-epoch error
into the vehicle frame with the reference heading gives:

| Run (P2 FIX epochs) | Forward | Right | Up |
|---|---|---|---|
| HK Deep (n = 251) | median −0.57 m (IQR 0.14 m) | −0.01 m (IQR 0.05 m) | −0.17 m |
| HK Harsh (n = 53) | median −0.53 m (IQR 0.23 m) | 0.12 m (IQR 0.96 m) | −2.73 m |

A constant along-track offset of about 0.55 m, with a small spread, shows up
in both runs independently. That is the signature of a lever arm, not of
integer errors. The UrbanNav `extrinsic.yaml` of both runs gives the
antenna-to-IMU translation as (0, 0.86, −0.31) m (y forward, ±0.1 m, measured
by hand) and an IMU-to-SPAN offset of 0.14 m in z. The truth is produced by
the SPAN-CPT. The published offset and the fitted one differ by about 0.3 m,
and the frame-direction convention is not stated, so the exact correction is
unresolved.

The repository's HK conversion uses a zero lever arm
([holdout v3](online_pva_default_switch_holdout_v3.md) declares this). Until a
lever arm is applied:

- the HK runs can be used for metre-level statistics only;
- no 0.5 m wrong/correct-FIX label on HK is meaningful;
- every centimetre-to-decimetre HK figure carries a bias of about 0.5 m along
  track.

HK Harsh additionally has a genuine vertical error on its fixes (median
−2.7 m).

## Related integrity results (same effort, No-Go)

These experiments in this effort were run under frozen contracts and were
No-Go:

- **Honest reported covariance.** `SPP_CONSISTENCY_SCALED` for FLOAT, and for
  FIXED the fixed filter covariance unless the bootstrapped success rate is
  below 0.999.
  - FIXED sigma became 2.5–5 mm. The success rate of the held-ambiguity
    filter was effectively 1, so only 0.57 % of FIXED epochs fell back.
  - Misleading FIXED epochs at K = 5.33 rose from 850 to 21,860 (horizontal).
  - FLOAT misleading stayed near 50 % against the required 10 %.
  - The model covariance is too optimistic for urban data, and thresholding it
    does not make it honest.
- **Carrier-prior FIX safeguard.** No FIXED is published while
  `--rtk-canyon-carrier-prior` is engaged, or within 30 s after release.
  - It removed the `--preset odaiba` + SNR + prior wrong-fix burst (1009 → 6).
  - It cost 18.5 % of preset fixes and −0.29 pp PPC official.
- **Calibrated reported uncertainty for `gnss live` (2026-10-12).** The
  reported σ was fitted to measured errors.
  - Form:
    - FLOAT: σ = max(f, k · σ_float).
    - FIXED: σ = max(f, k · sqrt(σ_float² + d²)), where d is the distance
      between the epoch's float and fixed positions.
  - Fitting: one floor f and one factor k per status and component (8
    constants), fitted on the six PPC runs (G+R+E+J+C MSM7,
    `--preset low-cost`) by a fixed rule. The rule targets 1 % of errors
    above 3σ.
  - Check: once on the four UrbanNav Tokyo runs, which were never used for σ
    (29,273 FIXED and 13,592 FLOAT epochs).

  | Holdout result | Calibrated | Legacy (constant 0.1 m) | Gate |
  |---|---|---|---|
  | FLOAT, share of errors above 3σ (horizontal / vertical) | 0.85 % / 0.24 % | 88 % / 91 % | ≤ 2 %: pass |
  | FIXED, share of errors above 3σ (horizontal / vertical) | 0.80 % / **2.43 %** | 8.2 % / 8.6 % | ≤ 2 %: **fail** (vertical) |
  | AUROC of σ_H, wrong vs correct FIX | **0.88** (0.82–0.99 per run) | 0.50 | ≥ 0.75: pass |
  | Median σ_H of correct FIX | **0.168 m** | 0.141 m | ≤ 0.10 m: **fail** |

  **Decision: No-Go**, and the option was not merged. The float–fixed
  distance separates wrong fixes well: AUROC 0.88, where the constant σ
  cannot separate them at all. But the vertical FIXED σ is too small on two
  runs, and correct fixes get σ_H of 0.17 m (0.79 m on Odaiba u-blox). The
  FLOAT σ comes out at metre level (median about 11 m), because the training
  FLOAT errors have a heavy tail. A follow-up needs a holdout that has not
  been used for σ. The UrbanNav Tokyo runs now have been.
- **Fresh evaluation data.** A search on 2026-10-12 found one fresh
  dual-frequency urban run with an open base and post-processed truth:
  UrbanNav HK Medium-Urban-1 (TST, 13 min). It is too weak to decide a 0.5 m
  wrong-FIX label.
  - Dual-frequency carrier phase is sparse: 4 or more GPS satellites in only
    53 epochs on the NovAtel rover.
  - The truth is mostly quality class 2.
  - It shares the lever-arm offset of the Deep and Harsh runs, about −0.57 m
    forward. Estimated on Deep and Harsh, applying that offset left a median
    horizontal error of 0.105 m.

Measurement tools for this work are `gnss_solve --pos-uncertainty` and
`gnss uncertainty-eval` (see [Interfaces](interfaces.md)).
