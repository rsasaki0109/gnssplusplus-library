# Velocity-consistency candidate v10 results: PPC Go (0/558), UrbanNav 2/186 -> No-Go overall

The frozen contract is [online_pva_candidate_v10.md](online_pva_candidate_v10.md).

- **Freeze:** commit `4e0d4914`, before any comparison replay.
- **Contract SHA256:** `e25a9b960ea691b41e3aa665e9938b0ef20349f3e7f20787d4265f6c4f0a9637`.
- **Default:** unchanged.
- **Tuning:** nothing was tuned after the results were seen.
- **Machine-readable record:**
  [online_pva_decision_v10.json](online_pva_decision_v10.json).

## Decision

**No-Go.** The contract requires every gate to pass on all 24 run/scenarios.

- **PPC: 0 of 558 gates fail.** This is the first candidate to pass every
  PPC gate. The targeted rotation improvement is present.
- **UrbanNav: 2 of 186 gates fail.** Both are Odaiba IMU gap (60-64 s), RTK
  position P95, in the all-output and common-valid cohorts: control 26.8 m,
  candidate 28.5 m (+6 %).
  - The same gate failed in v7 (27.9 m) and v8 (28.5 m).
  - It comes from the RTK input introduced in v7, not from this candidate's
    change.

Run details:

- 48 replays from one binary (SHA256 `5cf2b2c6...`, built from the frozen
  worktree), interleaved. All passed.
- Gate 7: control `none` is bit-identical to develop `c37979c3` on the 18 PPC
  runs (175,902 rows).
- Processor P95, candidate/control: min 1.06, mean 1.33, max 1.89.

## Normal runs, control -> candidate

Values are RMSE / P95, except fused velocity, which is RMSE.

| Run | RTK pos m | RTK vel m/s | Fused pos m | Fused vel m/s | Rotation deg |
|---|---|---|---|---|---|
| Tokyo 1 | 32.2/61.7 -> 21.1/34.7 | 2.10/3.29 -> 1.17/2.22 | 71.7/155.2 -> **4.0/6.2** | 7.05 -> **0.18** | 105.1/171.1 -> **1.8/2.5** |
| Tokyo 2 | 19.5/29.7 -> 6.3/10.6 | 2.26/3.71 -> 0.75/1.59 | 74.2/179.5 -> **2.4/5.1** | 6.85 -> **0.10** | 100.3/170.0 -> **1.4/1.9** |
| Tokyo 3 | 18.6/31.9 -> 12.0/23.9 | 2.10/1.76 -> 0.66/1.30 | 41.9/115.0 -> **10.4/9.0** | 3.22 -> **0.15** | 63.5/144.4 -> **1.6/2.6** |
| Nagoya 1 | 25.7/29.0 -> 20.9/26.7 | 2.51/4.26 -> 1.28/2.36 | 49.3/118.1 -> **12.5/11.3** | 4.61 -> **0.88** | 88.2/168.7 -> **1.4/2.0** |
| Nagoya 2 | 37.7/81.8 -> 30.9/71.7 | 2.30/5.09 -> 1.49/2.82 | 81.0/168.0 -> **7.3/17.1** | 3.73 -> **0.33** | 18.5/39.8 -> **3.2/5.0** |
| Nagoya 3 | 43.7/101.1 -> 37.0/99.5 | 2.47/5.13 -> 1.50/3.31 | 58.9/128.1 -> **14.2/19.9** | 5.54 -> **0.21** | 105.0/170.9 -> **2.2/2.7** |
| Odaiba | 199.9/47.8 -> 199.0/28.3 | 3.84/9.31 -> 2.32/5.00 | 81.2/157.1 -> **23.7/58.1** | 6.26 -> **2.14** | 105.6/171.4 -> 103.9/170.6 |
| Shinjuku | 27.6/65.6 -> 18.2/38.5 | 1.96/3.96 -> 1.02/1.31 | 82.8/175.1 -> **7.3/13.7** | 9.27 -> **0.28** | 68.7/152.6 -> **3.8/4.1** |

- **Latch re-anchor on Tokyo 1 and Tokyo 2.** It restored their attitude (1.8 and
  1.4 deg) with the honest SPP velocity covariance kept as the fusion input,
  as predicted.
- **Further gains.** Fused position on Tokyo 1 and Tokyo 2 improved further
  against v8: 10.8 -> 4.0 m and 3.6 -> 2.4 m.
- **Shinjuku.** Rotation is now 3.8 deg.
- **Odaiba.** Attitude stays unusable (104 deg), as for the control and every
  candidate so far.

## Remaining work

The only failing gate is the RTK position P95 on the Odaiba IMU gap scenario.
The RTK filter output there must not be worse than the control at P95.
A follow-up candidate needs a new frozen contract. All eight runs are now
development data, so a credible default switch will also need a fresh
holdout.

## Post-hoc findings after the decision (recorded after the results; no gate or result above changes)

1. **The Odaiba IMU gap RTK P95 failure is the control's variance, not a
   candidate regression.** Candidate RTK P95 is 28.28 / 28.41 / 28.46 m on the
   normal / GNSS outage / IMU gap scenarios. The control's is
   47.84 / 28.45 / 26.79 m, and its best draw is the IMU gap scenario. Both arms
   recreate the RTK filter at 60.2 s, and the candidate fixes again about 0.8 s
   later. A scratch counterfactual that kept the RTK filter across the IMU gap
   left the P95 at 28.46 m. The candidate's P95 comes from a FLOAT bias of about
   50 m at 455-477 s, and that bias is present in every scenario.
2. **RINEX 3.00-3.02 BeiDou B1 was read as B1C.** UrbanNav Tokyo is RINEX 3.02,
   and the reader mapped BeiDou band 1 (`C1I`) to B1C at 1575.42 MHz instead of
   B1I at 1561.098 MHz. PPC is RINEX 3.04 and is not affected. A separate fix
   reads band 1 as B1I in 3.00-3.02. With that fix, the candidate's Odaiba
   results change as follows:
   - fused P95: 58.1 -> 17.4 m;
   - RTK velocity P95: 5.00 -> 0.51 m/s.
3. **Early attitude loss of about 180 deg.**
   - **Odaiba.** The candidate's error exceeds 90 deg on about half of the
     epochs, from 29.6 s, before and after the reader fix.
   - **Shinjuku.** With the reader fix, the candidate starts doing the same
     from 25.4 s: rotation RMSE 3.8 -> 105 deg.
   - **IMU gap scenario.** On Odaiba, the re-initialisation after the gap
     recovers the attitude (P95 5.7 deg).
   - **Consequences.** This candidate is not a default-switch candidate. The
     relative gates did not show the defect, because the control fails the
     same way. A later contract needs an absolute attitude-integrity check as
     well as the relative gates.
