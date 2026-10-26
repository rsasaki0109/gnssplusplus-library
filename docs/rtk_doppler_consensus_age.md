# Doppler consensus track age limit: more FIX than RTKLIB, but opt-in

This note follows the [PPC profile note](rtk_cd_ppc_profile.md), which found that
the fixed Doppler consensus check (D, `--max-fixed-doppler-consensus`) locks out
correct fixes once its Doppler-integrated track drifts. It adds an opt-in age
limit, `--max-fixed-doppler-consensus-age <s>`, and evaluates it against the
default profile, the CDM recipe and RTKLIB demo5 with every constellation (the
fair baseline since [#657](https://github.com/rsasaki0109/gnssplusplus-library/pull/657)).
**The default is unchanged.** Record: [JSON](urbannav_doppler_consensus_age_evaluation.json).

The reference trajectory is used only for scoring.

## Why: the D lock-out also limits Tokyo

On the full UrbanNav Odaiba drive (Trimble), RTKLIB demo5 has 5839 FIX epochs
and the default profile 1298. RTKLIB has a correct FIX (3D ≤ 0.5 m) at 2129
epochs where CDM has none. CDM's debug log rejects 1888 of them with
`fixed_doppler_consensus`.

`scripts/analysis/truth_dd/doppler_consensus_age.py` on that log:

- every rejection happens 30 s or more after an accepted FIX last re-anchored
  the track, when the track is 7–11 m off;
- the check rejects 2703 correct candidates and one wrong one;
- the longest lock-out lasts 2704 epochs (from TOW 273981.7);
- below 30 s it rejects nothing, and the track error p95 is 0.5 m at 2–5 s.

Only an accepted FIX can re-anchor the track, so a drifted track never recovers
by itself.

## Change

`max_fixed_doppler_consensus_age_s` (CLI `--max-fixed-doppler-consensus-age`,
default 0 = no limit) stops the track vetoing once its last anchor is older than
the limit. The next accepted FIX re-anchors it. The debug CSV gains
`fixed_candidate_doppler_consensus_age_s`.

- `RTKDopplerConsensusTest.DriftedTrackLocksOutUntilItsAnchorExpires` reproduces
  the lock-out (a 270 s old track 9 m off still vetoes), then shows the limit
  lifting it and a 2 s old track still vetoing.
- Without the flag, the Odaiba default and CDM POS are byte-identical to develop.

The evaluated value is 5 s, the CD recipe's existing `--max-fixed-anchor-age`;
it was not tuned. CDME = CDM + `--max-fixed-doppler-consensus-age 5`.
`gnss solve --preset urban` sets exactly these flags, and its POS is
byte-identical to them. `gnss live` and `gnss replay` accept the same flags
and preset.

## Result against RTKLIB demo5 (the goal: more FIX, no more wrong FIX)

Every solver is scored with the same matching (0.11 s) and 3D error.

| Run | RTKLIB: FIX / wrong >0.5, 2, 3 m | default | CDM | **CDME** |
| --- | --- | --- | --- | --- |
| Trimble Odaiba | 5839 / 19, 4, 4 | 1298 / 0 | 4450 / 6, 0, 0 | **7467 / 9, 0, 0** |
| Trimble Shinjuku | 6313 / 128, 112, 112 | 122 / 0 | 2877 / 4, 0, 0 | 11491 / 242, 0, 0 |
| u-blox Odaiba | 1857 / 92, 69, 44 | 461 / 0 | 871 / 0 | **3130 / 0** |
| u-blox Shinjuku | 792 / 93, 81, 81 | 1 / 1, 1, 0 | 16 / 0 | **3582 / 10, 0, 0** |
| PPC tokyo/run1 | 995 / 123, 105, 104 | 772 / 1, 0, 0 | 1395 / 5, 0, 0 | **4328 / 24, 1, 1** |
| PPC tokyo/run2 | 1151 / 15, 13, 13 | 879 / 1, 0, 0 | 1310 / 1, 0, 0 | **6669 / 5, 0, 0** |
| PPC tokyo/run3 | 6675 / 85, 48, 44 | 749 / 0 | 1134 / 0 | **10910 / 35, 0, 0** |
| PPC nagoya/run1 | 1577 / 81, 77, 76 | 18 / 0 | 2033 / 10, 0, 0 | **5280 / 19, 0, 0** |
| PPC nagoya/run2 | 2133 / 239, 229, 223 | 1130 / 1, 0, 0 | 1173 / 2, 0, 0 | **4404 / 26, 0, 0** |
| PPC nagoya/run3 | 291 / 92, 82, 75 | 12 / 0 | 591 / 15, 0, 0 | **1069 / 18, 2, 0** |

- **CDME beats RTKLIB on 9 of 10 runs**: more FIX epochs and no more wrong FIX at
  any threshold. The frozen Odaiba prediction (FIX > 5839, wrong ≤ 19) is met.
- **The exception is Trimble Shinjuku**: 242 wrong FIX at 0.5–2 m against
  RTKLIB's 128, although RTKLIB has 112 beyond 3 m and CDME none.
- Over the ten runs RTKLIB has 776 wrong FIX beyond 3 m; CDME has one.

On the PPC RTK profile, P+CDME beats RTKLIB on tokyo/run1, tokyo/run3 and
nagoya/run1. Against P alone it cuts wrong FIX >0.5 m from 788 to 36, 340 to 3 and
837 to 10 there, at a loss of 10–14% of P's correct FIX. On nagoya/run2 (575
wrong) and nagoya/run3 (95 beyond 3 m) it is worse. The frozen prediction of a
loss below 20% on every PPC run is not met (42.3% and 24.7% on nagoya/run2–3).

## Result against CDM (the gate for keeping it): fails

Against CDM, CDME loses no CDM-correct FIX on any run. The gate still fails on
every run except HK Harsh:

- wrong FIX >0.5 m rises, for example 4 → 242 on Trimble Shinjuku and 0 → 35 on
  PPC tokyo/run3;
- PPC tokyo/run1 and nagoya/run3 gain 1–2 wrong FIX beyond 2 m;
- missing epochs rise, for example 1021 → 1510 on PPC tokyo/run3.

The gate against default ([fix_recovery_guard.md](fix_recovery_guard.md)) fails
as it does for CDM.

## Where Shinjuku's new wrong FIX come from

All 242 were accepted with a fresh track (age ≤ 5 s), not at an expiry: 124 AR
fixes and 118 hold fixes in 19 episodes. Three of them hold 209 epochs:

- TOW 283255.5–283265.1 (96 epochs);
- TOW 281650.1–281656.4 (64 epochs);
- TOW 283284.3–283289.1 (49 epochs).

These are FIX streaks that drift into a wrong basin. The track re-anchors at
every FIX and cannot see them; the [FIX streak drift note](rtk_fix_streak_drift.md)
describes the same shape. CDM had no wrong FIX there only because the lock-out
kept it out of these spans.

## Decision

Per the frozen rule, gate C fails, so:

- the flag stays opt-in;
- it is not added to the CD-family recipe;
- the default is unchanged.

The next step for "more FIX than RTKLIB without more wrong FIX" is the
drifting-streak mechanism itself, not the D gate.

## Reproduce

Build this branch. Run each recipe with the Tokyo, HK and PPC argv of the
[CDMH note](rtk_cdmh_ppc_evaluation.md) and the
[PPC profile note](rtk_cd_ppc_profile.md). Run RTKLIB demo5 b34k with
`scripts/rtklib_odaiba.conf`. The JSON records the frozen rule, the binary hash,
every summary and both gates.
