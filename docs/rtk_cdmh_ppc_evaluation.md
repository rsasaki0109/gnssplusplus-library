# CDMH on PPC, Tokyo and HK: the Kalman hold shortens the hold basin, NO_GO

This note follows the [FIX streak drift note](rtk_fix_streak_drift.md). It
adds the PPC-Dataset to the opt-in CD-family evaluation and replays one frozen
recipe, CDMH. **No code and no default change.** Every flag already exists.
Record: [JSON](ppc_cdmh_evaluation.json).

The reference trajectory is used only for scoring and 0.5 m labels.

## PPC-Dataset acquisition

The [taroz/PPC-Dataset](https://github.com/taroz/PPC-Dataset) README links a
SharePoint download. Its redirect chain sets session cookies. Without a
cookie jar, curl ends at HTTP 403, which looks like a network block but is
not one. With a cookie jar it downloads:

```sh
curl -sS -L -c cj.txt -b cj.txt -o PPC-Dataset.zip \
  "https://chibakoudai-my.sharepoint.com/:u:/g/personal/66lsm6_chibatech_ac_jp/ETmyNr1VrcpFjqxgjXJdgkQBkfTwnykhSPOVKClOUBNOMQ?download=1"
sha256sum PPC-Dataset.zip   # 460b2174...b76658, 162247330 bytes
```

Extract it to `data/PPC-Dataset/` (gitignored) without the `.git/` members.
The 24 input hashes (rover, base, nav and reference for six runs) are in the
JSON. The rover and base hashes match those already recorded in `docs/`. PPC
is development data in this repository, not a holdout.

The runs use the same bare command line as Tokyo:

```
gnss_solve --rover rover.obs --base base.obs --nav base.nav --mode kinematic
           --glonass-ar off --base-ecef <base.obs APPROX POSITION XYZ> --no-kml
```

They do **not** use the repository's PPC RTK profile
(`--preset low-cost --ratio 2.4 --max-subset-ar-drop-steps 18
--rtk-snr-weighting --no-arfilter` in `gnss_ppc_native_replay.py`). Their
numbers therefore cannot be compared with
[ppc_development_results.md](ppc_development_results.md) or
[ppc_fix_recovery_evaluation.json](ppc_fix_recovery_evaluation.json). The
bare default fixes only 12 and 18 epochs on nagoya/run3 and nagoya/run1.

## Why CDMH

On PPC, CDM (CD with `--max-position-jump-motion-compensation`) raises FIX
from 3560 to 7636 over the six runs. Its wrong FIX stays below 2 m. Its worst
loss is tokyo/run2: 55 default-correct epochs (6.26%).

The tokyo/run2 loss has this mechanism (CDM debug log):

1. From TOW 177054, the AR fixes are already 0.25–0.44 m off.
2. From 177058.8, AR mostly fails: `lambda_not_fixed` in 109 of 122 epochs,
   and the low-ratio floor of CD in 10. The solver falls back to hold fixes.
3. The legacy hold overwrite keeps the ambiguities in that basin until
   177083.0, 124 FIX epochs more than 0.25 m off in total.
4. After the hold stops, LAMBDA still fails from that state. 40
   default-correct epochs (177083.4–177094.8) become FLOAT.

The [Kalman hold](rtk_hold_ambiguity_update.md) (`--hold-kalman-update`)
conditions the state instead of overwriting it. The frozen rule therefore
predicted that, under CDMH = CDM + `--hold-kalman-update`:

- the biased hold streak would be shorter;
- the tokyo/run2 loss would be below 2%.

The rule, frozen before the replay, also fixed:

- the gate of [fix_recovery_guard.md](fix_recovery_guard.md) on every run,
  including the clause that at least one wrong-FIX population must decrease;
- the binary, develop `2e224ea`;
- no retuning after the replay.

## Result

**The prediction is half met.**

- **Streak:** met. FIX epochs more than 0.25 m off in TOW 177050–177100 fall
  from 124 to 2.
- **Loss:** not met. The tokyo/run2 loss is 2.28% (20 epochs), not below 2%.
  The two long clusters are gone. Each remaining cluster is 1–5 FLOAT epochs.

Every default-correct epoch that CDM or CDMH loses on PPC becomes FLOAT. None
becomes a wrong FIX or a missing epoch.

| Run | FIX default / CDM / CDMH | Wrong >0.5 m default / CDM / CDMH | Loss CDM / CDMH | Missing default / CDMH |
| --- | ---: | ---: | ---: | ---: |
| PPC tokyo/run1 | 772 / 1395 / 1439 | 1 / 5 / 3 | 0.13% / 0.13% | 1196 / 1197 |
| PPC tokyo/run2 | 879 / 1310 / 1386 | 1 / 1 / 1 | 6.26% / 2.28% | 602 / 604 |
| PPC tokyo/run3 | 749 / 1134 / 1138 | 0 / 0 / 0 | 1.07% / 0.53% | 1021 / 1021 |
| PPC nagoya/run1 | 18 / 2033 / 2086 | 0 / 10 / 11 | 0 / 0 | 841 / 846 |
| PPC nagoya/run2 | 1130 / 1173 / 1177 | 1 / 2 / 2 | 0.53% / 0.35% | 1770 / 1780 |
| PPC nagoya/run3 | 12 / 591 / 664 | 0 / 15 / 4 | 0 / 66.67% (8 of 12) | 1460 / 1466 |
| Trimble Odaiba | 1298 / 4450 / 4366 | 0 / 6 / 7 | 0.54% / 0.54% | 1426 / 1421 |
| Trimble Shinjuku | 122 / 2877 / 2869 | 0 / 4 / 3 | 0 / 0 | 3234 / 3273 |
| u-blox Odaiba | 461 / 871 / 870 | 0 / 0 / 0 | 0.87% / 1.08% | 922 / 921 |
| u-blox Shinjuku | 1 / 16 / 16 | 1 / 0 / 0 | – | 2374 / 2345 |

Notes on the table:

- No PPC, Trimble or u-blox run has a wrong FIX beyond 2 m under any recipe.
- HK is descriptive only (2/3 m). There, CDMH scores the same as CDM. The
  Harsh POS is byte-identical; the Deep POS differs but scores the same.
- On PPC the horizontal P95 changes by −4.8 to +0.5 cm. Trimble Odaiba goes
  from 4.78 to 4.92 m.

## Gate

CDMH fails the gate against default on 11 of 12 runs. HK Deep passes at
2/3 m. The main failures are:

- **Wrong >0.5 m increases** on 6 runs: PPC tokyo/run1, nagoya/run1–3 and
  both Trimble drives.
- **Default-correct loss above 1%** on tokyo/run2, nagoya/run3 and u-blox
  Odaiba. On nagoya/run3, default has only 12 FIX epochs.
- **Missing epochs increase by 1–39** on 6 runs. On PPC, CDM already adds
  1–12. Under CDM, 15 of the 53 default-only epochs lie within 5 s after a
  CDM FIX. That is the window of the FLOAT drift-from-fixed guard in
  `gnss_solve`. The tokyo/run2 debug log also shows solver-invalid epochs.
  The mechanism is not established.
- **The recovery P95 is listed as an increase** wherever default has no
  correct-FIX gap of 1 s or longer. With more FIX, the recipes create such
  gaps. The criterion is the same as in earlier rounds.

**Decision: NO_GO.** No default change, and no retuning after the replay.
CDMH stays a combination of existing opt-in flags.

Compared with CDM, the Kalman hold mostly helps where the legacy hold
overwrite kept a biased basin:

- nagoya/run3 wrong >0.5 m: 15 → 4;
- tokyo/run2 loss: 6.26% → 2.28%;
- tokyo/run1 wrong: 5 → 3.

It is neutral or slightly worse elsewhere: Odaiba wrong 6 → 7, u-blox Odaiba
loss 0.87% → 1.08%. It does not touch the single-satellite carrier ramps of
the [FIX streak drift note](rtk_fix_streak_drift.md).

## Reproduce

1. Build develop `2e224ea`.
2. Fetch the PPC data as above and Tokyo/HK as in the
   [handoff](claude_handoff_gnss_fix.md).
3. Run default, CDM and CDMH with the argv in the JSON (`native_argv`,
   `recipes`).
4. Score with exact 1 ms matching for PPC and Trimble; u-blox on the 0.2 s
   grid with 5 ms matching.

The JSON records the binary, input, POS and scorer hashes.

## Open

- **CD family on the PPC RTK profile.** Evaluated in the
  [PPC profile note](rtk_cd_ppc_profile.md): NO_GO. There the Doppler
  consensus check locks out, and it does so under the bare argv here too.
- **Missing epochs.** The extra missing epochs under CD on PPC need their
  own reproduction.
