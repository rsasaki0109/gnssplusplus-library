# Urban canyon RTK: when to try the opt-in robustness options

Use this page when a kinematic RTK run in a dense city has metre-level or worse
FLOAT errors, typically while the vehicle is **stopped** under an overpass or
between tall buildings. Three opt-in `gnss_solve` options target that case.
All are off by default and leave the default output byte-identical. None of
them is a universal improvement: each helps some runs and costs a little on
others, so compare OFF and ON on your own data before keeping one.

| Option | Targets | Try it when |
|---|---|---|
| `--rtk-canyon-carrier-prior` | Blocked-sky NLOS code while the carrier is intact. Holds the previous float position, advanced by a validated time-differenced carrier displacement, instead of re-seeding it from the biased code; seeds new ambiguities from the carrier. | Large vertical errors while stopped or crawling in a canyon, with the carrier still tracking |
| `--rtk-snr-weighting --rtk-snr-elevation-aware` | Signals whose C/N0 is far below what their elevation should give (an overpass blocks a 66° satellite: 48 → 33 dB-Hz). | Many satellites with sudden C/N0 drops at high elevation |
| `--rtk-snr-code-only` (with the two above) | Same, but inflates only the code variance, keeping the carrier weight. | The elevation-aware option costs you fixes |

These target FLOAT errors. For wrong fixes, two opt-in rules restrict the hold-fix
fallback, which keeps a FIX without a ratio test when ambiguity resolution fails.
The figures are FIXED epochs more than 0.5 m (3D) off the reference, with
`gnss_solve --preset low-cost`:

- **`--hold-fix-min-matched-pairs 12`** refuses the fallback when fewer than 12
  pairs carry a held integer.
  - PPC and UrbanNav Tokyo runs: 5.77 % to 2.85 %, keeping 101 % of the correct
    FIXED epochs.
  - SmartPNT-POS and TEX-CUP urban runs never used before: 1.69 % to 1.07 %,
    keeping 98.5 %.
- **`--hold-fix-unchanged-set`** refuses it when a satellite has joined or an
  ambiguity was reset, instead of rounding that ambiguity. PPC and UrbanNav Tokyo
  runs: 5.77 % to 3.10 %, at the cost of 7.5 % of the FIXED epochs.

See [details](../interfaces.md#rtk-hold-fix-fallback-known-weakness).

A third opt-in, **`--min-fix-satellites 4`**, refuses a fix from ambiguity
resolution that rests on fewer than 4 non-reference satellites. Added to
`--hold-fix-min-matched-pairs 12`:

- Held-out GREAT-WHU, SmartPNT-POS and TEX-CUP urban runs: 0.95 % to 0.37 %,
  keeping 97.5 % of the correct FIXED epochs.
- PPC and UrbanNav Tokyo runs: 2.85 % to 1.22 %, keeping 97.6 %.

See [details](../interfaces.md#rtk-fixes-from-few-satellites).

## First output

Run the same data twice, once without and once with the option. A PPC-style
run directory works as is:

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --target gnss_solve --parallel 2
mkdir -p output/use_cases/canyon

RUN=data/PPC-Dataset/nagoya/run2
COMMON="--rover $RUN/rover.obs --base $RUN/base.obs --nav $RUN/base.nav \
  --mode kinematic --preset low-cost --no-kml"

./build/apps/gnss_solve $COMMON --out output/use_cases/canyon/off.pos
./build/apps/gnss_solve $COMMON --rtk-canyon-carrier-prior \
  --out output/use_cases/canyon/prior.pos | tee output/use_cases/canyon/prior.log
```

The ON run prints whether the prior actually engaged:

```text
  canyon carrier prior: enabled epochs=9390 tdcp_valid=8909 bridged=22 engaged_epochs=525 engagements=3 contradictions=0
```

`engaged_epochs=0` means the run never met the blocked-sky condition and the
option changed nothing that matters; `contradictions` counts releases because
the measurement update moved more than 3 m away from the prior.

The same options work from a TOML config (`rtk_canyon_carrier_prior = true`,
`rtk_snr_weighting = true`, `rtk_snr_elevation_aware = true`) and pass through
`python3 apps/gnss.py ppc-demo` and `python3 apps/gnss.py ppc-coverage-matrix`.

## Score both runs against your reference

With a PPC run, `ppc-demo` scores each arm against `reference.csv`
(`--max-epochs -1` processes the whole run; the default is a 120-epoch smoke):

```bash
for arm in off prior; do
  extra=""; [ "$arm" = prior ] && extra="--rtk-canyon-carrier-prior"
  python3 apps/gnss.py ppc-demo --dataset-root data/PPC-Dataset \
    --city nagoya --run run2 --solver rtk --preset low-cost --max-epochs -1 \
    --out output/use_cases/canyon/ppc_$arm.pos \
    --summary-json output/use_cases/canyon/ppc_$arm.json $extra
done
```

Compare `ppc_official_score_pct`, `fix_rate_pct`, `p95_h_m` and
`p95_abs_up_m` in the two JSON files. The answer depends on the profile:
on nagoya run2 the carrier prior moved the official score by +2.63 pp in the
coverage profile (`ppc-coverage-matrix --ratio 2.4`) but by −0.43 pp in the
`ppc-demo --preset low-cost` profile above.

## Measured effect (OFF → ON)

All numbers below were reproduced on develop `82d73d3f` (Linux, gcc 13.3,
Release). Odaiba uses the surveyed base and the 10 Hz Trimble rover; HK uses
the default profile; PPC uses the coverage profile (`--ratio 2.4`).

**UrbanNav Tokyo Odaiba, default profile** (Hp95 / Vp95 in metres; the
stopped window is tow 273750–273860):

| Arm | All epochs Hp95 / Vp95 | Stopped window Vp95 | Fixes |
|---|---|---|---|
| OFF | 4.77 / 11.54 | 27.19 | 1298 |
| `--rtk-canyon-carrier-prior` | 3.37 / 6.95 | 2.04 | 1298 |
| `--rtk-snr-weighting --rtk-snr-elevation-aware` | 5.58 / 5.84 | 15.38 | 1298 |
| elevation-aware + `--rtk-snr-code-only` | 5.88 / 6.45 | 25.68 | 1298 |
| carrier prior + elevation-aware | 2.72 / 4.72 | 1.70 | 1298 |

**UrbanNav Tokyo Odaiba, `--preset odaiba`:**

| Arm | All epochs Hp95 / Vp95 | Fixes | Wrong fixes (H > 1 m) |
|---|---|---|---|
| OFF | 5.28 / 13.89 | 7961 | 12 |
| `--rtk-canyon-carrier-prior` | 2.45 / 3.19 | 8089 | 12 |
| `--rtk-snr-weighting --rtk-snr-elevation-aware` | 4.77 / 11.31 | 7734 | 7 |
| carrier prior + elevation-aware | 2.42 / 3.17 | 8796 | **1009** |

**Do not combine the carrier prior with the SNR options under
`--preset odaiba`.** That combination fixes 1,040 epochs in the stopped window,
1,003 of them wrong by about 2.4 m horizontally, which a downstream user would
treat as centimetre-level. With the default profile the same combination has
no wrong fixes and is the best Odaiba arm.

**UrbanNav Hong Kong** (all epochs, Hp95 / Vp95 in metres):

| Arm | Deep-Urban-1 | Harsh-Urban-1 |
|---|---|---|
| OFF | 7.24 / 25.09 | 13.90 / 34.87 |
| `--rtk-canyon-carrier-prior` | 6.61 / 6.37 | 13.87 / 34.41 |
| `--rtk-snr-weighting --rtk-snr-elevation-aware` | 7.65 / 18.06 | 15.61 / 35.72 |
| carrier prior + elevation-aware | 7.52 / 17.50 | 15.53 / 34.46 |

**PPC Tokyo/Nagoya, coverage profile** (official score, pp change per run):

| Arm | Weighted | tokyo1 | tokyo2 | tokyo3 | nagoya1 | nagoya2 | nagoya3 |
|---|---|---|---|---|---|---|---|
| OFF (score) | 68.120 | 69.62 | 83.32 | 79.01 | 52.84 | 30.02 | 52.12 |
| `--rtk-canyon-carrier-prior` | −0.09 | −0.27 | −0.75 | −0.65 | 0.00 | +2.63 | +0.68 |
| `--rtk-snr-weighting --rtk-snr-elevation-aware` | +1.04 | +3.16 | −2.47 | +0.44 | +3.95 | +6.71 | −6.97 |
| elevation-aware + `--rtk-snr-code-only` | +0.22 | −0.47 | +0.42 | +0.45 | +0.66 | +2.11 | −2.51 |
| carrier prior + elevation-aware | +1.31 | +3.39 | −0.32 | +0.11 | +3.95 | +6.71 | −6.97 |

On PPC the combination does not repeat the Odaiba wrong-fix failure: FIXED
epochs outside the PPC correct-fix tolerance fall on five runs (for example
tokyo3 455 → 199) but rise on nagoya3 (467 → 602 of 2,744 → 2,170 fixes),
the run where both SNR arms also lose about 7 pp.

## Exit criteria

Keep an option only when, on your own runs with an independent reference:

- the error statistic you care about (for example Vp95 over all epochs) drops;
- the fix count does not fall more than you can accept; and
- wrong fixes (fixed epochs more than 1 m off the reference) do not increase.

The last check matters most: a FLOAT epoch that is 3 m off is labelled as such,
a wrong FIX is not.

## Boundary

- The options do not create fixes in a blocked sky; they keep the FLOAT
  solution from following biased code. On Odaiba the stopped window stays
  FLOAT with a 1–3 m horizontal error.
- The carrier prior does not engage on the second Odaiba stop
  (tow 274230–274300), whose error is unchanged.
- Odaiba was the development set for the carrier prior, so its Odaiba gain is
  not independent evidence; PPC and Hong Kong were the check set
  ([PR #596](https://github.com/rsasaki0109/gnssplusplus-library/pull/596)).
- The carrier prior needs kinematic mode with dual-receiver L1 carrier; it is
  inactive in static, moving-base and external-prior modes.

## Next step

For a vehicle with an IMU, combine the chosen RTK option with the continuity
bridge in [Urban RTK and IMU continuity](urban_rtk_fgo.md). Every knob is
listed in [RTK tuning gates](../rtk_tuning_gates.md); the `.pos` format and the
solver API are in [Interfaces](../interfaces.md), the sign-off process in
[Validation](../validation.md), and the default-profile Odaiba and PPC
baselines in [Benchmarks](../benchmarks.md).
