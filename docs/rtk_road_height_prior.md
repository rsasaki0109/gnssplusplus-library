# Road-surface height prior for `gnss_solve` (PLATEAU / GSI DEM)

This note adds an opt-in height prior to the RTK float filter. It takes the
road (or ground) height from a truth-free map. **No default and no preset
change.** The publish-only mode is the recommended one. Record:
[JSON](road_height_prior_evaluation.json).

The reference trajectory is used only for scoring and for the analysis-only
[phase audit](rtk_phase_audit.md). Maps, antenna offsets and sigmas come from
PLATEAU or GSI data and from each run's own solution.

## Why

- **The float is the bottleneck, not AR acceptance.**
  - Under `--preset urban` (U) on the ten UrbanNav Tokyo and PPC runs, the
    epochs where LAMBDA fails have a float 3D error with a median of 3 m.
  - Only 3% of them are within 0.2 m.
  - Relaxing AR acceptance could add at most about 2 pp of FIX.
- **PLATEAU has heights, not lanes.**
  - PLATEAU LOD3 `tran` tiles split the road into carriageway, intersection,
    sidewalk and island. There are no lanes.
  - The reference trajectory is a median of 3.5–5 m from the carriageway edge,
    so a horizontal constraint would be weak.
  - The height is a strong constraint. The runs' own FIX epochs put the antenna
    1.50–1.55 m above the road surface, with an IQR of 0.10–0.17 m.

## Change

`gnss_solve` options, all off by default:

| Option | Meaning |
| --- | --- |
| `--road-height-map <csv>` | `lat_deg,lon_deg,height_m` (ellipsoidal) road or ground heights |
| `--road-height-sigma-m <s>` | prior sigma; 0 is off |
| `--road-height-offset-m <h>` | antenna height above the map surface |
| `--road-height-radius-m <r>` | max horizontal distance to a map point (default 1.5) |
| `--road-height-publish-only` | apply the update only to the published FLOAT |

There are two modes:

- **In-filter (default when the map is given).**
  - The prior runs in every epoch that reaches the float update: satellite
    data collected and at least 4 satellites.
  - Before that update, one scalar Kalman update pulls the re-seeded height
    toward the road height at the seed plus the offset.
  - Its variance is sigma² plus the road gradient times the seed's horizontal
    covariance. The gradient comes from a plane fit over the map points within
    two cells.
- **Publish-only.**
  - After the float update, the same update is applied to the position block
    of the published FLOAT only.
  - The filter state and ambiguity resolution stay GNSS-only. The published
    FLOAT also enters the solution history.

The debug epoch log gains three columns: `road_height_applied`,
`road_height_innovation_m` and `road_height_sigma_m`. The tests are
`RoadHeightMapTest.*`, `RTKRoadHeightPriorTest.*` and
`tests/test_road_height_maps.py`.

## Maps

`scripts/experiments/road_height/road_height_maps.py` builds one map per run.
The grid is 1 m, within 30 m of the run's own U solution (any status).

- **Tokyo: PLATEAU LOD3 `tran`.**
  - Wards: Chiyoda, Chuo, Minato, Koto, Shinagawa, Shinjuku and Shibuya, plus
    the MLIT road set. Every leaf tile is used, not only those near a run.
  - Surfaces: carriageway (車道部) and intersection (車道交差部).
  - A node whose surfaces disagree by more than 0.5 m (an overpass) is dropped.
- **Nagoya: GSI DEM5A.**
  - z15 tiles plus a geoid plane fitted to five GSI geoid points.
  - The geoid changes by about 0.2 m across the area.
  - A DEM is ground height, so overpasses cannot be removed.

The offset and sigma come from U's FIX epochs on the map: the median and
1.4826 × MAD of (FIX height − map height). The sigma used is
max(MAD sigma, 0.10 m). The tool rebuilds all ten evaluated maps byte for byte.

| Map | Run | Points | Offset (m) | sigma MAD (m) |
| --- | --- | ---: | ---: | ---: |
| PLATEAU | PPC tokyo/run1 | 160066 | 1.544 | 0.073 |
| PLATEAU | PPC tokyo/run2 | 99162 | 1.493 | 0.124 |
| PLATEAU | PPC tokyo/run3 | 225780 | 1.530 | 0.174 |
| PLATEAU | Trimble Odaiba | 93948 | 1.938 | 0.125 |
| PLATEAU | u-blox Odaiba | 89144 | 1.942 | 0.120 |
| PLATEAU | Trimble Shinjuku | 126225 | 1.944 | 0.065 |
| PLATEAU | u-blox Shinjuku | 130117 | 1.956 | 0.054 |
| DEM | PPC nagoya/run1 | 256737 | 1.553 | 0.085 |
| DEM | PPC nagoya/run2 | 258447 | 1.604 | 0.060 |
| DEM | PPC nagoya/run3 | 190797 | 1.672 | 0.068 |

## Results

Each cell gives the official score (PPC-style: 3D ≤ 0.5 m, distance weighted),
the FIX count and the audited wrong FIX (> 0.5 m).

| Run | U | In-filter (UR3) | Publish-only (URP) |
| --- | --- | --- | --- |
| PPC tokyo/run1 | 39.7% / 4328 / 24 | 55.3% / 5538 / 40 | 41.5% / 4348 / 24 |
| PPC tokyo/run2 | 75.9% / 6669 / 5 | 79.8% / 6838 / 4 | 78.5% / 6691 / 5 |
| PPC tokyo/run3 | 75.2% / 10910 / 35 | 75.5% / 10751 / 25 | 75.9% / 10933 / 35 |
| Trimble Odaiba | 69.2% / 7467 / 9 | 68.6% / 7435 / 18 | 71.0% / 7470 / 9 |
| u-blox Odaiba | 31.8% / 3130 / 0 | 32.8% / 3218 / 1 | 32.8% / 3131 / 0 |
| Trimble Shinjuku | 49.8% / 11491 / 119 | 58.1% / 13597 / 75 | 54.8% / 11565 / 119 |
| u-blox Shinjuku | 15.7% / 3582 / 3 | 22.3% / 4478 / 25 | 18.0% / 3619 / 3 |
| PPC nagoya/run1 | 54.5% / 5280 / 19 | 59.1% / 5239 / 84 | 58.8% / 5291 / 19 |
| PPC nagoya/run2 | 32.9% / 4404 / 17 | 37.3% / 4779 / 18 | 37.0% / 4410 / 17 |
| PPC nagoya/run3 | 33.1% / 1069 / 18 | 45.5% / 1733 / 97 | 37.0% / 1077 / 17 |

| Totals | U | In-filter (UR3) | Publish-only (URP) |
| --- | ---: | ---: | ---: |
| Tokyo (7 runs) official mean | 51.04% | 56.05% | 53.19% |
| Tokyo audited wrong FIX | 195 | 188 | 195 |
| Nagoya (3 runs) official mean | 40.15% | 47.31% | 44.30% |
| Nagoya audited wrong FIX | 54 | 199 | 53 |
| U-correct FIX lost (10 runs) | – | 1996 | 2 |

What the two modes do:

- **Publish-only keeps every run's wrong FIX at U** (equal, or one lower) and
  loses 2 correct FIX over the ten runs. It raises the score on all ten runs: +2.2 pp in Tokyo
  and +4.2 pp in Nagoya. The vertical tail shrinks too; for example,
  nagoya/run3 Vp95 goes from 46.0 m to 9.2 m.
- **In-filter raises the score more**: +5.0 pp in Tokyo and +7.2 pp in Nagoya.
  It also adds about 9% FIX, but it adds wrong FIX as well:
  - Nagoya 54 → 199;
  - tokyo/run1 24 → 40;
  - Trimble Odaiba 9 → 18;
  - u-blox Shinjuku 3 → 25.

## Frozen rules and predictions

Every variant was frozen before its replay. Each later rule discloses what the
earlier replays showed. Nagoya was not used to design any variant.

| Variant | Prediction | Outcome |
| --- | --- | --- |
| UR (v1: after the re-seed, every epoch) | official up on the three Tokyo PPC runs; wrong ≤ +10%; Vp95 down | failed: tokyo/run3 74.65%; Odaiba wrong 9 → 17 |
| UR2 / UR2F (v2: after the update; + fixed-height check) | same | failed: Shinjuku wrong 119 → 219 (UR2); the check loses 989 / 1254 correct FIX on tokyo/run2 / run3 |
| UR3 (in-filter, the code) | official up on the three Tokyo PPC runs; wrong ≤ U + max(10%, 2); Vp95 down | official held; wrong and Vp95 failed (above) |
| UR3 on Nagoya (held out) | mean official up; wrong ≤ +10%; Vp95 down on ≥ 2 runs | official and Vp95 held; wrong failed (54 → 199) |
| URP (publish-only, the code) | wrong ≤ +5% and no run above U + max(10%, 2); FIX within ±1%; official up in both cities | wrong and official held; FIX +1.03% on u-blox Shinjuku (more FIX) |

## What did not work

- **Prior in epochs without satellites (v1).** On tokyo/run3 at TOW 180561.2 the
  innovation was 32 m in an epoch with no satellites. The float then stayed
  0.48 m off. The code now applies the prior only in epochs that reach the
  float update.
- **Prior after the float update (v2).** The code outliers are then no longer
  rejected against a height-constrained prediction. Shinjuku wrong FIX went
  from 119 to 219, and u-blox Shinjuku from 3 to 108.
- **Rejecting a FIX whose height is off the map (3 sigma).** It removed
  989 / 1254 correct FIX on tokyo/run2 / run3, where the map is locally off.
  The option is not in the code.
- **Separating the wrong in-filter FIX.**
  - The in-filter prior creates 5659 new FIX, of which 78 are wrong (1.4%).
    U's own FIX are about 0.16% wrong.
  - Post-fix residuals, the phase fit and the ratio do not separate them. A
    post-fix RMS above 0.10 m catches 23 of the 35 wrong AR fixes, but also
    rejects 10% of the correct new ones.
  - This matches the [ambiguity diagnostics note](rtk_ambiguity_consistency_diagnostics.md).

## Decision

Both modes stay opt-in, and `--preset urban` is unchanged. Use
`--road-height-publish-only` when wrong FIX must not rise. The in-filter mode
is for scoring by 0.5 m availability, where its extra FIX outweigh its wrong
FIX.

## Reproduce

1. Run U: `gnss_solve --preset urban` with the argv of the
   [CDMH note](rtk_cdmh_ppc_evaluation.md).
2. Build the maps. `pip install DracoPy` is needed for the PLATEAU tiles; the
   tileset URLs are in the PLATEAU data catalog,
   `https://api.plateauview.mlit.go.jp/datacatalog/plateau-datasets`, under
   `type_en == "tran"`, `lod == 3`.

   ```sh
   T=scripts/experiments/road_height/road_height_maps.py
   python3 $T plateau-fetch --tileset-url <ward tileset.json ...> --out tiles/
   python3 $T plateau-cache --tiles tiles/ --out road_triangles.npy
   python3 $T plateau-map --triangles road_triangles.npy --solution U/tokyo-run1.pos --out maps/tokyo-run1.csv
   python3 $T dem-fetch --solution U/nagoya-run*.pos --out dem/
   python3 $T dem-geoid --solution U/nagoya-run*.pos --out geoid.txt
   python3 $T dem-map --tiles dem/ --geoid geoid.txt --solution U/nagoya-run1.pos --out maps/nagoya-run1.csv
   ```

3. Run the recipe with `offset_m` and max(`sigma_mad_m`, 0.10) from the map's JSON:

   ```sh
   gnss_solve ... --preset urban --road-height-map maps/<run>.csv \
     --road-height-sigma-m <sigma> --road-height-offset-m <offset> [--road-height-publish-only]
   ```

4. Score as in the [phase-fit note](rtk_hold_fix_phase_fit.md). The official
   score comes from `ppc_official_distance_score()` in
   `apps/commands/benchmarks/gnss_ppc_metrics.py`.
