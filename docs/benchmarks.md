# Benchmarks

gnssplusplus `develop` (post PR #19–#23) dominates RTKLIB `demo5` on the
PPC-Dataset Tokyo and Nagoya urban runs across Fix count, rate, and precision
— with no Phase 2 opt-in flags. UrbanNav-Odaiba is dominated on Fix count,
Hp95, and Vp95; Hmed is within 9 cm of demo5 with
`--enable-wide-lane-ar --wide-lane-threshold 0.10` opted in.

All runs below use `--mode kinematic --preset low-cost --match-tolerance-s 0.25`.

## PPC Tokyo (kinematic, low-cost preset, no Phase 2 flags)

| Run  | gnssplusplus Fix / rate | RTKLIB Fix / rate | Hmed (m)              | Vp95 (m)               |
|------|------------------------:|------------------:|:---------------------:|:----------------------:|
| run1 | **3572 / 81.26%**       | 2418 / 30.52%     | **0.037** vs 1.567 (42×) | **1.259** vs 36.703 (29×) |
| run2 | **4674 / 80.12%**       | 2127 / 27.58%     | **0.016** vs 0.835 (52×) | **0.313** vs 42.624 (136×) |
| run3 | **7516 / 86.84%**       | 5778 / 40.55%     | **0.012** vs 0.666 (56×) | **0.137** vs 24.521 (179×) |

## PPC Nagoya (same preset)

| Run  | Fix delta    | rate delta    | Hmed delta     |
|------|-------------:|--------------:|---------------:|
| run1 | **+1743**    | **+58.03 pp** | **9× better**  |
| run2 | **+1735**    | **+64.00 pp** | **10× better** |
| run3 | **+154**     | **+50.16 pp** | **44× better** |

## UrbanNav Tokyo Odaiba

Dataset: [UrbanNav Tokyo Odaiba](https://github.com/IPNL-POLYU/UrbanNavDataset)  
Comparison baseline: [RTKLIB](https://github.com/tomojitakasu/RTKLIB)

Current checked-in snapshot (kinematic, low-cost preset):

| Config                                                                     | Fix              | Rate        | Hmed (m)              | Hp95 (m)    | Vp95 (m)    |
|----------------------------------------------------------------------------|-----------------:|------------:|:---------------------:|:-----------:|:-----------:|
| RTKLIB demo5                                                               | 595              | 7.22%       | **0.707**             | 27.878      | 45.212      |
| libgnss++ default                                                          | **1268** (+673)  | **36.98%**  | 1.707                 | **19.585**  | **25.495**  |
| libgnss++ `--enable-wide-lane-ar --wide-lane-threshold 0.10`               | 818 (+223)       | 33.65%      | **0.799** (9 cm gap)  | **19.971**  | **26.429**  |

| RTKLIB 2D | libgnss++ 2D |
|---|---|
| ![RTKLIB 2D trajectory](driving_odaiba_comparison_rtklib_2d.png) | ![libgnss++ 2D trajectory](driving_odaiba_comparison_libgnss_2d.png) |

More artifacts:

- [Odaiba social card](driving_odaiba_social_card.png)
- [Full comparison figure](driving_odaiba_comparison.png)
- [Scorecard](driving_odaiba_scorecard.png)
- Optional side-by-side PPP reference: [JAXA-SNU/MALIB](https://github.com/JAXA-SNU/MALIB)
- Additional low-cost GNSS reference: [rtklibexplorer/RTKLIB](https://github.com/rtklibexplorer/RTKLIB)

## PPC-Dataset

External dataset source: [taroz/PPC-Dataset](https://github.com/taroz/PPC-Dataset)

Example:

```bash
python3 apps/gnss.py ppc-demo \
  --dataset-root /datasets/PPC-Dataset \
  --city tokyo \
  --run run1 \
  --solver rtk \
  --require-realtime-factor-min 1.0 \
  --summary-json output/ppc_tokyo_run1_rtk_summary.json

python3 apps/gnss.py ppc-rtk-signoff \
  --dataset-root /datasets/PPC-Dataset \
  --city tokyo \
  --rtklib-bin /path/to/rnx2rtkp \
  --summary-json output/ppc_tokyo_run1_rtk_signoff.json
```

`ppc-rtk-signoff` is the fixed-threshold path for Tokyo/Nagoya quality and
runtime checks, with optional RTKLIB delta gates.
