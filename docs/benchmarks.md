# Benchmarks

## UrbanNav Tokyo Odaiba

Dataset: [UrbanNav Tokyo Odaiba](https://github.com/IPNL-POLYU/UrbanNavDataset)  
Comparison baseline: [RTKLIB](https://github.com/tomojitakasu/RTKLIB)

Current checked-in snapshot:

- All matched epochs: libgnss++ `11637` vs RTKLIB `8241`
- Fix rate: libgnss++ `8.11%` vs RTKLIB `7.22%`
- All-epoch p95 horizontal: libgnss++ `7.58 m` vs RTKLIB `27.88 m`
- Common-epoch median horizontal: libgnss++ `0.733 m` vs RTKLIB `0.704 m`
- Common-epoch p95 horizontal: libgnss++ `5.94 m` vs RTKLIB `27.67 m`

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
