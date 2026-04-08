# Track A ベンチマーク基準線

**Updated:** 2026-04-08

## 概要

PLAN.md Track A §2.4 の「ベンチスナップショット 1 セット固定」として、
2018/2019 の 2 ケースで `--claslib-parity` 実行結果を記録する。

## データ

| ケース | OBS | NAV | SSR (L6) |
|--------|-----|-----|----------|
| 2018 | `0161329A.obs` | `tskc2018329.nav` | `2018328X_329A.l6` |
| 2019 | `0627239Q.obs` | `sept_2019239.nav` | `2019239Q.l6` |

データ置き場: `/media/sasaki/aiueo/ai_coding_ws/gnssplusplus_thesis_ws/data/clas/claslib/data/`

## 基準座標 (RINEX APPROX POSITION XYZ)

| ケース | REF_X | REF_Y | REF_Z |
|--------|-------|-------|-------|
| 2018 | -3815683.6319 | 3065138.8371 | 4076533.5529 |
| 2019 | -3957240.1233 | 3310370.8778 | 3737527.7041 |

## 実行コマンド

```bash
cd /media/sasaki/aiueo/ai_coding_ws/rtklib_v2_ws/gnssplusplus-library
DATA=/media/sasaki/aiueo/ai_coding_ws/gnssplusplus_thesis_ws/data/clas/claslib/data

# 2019 case (hybrid-standard-ppp)
GNSS_PPP=./build/apps/gnss_ppp ./scripts/run_gnss_ppp_claslib_parity.sh \
  --obs $DATA/0627239Q.obs \
  --nav $DATA/sept_2019239.nav \
  --ssr $DATA/2019239Q.l6 \
  --clas-epoch-policy hybrid-standard-ppp \
  --out /tmp/bench_2019_hybrid.pos \
  --summary-json /tmp/bench_2019_hybrid_summary.json

# 2018 case (hybrid-standard-ppp)
GNSS_PPP=./build/apps/gnss_ppp ./scripts/run_gnss_ppp_claslib_parity.sh \
  --obs $DATA/0161329A.obs \
  --nav $DATA/tskc2018329.nav \
  --ssr $DATA/2018328X_329A.l6 \
  --clas-epoch-policy hybrid-standard-ppp \
  --out /tmp/bench_2018_hybrid.pos \
  --summary-json /tmp/bench_2018_hybrid_summary.json

# Benchmark evaluation
export REF_X=... REF_Y=... REF_Z=...  # see table above
export JSON_OUT_DIR=/tmp/bench_track_a
./scripts/benchmark_fair_vs_claslib.sh /tmp/bench_XXXX_hybrid.pos
```

## 結果 (2026-04-08, develop branch)

### 2019 ケース (hybrid-standard-ppp)

| 項目 | 値 |
|------|-----|
| Processed epochs | 3599 |
| Float solutions | 3520 |
| Fixed solutions | 79 |
| Fallback solutions | 0 |
| Solution rate | 100% |
| Fix rate | 2.20% |
| **RMS H (last 100)** | **4.83m** |
| **RMS V (last 100)** | **2.99m** |
| **RMS 3D (last 100)** | **5.68m** |

### 2018 ケース (hybrid-standard-ppp)

| 項目 | 値 |
|------|-----|
| Processed epochs | 3629 |
| Float solutions | 2750 |
| Fixed solutions | 0 |
| Fallback solutions | 879 |
| Solution rate | 75.8% |
| Fix rate | 0% |
| **RMS H (last 100)** | **6.59m** |
| **RMS V (last 100)** | **0.35m** |
| **RMS 3D (last 100)** | **6.60m** |

### 2018 ケース (strict-osr)

| 項目 | 値 |
|------|-----|
| Processed epochs | 3629 |
| Float solutions | 0 |
| Fallback solutions | 3629 |
| Solution rate | 0% |
| **RMS 3D (last 100)** | **5.95m** |

### 過去実験との比較 (2019)

| 設定 | RMS 3D |
|------|--------|
| 今回 claslib-parity hybrid | 5.68m |
| 過去 iflc_float_baseline | 4.12m |
| 過去 osr_ar_strict | 4.97m |
| 過去 osr_float_strict | 4.97m |

## 所見

1. **2018 ケースは CLAS 補正が機能しない** — strict-osr で全エポック fallback、過去実験でも 44-58m 級。データ or フォーマットの問題。
2. **2019 ケースは CLAS OSR が動作** — fallback=0、float 97.8%。ただし精度は 5m 級で CLASLIB 実用レベル (cm) には遠い。
3. **Fix rate 2.2%** — AR がほぼ効いていない。Track B の subtype-4/6 調査が必要。
4. **基準線として**: 今後の Track B arm 変更で、この RMS 3D 5.68m (2019) が改善するか劣化するかを見る。
