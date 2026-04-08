# Codex タスク: SD 位相残差 m 級の3つの根本原因を修正

## デバッグで判明した事実

2019ケースで `GNSS_PPP_DEBUG=1` 実行した結果:
- **SD 位相残差 RMS = 2.8〜9.1m**（正常なら数 mm〜数 cm）
- **code_bias_m = 0** for 全衛星・全エポック（SSRには cbias_n=2〜4 のデータあり）
- **phase_bias_m がジャンプ**: G14 `-1.165→5.39→0`、SSR エポック切替で不連続
- **iono がジャンプ**: G26 `5.59m→-0.36m`

## 問題1: code_bias = 0（signal ID マッピング不一致）

### 症状
`ppp_osr.cpp:805-807`:
```cpp
const uint8_t sid = rtcmSsrSignalId(sat.system, osr.signals[f]);
auto cb_it = ssr_cbias.find(sid);
osr.code_bias_m[f] = (cb_it != ssr_cbias.end()) ? cb_it->second : 0.0;
```
`ssr_cbias` にはデータがある（`cbias_n=2〜4`）のに、`find(sid)` で見つからない。

### 調査手順
1. `ssr_cbias` の実際のキー値を調べる（デバッグログ追加）
2. `rtcmSsrSignalId()` が返す値と比較
3. L6 デコーダーが SSR にどの signal ID で code_bias を格納しているか確認
4. マッピングを修正

### 修正場所
- `include/libgnss++/core/signals.hpp` の `rtcmSsrSignalId()` or
- L6 デコーダーの bias 格納ロジック or
- `ppp_osr.cpp:805` の lookup ロジック

## 問題2: phase_bias のジャンプ（phase continuity repair 不十分）

### 症状
SSR エポック切替（`pbias_ref_tow` が 230400→230425→230430）のたびに `phase_bias_m` が数 m ジャンプ。
phase_bias_repair (ppp_osr.cpp:890-905) がジャンプを検出・補償できていない。

### 調査手順
1. `kPhaseBiasJumpLowerCycles` と `kPhaseBiasJumpUpperCycles` の閾値を確認
2. 実際のジャンプ量（cycles）が閾値内に収まっているか
3. `compensatedisp` 相当の GF 差分ベース修復ロジックの有無を確認
4. CLASLIB の `compensatedisp()` (cssr2osr.c:488) と比較

### 修正場所
- `src/algorithms/ppp_osr.cpp` の phase_bias_repair セクション（行890-915）

## 問題3: iono のジャンプ

### 症状
G26 の `iono_l1` が `5.59m→-0.36m` にジャンプ。SSR エポック切替時に大気補正が不連続。

### 調査手順
1. STEC 補間で SSR エポック切替時にどう値が変わるか確認
2. SSR データの STEC が実際に不連続か、それとも補間ロジックの問題か

### 修正場所
- `src/algorithms/ppp_osr.cpp` or `src/algorithms/ppp_atmosphere.cpp`

## 実行手順

**まず問題1（cbias=0）から修正。最もシンプルで効果が確実。**

1. `ppp_osr.cpp:805` の直前にデバッグログを追加して `sid` と `ssr_cbias` のキーを出力
2. 不一致の原因を特定
3. マッピングを修正
4. ビルド確認
5. ベンチマーク実行

次に問題2（pbias ジャンプ）。

## ビルド・ベンチマーク

```bash
cmake --build build -j$(nproc)

GNSS_PPP=./build/apps/gnss_ppp ./scripts/run_gnss_ppp_claslib_parity.sh \
  --obs /workspace/ai_coding_ws/gnssplusplus_thesis_ws/data/clas/claslib/data/0627239Q.obs \
  --nav /workspace/ai_coding_ws/gnssplusplus_thesis_ws/data/clas/claslib/data/sept_2019239.nav \
  --ssr /workspace/ai_coding_ws/gnssplusplus_thesis_ws/data/clas/claslib/data/2019239Q.l6 \
  --clas-epoch-policy hybrid-standard-ppp \
  --out /tmp/bench_2019_resfix.pos \
  --summary-json /tmp/bench_2019_resfix_summary.json

export REF_X=-3957240.1233 REF_Y=3310370.8778 REF_Z=3737527.7041
export JSON_OUT_DIR=/tmp/bench_resfix
mkdir -p /tmp/bench_resfix
./scripts/benchmark_fair_vs_claslib.sh /tmp/bench_2019_resfix.pos
```

## 制約

- **ppp.cpp は変更しない**
- **ppp_ar.cpp, ppp_wlnl.cpp は変更しない**
- 変更してよいファイル: `ppp_osr.cpp`, `signals.hpp`, `ppp_clas.cpp`（必要な場合）
- RMS 3D が 5.68m より悪化したらリバートして原因報告
