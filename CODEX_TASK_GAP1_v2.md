# Codex タスク: ギャップ1 v2 — WL FCB 適用の正しい場所

## 前回の失敗と原因

### Codex (v1) の失敗: RMS 5.68m → 50.28m に退化
**原因**: SD (Single Difference = 単衛星) の MW 平均に FCB を引いた。
しかし CLASLIB は **DD (Double Difference = 衛星ペア差分)** で FCB を適用している。
SD に FCB を引くと、全衛星の WL 整数値がずれ、NL 計算が連鎖的に壊れる。

### Cursor の失敗: PPP core (ppp.cpp +630行) を広範囲に書き換えた
**原因**: フィルタの状態遷移やプロセスノイズを変更し、float 解自体を壊した。

## 重要な制約: 変えてよいファイルと変えてはいけないファイル

**変えてよい**: `src/algorithms/ppp_ar.cpp` の `applyWideLaneFixes()` のみ
**変えてはいけない**: `ppp.cpp`, `ppp_wlnl.cpp`, `ppp_osr.cpp`, `ppp_clas.cpp`, `ppp_clas_epoch.cpp`, `ppp_shared.hpp`, その他全ファイル

## アーキテクチャ理解（これが最重要）

libgnss++ の WL/NL AR の処理フロー:

```
1. MW (Melbourne-Wübbena) 平均の蓄積 (ppp.cpp:2318-2324)
   - 各衛星の SD MW を cycles に変換して蓄積
   - ambiguity.mw_mean_cycles = mw_sum_cycles / mw_count

2. WL 固定 (ppp_ar.cpp:applyWideLaneFixes, 行90-130)
   - SD MW 平均を四捨五入して WL 整数値を決定
   - ★ここが修正対象 ★

3. NL DD 構成 (ppp_ar.cpp:tryWlnlFix, 行419-524)
   - WL 固定済み衛星からシステム毎に参照衛星を選択
   - DD NL = ref.nl_ambiguity_cycles - sat.nl_ambiguity_cycles
   - LAMBDA で DD NL 整数解を探索

4. 位置制約更新 (ppp_ar.cpp:tryWlnlFix, 行534-)
   - 固定した DD 整数解で KF に制約を入れる
```

## CLASLIB の WL 固定方式 (ppp_ar.c:192-215)

```c
// CLASLIB: DD の WL アンビギュイティ = 衛星ペアの MW 差 + FCB の DD
BW = (amb1->LC[0] - amb2->LC[0]) / lam_WL + nav->wlbias[sat1-1] - nav->wlbias[sat2-1];
*NW = ROUND(BW);
```

CLASLIB は DD ベースなので、`BW` は衛星ペアの差。`wlbias` も DD (差分) で入る。

## libgnss++ での正しい FCB 適用方法

libgnss++ は SD ベースで WL を固定している（行110-111）:
```cpp
const double mw_mean = ambiguity.mw_mean_cycles;  // SD MW平均
const int wl_int = static_cast<int>(std::round(mw_mean));  // 四捨五入
```

**CLAS の phase_bias_m から SD の WL FCB (cycles) を計算して、MW 平均から引いてから丸める。**

SD では:
```
WL_corrected = mw_mean_cycles - wl_fcb_cycles
wl_int = round(WL_corrected)
```

ここで `wl_fcb_cycles` は:
```
wl_fcb_cycles = (f1 * pb1_m - f2 * pb2_m) / ((f1 - f2) * lambda_wl)
```
- `pb1_m` = `OSRCorrection::phase_bias_m[0]` (L1 phase bias in meters)
- `pb2_m` = `OSRCorrection::phase_bias_m[1]` (L2 phase bias in meters)
- `f1` = L1 frequency (Hz), `f2` = L2 frequency (Hz)
- `lambda_wl` = c / (f1 - f2)

SD で FCB を引いておけば、DD を取ったときに自動的に `wlbias[sat1] - wlbias[sat2]` と等価になる。
**これが SD ベースの libgnss++ で CLASLIB と同じ効果を得る正しい方法。**

## しかし: FCB が MW 蓄積に既に含まれていないか確認が必要

`ppp.cpp:2258` で MW を計算するとき、`ppp_utils::calculateMelbourneWubbena()` に渡す
`carrier_phase` が、CLAS OSR 補正済み（phase_bias_m が既に引かれている）なら、
MW にすでに FCB が反映されている可能性がある。

**確認手順:**
1. `calculateMelbourneWubbena()` に渡される `primary->carrier_phase` が raw か corrected か確認
2. CLAS OSR パス (`ppp_clas.cpp`) で観測値に phase_bias_m を適用しているか確認
3. 既に適用済みなら、`applyWideLaneFixes()` での追加補正は不要（二重適用になる）
4. 未適用なら、上記の補正を `applyWideLaneFixes()` に追加する

**二重適用したら退化する。これが Codex v1 で起きたこと。**

## 具体的なタスク

### Step 1: 調査（コード変更なし）

以下を調べて、結論を stderr に出力するデバッグコードを一時的に追加:

1. `ppp.cpp` の MW 計算 (行2258) に渡される `carrier_phase` が:
   - raw 観測値なのか
   - CLAS OSR で phase_bias_m が引かれた後なのか

2. `ppp_clas.cpp` / `ppp_osr.cpp` で `carrier_phase` に `phase_bias_m` を適用しているか

3. `OSRCorrection::phase_bias_m[0]` と `[1]` に実際に値が入っているか（0 ではないか）

### Step 2: 判断

- phase_bias_m が carrier_phase に **未適用** → `applyWideLaneFixes()` で FCB 補正を追加
- phase_bias_m が carrier_phase に **適用済み** → MW にすでに FCB が入っている → WL 固定に追加補正は不要。問題は別の場所にある

### Step 3: 実装（Step 2 の結論に基づく）

**未適用の場合のみ**: `applyWideLaneFixes()` (ppp_ar.cpp 行110-111) を修正:

```cpp
// 現在:
const double mw_mean = ambiguity.mw_mean_cycles;
const int wl_int = static_cast<int>(std::round(mw_mean));

// 修正案 (FCB が未適用の場合):
double wl_fcb_cycles = 0.0;
if (ambiguity.wide_lane_fcb_valid) {
    wl_fcb_cycles = ambiguity.wide_lane_fcb_cycles;
}
const double mw_corrected = ambiguity.mw_mean_cycles - wl_fcb_cycles;
const int wl_int = static_cast<int>(std::round(mw_corrected));
const double frac = mw_corrected - wl_int;
```

ただし `wide_lane_fcb_cycles` の計算と `PPPAmbiguityInfo` への追加も必要。
**ppp_wlnl.cpp で OSR 情報から FCB を計算して ambiguity_states_ に設定する**
（Codex v1 の `updateWideLaneFcbCycles()` のロジック自体は正しかった。
 問題は `tryWlnlFix()` の KF 更新を壊したこと。）

## ビルド・テスト・ベンチマーク

```bash
cmake --build build -j$(nproc)

# 2019ベンチマーク (基準線: RMS 3D = 5.68m, fix rate = 2.2%)
GNSS_PPP=./build/apps/gnss_ppp ./scripts/run_gnss_ppp_claslib_parity.sh \
  --obs /media/sasaki/aiueo/ai_coding_ws/gnssplusplus_thesis_ws/data/clas/claslib/data/0627239Q.obs \
  --nav /media/sasaki/aiueo/ai_coding_ws/gnssplusplus_thesis_ws/data/clas/claslib/data/sept_2019239.nav \
  --ssr /media/sasaki/aiueo/ai_coding_ws/gnssplusplus_thesis_ws/data/clas/claslib/data/2019239Q.l6 \
  --clas-epoch-policy hybrid-standard-ppp \
  --out /tmp/bench_2019_gap1v2.pos \
  --summary-json /tmp/bench_2019_gap1v2_summary.json

export REF_X=-3957240.1233 REF_Y=3310370.8778 REF_Z=3737527.7041
export JSON_OUT_DIR=/tmp/bench_gap1v2
mkdir -p /tmp/bench_gap1v2
./scripts/benchmark_fair_vs_claslib.sh /tmp/bench_2019_gap1v2.pos
```

## 完了条件

- [ ] Step 1 の調査結果を報告（phase_bias_m が MW に既に含まれているか否か）
- [ ] Step 2 の判断を明示
- [ ] Step 3 の実装（必要な場合のみ）
- [ ] ビルドが通る
- [ ] ベンチマーク結果が基準線 (RMS 3D = 5.68m) 以下、または同等
- [ ] **RMS 3D が 5.68m より悪化していたら、変更をリバートして「なぜ悪化したか」を報告すること**
