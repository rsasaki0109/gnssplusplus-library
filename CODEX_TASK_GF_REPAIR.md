# Codex タスク: GF差分ベース phase bias repair の実装

## 背景

SSR エポック切替時に phase_bias_m が数m ジャンプし、SD位相残差が 2.8〜9.1m になっている。
現在の phase repair（ppp_osr.cpp:912-918）は 95-105 cycles の閾値で100 cycles単位のジャンプしか検出しない。
実際のジャンプは 15-35 cycles なので全く検出できていない。

前回「全差分を offset_cycles に蓄積」したら 164m に大暴走した。
原因: `continuity_term` には幾何変化（位置変化による衛星-受信機距離変化）が含まれるため、
全差分を補償すると幾何変化まで打ち消してしまう。

## CLASLIB の compensatedisp() のロジック (cssr2osr.c:488-593)

CLASLIB は **Geometry-Free (GF) 差分**を使う。GF は L1 - L2 の差なので幾何（距離）が消える。

### opt->posopt[5]==1 の場合（SSR ベース、行520-561）

SSR エポックが変わったとき:
```c
// 前エポックと現エポックの dispersion（電離層+バイアス）差を計算
dispm = -FREQ2/FREQ1 * (1-fi²) * ionom + bm[L1] - bm[L2];  // 前エポック
disp0 = -FREQ2/FREQ1 * (1-fi²) * iono0 + b0[L1] - b0[L2];  // 現エポック
coef = (disp0 - dispm) / dt;  // 変化率 (m/s)
```

エポック内の補償:
```c
dt = timediff(time, t0);  // SSRエポック開始からの経過時間
compL[0] = 1/(1-fi²) * coef * dt;       // L1 補償量
compL[1] = fi²/(1-fi²) * coef * dt;     // L2 補償量
```

ここで:
- `fi = λ2/λ1` (= f1/f2 の逆数の波長比)
- `b0[i]`, `bm[i]` = phase_bias_m[i] の現エポック/前エポック値
- `iono0`, `ionom` = 電離層補正の現エポック/前エポック値
- `coef` にはサニティチェックがある: `|coef / (FREQ2/FREQ1*(1-fi²))| > 0.008` なら skip

### opt->posopt[5]==0 の場合（観測ベース、行580-591）

```c
dgf = L1*λ1 - L2*λ2 - (b0[L1] - b0[L2]);  // GF差分
compL[0] = 1/(1-fi²) * dgf;
compL[1] = fi²/(1-fi²) * dgf;
```

## libgnss++ での実装方針

libgnss++ の CLAS 経路は SSR ベースなので、opt->posopt[5]==1 のロジックを移植する。

### 修正場所: `src/algorithms/ppp_osr.cpp`

1. phase_bias_m と iono_l1_m の前エポック値を `CLASPhaseBiasRepairInfo` に保存
2. SSR エポック切替時（`phase_bias_epoch_changed == true`）に dispersion 変化率 `coef` を計算
3. 各観測エポックで `compL = coef * dt` を計算して CPC に適用
4. 既存の 95-105 cycles 閾値ロジックは削除

### 具体的な変更

#### CLASPhaseBiasRepairInfo に追加するフィールド

`include/libgnss++/algorithms/ppp_osr.hpp` の `CLASPhaseBiasRepairInfo` 構造体に以下を追加:
```cpp
std::array<double, 3> prev_phase_bias_m = {0.0, 0.0, 0.0};  // 前SSRエポックのphase_bias
double prev_iono_l1_m = 0.0;  // 前SSRエポックの電離層L1
GNSSTime prev_ssr_epoch_time = {};  // 前SSRエポックの時刻
bool has_prev_ssr_epoch = false;
std::array<double, 3> dispersion_rate_m_per_s = {0.0, 0.0, 0.0};  // compL の変化率
```

#### ppp_osr.cpp の修正

`phase_bias_epoch_changed` のブロック（行903-918）を以下に置換:

```cpp
if (phase_bias_epoch_changed &&
    phase_bias_repair_info.has_prev_ssr_epoch &&
    osr.num_frequencies >= 2 &&
    usesClasPhaseBiasRepair(phase_continuity_policy)) {
    
    const double f1 = osr.frequencies[0];
    const double f2 = osr.frequencies[1];
    if (f1 > 0.0 && f2 > 0.0) {
        const double fi = f1 / f2;  // λ2/λ1 in freq domain = f1/f2
        const double fi2 = fi * fi;
        // Current dispersion: iono + bias
        const double disp_curr = -(f2/f1) * (1.0 - fi2) * osr.iono_l1_m
            + osr.phase_bias_m[0] - osr.phase_bias_m[1];
        const double disp_prev = -(f2/f1) * (1.0 - fi2) * phase_bias_repair_info.prev_iono_l1_m
            + phase_bias_repair_info.prev_phase_bias_m[0] - phase_bias_repair_info.prev_phase_bias_m[1];
        
        const double dt_ssr = phase_bias_dt;  // 現SSRエポック - 前SSRエポック
        if (std::abs(dt_ssr) > 0.1 && std::abs(dt_ssr) < kPhaseBiasRepairTimeoutSeconds) {
            const double coef = (disp_curr - disp_prev) / dt_ssr;
            // Sanity check: CLASLIB uses |coef / (f2/f1*(1-fi²))| > 0.008
            const double norm = (f2/f1) * (1.0 - fi2);
            if (norm > 0.0 && std::abs(coef / norm) <= 0.008) {
                phase_bias_repair_info.dispersion_rate_m_per_s[0] = coef / (1.0 - fi2);
                phase_bias_repair_info.dispersion_rate_m_per_s[1] = fi2 / (1.0 - fi2) * coef;
            } else {
                phase_bias_repair_info.dispersion_rate_m_per_s[0] = 0.0;
                phase_bias_repair_info.dispersion_rate_m_per_s[1] = 0.0;
            }
        }
    }
}

// Save current epoch values for next SSR epoch comparison
if (phase_bias_epoch_changed) {
    for (int ff = 0; ff < osr.num_frequencies; ++ff) {
        phase_bias_repair_info.prev_phase_bias_m[ff] = osr.phase_bias_m[ff];
    }
    phase_bias_repair_info.prev_iono_l1_m = osr.iono_l1_m;
    phase_bias_repair_info.prev_ssr_epoch_time = effective_phase_bias_reference_time;
    phase_bias_repair_info.has_prev_ssr_epoch = true;
}

// Apply dispersion compensation
if (usesClasPhaseBiasRepair(phase_continuity_policy) &&
    phase_bias_repair_info.has_prev_ssr_epoch) {
    const double dt_from_ssr = obs.time - phase_bias_repair_info.prev_ssr_epoch_time;
    osr.phase_compensation_m[f] = phase_bias_repair_info.dispersion_rate_m_per_s[f] * dt_from_ssr;
}
```

注意: `osr.phase_compensation_m[f]` は CPC 計算時に `phase_compensation_term` として既に使われている (行867-871)。

**既存の 95-105 cycles 閾値ロジック（行912-918）は削除する。**

## 重要な制約

- **変更してよいファイル**: `ppp_osr.cpp`, `ppp_osr.hpp` (CLASPhaseBiasRepairInfo のみ)
- **変更してはいけないファイル**: ppp.cpp, ppp_ar.cpp, ppp_wlnl.cpp, ppp_clas.cpp, ppp_clas_epoch.cpp
- `phase_compensation_m[f]` に値を設定すれば、CPC 計算で自動的に適用される（行867-871の既存パス）
- `offset_cycles` は使わない（前回これで暴走した）。代わりに `phase_compensation_m[f]` を使う

## ビルド・ベンチマーク

```bash
cmake --build build -j$(nproc)

GNSS_PPP=./build/apps/gnss_ppp ./scripts/run_gnss_ppp_claslib_parity.sh \
  --obs /workspace/ai_coding_ws/gnssplusplus_thesis_ws/data/clas/claslib/data/0627239Q.obs \
  --nav /workspace/ai_coding_ws/gnssplusplus_thesis_ws/data/clas/claslib/data/sept_2019239.nav \
  --ssr /workspace/ai_coding_ws/gnssplusplus_thesis_ws/data/clas/claslib/data/2019239Q.l6 \
  --clas-epoch-policy hybrid-standard-ppp \
  --out /tmp/bench_2019_gf_repair.pos \
  --summary-json /tmp/bench_2019_gf_repair_summary.json

export REF_X=-3957240.1233 REF_Y=3310370.8778 REF_Z=3737527.7041
export JSON_OUT_DIR=/tmp/bench_gf_repair
mkdir -p /tmp/bench_gf_repair
./scripts/benchmark_fair_vs_claslib.sh /tmp/bench_2019_gf_repair.pos
```

## 完了条件

- [ ] CLASPhaseBiasRepairInfo にフィールド追加
- [ ] GF差分ベースの dispersion rate 計算を実装
- [ ] 95-105 cycles 閾値ロジックを削除
- [ ] phase_compensation_m[f] に dispersion 補償を設定
- [ ] ビルドが通る
- [ ] **RMS 3D が 6.20m（現在の cbias fix 後の値）以下であること**
- [ ] 悪化したらリバートして原因報告
