# OSR (Observation Space Representation) 再設計

## 目標
CLASLIB同等のcm級CLAS-PPP精度をlibgnss++で達成する。

## CLASLIBの処理フロー
```
L6 decode → CSSR data → corrmeas() → OSR per-satellite
  OSR = {trop, relativity, antenna, iono(STEC), cbias, pbias, windup, comp}
  PRC = trop + relatv + antr + iono_scaled + cbias
  CPC = trop + relatv + antr - iono_scaled + pbias + windup + comp

Corrected obs:
  P_corr = P_obs - PRC  → 幾何距離 + 受信機時計
  L_corr = L_obs - CPC  → 幾何距離 + 受信機時計 + N*λ

PPP-RTK filter:
  State = [pos(3), vel(3), clk(1), glo_isb(1), trop(1), iono(MAXSAT), amb(MAXSAT*NF)]
  H_code = [-los, 1, mapping, +didx, 0]
  H_phase = [-los, 1, mapping, -didx, 1]

  didx = ionosphere mapping factor (frequency-dependent)
  iono state absorbs residual ionosphere after STEC correction
```

## libgnss++ 再設計方針

### 新モジュール: `ppp_osr.hpp/cpp`
- CLASLIBの`cssr2osr.c`に相当する独立モジュール
- 全CLAS補正を観測量に直接適用してOSRを生成
- PPPフィルタからは独立（補正済み観測量を返すだけ）

### OSR構造体
```cpp
struct OSRCorrection {
    SatelliteId satellite;
    double trop_correction_m;      // トロポスフィア補正
    double relativity_correction_m; // 相対論的補正
    double antenna_correction_m;    // 受信機アンテナPCO/PCV
    double iono_l1_m;              // L1電離層遅延 (STEC based)
    double code_bias_m[MAX_FREQ];  // コードバイアス per frequency
    double phase_bias_m[MAX_FREQ]; // フェーズバイアス per frequency
    double windup_cycles;          // Phase wind-up (cycles)
    double PRC[MAX_FREQ];          // 疑似距離補正 (全補正合計)
    double CPC[MAX_FREQ];          // 搬送波位相補正 (全補正合計)
    bool valid;
};
```

### PPPフィルタの変更
1. **anchor blend廃止** → プロセスノイズのみで安定化
2. **個別周波数アンビギュイティ** → L1/L2それぞれを整数として管理
3. **イオノ状態** → OSR補正後の残差を吸収（小さい分散）
4. **DD-AR** → L1/L2の個別DDアンビギュイティを固定

### 実装順序
1. `ppp_osr.hpp/cpp` の作成（OSR補正計算）
2. PPPフィルタの状態ベクトル拡張（L1/L2個別amb）
3. 観測方程式の書き直し（OSR補正済み観測量を使用）
4. DD-ARの書き直し（L1/L2個別整数固定）
5. テスト・検証
