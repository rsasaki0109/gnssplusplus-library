# RTK Dual-Frequency 実装に関する知見

## 問題の背景

gnssplusplus-library の RTK 実装で、RTKLIB の精度（水平RMS 0.027m, 垂直RMS 0.039m, fix rate >95%）に対して大きく劣っている状態だった。

| Metric | 現状 | RTKLIB目標 |
|--------|------|-----------|
| 水平 RMS (ENU) | 0.361m | 0.027m |
| 垂直 RMS (ENU) | 0.406m | 0.039m |
| Fix rate | 37.5% | >95% |

## 根本原因の特定

### 1. DD電離層残差がambiguityを非整数にする

- RTKLIB真値位置でのDD残差テスト: L1位相のfrac ~0.36 cycles（コード残差 < 1.4m は正常）
- KF真値位置初期化テスト: 毎エポック真値位置を強制しても frac は改善しない → 位置誤差ではなく測定モデルの問題
- 低仰角衛星ほど大きい（G03, 仰角9.7°: frac=-0.36）→ 電離層パス長と一致
- **DD電離層残差 ~0.069m（L1で~0.36 cycles）** がN1推定値にバイアスとして吸収され、整数に収束しない

### 2. LAMBDAメソッドの逆変換バグ（修正済み）

- Z変換の逆変換で `Z.solve(fixed_z)` → `Z.transpose().solve(fixed_z)` に修正
- 順変換が `z = Z^T * a` なので、逆変換は `a = (Z^T)^{-1} * z`
- Fix rate: 24% → 38%、水平RMS: 0.517m → 0.382m に改善

## RTKLIBのアプローチ分析（RTKLIB rtkpos.c 調査結果）

### RTKLIBの3つのモード

| モード | 状態ベクトル | 整数AR | 用途 |
|--------|-------------|--------|------|
| IONOOPT_OFF（短基線デフォルト） | pos + N1 + N2 | Yes | < 10km |
| IONOOPT_EST（中基線） | pos + N1 + N2 + I | Yes | 10-30km |
| IONOOPT_IFLC（IF結合） | pos + N_IF | No | PPP / 長基線 |

### 短基線モードの特徴（IONOOPT_OFF）

- **電離層を推定しない**。DD形成で大部分が消え、残差は十分小さい
- N1, N2を**独立な整数ambiguity**として推定
- LAMBDAで**N1とN2を同時に**整数解決
- SD（single-difference）ambiguityで内部管理、DDに変換してLAMBDAへ

### なぜ電離層推定が短基線で逆効果になるか

1. **L1位相からN1とIの分離が困難**: `L1_DD = geom + λ1·N1 - I` でN1とIが強相関
2. **コード測定のノイズが大きい**: Code DD = geom + I でIを推定するが精度~1m
3. **過剰パラメータ**: iono状態の追加でKFの収束が遅くなる
4. 実験でも確認: iono推定追加で改善は限定的（水平0.382→0.361m, fix rate変化なし）

### 正しいアプローチ: 独立N2推定

**現在のアプローチ（問題あり）:**
- 状態: `[baseline(3), N1_per_sat, I_per_sat]`
- L2制約: `N2 = N1 - N_WL`（Melbourne-Wübbenaで固定）
- 問題: N1-I相関、WL制約がLAMBDAのN2情報を制限

**RTKLIBアプローチ（目標）:**
- 状態: `[baseline(3), N1_per_sat, N2_per_sat]`
- L1行: `L1_DD = geom + λ1·N1`（ionoなし）
- L2行: `L2_DD = geom + λ2·N2`（独立、N1と無関係）
- Code行: `P_DD = geom`（ionoなし）
- LAMBDAでN1とN2を同時に整数解決

**メリット:**
- 高速収束（相関の強い余分な状態がない）
- LAMBDAの性能向上（より多くのDDambiguityで非相関化）
- L2が幾何学的制約とambiguity観測量の両方に寄与

## Melbourne-Wubbena (MW) の役割

独立N2推定でも、MWは以下の用途で有用:
- **サイクルスリップ検出**
- **float ambiguity品質評価**
- **部分AR戦略**: WLを先にfix → NLをfix（段階的解決）
- KFのハード制約としてではなく、**品質チェック・初期化補助**として使用

## 定数・パラメータ

- λ_L1 = c / f_L1 = 0.190294 m
- λ_L2 = c / f_L2 = 0.244210 m
- γ = (f1/f2)² ≈ 1.647
- RINEX 2.10 観測タイプ: L1, C1, L2, P2
- 搬送波位相はcycles単位で格納（メートルではない）
- データ: Trimble 5700, 30s間隔, 2005/04/02, 8-9 GPS衛星/エポック

## 実験結果まとめ

| アプローチ | 水平RMS | 垂直RMS | Fix rate | 備考 |
|-----------|---------|---------|----------|------|
| LAMBDA修正前（基準） | 0.517m | - | 24% | Z^{-1}ではなく(Z^T)^{-1}を使うべき |
| LAMBDA修正後 | 0.382m | 0.507m | 38% | 現在のベースライン |
| +iono推定（process_noise=0.01） | 0.361m | 0.406m | 37.5% | 微改善だが根本解決にならず |
| +iono推定（process_noise=1e-3） | 0.361m | 0.405m | 37.5% | ↑とほぼ同じ |
| +iono推定（process_noise=1e-5） | 0.380m | 0.517m | 13.3% | 小さすぎて効かない |
| 独立N2（N1+N2のLAMBDA同時fix） | 0.652m | 2.18m | 0% | 探索空間14次元は大きすぎ |
| 独立N2（N1のみLAMBDA） | 0.650m | 2.18m | 19% | N2がKFを不安定化 |
| N2なし、L2行なし | 0.583m | 1.79m | 28% | L2の制約がないと悪化 |
| WL制約L2行復帰（現状態） | 0.380m | 0.503m | 35% | ≈ベースラインに戻った |
| **RTKLIB目標** | **0.027m** | **0.039m** | **>95%** | |

### 得られた教訓
1. **iono推定は短基線で逆効果** — N1-I相関でKF収束が遅くなる
2. **独立N2をLAMBDAに入れるのは非効率** — 14次元探索はratioが出にくい
3. **L2のWL制約はN1収束に有効** — L2を除外するとfix rateが低下
4. **max_frac ~0.4-0.5が根本問題** — float ambiguityが整数に収束しない
5. **RTKLIBとの差は測定モデルだけでなく、KFの初期化・収束戦略にもある可能性**

## テストデータ

- `data/rover.obs`, `data/base.obs`: RINEX 2.10
- `output/rtklib_rtk_result.pos`: RTKLIB参照結果（115エポック, 全Q=1 fixed, ratio > 25）
- テスト実行は**プロジェクトルート**から（`./build/tests/run_tests`）、buildディレクトリからだとパスが合わない
