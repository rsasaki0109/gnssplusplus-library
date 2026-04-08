# Codex タスク: CLASLIB パリティ — 5 ギャップ修正

**作業ディレクトリ:** `/workspace/ai_coding_ws/rtklib_v2_ws/gnssplusplus-library`

## 目的

libgnss++ の CLAS PPP-RTK を CLASLIB と同等精度（cm級）にする。
現状: 2019ケースで RMS 3D = 5.68m、fix rate = 2.2%。
目標: RMS 3D < 0.5m、fix rate > 50%。

## CLASLIB ソース（参照のみ、コピペ禁止、ロジックを理解して C++17+Eigen で再実装）

```
/workspace/ai_coding_ws/gnssplusplus_thesis_ws/data/clas/claslib/src/
├── ppprtk.c    (1857行) — PPP-RTK solver本体
├── cssr2osr.c  (979行)  — SSR→OSR変換
├── cssr2osr.h           — OSR型定義
├── ppp_ar.c    (469行)  — アンビギュイティ解決
├── ppp.c       (1167行) — 標準PPP
├── grid.c               — グリッド補間
├── stec.c               — STEC計算
└── rtklib.h             — 共通定義
```

## libgnss++ 対象ファイル

```
src/algorithms/ppp.cpp          (3396行) — PPP solver
src/algorithms/ppp_ar.cpp       (810行)  — AR
src/algorithms/ppp_clas.cpp     (1158行) — CLAS OSR統合
src/algorithms/ppp_clas_epoch.cpp (296行) — CLASエポック管理
src/algorithms/ppp_osr.cpp      (946行)  — OSR補正計算
src/algorithms/ppp_wlnl.cpp     (439行)  — WL/NL AR
include/libgnss++/algorithms/ppp.hpp
include/libgnss++/algorithms/ppp_shared.hpp
include/libgnss++/algorithms/ppp_osr.hpp
include/libgnss++/algorithms/ppp_clas.hpp
```

## ギャップ 1: Phase Bias を FCB 確定値として使う（最優先）

### CLASLIB の方式 (ppp_ar.c)
- `fix_amb_WL()` (行192): `nav->wlbias[sat]` から CLAS の FCB（Fractional Cycle Bias）を取得
- WL搬送波 LC(1,-1,0) を構成し、FCB を**既知の確定値として差し引く**
- 四捨五入で WL 整数値を固定
- `fix_amb_ILS()` (行304): NL は LAMBDA で解く
- `REV_WL_FCB` フラグで符号反転対応

### libgnss++ の現状
- `ppp_wlnl.cpp` で WL/NL AR を実装しているが、CLAS FCB の扱いが不明確
- Phase bias を推定パラメータとして扱っている可能性

### 修正内容
1. `ppp_wlnl.cpp` の WL 固定で、CLAS phase bias (`OSRCorrection::phase_bias_m[]`) を**確定値として観測値から事前に差し引く**
2. WL = LC(1,-1,0) の構成: `L_WL = (f1*L1 - f2*L2) / (f1-f2)` から `wlbias` を引く
3. NL は LAMBDA で解く（既存の `lambdaSearch` を使用）
4. Fix 後に制約カルマンフィルタで位置更新（CLASLIB `fix_sol()` 行263-302 参照）

## ギャップ 2: DD 観測モデルの整合

### CLASLIB の方式 (ppprtk.c)
- `ddres()` (行794-1032): 参照衛星を**システム・周波数毎に最高仰角衛星**として動的選択
- DD残差を直接計算: `v[nv] = (L_i - L_j) - (λ_i*N_i - λ_j*N_j) - (I_i - I_j) - (T_i - T_j)`
- 設計行列 H: 位置成分 = `-(e_i - e_j)` (LOS差分)
- 電離層: `H[II(sat_i)] = f^2 * im_i`, `H[II(sat_j)] = -f^2 * im_j`
- 対流圏: `H[ITT+k] = m_w_i - m_w_j`
- 参照衛星変更時: phase bias リセット (`set_init_pb`, 行941)
- 3段階反復: FST → SND → TRD、各段でchi-square検定

### libgnss++ の現状
- SD状態ベクトル + DD観測行列（間接的なDD）
- 参照衛星の選択ロジックが CLASLIB と異なる可能性

### 修正内容
1. `ppp.cpp` の残差計算を CLASLIB の `ddres()` に合わせる
2. 参照衛星選択: システム・周波数毎に最高仰角衛星を選択
3. 参照衛星変更時の phase bias リセットロジック追加
4. 3段階反復の導入（FST: 初期計算、SND: 収束確認、TRD: 固定検証）
5. Chi-square検定（F分布テーブル使用）での外れ値排除

## ギャップ 3: compensatedisp — Phase Continuity 修復

### CLASLIB の方式 (cssr2osr.c)
- `compensatedisp()` (行488): 幾何フリー(GF)差分から時間変動を補償
- `dgf = (L1/λ1 - L2/λ2) - (prev_L1/λ1 - prev_L2/λ2)` で位相変化量を計算
- 補償値: `compL[0] = dgf/(1-γ)`, `compL[1] = γ/(1-γ)*dgf` (γ = f1²/f2²)
- サイクルスリップ検出: `|dgf| > threshold` で破棄
- 100サイクル単位の `pbias_ofst` 追跡（行325-336）
- CPC差分が `MAXPBCORSSR=20m` 超過で検出（行323）

### libgnss++ の現状
- `CLASPhaseBiasRepairInfo` 構造体で状態保持
- `ClasPhaseContinuityPolicy` でポリシー選択可能
- `updatePhaseBiasRepairState()` (ppp_osr.cpp 行283) で更新

### 修正内容
1. `ppp_osr.cpp` の `compensatedisp` 相当ロジックを CLASLIB に合わせる
2. GF差分ベースの時間変動補償の計算式を正確に移植
3. `compL[0]`, `compL[1]` の計算: `γ = (f1/f2)²` を使用
4. サイクルスリップ検出閾値の整合
5. `pbias_ofst` の100サイクル単位追跡

## ギャップ 4: STEC/Trop グリッド補間

### CLASLIB の方式
- `stec_grid_data()` (cssr2osr.c 行662): 4点グリッド補間 (`MAX_NGRID=4`)
- グリッドインデックス・重み行列 `grid->index`, `grid->Gmat`, `grid->Emat` を使用
- `trop_grid_data()` (行195): ZWD/ZTD を4点補間で取得
- `prectrop()` (行242): Niell写像関数 (`tropmapf`) で天頂→仰角変換
- `grid.c`: 緯度経度ベースの面的補間

### libgnss++ の現状
- `ClasGridReference` クラスで内部実装
- `selectClasEpochAtmosTokens` で大気トークン選択

### 修正内容
1. STEC 補間が CLASLIB の4点補間 (`Gmat`/`Emat` ベース) と同じ計算をしているか検証
2. Niell写像関数の実装確認・整合
3. グリッド外の場合の fallback 処理を CLASLIB に合わせる
4. 大気補正のタイミング（OSR計算時 vs フィルタ更新時）を整合

## ギャップ 5: 3段階DD反復 + Chi-square検定

### CLASLIB の方式 (ppprtk.c)
- `ppp_rtk_pos()` で3回の `ddres()` + `filter2()` を呼ぶ:
  - FST (行1590-1617): 1次反復 — 粗い外れ値除去
  - SND (行1625-1632): 2次反復 — 収束確認
  - TRD (行1746-1757): 3次反復 — 固定検証（AR後）
- 各段で `max_inno[3]`/`max_inno[4]` の閾値で innovation 検定
- F分布テーブル `qf[][]` (行71-100) で有意水準を判定

### 修正内容
1. `ppp.cpp` の `processEpoch` に3段階反復を導入
2. 各段階でイノベーション（残差）の統計検定を追加
3. 閾値超過の観測を除外してフィルタ再更新
4. F分布テーブルによる chi-square 検定

## ビルド・テスト

```bash
# ビルド
cmake --build build -j$(nproc)

# ユニットテスト
ctest --test-dir build --output-on-failure

# 2019ベンチマーク（基準線: RMS 3D = 5.68m）
GNSS_PPP=./build/apps/gnss_ppp ./scripts/run_gnss_ppp_claslib_parity.sh \
  --obs /workspace/ai_coding_ws/gnssplusplus_thesis_ws/data/clas/claslib/data/0627239Q.obs \
  --nav /workspace/ai_coding_ws/gnssplusplus_thesis_ws/data/clas/claslib/data/sept_2019239.nav \
  --ssr /workspace/ai_coding_ws/gnssplusplus_thesis_ws/data/clas/claslib/data/2019239Q.l6 \
  --clas-epoch-policy hybrid-standard-ppp \
  --out /tmp/bench_2019_after.pos \
  --summary-json /tmp/bench_2019_after_summary.json

# ベンチマーク評価
export REF_X=-3957240.1233 REF_Y=3310370.8778 REF_Z=3737527.7041
export JSON_OUT_DIR=/tmp/bench_track_a_after
./scripts/benchmark_fair_vs_claslib.sh /tmp/bench_2019_after.pos
```

## 完了条件

- [ ] ギャップ1: Phase bias を FCB確定値として WL固定に使用
- [ ] ギャップ2: DD観測モデルの参照衛星選択が CLASLIB と同等
- [ ] ギャップ3: compensatedisp の GF差分補償が正確に移植
- [ ] ギャップ4: STEC/Trop グリッド補間が CLASLIB と整合
- [ ] ギャップ5: 3段階DD反復 + chi-square検定が動作
- [ ] ビルドが通る
- [ ] 既存テストが壊れない
- [ ] 2019ベンチマークで RMS 3D < 5.68m（改善方向であること）

## 注意

- CLASLIB のコードを**そのままコピペしない**。ロジックを理解して C++17+Eigen で再実装する
- 既存の libgnss++ のアーキテクチャ（クラス構造、名前空間）を尊重する
- 1ギャップずつ修正→ビルド→テスト→ベンチマークで効果確認するのが理想
- 最も効果が大きいのはギャップ1（Phase Bias/FCB）。ここから始めること
