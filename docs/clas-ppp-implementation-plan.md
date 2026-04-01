# CLAS-PPP cm級精度達成 実装計画

最終更新: 2026-04-01 (Session 14終了時)
引き継ぎ先: Codex CLI / Claude Code

---

## 1. 目標

QZSS CLAS補正を使ったPPP-RTKで**cm級Fix精度**を達成する。

| 方式 | 現状精度 | 目標 |
|------|---------|------|
| libgnss++ IFLC PPP+DD (SSR) | 3.9m median | — |
| libgnss++ per-freq L1 PPP (SSR, iono推定) | 3.9m median | < 0.06m |
| CLASLIB Float | 4.9m | — |
| CLASLIB Fix | 0.06m | これを超える |

基準点: ECEF (-3957240.1233, 3310370.8778, 3737527.7041)

## 2. データ

| ファイル | パス |
|---------|------|
| 観測 | `/workspace/ai_coding_ws/gnssplusplus_thesis_ws/data/clas/claslib/data/0627239Q.obs` |
| 航法 | `.../sept_2019239.nav` |
| CLAS L6 | `.../2019239Q.l6` |
| SSR CSV | `/tmp/clas_expanded4.csv` (L6デコーダで生成、cbias/pbias/atmos含む) |
| CLASLIBソース | `.../claslib/src/` (ppp.c, cssr2osr.c, ppp_ar.c等) |
| cssrlib参考 | `https://github.com/hirokawa/cssrlib` (Python CSSR+PPP-RTK、将来取り込み候補) |

2019-08-27 16:00-17:00 UTC, GPS+Galileo+QZSS, GPS week 2068, TOW ~230420

## 3. 現在のアーキテクチャ

### ファイル構成

```
src/algorithms/
  ppp.cpp              (~3800行) メインPPPプロセッサ
  ppp_atmosphere.cpp   (~430行) STEC/trop計算
  ppp_osr.cpp          (~310行) OSR補正パイプライン（processEpochCLAS用、現在未使用）
include/libgnss++/algorithms/
  ppp.hpp              PPPProcessor, PPPConfig, PPPState, ARMethod
  ppp_osr.hpp          OSRCorrection構造体
  ppp_atmosphere.hpp   大気補正関数
src/core/
  navigation.cpp       SSRProducts (loadCSVFile, interpolateCorrection, addCorrection)
apps/
  gnss_ppp.cpp         CLI (--no-ionosphere-free, --estimate-ionosphere等)
  gnss_clas_ppp.py     CLAS-PPP自動化スクリプト
  gnss_qzss_l6_info.py L6デコーダ (CSSR subtype 1-12)
scripts/
  test_clas_ppp.sh     ビルド→実行→精度計算の一発スクリプト
```

### 処理フロー (現在)

```
processEpoch()
  ├─ [use_ionosphere_free=true] → formIonosphereFree(IFLC) → updateFilter → DD変換 → WLNL AR
  │  結果: 3.9m, DD変換でSSR残差(~40m)を相殺
  │
  └─ [use_ionosphere_free=false] → formIonosphereFree(L1のみ) → updateFilter → DD変換なし → DD_IFLC AR
     結果: 3.9m float, AR 2/500エポック(fractional partが大きい)
     ※ processEpochCLAS()は無効化済み (line ~1037)
```

### テスト実行

```bash
# ビルド + per-freq CLAS-PPP テスト
bash scripts/test_clas_ppp.sh 200

# 標準IFLC PPP テスト（比較用）
./build_local/apps/gnss_ppp \
  --obs .../0627239Q.obs --nav .../sept_2019239.nav \
  --ssr /tmp/clas_expanded4.csv --out /tmp/ppp_iflc.pos \
  --static --estimate-troposphere --max-epochs 200

# デバッグ出力
GNSS_PPP_DEBUG=1 bash scripts/test_clas_ppp.sh 3 2>&1 | head -50

# SSR CSVが無い場合の再生成
rm /tmp/clas_expanded4.csv && bash scripts/test_clas_ppp.sh 1
```

## 4. 発見した事実（全セッションの累積）

### 4.1 SSR残差が衛星間で20-40mばらつく（最重要）

```
G14: -1.5m, G25: -40.2m, G26: +24.7m, G29: +5.7m
G31: +6.8m, G32: -27.0m, E07: -32.3m, E27: +7.6m
```

**原因**: PPPの`calculateSatelliteState(sat, obs.time)`が受信時刻で衛星位置を計算。衛星は~0.067秒前の放射時刻位置を使うべき。衛星軌道速度~3.9km/s × 0.067s ≈ 260mの位置誤差がLOS投影で衛星ごとに20-40mの残差に。

**SPPは正しく補正済み** (spp.cpp L258-277): `emission_time = time - pseudorange/c`で反復計算。PPPには未実装。

**影響**: 受信機クロック推定がcommon-modeを吸収し、Kalmanフィルタが時間平均するため3.9mに収束するが、**これが精度の天井を決めている**。

### 4.2 アンカー制約が精度の天井を作る

`constrainStaticAnchorPosition()` (ppp.cpp L4096):
```cpp
filter_position = 0.7 * SPP_position + 0.3 * filter_position  // SSR, not converged
```

SPPが~5m誤差 → フィルタは最大でも~3m以上SPPから離れられない → 4m精度の天井。

**実験結果**:

| アンカー設定 | 精度 |
|------------|------|
| ハードブレンド70% (現在) | 3.9m |
| ソフトアンカー sigma=3m | 4.3m |
| ソフトアンカー sigma=5m | 4.8m |
| ソフトアンカー sigma=10m | 6.2m |
| アンカーなし | 10.6m (per-freq) / 40m (IFLC) |

**結論**: アンカーなしでは発散する。SSR残差(20-40m)を先に改善しないとアンカーは緩められない。**Signal travel time補正が最優先**。

### 4.3 AR未発火の原因（修正済み、ただしfractional partが問題）

**原因1** (修正済み): 位相測定のlock_count閾値がSSRモードで高すぎた
- `ppp.cpp L3805`: `precise_products_loaded_`がSSRでもtrueになり、lock_count >= 20を要求
- **修正**: SSRモードでは`phase_measurement_min_lock_count`(=1)を使用

**原因2** (修正済み): GF/MWサイクルスリップ検出がSSR残差で誤爆
- `ppp.cpp L2241`: `use_combination_slip_detection`がSSRでtrue
- **修正**: per-freqモード(`!use_ionosphere_free`)ではGF/MW検出を無効化

**原因3** (未解決): AR候補の`convergence_min_epochs`閾値
- `ppp.cpp L2600`: SSRモードでは`min(convergence_min_epochs, 10)`に緩和済み

**現在のAR状況**: 500エポック中2回Fix、ratio=3.3。しかしDD fractional partsが0.1-0.5サイクル（整数に非収束）。Float位置の4m誤差がアンビギュイティ推定を汚染。

### 4.4 CLASLIBはDDを使わない

CLASLIB (`ppp.c`)の分析:
- **Undifferenced PPP** (DDなし)
- 観測方程式: `v = meas - (r - c*dts + dtrp) - rcv_clk - N*λ`
- 軌道補正: RAC→ECEF変換後に**減算** `rs += -(er*dr + ea*da + ec*dc)`
- クロック: 加算 `dts += dclk/c`

### 4.5 STEC品質

- STEC多項式評価: 正しい（c00+c01*dlat+c10*dlon+c11*dlat*dlon+残差）
- 符号: 正しい（+iono for code, -iono for phase）
- 0.5m固定sigma: 保守的。`atmos_stec_quality`フィールドは**完全に未使用**
- STEC品質範囲: 0-51、ほとんど17-27。品質0は無データ

### 4.6 CSVの軌道補正はRAC座標

- `orbit_correction_ecef`フィールドの名前が不正確（実際はRAC）
- 標準PPP: RAC値をそのままECEF加算（~2m誤差、軽微）
- OSR: RAC→ECEF変換あり（正しい）
- cross-track符号が`ssrRacToEcef`と`computeOSR`で異なる可能性あり（~0.1-0.5m）

### 4.7 Light Travel Time補正（実装済み、効果限定的）

ppp.cpp L1979付近に追加:
```cpp
const double travel_time = observation.pseudorange_if / constants::SPEED_OF_LIGHT;
const GNSSTime emission_time = time - travel_time + sat_clock_bias;
nav.calculateSatelliteState(sat, emission_time, ...)
```

**問題**: 残差は依然として20-40m。理由を調査中。

## 5. 現在の問題の根本原因図

```
SSR残差 20-40m (per-satellite)
    ↓
受信機クロックがcommon-mode吸収 → Float ~4m
    ↓
アンカー制約 (SPP 70%) → Float 4mでロック
    ↓
アンビギュイティ推定 biased → DD fractional 0.1-0.5サイクル
    ↓
AR ratio低い → Fix 2/500エポック
    ↓
Fixed位置 ≈ Float位置 (4m)
```

**ボトルネック**: SSR残差。Signal travel time補正は実装したが効果不十分。

## 6. 次のステップ（優先順、Codex引き継ぎ用）

### Step 1: Signal Travel Time補正の検証と改善 ★最優先

Light travel time補正を実装済みだが効果が限定的。原因を調査。

**タスク**:
1. デバッグ出力で補正前後の残差を比較:
```bash
GNSS_PPP_DEBUG=1 bash scripts/test_clas_ppp.sh 1 2>&1 | grep "PPP-OBS"
```
→ 残差が減っていなければ実装にバグ

2. SPPの信号伝搬時間補正と比較（spp.cpp L258-277参照）:
   - PPPでは`observation.pseudorange_if`がIFLC化/L1のみ → 値が正しいか？
   - Sagnac補正は含まれているか？（`geodist()`はSagnac込みだが、`calculateSatelliteState`後に再計算必要）

3. `calculateSatelliteState`が返す衛星位置と、SPPが計算する衛星位置を直接比較

**ファイル**: `ppp.cpp` L1962-1989, `spp.cpp` L258-277

### Step 2: SSR軌道補正のRAC→ECEF変換を正しくする

標準PPPパスではRAC値がECEFとして直接加算されている（ppp.cpp L1997）。
`computeOSR`と同じRAC→ECEF変換を適用すべき。

**タスク**:
1. `ppp.cpp` L1997付近の`sat_position += orbit_correction_ecef`を修正
2. `ssrRacToEcef()`関数を使用（ppp.cpp L907定義済み）
3. ただし`ssrRacToEcef`は衛星位置/速度が必要 → calculateSatelliteState後に変換

**期待改善**: ~2m（RAC方向の差分）

### Step 3: アンカー制約の段階的緩和

SSR残差が改善された後:
1. `constrainStaticAnchorPosition()`のSSRブレンド値を調整
   - 現在: `converged_ ? 0.5 : 0.7`
   - 提案: `converged_ ? 0.3 : 0.5` (carrier phaseの重みを増やす)
2. または `static_anchor_position_`をフィルタ推定値で徐々に更新:
   ```cpp
   static_anchor_position_ = 0.99 * static_anchor_position_ + 0.01 * filter_position;
   ```

### Step 4: AR品質の改善

Float精度がsub-meterになった後:
1. DD fractional partsが< 0.1サイクルになるか確認
2. LAMBDA ratioが3.0以上で安定するか確認
3. Fixed位置のcm級精度確認

### Step 5: STEC品質の活用

1. `atmos_stec_quality`フィールドをパース
2. 品質連動のsigmaスケーリング: `sigma = 0.5 / (quality / 25.0)`
3. 品質0の衛星はiono推定をスキップ

### Step 6: 論文更新

cm級Fixが達成できたら `paper_en.tex` を更新。

## 7. CLASLIBリファレンス

| CLASLIB関数 | ファイル | libgnss++対応 | 備考 |
|------------|---------|--------------|------|
| `satpos_ssr()` | ephemeris.c L880 | calculateSatelliteState + SSR | 軌道: `rs += -(er*dr+ea*da+ec*dc)`, クロック: `dts += dclk/c` |
| `cssr2osr()` | cssr2osr.c | computeOSR() | PRC/CPC計算 |
| `res_ppp()` | ppp.c L947 | updateFilter() | `v = meas - (r - c*dts + dtrp) - clk - N*λ` |
| `ifmeas()` | ppp.c L537 | formIonosphereFree() | IFLC組み合わせ |
| `ppp_ar()` | ppp_ar.c | resolveAmbiguities() | WL(FCB) + NL(ROUND/ILS) AR |

## 8. コード変更履歴（Session 13-14）

### ppp.cpp の変更（現在有効なもの）

1. **processEpochCLAS無効化** (L1037): コメントアウト → 標準updateFilter使用
2. **formIonosphereFree per-freq修正** (L1805): `!use_ionosphere_free`時はIFLC形成スキップ、L1のみ
3. **DD変換条件追加** (L1638): `use_ionosphere_free`の場合のみDD適用
4. **WLNL AR条件追加** (L2583): WLNL ARは`use_ionosphere_free`時のみ
5. **位相lock_count閾値修正** (L3805): SSRモードでは1エポック
6. **GF/MWスリップ検出修正** (L2241): per-freqモードでは無効化
7. **Light travel time補正** (L1979): emission_time = time - pseudorange/c + sat_clk
8. **AR候補閾値緩和** (L2635): SSRモードでは10エポック
9. **アンカー制約コメント** (L4101): SSR実験結果メモ
10. **ARデバッグ出力** (L2599): total_amb/ready/min_epochs表示

### ppp_osr.cpp の変更
1. **デバッグ出力拡大** (L130): 全衛星SSR値 + sat_clk_total_s

### processEpochCLAS内の修正（無効コード内、参考）
- tropをPRC/CPCから分離してH行列で推定
- L2アンビギュイティ事前割り当て
- elevation-dependent weighting
- IFLC vs per-freq残差比較デバッグ

## 9. 重要な定数・パラメータ

| パラメータ | 値 | 場所 |
|-----------|-----|------|
| GPS L1波長 | 0.190294m | constants.hpp |
| GPS L2波長 | 0.244210m | constants.hpp |
| WL波長 | 0.862m | c/(f1-f2) |
| NL波長 | 0.107m | c/(f1+f2) |
| convergence_min_epochs | 20 | ppp.hpp:93 |
| phase_measurement_min_lock_count | 1 | ppp.hpp:77 |
| ar_ratio_threshold | 2.0 | CLI引数 |
| STEC制約sigma | 0.5m | ppp.cpp:2090 |
| アンカーブレンド(SSR) | 0.7(未収束)/0.5(収束) | ppp.cpp:4109-4111 |
| SSR CSV GPS week | 2068 | — |
| 観測開始TOW | ~230420 | 16:00:20 UTC |

## 10. 既知のバグ・注意点

1. **`orbit_correction_ecef`は実際にはRAC** - navigation.cppのloadCSVFile。名前と内容が不一致。標準PPPパスではRAC→ECEF変換なしで直接加算（~2m誤差）。
2. **複数clock値のlast-wins** - 同一TOWに複数clock行。最後のnon-zero値が使われる。
3. **L6デコーダのsync=0 break** - `gnss_qzss_l6_info.py` L1554、大気メッセージ用に除去済み。
4. **ppp.cppが3800行** - リファクタリング計画 `docs/clas-ppp-refactoring-plan.md`。
5. **processEpochCLAS** - 無効化済み。独自フィルタは24-60mで実用不可。
6. **cross-track符号** - `ssrRacToEcef`と`computeOSR`で異なる可能性（~0.1-0.5m）。
7. **Light travel time** - 実装済みだが残差改善が限定的。検証必要。

## 11. 参考リンク

- IS-QZSS-L6-004: https://qzss.go.jp/en/technical/download/pdf/ps-is-qzss/is-qzss-l6-004.pdf
- CLASLIB: https://github.com/QZSS-Strategy-Office/claslib
- cssrlib: https://github.com/hirokawa/cssrlib (Python CSSR+PPP-RTK)
- RTKLIB: https://github.com/rtklibexplorer/RTKLIB (demo5)

---

*このドキュメントはCodex CLI / Claude Codeセッション引き継ぎ用。Step 1のlight travel time検証から開始すること。*
