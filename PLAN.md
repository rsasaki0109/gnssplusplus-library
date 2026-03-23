# libgnss++ 開発計画

## プロジェクト概要

RTKLIB 相当の GNSS 測位ライブラリを Modern C++17 + Eigen でゼロから構築。
RTK コア（Kalman filter, LAMBDA, 対流圏/電離圏モデル, 座標変換）は **全て自前実装** で RTKLIB ランタイム依存ゼロを達成。

## 現在の到達点 (2026-03-24)

### 性能

| データセット | 基線長 | Fix率 | RMS (水平) | RTKLIB Fix率 |
|-------------|--------|-------|-----------|-------------|
| **Kinematic** (DR, 1.2km) | 1.2 km | **100%** | **12 mm** | 98.3% |
| **Short static** (筑波, 36m) | 36 m | **99%** | 90 mm | 100% |
| **Long static** (日本, 3.3km) | 3.3 km | 52% | 104 mm | 60.8%* |

\* RTKLIB long static は高さ方向 2.2m のオフセットあり。libgnss++ の方が概略位置に近い。

### アーキテクチャ

```
libgnss++/
├── core/
│   ├── constants.hpp        物理定数, WGS84, GPS周波数
│   ├── coordinates.hpp      ECEF↔Geodetic↔ENU, geodist (Sagnac込み)
│   ├── types.hpp            SatelliteId, GNSSTime, SignalType
│   ├── observation.hpp      ObservationData
│   ├── navigation.hpp       Ephemeris, NavigationData (broadcast)
│   └── solution.hpp         PositionSolution
├── models/
│   ├── troposphere.hpp      Saastamoinen (1/cos(z) mapping)
│   └── ionosphere.hpp       Klobuchar (broadcast パラメータ対応)
├── algorithms/
│   ├── spp.hpp/cpp          SPP (WLS + iono + trop + TGD)
│   ├── rtk.hpp/cpp          RTK (DD + KF + AR + holdamb + WL-NL)
│   ├── kalman.hpp           Eigen Kalman Filter (active state skip)
│   └── lambda.hpp/cpp       LAMBDA (LD分解 + reduction + 整数探索)
├── io/
│   ├── rinex.hpp/cpp        RINEX 2/3 reader (obs + nav + iono params)
│   ├── rtcm.hpp             RTCM (stub)
│   └── solution_writer.hpp  POS/LLH/XYZ 出力
└── gnss.hpp                 メインヘッダー
```

### RTKLIB 依存状況

| コンポーネント | 状態 | 実装 |
|--------------|------|------|
| Kalman filter | ✅ 自前 | kalman.hpp (Eigen, active state skip) |
| LAMBDA AR | ✅ 自前 | lambda.cpp (LD + reduction + mlambda search) |
| 対流圏モデル | ✅ 自前 | troposphere.hpp (Saastamoinen) |
| 電離圏モデル | ✅ 自前 | ionosphere.hpp (Klobuchar broadcast) |
| 座標変換 | ✅ 自前 | coordinates.hpp (ecef2geodetic, geodist) |
| SPP | ✅ 自前 | spp.cpp (WLS + iono + trop + TGD) |
| RINEX parser | ✅ 自前 | rinex.cpp (v2 + v3, obs + nav) |

**外部依存**: Eigen3 のみ

### テスト

- **単体テスト**: `./build/tests/run_tests` (24/27 pass)
- **回帰テスト**: `bash tests/run_regression.sh` (3データセット, 4 checks, ALL PASS)
- **CI**: GitHub Actions (Ubuntu gcc/clang + macOS clang, green)

### 解析ツール

| ツール | 機能 |
|--------|------|
| `tools/rtk_stats.py` | テキスト統計 (Fix率, RMS, TTFF, 連続Fix) |
| `tools/plot_rtk.py` | matplotlib 可視化 (ENU, scatter, status) |
| `tools/compare_rtklib.py` | RTKLIB 比較 (精度, Fix率, epoch差分) |
| `tools/plot_trajectory.py` | 2D軌跡比較 (Fix/Float/SPP 色分け) |

---

## 完了した作業 (session 1-11)

### Session 1-3: RTK 基盤構築
- SD parameterization KF 実装
- DD observation model (H matrix)
- RTKLIB filter() wrapper
- RTKLIB lambda() wrapper
- bias 初期化 (cp - pr/λ)
- offset correction

### Session 4-6: RTKLIB 同等精度の達成
- **根本原因特定**: base satellite position を rover PR で計算していた → base PR で計算に修正
- **Sagnac correction**: rotation matrix → analytical geodist に変更
- **Troposphere**: RTKLIB NMF wrapper → Saastamoinen 自前実装
- **SD state 差を 0.55 cycle → 0.023 cycle に改善**
- **ratio 1.1 → 24.9** (RTKLIB 同等)
- **epoch 1 instant fix 達成**

### Session 7: Fix率改善 + 解析ツール
- High-variance DD pair exclusion (median-relative threshold)
- Position-based hold validation
- Last fixed position による position reset
- Position mode (static/kinematic) 切替
- 解析ツール 3 本作成

### Session 8: SPP 改善 + RINEX 3 対応
- **SPP Sagnac 符号修正** (60m → 1.9m)
- Geodetic latitude (iterative, RTKLIB 一致)
- TGD correction
- Broadcast Klobuchar iono パラメータ parsing (RINEX 2/3)
- ENU azimuth (proper ENU frame)
- Weighted LS in SPP
- **RINEX 3 parser** (nav: G/R/E/C system chars, obs: > epoch marker)
- **短基線 36m IGS データ取得** (筑波 TSK2/TSKB)

### Session 9: RTKLIB pntpos 統合 + holdamb 改善
- RTKLIB pntpos C wrapper
- holdamb direct state constraint (PSD-preserving)
- tryHoldFix (LAMBDA 失敗時の integer 保持)
- Position hold fallback
- LAMBDA regularization (eigenvalue clamping)

### Session 10: WL-NL AR + RTKLIB 依存除去
- **WL-NL ambiguity resolution** (Melbourne-Wubbena + IF combination)
- MW cross-validation (LAMBDA integer 検証)
- **RTKLIB 置換**: filter → Eigen kalmanFilter, lambda.c → lambda.cpp, trop → troposphere.hpp, ecef2pos → coordinates.hpp
- **SPP 完全自前化** (rtklib_pntpos 除去)
- **RTKLIB ランタイム依存ゼロ達成**

### Session 11: コード品質 + ドキュメント
- debug 出力一掃
- 回帰テスト (tests/run_regression.sh)
- API リファクタリング (constants.hpp, coordinates.hpp, models/, io/solution_writer.hpp)
- CI 修正 (Eigen3::Eigen target, macOS 対応)
- README.md 更新 (RTKLIB 比較テーブル + 画像)
- CLAUDE.md 作成
- **東京お台場 車載走行データ取得** (UrbanNav, 6.5km 走行, RINEX 3)

---

## 今後の計画

### Phase 1: 移動体データ検証 (優先度: 高)

**目標**: 東京お台場 車載走行データでの RTK fix + 軌跡画像を README に掲載

- [ ] お台場データ (RINEX 3, multi-GNSS) でのテスト
  - Rover: Trimble NETR9, 10Hz, GPS+GLO+GAL+BDS+QZSS
  - Base: Trimble NETR9 (近傍 ~170m)
  - RTKLIB: 7% fix (urban canyon で困難)
- [ ] fix 軌跡 vs RTKLIB 軌跡の 2D 比較画像生成
- [ ] ground truth (NovAtel SPAN-CPT) との精度比較
- [ ] README に画像追加

### Phase 2: multi-GNSS 対応 (優先度: 高)

**目標**: GPS 以外の衛星系を使えるようにする

- [ ] GLONASS 対応
  - FDMA frequency handling (衛星毎に異なる周波数)
  - inter-system bias estimation
- [ ] Galileo 対応
  - E1/E5a dual frequency
  - Galileo ephemeris parsing
- [ ] BeiDou 対応 (B1/B3)
- [ ] multi-GNSS DD (system 毎の ref satellite)

### Phase 3: 精度・堅牢性改善 (優先度: 中)

- [ ] NMF (Niell Mapping Function) 自前実装 (現在は 1/cos(z))
- [ ] Saastamoinen wet delay 改善 (GPT モデル)
- [ ] cycle slip detection 改善 (geometry-free, MW)
- [ ] 電離圏推定 (IONOOPT_EST) — DD iono state を KF に追加
- [ ] 精密暦 (SP3) 対応
- [ ] アンテナ位相中心補正 (ANTEX)
- [ ] 潮汐補正

### Phase 4: リアルタイム対応 (優先度: 中)

- [ ] NTRIP client (RTCM 受信)
- [ ] RTCM 3 decoder (MSM messages)
- [ ] リアルタイム KF update
- [ ] serial port receiver interface (u-blox UBX)

### Phase 5: PPP (優先度: 低)

- [ ] PPP-static
- [ ] PPP-kinematic
- [ ] PPP-AR (ambiguity resolution)
- [ ] CLAS/MADOCA 対応

### Phase 6: パッケージング (優先度: 低)

- [ ] CMake install target
- [ ] pkg-config support
- [ ] Python bindings (pybind11)
- [ ] ROS2 node
- [ ] Debian package
- [ ] API ドキュメント (Doxygen)

---

## 技術メモ

### RTK 処理フロー

```
processRTKEpoch()
├── resetPositionToSPP()      位置リセット (kinematic: SPP, static: process noise)
├── collectSatelliteData()    衛星データ収集 (rover/base 別衛星位置)
├── selectReferenceSatellite() 最高仰角衛星
├── updateBias()              SD bias 初期化/更新 (cp - pr/λ + offset)
├── updateFilter() × N        DD KF 更新 (geodist + trop)
├── incrementLockCounts()     lock count (KF 後)
├── resolveAmbiguities()
│   ├── DD pairs 構築 (L1 + L2, PRN 順 ref)
│   ├── D matrix → DD transform
│   ├── Qb variance outlier exclusion
│   ├── LAMBDA → ratio test
│   ├── WL-NL fallback (long baseline)
│   ├── Position-based hold validation
│   └── Partial AR (worst satellite removal)
├── validateFixedSolution()
├── applyHoldAmbiguity()      direct state constraint
└── generateSolution()
```

### 既知の制限

1. **GPS only** — multi-GNSS 未対応
2. **broadcast ephemeris only** — 精密暦未対応
3. **NMF 未実装** — 1/cos(z) mapping (短基線では影響微小)
4. **urban canyon** — 高層ビル環境では multipath で Fix 困難
5. **単体テスト不足** — RTK 系のテストは regression test のみ
6. **SPP 精度** — 自前実装 ~1.3m (RTKLIB ~0.5m) — 長基線で影響

### データセット

| ID | 場所 | 基線 | タイプ | RINEX | 衛星 | ソース |
|----|------|------|--------|-------|------|--------|
| static | 日本 | 3.3km | 静的 | 2.10 | GPS | RTKLIB sample |
| kinematic | DR | 1.2km | 静的* | 3.04 | GPS+R+E+C | geofis/ppk |
| short_baseline | 筑波 | 36m | 静的 | 3.02 | GPS+R+E+J | IGS (TSK2/TSKB) |
| driving | お台場 | 170m | **車載走行** | 3.02 | GPS+R+E+C+J | UrbanNav |

\* kinematic データは rover が実質静止

---

*最終更新: 2026-03-24*
