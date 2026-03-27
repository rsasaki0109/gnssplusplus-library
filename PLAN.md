# libgnss++ 開発計画

## プロジェクト概要

RTKLIB 相当の GNSS 測位ライブラリを Modern C++17 + Eigen でゼロから構築。
RTK コア（Kalman filter, LAMBDA, 対流圏/電離圏モデル, 座標変換）は **全て自前実装** で RTKLIB ランタイム依存ゼロを達成。

## 現在の到達点 (2026-03-26)

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
│   ├── rinex.hpp/cpp        RINEX 2/3 reader/writer (obs + nav + iono params)
│   ├── rtcm.hpp/cpp         RTCM 3 reader/writer (1003/1004, 1019/1020, MSM4/5/6/7)
│   ├── ubx.hpp/cpp          UBX NAV-PVT / RXM-RAWX / RXM-SFRBX + streaming decoder
│   ├── ntrip.hpp/cpp        NTRIP client
│   ├── rtcm_stream.hpp/cpp  RTCM stream context (week/station metadata)
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

- **GTest / CTest**: `ctest --test-dir build --output-on-failure` (green)
- **CLI 回帰**: `tests/test_cli_tools.py` (`stream / convert / replay / live / rcv / solve`)
- **Benchmark / 描画回帰**: `tests/test_benchmark_scripts.py`
- **live I/O 回帰**: `RTCM/NTRIP/serial/TCP`, `UBX file/serial`, `receiver reload`
- **CI**: GitHub Actions (Ubuntu gcc/clang + macOS clang, green)

### ツールチェイン到達点

- `gnss solve`: RINEX rover/base/nav からのオフライン RTK 後処理
- `gnss spp`: RINEX rover/nav からのオフライン SPP 後処理
- `gnss stream`: RTCM を `file / NTRIP / serial / TCP` で受信し、`file / serial / TCP` へ relay
- `gnss nav-products`: observation epoch + broadcast nav から minimal `SP3/CLK` を生成
- `gnss ubx-info`: `NAV-PVT / RXM-RAWX / RXM-SFRBX` の inspect と streaming decode
- `gnss nmea-info`: `NMEA GGA/RMC` の file/serial inspect
- `gnss novatel-info`: `NovAtel ASCII/Binary BESTPOS/BESTVEL` の file/serial inspect
- `gnss sbp-info`: Swift Binary Protocol `GPS_TIME / POS_LLH / VEL_NED` の file/serial inspect
- `gnss sbf-info`: Septentrio SBF `PVTGeodetic / LBandTrackerStatus / P2PPStatus` の file/serial inspect
- `gnss trimble-info`: Trimble GSOF `Type 1/2/8` GENOUT packet の file/serial inspect
- `gnss skytraq-info`: SkyTraq binary `0xDC/0xDD/0xE5/0x83/0x84` の file/serial inspect
- `gnss binex-info`: BINEX `0x00/0x01/0x7F` metadata/navigation/prototyping record の file/serial inspect
- `gnss qzss-l6-info`: direct QZSS L6 250-byte frame の header/data-part inspect、subframe CSV export、subtype `10` service-info packet assembly/export、Compact SSR subtype `1/2/3/4/5/6/7/8/9/11/12` message/correction export
- `gnss convert`: `UBX / RTCM -> RINEX obs/nav`, `SFRBX -> CSV`
- `gnss replay`: file replay による live-like RTK 実行
- `gnss live`: rover `RTCM / UBX` + base `RTCM` の live solver
- `gnss rcv`: config ベースの `run/start/restart/reload/status/stop/console`
- `UBX -> nav RINEX`: `GPS / QZSS / Galileo / GLONASS / BeiDou`

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

### Session 12: テスト基盤強化
- **CTest 有効化** (`enable_testing()` + `ctest --test-dir build`)
- **SPP 単体テスト刷新** (bundled RINEX 実データベースに変更)
- **RTK public API smoke test 追加** (kinematic dataset, valid/fixed solution 確認)
- **RTCM テスト整理** (invalid CRC rejection + message family classification)
- **RTCM utility 修正** (`isObservationMessage()` が station message を誤分類していた問題を修正)

---

## 今後の計画

### Phase 1: 移動体データ検証 (優先度: 高)

**目標**: Odaiba moving benchmark を README と回帰で維持する

- [x] お台場データ (RINEX 3, multi-GNSS) での solver/benchmark 導線
- [x] RTKLIB 比較画像, scorecard, 単独 2D 図の生成
- [x] ground truth (UrbanNav reference.csv) ベースの比較
- [x] common-epoch / all-epoch の pass 条件を benchmark command に固定
  - `gnss odaiba-benchmark --require-*` で sign-off threshold を明示できる
- [x] multi-GNSS RTK の Odaiba full-dataset sign-off 条件を固定
  - `output/odaiba_summary.json` に all/common epoch 指標を記録
- [x] optional MALIB comparison path を benchmark summary に追加
  - `--malib-bin` / `--malib-pos` 指定時は `output/odaiba_summary.json` に MALIB 指標も記録

### Phase 2: multi-GNSS 対応 (優先度: 高)

**目標**: GPS-only 前提を外し、system-aware RTK を既定経路にする

- [x] GLONASS 対応
  - FDMA frequency handling
  - inter-system bias / GLONASS AR mode
- [x] Galileo 対応
  - broadcast nav / SPP / RTK ingest path
- [x] BeiDou 対応
  - BDS nav, SPP path, RTK ingest path
- [x] multi-GNSS DD (system 毎の ref satellite, ambiguity slot 分離)
- [x] short_baseline mixed-GNSS sign-off command
  - `gnss short-baseline-signoff` で static sign-off summary と threshold check を出力
- [x] Odaiba 以外の mixed-GNSS dataset で通し sign-off
  - `gnss rtk-kinematic-signoff` で bundled mixed-GNSS DR sample の sign-off summary と threshold check を出力

### Phase 3: 精度・堅牢性改善 (優先度: 中)

- [x] NMF (Niell Mapping Function) 自前実装
- [x] Saastamoinen wet delay 改善 (GPT モデル)
  - [x] latitude/season/height zenith wet climatology seed
  - [x] `estimate_troposphere=false` PPP observation model に climatology-based modeled delay を適用
  - [x] full GPT-style hydrostatic/wet slant delay model
- [x] cycle slip detection 改善 (geometry-free, MW)
- [x] 電離圏推定 (IONOOPT_EST) — DD iono state を KF に追加
  - [x] `RTKConfig::IonoOpt::EST` を state vector / DD observation model / `gnss solve --iono est` に追加
  - [x] `EST` mode では float KF path を優先し、bundled kinematic/CLI 回帰で valid solution を固定
- [x] 精密暦 (SP3) 対応
- [x] アンテナ位相中心補正 (ANTEX)
  - receiver ANTEX (`--antex`) を PPP geometry に適用
- [x] 潮汐補正
  - [x] solid Earth tide の近似 geophysical correction を PPP path に適用
  - [x] station coefficient 前提の ocean loading (`--blq`) を PPP path に適用

### Phase 4: リアルタイム対応 (優先度: 中)

- [x] NTRIP client (RTCM 受信)
- [x] RTCM 3 decoder (MSM4/5/6/7, 1003/1004, 1019/1020)
- [x] serial port receiver interface (u-blox UBX rover + RTCM serial ingest)
- [x] raw TCP / serial / file relay sink/source
- [x] リアルタイム KF update の基本導線
  - `gnss live` / `gnss rcv` / `gnss stream` の live path は動作確認済み
  - rover UBX file/serial, base RTCM file/NTRIP/serial を回帰で固定
- [x] receiver control (`run/start/restart/reload/status/stop/console`)
- [x] 長時間運用向けの receiver state / monitoring 拡張
  - JSON status, waitable polling, restart countdown, log tail, summary metrics
- [x] UBX/RTCM 以外の raw decoder coverage 拡張
  - [x] `gnss nmea-info` (`GGA/RMC`)
  - [x] `gnss novatel-info` (ASCII/Binary `BESTPOS/BESTVEL`)
  - [x] `gnss sbp-info` (Swift SBP `GPS_TIME/POS_LLH/VEL_NED`)
  - [x] `gnss sbf-info` (Septentrio SBF `PVTGeodetic/LBandTrackerStatus/P2PPStatus`)
  - [x] `gnss trimble-info` (Trimble GSOF `Type 1/2/8`)
  - [x] `gnss qzss-l6-info` (direct QZSS L6 frame/subframe + Compact SSR export)
  - [x] NMEA GGA/RMC inspect path (`gnss nmea-info`)
  - [x] first proprietary raw family (`gnss novatel-info`, ASCII/Binary)
  - [x] additional proprietary raw family (`gnss sbp-info`, Swift Binary Protocol)
  - [x] additional proprietary raw family (`gnss sbf-info`, Septentrio SBF PVT/L-band/P2PP)
  - [x] additional proprietary raw family (`gnss trimble-info`, Trimble GSOF Type 1/2/8)

### Phase 5: PPP (優先度: 低)

- 参考: [JAXA-SNU/MALIB](https://github.com/JAXA-SNU/MALIB)
  - RTKLIB fork ベースの MADOCA-PPP / PPP-AR / L6E 対応実装。Phase 5 の外部リファレンスとして扱う。
- 参考: [rtklibexplorer/RTKLIB](https://github.com/rtklibexplorer/RTKLIB)
  - low-cost GNSS receiver 向けに最適化された RTKLIB fork。PPP/RTK の tuning、運用導線、demo5 系の実装比較リファレンスとして扱う。
- 参考: [commaai/laika](https://github.com/commaai/laika)
  - precise product downloader/cache、補正モデルの切り分け、raw measurement abstraction の設計リファレンスとして扱う。
- 参考: [Swift Navigation libsbp navigation docs](https://swift-nav.github.io/libsbp/c/build/docs/html/group__navigation.html)
  - `GPS_TIME / POS_LLH / VEL_NED` の official message layout。`gnss sbp-info` 実装時の protocol reference。
- 参考: [Septentrio AsteRx SB3 Pro+ Reference Guide](https://www.septentrio.com/system/files/support/asterx_sb3_pro_firmware_v4.10.1_reference_guide.pdf)
  - `PVTGeodetic / LBandTrackerStatus / P2PPStatus` の official SBF block layout。`gnss sbf-info` 実装時の protocol reference。
- 参考: [Trimble R780 GSOF Messages Overview](https://help.fieldsystems.trimble.com/r780/gsof-messages-overview.htm)
  - `Type 1/2/8` の official GSOF record index。`gnss trimble-info` 実装時の protocol reference。
- 参考: [QZSS L6 Signal Interface Specification IS-QZSS-L6-004](https://qzss.go.jp/en/technical/download/pdf/ps-is-qzss/is-qzss-l6-004.pdf)
  - direct QZSS L6 250-byte frame の official header/data-part structure。`gnss qzss-l6-info` 実装時の protocol reference。
- 参考: [QZSS-Strategy-Office/claslib](https://github.com/QZSS-Strategy-Office/claslib)
  - official CLAS reference implementation。`data/clas_grid.def` はこの `clas_grid.def` を mirror したもので、direct QZSS L6 subtype `12` の nearest-grid atmospheric application に使う。

## 今後の外部実装取り込み方針

- `develop` へ機能の直 push は増やさない
- 外部実装の取り込みは `1機能 = 1PR` で分割する
- 各 PR で最低 1 本の regression / sign-off / dogfooding check を増やす

予定している分割例:

- `laika-fetch-products`
- `laika-ppp-pipeline`
- `rtklibexplorer-low-cost-tuning`
- `rtklibexplorer-stream-ux`
- `ppc-signoff-expansion`
- [x] precise products loader / interpolation (`SP3` + `CLK`)
- [x] PPP float core groundwork
  - precise orbit/clock fallback
  - ionosphere-free dual-frequency observation path
  - pseudorange PPP Kalman update
  - synthetic regression tests in `tests/test_ppp.cpp`
- [x] public PPP batch CLI groundwork (`gnss ppp`)
- [x] broadcast-nav to precise-product bridge (`gnss nav-products`)
  - real observation epochs + broadcast nav から minimal `SP3/CLK` を生成
- [x] PPP-static sign-off
  - bundled `data/rover_static.obs` + `data/navigation_static.nav`
  - `gnss ppp-static-signoff` emits summary JSON and threshold checks
- [x] PPP mode split (`static` / `kinematic`)
- [x] PPP-kinematic sign-off
  - bundled `data/rover_kinematic.obs` + `data/base_kinematic.obs` + `data/navigation_kinematic.nav`
  - `gnss ppp-kinematic-signoff` emits summary JSON and threshold checks against an RTK reference trajectory
- [x] PPP-AR (ambiguity resolution)
  - [x] synthetic precise-products fixed path + regression
  - [x] real-data precise-path regression (`gnss nav-products` + `gnss ppp --enable-ar`, static sample, `fallback=0`)
  - [x] real-data PPP-AR slice regression (`20 epoch` static sample slice, generated `SP3/CLK`, `PPP fixed solutions > 0`, `fallback=0`)
  - [x] real-data PPP-AR full-run regression (`gnss ppp-static-signoff --enable-ar --generate-products --ar-ratio-threshold 1.5`, bundled `120 epoch` static sample, `PPP fixed epochs >= 1`, `fallback=0`)
  - [x] real-data PPP-AR sign-off (`gnss ppp-static-signoff --enable-ar --generate-products --ar-ratio-threshold 1.5`, bundled `120 epoch` static sample, `mean <= 5.0 m`, `max <= 6.0 m`, `PPP fixed epochs >= 1`, `fallback=0`)
- [x] CLAS/MADOCA 対応
  - [x] minimal SSR orbit/clock correction manager (`SSRProducts`, CSV loader, `gnss ppp --ssr`)
  - [x] minimal RTCM SSR 1/2/4 decoder (`RTCMProcessor::decodeSSRCorrections()` for GPS/GLONASS/Galileo/QZSS/BeiDou)
  - [x] RTCM SSR file -> sampled PPP correction application (`gnss ppp --ssr-rtcm`)
  - [x] RTCM SSR code-bias/URA/high-rate clock decoder
  - [x] RTCM SSR sampled high-rate clock application (`gnss ppp --ssr-rtcm`)
  - [x] RTCM SSR URA application in PPP weighting
  - [x] RTCM SSR code-bias application in PPP pseudorange model
  - [x] RTCM SSR transport path into PPP (`--ssr-rtcm` via file / NTRIP / TCP / serial)
  - [x] CLAS/MADOCA transport and application path (`gnss clas-ppp --profile clas|madoca` over RTCM-carried SSR)
  - [x] compact sampled CLAS/MADOCA transport path (`gnss clas-ppp --compact-ssr`)
  - [x] direct QZSS L6 frame/subframe inspect/export (`gnss qzss-l6-info`)
  - [x] raw QZSS L6 subtype `1/2/3/4/5/6/7/8/9/11/12` -> sampled Compact SSR correction path (`gnss qzss-l6-info`, `gnss clas-ppp --qzss-l6`)
  - [x] raw QZSS L6 subtype `10` service information packet assembly/export (`gnss qzss-l6-info`)
  - [x] sampled SSR atmospheric metadata application in PPP (official CLAS grid based nearest-grid trop/STEC path)

### Phase 6: パッケージング (優先度: 低)

- [x] CMake install target
- [x] pkg-config support
- [x] Python bindings (pybind11)
  - `RINEX header` / observation epoch summary / `.pos solution` inspection
  - file-based `SPP / PPP / RTK` solve helpers
- [x] ROS2 node
  - optional `gnss_solution_node` playback publisher for `.pos -> NavSatFix/PoseStamped/Path/status/satellite-count`
- [x] Debian package
- [x] API ドキュメント (Doxygen)

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

1. **PPP/CLAS scope is still bounded** — current sampled CLAS/MADOCA path covers RTCM SSR plus direct raw QZSS L6 subtype `1/2/3/4/5/6/7/8/9/10/11/12`, but it still stops at the sampled correction application model rather than a full external CLAS reference-stack reimplementation
2. **raw decoder coverage is broad but curated** — `UBX/RTCM/NMEA/NovAtel ASCII/Binary/Swift SBP/Septentrio SBF/Trimble GSOF/SkyTraq/BINEX/QZSS L6` は対応済みだが、RTKLIB `convbin` の全 vendor matrix を網羅する方針ではない
3. **Python bindings are file-oriented** — current `libgnsspp` package now exposes inspection plus file-based `SPP/PPP/RTK` solve helpers, but live stream / receiver-control APIs areまだ expose していない
4. **ROS2 integration is playback-oriented** — current node publishes `.pos` playback as `NavSatFix/PoseStamped/Path/status/satellite-count`; it is intentionally lighter than a full in-node live receiver stack

### データセット

| ID | 場所 | 基線 | タイプ | RINEX | 衛星 | ソース |
|----|------|------|--------|-------|------|--------|
| static | 日本 | 3.3km | 静的 | 2.10 | GPS | RTKLIB sample |
| kinematic | DR | 1.2km | 静的* | 3.04 | GPS+R+E+C | geofis/ppk |
| short_baseline | 筑波 | 36m | 静的 | 3.02 | GPS+R+E+J | IGS (TSK2/TSKB) |
| driving | お台場 | 170m | **車載走行** | 3.02 | GPS+R+E+C+J | UrbanNav |

\* kinematic データは rover が実質静止

---

*最終更新: 2026-03-27*
