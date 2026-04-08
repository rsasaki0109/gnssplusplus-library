# Cursor CLI タスク: Track A — ベンチ固定 + SSR2OBS 実運用確認

**作業ディレクトリ:** `/workspace/ai_coding_ws/rtklib_v2_ws/gnssplusplus-library`

## 背景

PLAN.md の Track A（ネイティブ C++ 境界 + 公平ベンチ）の「次の一手」を実行する。
目的は以下の 2 つ:

1. **ベンチスナップショット 1 セット固定** — `--claslib-parity` プリセットで gnss_ppp を走らせ、JSON 結果を保存し再現手順を記録する
2. **SSR2OBS の 1 エポック中間値ダンプ** — `buildSsrCorrectedObservations` の入出力を確認できるログ/小ツールを追加する

## 前提

- `build/apps/gnss_ppp` はビルド済み（動く）
- データ: `/workspace/ai_coding_ws/gnssplusplus_thesis_ws/data/clas/claslib/data/`
  - 2018 ケース: `0161329A.obs`, `tskc2018329.nav`, `2018328X_329A.l6`
  - 2019 ケース: `0627239Q.obs` (or `.bnx`), `sept_2019239.nav`, `2019239Q.l6`
- CLASLIB の `.pos` 出力があれば比較に使えるが、なくても libgnss 単体のベンチは可能

## Task 1: ベンチスナップショット固定

### 1.1 gnss_ppp を CLASLIB パリティモードで実行

まず、各ケースの OBS/NAV/SSR ファイルの組み合わせを確認する。
gnss_ppp は `--ssr` に CSV を取るが、L6 バイナリを直接食えるかも `--help` や `gnss_ppp.cpp` で確認すること。
L6 直接入力は最近追加された可能性がある（git log 参照: "Add C++ L6 decoder"）。

実行例（パスは確認の上で修正）:

```bash
cd /workspace/ai_coding_ws/rtklib_v2_ws/gnssplusplus-library

# 2018 case
./scripts/run_gnss_ppp_claslib_parity.sh \
  --obs /workspace/ai_coding_ws/gnssplusplus_thesis_ws/data/clas/claslib/data/0161329A.obs \
  --nav /workspace/ai_coding_ws/gnssplusplus_thesis_ws/data/clas/claslib/data/tskc2018329.nav \
  --ssr <適切なSSR CSV or L6ファイル> \
  --out /tmp/bench_2018_parity.pos \
  --summary-json /tmp/bench_2018_parity_summary.json
```

L6 入力の場合、`gnss_ppp.cpp` を読んで `--l6` や `--ssr-l6` 等のフラグがないか確認。
なければ、先に Python ツール (`apps/gnss_qzss_l6_info.py` 等) で L6 → CSV 変換が必要かもしれない。

### 1.2 benchmark スクリプトで評価

```bash
# REF_XYZ は基準座標（CLASLIB 設定や RINEX ヘッダから取得）
export REF_X=... REF_Y=... REF_Z=...
export JSON_OUT_DIR=/tmp/bench_track_a

./scripts/benchmark_fair_vs_claslib.sh /tmp/bench_2018_parity.pos
```

基準座標は RINEX obs ファイルのヘッダ `APPROX POSITION XYZ` から取得可能。

### 1.3 結果を記録

`docs/benchmarks/` ディレクトリを作り、以下を記録:
- 実行コマンド（コピペ可能な形で）
- gnssplusplus-library の git commit hash
- JSON 出力のコピー or パス
- 基準座標の出典

**秘密の座標・生ログはコミットしない**（PLAN.md のガードレール）。
代わりにコマンドとパスのテンプレートを記録する。

## Task 2: SSR2OBS 1 エポックダンプ

### 2.1 目的

`prepareClasEpochContext` → `buildSsrCorrectedObservationsFromClasContext` のパイプラインで、
1 エポック分の:
- 入力: raw observations（衛星ID, signal, pseudorange, carrier_phase）
- 中間: OSRCorrection の各項（orbit, clock, iono, tropo, phase_bias 等）
- 出力: corrected observations

をダンプして、補正が正しく適用されているか目視確認できるようにする。

### 2.2 実装方針

**選択肢 A（推奨）: gnss_ppp にデバッグフラグ追加**

`gnss_ppp.cpp` に `--dump-ssr2obs-epoch <epoch_number>` フラグを追加。
指定エポックで `buildSsrCorrectedObservations` の前後を stderr or 別ファイルにダンプ。

出力例:
```
=== SSR2OBS Epoch 50 ===
SAT  SIG     RAW_PR(m)        RAW_CP(cyc)      OSR_PR_CORR(m)   OSR_CP_CORR(m)   ADJ_PR(m)        ADJ_CP(cyc)
G01  L1CA    22345678.123     117456789.456     -12.345          -12.678          22345665.778     117456723.123
...
```

**選択肢 B: スタンドアロン小ツール**

`tools/dump_ssr2obs_epoch.cpp` を新規作成。
OBS/NAV/SSR を読んで N エポック目の SSR2OBS 入出力を表示するだけのツール。

### 2.3 確認事項

- `OSRCorrection` 構造体のフィールド一覧は `ppp_osr_types.hpp` を参照
- `selectAppliedOsrCorrections` の返り値が実際に何を返すか `ppp_clas.cpp` で確認
- ダンプ内容は human-readable テキストで十分（JSON 不要）

## 参考ファイル

主要コードの場所（PLAN.md §2.2）:
- `include/libgnss++/algorithms/ssr2obs.hpp` — SSR2OBS API
- `src/algorithms/ssr2obs.cpp` — 実装
- `include/libgnss++/algorithms/ppp_osr.hpp` — OSRCorrection 型, computeOSR
- `include/libgnss++/algorithms/ssr2osr.hpp` — ssr2osr namespace エイリアス
- `apps/gnss_ppp.cpp` — CLI アプリ（--claslib-parity 含む）
- `scripts/run_gnss_ppp_claslib_parity.sh` — ラッパースクリプト
- `scripts/benchmark_fair_vs_claslib.sh` — ベンチマーク比較
- `tools/compare_ppp_solutions.py` — .pos 比較ツール
- `docs/references/claslib-gap.md` — CLASLIB 対照表

## 完了条件

- [ ] 最低 1 ケース（2018 or 2019）で gnss_ppp --claslib-parity が走り、.pos と summary JSON が得られる
- [ ] benchmark スクリプトで REF に対する RMS が計算できる
- [ ] 再現コマンドが docs/ に記録されている
- [ ] SSR2OBS の 1 エポックダンプが動く（選択肢 A or B）
- [ ] ビルドが通る (`cmake --build build -j$(nproc)`)
- [ ] 既存テストが壊れない (`ctest --test-dir build --output-on-failure`)
