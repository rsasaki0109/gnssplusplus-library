# CLAS-PPP リファクタリング計画

## 背景

ppp.cppが3000行超に膨張し、Claude Codeのコンテキストウィンドウを圧迫。
毎セッションで同じファイルを何度もRead/Grepし、デバッグのtrial & errorで
コンテキストが枯渇する。

## 対策1: ソースのモジュール分割

### 現在の構造
```
src/algorithms/
  ppp.cpp            (~3500行) ← 全部入り
  ppp_atmosphere.cpp  (~430行) ← 大気補正計算
  ppp_osr.cpp        (~270行) ← OSR補正（新規追加済み）
```

### 分割後の構造
```
src/algorithms/
  ppp_core.cpp          — processEpoch, initialize, stats (~300行)
  ppp_clas.cpp          — processEpochCLAS (OSRモード) (~200行)
  ppp_filter.cpp        — updateFilter, predictState, constrainAnchor (~400行)
  ppp_measurement.cpp   — formIonosphereFree, formMeasurementEquations (~500行)
  ppp_ar.cpp            — resolveAmbiguities, resolveAmbiguitiesWLNL, solveFixedPosition (~500行)
  ppp_slip.cpp          — detectCycleSlips, updateAmbiguityStates (~300行)
  ppp_utils.cpp         — signalFrequencyHz, rtcmSsrSignalId, geodist等 (~200行)
  ppp_corrections.cpp   — applyPreciseCorrections (~300行)
  ppp_atmosphere.cpp    — STEC/trop計算 (既存、変更なし)
  ppp_osr.cpp           — OSR補正パイプライン (既存、変更なし)
```

### ヘッダの分割
```
include/libgnss++/algorithms/
  ppp.hpp              — PPPProcessor class定義、PPPConfig、PPPState
  ppp_types.hpp        — IonosphereFreeObs, OSRCorrection等の構造体
  ppp_ar.hpp           — AR関連の型と宣言
  ppp_osr.hpp          — OSR関連 (既存)
  ppp_atmosphere.hpp   — 大気補正 (既存)
```

### 分割の優先順位
1. **ppp_clas.cpp** — processEpochCLASを独立ファイルに (最優先、現在開発中)
2. **ppp_ar.cpp** — AR関連を分離 (DD-AR, WL+NL, solveFixedPosition)
3. **ppp_corrections.cpp** — applyPreciseCorrectionsを分離
4. **ppp_utils.cpp** — ユーティリティ関数を分離
5. 残りは安定後に段階的に分割

## 対策2: 実装ドキュメントの強化

### セッション開始時に読むファイル
```
docs/clas-ppp-refactoring-plan.md   ← このファイル（全体計画）
CLAS_PPP_AR_IMPLEMENTATION.md       ← 実装進捗と次のステップ
docs/osr-design.md                  ← OSR設計仕様
```

### 各モジュールのインタフェース仕様

#### ppp_clas.cpp
```
入力: ObservationData, NavigationData, SSRProducts, atmos_tokens
出力: PositionSolution

処理フロー:
1. SPP seed取得
2. フィルタ初期化 (初回のみ)
3. 時間予測 (プロセスノイズ追加、クロックリセット)
4. OSR補正計算 (computeOSR)
5. 観測方程式構築 (P_corr, L_corr per frequency)
6. Kalman更新
7. Solution生成

フィルタ状態: pos(3) + vel(3) + clk(1) + glo_isb(1) + trop(1) + {amb per sat-freq}
anchor blend: なし（OSR補正で安定）
```

#### ppp_osr.cpp (既存)
```
入力: ObservationData, NavigationData, SSRProducts, atmos_tokens, receiver_pos/clk/trop
出力: vector<OSRCorrection>

各衛星の補正:
  PRC[f] = trop + relativity + antenna + iono_scaled + cbias
  CPC[f] = trop + relativity + antenna - iono_scaled + pbias + windup

SSR/STECなしの衛星は除外 (CLASLIBのcorrmeas return 0相当)
```

#### ppp_ar.cpp
```
WL+NL AR:
1. MW平均からWL整数固定 (86cm波長、簡単)
2. 観測量ベースでNL DD float計算
3. LAMBDA でNL整数固定
4. holdamb方式でフィルタに制約注入

solveFixedPosition:
  固定NLアンビギュイティでWLS位置計算
```

### 「次にやること」テンプレート
各セッションの末尾に以下を `CLAS_PPP_AR_IMPLEMENTATION.md` に記録:
```markdown
## 次セッション開始時の手順
1. このファイルの「最新セッション」セクションを読む
2. 指定されたソースファイル（1-2個のみ）の指定行を読む
3. 記述された修正を実行
4. テストスクリプトで検証
```

## 対策3: テストスクリプト

### scripts/test_clas_ppp.sh
```bash
#!/bin/bash
set -e
GNSSPP=
THESIS=
CLAS=$THESIS/data/clas/claslib/data

# 1. ビルド
cd $GNSSPP
rm -f build_local/CMakeFiles/gnss_lib.dir/src/algorithms/ppp*.o build_local/libgnss_lib.a build_local/apps/gnss_ppp
cmake --build build_local --target gnss_ppp -j$(nproc) 2>&1 | tail -3

# 2. CSVが存在しなければ再生成
if [ ! -f /tmp/clas_expanded4.csv ]; then
    python3 -c "
import sys; sys.path.insert(0, 'apps')
from gnss_qzss_l6_info import decode_source, decode_cssr_messages, write_compact_corrections
from pathlib import Path
from apps.gnss_clas_ppp import expand_compact_ssr_text
frames, subframes, _ = decode_source('$CLAS/2019239Q.l6')
messages, corrections, _ = decode_cssr_messages(subframes, gps_week=2068)
compact = Path('/tmp/clas_compact.csv')
expanded = Path('/tmp/clas_expanded4.csv')
write_compact_corrections(compact, corrections)
expand_compact_ssr_text(compact.read_text(encoding='ascii'), expanded)
"
fi

# 3. CLAS-PPP実行
./build_local/apps/gnss_ppp \
  --obs $CLAS/0627239Q.obs \
  --nav $CLAS/sept_2019239.nav \
  --ssr /tmp/clas_expanded4.csv \
  --out /tmp/ppp_test.pos \
  --static --estimate-troposphere --no-ionosphere-free --estimate-ionosphere \
  --enable-ar --ar-ratio-threshold 2.0 \
  --max-epochs ${1:-3600}

# 4. 精度計算
python3 -c "
import math, numpy as np
ref_x, ref_y, ref_z = -3957240.1233, 3310370.8778, 3737527.7041
xs, ys, zs, sts = [], [], [], []
with open('/tmp/ppp_test.pos') as f:
    for line in f:
        if line.startswith('%'): continue
        parts = line.split()
        if len(parts) >= 9:
            xs.append(float(parts[2])); ys.append(float(parts[3])); zs.append(float(parts[4]))
            sts.append(int(parts[8]))
errors = [math.sqrt((xs[i]-ref_x)**2+(ys[i]-ref_y)**2+(zs[i]-ref_z)**2) for i in range(len(xs))]
fixed = [errors[i] for i in range(len(sts)) if sts[i]==6]
flt = [errors[i] for i in range(len(sts)) if sts[i]==5]
print(f'Epochs: {len(xs)}, Fixed: {len(fixed)}, Float: {len(flt)}')
if errors: print(f'All: median={np.median(errors):.4f}m, last50={np.median(errors[-50:]):.4f}m, min={np.min(errors):.4f}m')
if fixed: print(f'Fixed: median={np.median(fixed):.4f}m, min={np.min(fixed):.4f}m')
"

# 5. CLASLIB Float比較
echo "--- CLASLIB Float reference: median=4.9m ---"
```

### scripts/test_clas_ppp_debug.sh
```bash
#!/bin/bash
# 短時間デバッグ用（10エポック）
GNSS_PPP_DEBUG=1 $(dirname $0)/test_clas_ppp.sh 10 2>&1 | grep -E "OSR|CLAS-PPP|error"
```

## 実装順序

### Phase 1: モジュール分割 + テストスクリプト
1. `scripts/test_clas_ppp.sh` 作成
2. `ppp_clas.cpp` を分離（processEpochCLAS）
3. `ppp_ar.cpp` を分離
4. ビルド確認・テスト実行

### Phase 2: OSRモードのデバッグ
1. SSR orbit/clock補正値のデバッグ（code_rms=21mの原因特定）
2. 補正値の検証（CLASLIBと数値比較）
3. code_rms < 1m を達成

### Phase 3: AR + cm級精度
1. 個別周波数DD-ARの実装
2. cm級Float精度の達成
3. AR Fix後のcm級精度確認

### Phase 4: 論文更新
1. cm級結果を論文に反映
2. CLASLIBとの比較表更新
3. レビュー・投稿

## Claude Codeセッションのルール

### やること
- セッション開始時に `CLAS_PPP_AR_IMPLEMENTATION.md` と `clas-ppp-refactoring-plan.md` を読む
- 修正対象のファイルのみ読む（1-2ファイル、必要な行だけ）
- テストスクリプトで検証
- セッション終了時に進捗を `CLAS_PPP_AR_IMPLEMENTATION.md` に記録

### やらないこと
- ppp.cppの全体を読まない
- 同じファイルを何度もGrepしない
- デバッグのために大量のcerr出力を入れて何度もビルドしない → 1回のデバッグで最大限の情報を取る
- 関連しないファイルを読まない
