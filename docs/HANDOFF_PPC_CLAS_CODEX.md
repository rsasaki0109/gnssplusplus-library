# 引き継ぎ資料: PPC-Dataset キネマティック CLAS 評価トラック

全面改訂 2026-07-27(Claude Code → codex 引き継ぎ)。旧版(07-18)は全て陳腐化しており、本書が現行の唯一の状態記述。

---

## 1. ゴールと規律

**PPC-Dataset(東京/名古屋 6走行、都市部キネマティック)で、native CLAS PPP-RTK パイプラインを MRTKLIB v0.4.2 の公表結果と同等以上にする。**

- 公表結果: https://zenn.dev/hatognss/articles/7a54dd82606faf
- **ユーザー最重要指示: ベタ移植** — MRTKLIB v0.5.1 を忠実に移植。推測禁止、per-epoch 計測で設計。闇雲なチューニング禁止(3連敗済み)。
- **受け入れ規律(#333/#334 の教訓)**: ソルバ変更は必ず fix% と rmsFIX と >3m バッドフィックス数の3点で評価。6走行検証なしのデフォルト変更禁止。
- 検証の分業: 実装・実走は subagent、**スコアの独立再計算・md5・重要主張の裏取りはオーケストレータが必ず自分でやる**(agent の数字は再検証するまで信じない)。

### MRTKLIB v0.4.2 公表目標値(fix% / RMS2D)

| case | Fix% | RMS2D |
|---|---|---|
| tokyo_run1 | 4.9 | 0.747 |
| tokyo_run2 | 21.7 | 0.514 |
| tokyo_run3 | 7.4 | 0.801 |
| nagoya_run1 | 17.0 | 1.105 |
| nagoya_run2 | 23.4 | 1.119 |
| nagoya_run3 | 6.3 | 0.318 |

**重要な発見(07-27)**: ピン留めの MRTKLIB **v0.5.1** を実走させると t2 は 1921/9091=21.13% しか出ない(公表 v0.4.2 は 21.7%)。バージョン間スキューでありローカル再現不能。**実走可能な参照に対しては我々が既に全面勝ち**。公表値は「参考目標」として扱う。

---

## 2. 現状 (2026-07-27, develop = 8a44ef2)

### 直近マージ済み PR(順序どおり)

| PR | 内容 |
|---|---|
| #349 (07-24) | maxdiff hold-continuation carve-out(エイジゲート min_track_record=60) |
| #350 (07-27) | **採点方法論修正: レバーアーム二重補正を除去**(下記) + docs |
| #351 (07-27) | **outage_count 偽陽性修正(本命)**: 有効観測でカウンタをクリアする MRTKLIB パリティ。n3 fix 倍増の主因 |
| #352 (07-27) | 6走行ベンチ表・README・validation 文書の更新 |

### 採点方法論(必読)

**reference.csv は最初からアンテナ位置**。旧採点の apply_lever_arm=True は IMU→アンテナ補正の二重適用で、名古屋 ~0.89m/東京 ~0.31m の偽バイアスを載せていた。決定的証拠: 補正なし誤差ベクトルを heading で車体系投影すると平均 (0.075, 0.088)m ≈ 0(レバー (0.593,-0.670) ではない)。両ソルバの co-fixed 誤差相関 r=0.991 = 共通モード基準バイアス。**現行の正準採点は apply_lever_arm=False(raw antenna 基準)**。旧方式は `--apply-lever-arm` でオプトイン再現可能。MRTKLIB 公表値も raw 基準なので直接比較可能。

### 公表中の6走行結果(develop 8a44ef2, no-lever 基準)

| Run | fix% (目標) | rmsFIX (MRTKLIB) | >3m |
|---|---|---|---|
| tokyo_run1 | 10.704 (4.9) ✅ | 0.352 (0.747) ✅ | 0 |
| tokyo_run2 | 21.507 (21.7) ❌僅差 | 0.322 (0.514) ✅ | 0 |
| tokyo_run3 | 37.951 (7.4) ✅ | 0.192 (0.801) ✅ | 0 |
| nagoya_run1 | 36.737 (17.0) ✅ | 0.450 (1.105) ✅ | 0 |
| nagoya_run2 | 23.959 (23.4) ✅ | 0.625 (1.119) ✅ | **19**(既知バースト) |
| nagoya_run3 | 8.776 (6.3) ✅ | 0.304 (0.318) ✅ | 0 |

集計: **24.851% / 0.377m / >3m 19**(n2 の単一4秒バースト tow 556406.4–556410.4、全て3.17–3.20m、#349以前から存在、旧採点ではバイアスが打ち消し方向で2.49mに見えていた)。品質バーは「監査済みベースライン(19)に対し新規>3mゼロ」。

---

## 3. 進行中(未コミットあり・最優先で拾うこと)

### 07-27 21:10 Codex 追記（A4 6走行判定）

A4=5.0 の残り5走行は全て正常完走し、`apply_lever_arm=False` で6走行を
独立採点済み。**A4のデフォルト化は不採用、PRも作成しない**。

- tokyo_run2/run3, nagoya_run1/run3 は正準baselineと `.pos` のmd5まで同一。
- nagoya_run2 は既報どおり、悪性19 fixだけを除去:
  fixed 2249→2230、rmsFIX 0.624999→0.554480m、
  >1m/>3m 19/19→0/0、max 3.200100→0.767281m。
- tokyo_run1 で良性120 fixを誤検疫:
  fixed 1270→1150、fix% 10.703751→9.692373%、
  rmsFIX 0.351834→0.369342m、lost 120/gained 0。
  lost区間 tow 187698.8–187722.6、lost RMS 0.052781m、
  lost max 0.134573m（全て <1m）。
- t1検疫系列は `nfix=13`（後半11）、n2悪性系列は `nfix=7`。
  固定ペア数は次の候補識別子になり得るが、閾値追加は未実装・未検証。

全体完走は19:53。実験出力は SSD
`rtklib_v2_ws_output/clas_a4_sixrun/`、n2だけは既存の
`clas_gate_experiments/nagoya_run2_expA4_publish_quarantine_full.pos`。
コミット/push済み `a3cedd3` はenvデフォルトOFFの実験基盤として保持。
デフォルト値は0のまま。マージ未実施。

次の候補は、A4検疫を少数固定ペア（まず `nfix<=7` 仮説）に限定して
n2/t1の対象窓で反証テストし、通った場合だけ再度6走行検証すること。

### 07-27 17:05 Codex 追記（旧A2計画を更新）

ブランチ `agent/clas-gate-experiments-20260727` は **a3cedd3** まで
コミット・push済み。PRは未作成、マージも未実施。`data` symlinkだけが
untracked。

旧A2の「有効な既存WLNL holdならfloor免除」はper-epoch実走で反証された。
556459.8の良性seedはFLOATCNT直後のlock=1新規holdであり、入口・reset直前の
どちらでも `wlnlHoldStillValid()==false`。旧A/A2はbad 19 fixを消す一方、
内部ARをearly-returnした下流影響で良性322 fixブロックも失う。

採用した実験実装A4は、reset後1秒窓内でmaxdiff-onlyを観測したratio<floor
候補について、AR/holdの内部効果はbaseline同様に維持し、そのhold系列だけ
FIX公開を検疫する。行政的FLOAT公開はFLOATCNTに数えず、次のフルリセットで
検疫解除する。

**A4=5.0 n2全区間（apply_lever_arm=False、独立採点）**

- fixed 2249→2230（lost 19 / gained 0、悪性burstだけ）
- fix% 23.958666→23.756259%
- rmsFIX 0.624999→0.554480m
- >1m / >3m: 19/19→0/0、max 3.200100→0.767281m
- 555856クラスタ92 fix維持、556459–556524良性322 fix完全維持
- 検疫ログ19行。full出力md5 `fabd799a9ea29079177852fff5394687`

envなしn2先頭4025行はbaseline prefixとmd5
`3bd66f2a9cacddebf025513d508241a3`でバイト同一。A4 t2先頭4000 epochも
検疫0、baseline同範囲4003出力行とmd5
`c41fc5a7d642bcf9f4e465f30d45bcaa`でバイト同一。

**B=mintrack10は不採用**。n2先頭4025 epochでfixed 1215→996、
良性322ブロック全滅、lost 322/gained 103、>3m 19→42、
rms 0.669→0.783m。したがってBデフォルトflipとA4+B 6走行は中止。
envノブ自体は実験用・デフォルト60のままコミットに含む。

次の必須作業:

1. A4=5.0単独で残り5走行をフル検証（t2前半はmd5同一済み）。
2. 全6走行でfix% / rmsFIX / >3m / max / lost-gainedを独立再計算。
3. 通ればA4のデフォルト0→5.0を別PRで提案。Bは変更しない。
4. PR作成後もマージはユーザー明示指示待ち。

### ワークツリー gnssplusplus-worktrees/clas-n3-kinchisq、ブランチ agent/clas-gate-experiments-20260727

**7ファイル未コミット**(ppp.hpp, ppp_ar.hpp, ppp_env_overrides.hpp/.cpp, ppp_ar.cpp, ppp_clas_epoch.cpp, ppp_wlnl.cpp)。2つの env ノブ(デフォルトは現行動作とビット同一、n2 bitcheck md5 一致検証済み):

- **ノブA2** `GNSS_PPP_CLAS_POST_RESET_RATIO_FLOOR`(float, 0=OFF): フルリセット(FLOATCNT/MAXDIFFP/HOLDCONT-RESET)後 1.0s 以内の AR accept に ratio フロアを課す。**ただし有効な既存 WLNL ホールド下の accept は免除**(A→A2 の設計修正。免除なし版(A)は n2 バースト完殺と引き換えに #349 の317ブロック(556459.8–556523.0)を全滅させ fix −342 で不採用)。
- **ノブB** `GNSS_PPP_CLAS_HOLD_CONT_MIN_TRACK`(int, デフォルト60): #349 エイジフロアの可変化。

### 実験結果(オーケストレータ独立採点済み)

| 実験 | 結果 |
|---|---|
| n2 bitcheck(env なし) | md5 01a2dff7… 完全一致 ✅ |
| **実験A(旧設計・不採用)** n2 floor5.0 | fixed 2249→1907(317ブロック全滅)、ただしバースト19→0・rms 0.625→0.517・max 0.659。→ A2 へ設計修正 |
| **実験B** t2 mintrack=10 | **fixed 1955→2095(21.51→23.05%、目標21.7超え)**、rms 0.322→0.377、>3m 0、>1m 18→17、max 不変。7つの若ホールド窓全回収。新規258 fix は全て<0.77m。lost 118 は churn(要注視) |
| **実験A2** n2 floor5.0+ホールド免除 | **実行中/未完**(07-27 13時時点)。期待: バースト19→0、317ブロック無傷、fixed ≈2249−19 |

### 残ステップ(このスライスの完了条件)

1. 実験A2 の n2 採点: バースト窓(556406.4–556410.4)→0、317ブロック(556459.0–556523.0)≈317 維持、555856クラスタ(~91)維持、>3m=0 を確認。
2. クリーンなら 2×2 交差(n2 に B=10 単独、t2 に A2=5.0 単独)で相互無害を確認。
3. コミット・push(ブランチ agent/clas-gate-experiments-20260727、PR はまだ)。
4. **両ノブ ON(A2=5.0, mintrack=10)で6走行フル検証**(受け入れ: >3m は理想的に全ランゼロ — A2 が n2 バーストを殺すため。rmsFIX 悪化幅と fix% と max を全ラン確認)。
5. 通れば「デフォルト値変更(0→5.0, 60→10)」の PR。マージは必ずユーザーの明示指示。

---

## 4. 検証済み知見バンク(全て per-epoch 計測・裏取り済み)

### n3(解決済み)
- 真因は **outage_count が maxout 超過後に永久張り付き**(オーバーフロー分岐が値保持、全フィルタリセットまでクリアなし)→ G15 偽スリップ(RINEX には25/25エポック観測あり)→ 健全ホールド崩壊→ 15フロートエポック全状態ワイプのループ。#351 で修正、fix 250→451。
- #351 の注意点: 我々のクリアは raw 観測ゲート、MRTKLIB は vsat(残差テスト生存)ゲートで僅かに緩い近似(6走行検証で無害を確認済み)。
- KIN-CHISQ ゲート(閾値5)は n3 で本物のχ²を一度も評価していない(全棄却が phase_chisq=100 センチネル=位相検証行ゼロ)。閾値調整は無効(=20 は md5 同一の no-op)。env `GNSS_PPP_CLAS_KIN_CHISQ_MAX` あり。
- リカバリ検疫フロア(nb≥7)のコードコメント前提「MRTKLIB は nb=8 から」は**トレース実測で反証**(MRTKLIB の実オンセットは nb=6 ratio 6.01)。フロア自体は未変更。

### n2 バースト(機構解明済み、A2 で対処中)
- 生成: FLOATCNT ワイプ 1.0s 後、全衛星 lock=1 の単一エポック新規 LAMBDA fix(nb=7 ratio 4.12 vs 閾値2.78)→ ホールドが誤整数を19エポック固定(ratio 上昇は自己整合であって正しさの証拠ではない)。
- 判別条件: 悪い種=ホールド無し新規 fix、良い種(317ブロック起点 556459.8 ratio 3.17)=成熟ホールド継続。**ratio では分離不能、ホールド有無で分離**。
- ポスト#351 の n2 に「破滅的若ホールド継続」シナリオは**もう存在しない**(候補5エポックのみ、全て良性/無関係)→ エイジフロア緩和の根拠。

### t2(目標実質達成)
- MRTKLIB-only 321エポックの分類: post-wipe死時間110 / ratio再収束遅れ67 / 真のAR失敗36 / **エイジゲート殺し34(第一次シグネチャは13)** / posvar-gate 25 / 参照ロールバック等31 / リカバリフロア14 / chisqセンチネル3。
- エイジゲート殺しは実験B(mintrack=10)で回収済み → 23.05%。
- 未着手リード: posvar-gate(リセット後共分散収束が MRTKLIB より遅い、ppp_ar.cpp:1548-1555)、[CLAS-FILTER2] 参照ロールバック(G04 未検出スリップ、t2 tow 178645-178647)、FLOATCNT タイマーが「ratio合格だが行政棄却」エポックを FLOAT として数える問題。

---

## 5. 実行環境・コマンド(全て検証済みの現物)

### ソルバ実行(パリティ env 必須)
```bash
cd <worktree or repo root>  # data/ シンボリックリンク必須
env GNSS_PPP_CLAS_BASE_CLOCK_PARITY=1 GNSS_PPP_CLAS_SIS_BOUNDARY=1 \
    GNSS_PPP_CLAS_TROP_GRID_PARITY=1 GNSS_PPP_CLAS_QZSS_S_PRN_FIX=1 \
  build/apps/gnss_ppp \
  --obs data/PPC-Dataset/<city>/<runN>/rover.obs \
  --nav data/PPC-Dataset/<city>/<runN>/base.nav \
  --ssr /media/sasaki/aiueo/rtklib_v2_ws_output/ppc_clas_full/<name>/ssr/<name>_expanded.csv \
  --kinematic --use-dynamics-model --no-ionosphere-free --estimate-ionosphere \
  --enable-ar --ar-method wlnl --clas-osr --emit-epoch-time \
  --estimate-troposphere --ar-ratio-threshold 3.0 --out <SSD>/<name>.pos
```
所要: n3 ~20分, n2 ~40分, n1 ~45分, t2 ~50分, t1 ~65分, t3 ~2時間。デバッグは `GNSS_PPP_DEBUG=1` を追加(stderr 100–200MB)。**デバッグ有無で .pos は md5 同一になるはず — 毎回確認**。ビルドは `cmake --build build -j --target gnss_ppp`。

### 採点(正準)
```python
# リポジトリ(または worktree)ルートで
import sys, math; from pathlib import Path
sys.path.insert(0,'scripts')
import generate_ppc_clas_scorecard as s
ref=s.read_reference_csv(Path('data/PPC-Dataset')/city/run/'reference.csv',
                         apply_lever_arm=False, city=city)  # False が正準!
sol=s.read_ppp_pos(Path(pos))
m=s.comparison.match_to_reference(sol,ref,s.MATCH_TOLERANCE_S)[s.ARTICLE_SKIP_EPOCHS:]
fx=[x for x in m if x.status==s.PPP_FIXED_STATUS]
# x.tow / x.horiz_error_m / x.status。キーは round(x.tow,1)。>3m と >1m を必ず数える
```
worktree の scripts/ は旧デフォルトの可能性 → **apply_lever_arm は常に明示**。

### docs 再生成(表更新 PR 用)
```bash
python3 scripts/generate_ppc_clas_full_comparison.py --dataset-root data/PPC-Dataset \
  --results-dir <6走行の .pos ディレクトリ> \
  --metrics docs/ppc_clas_full_metrics.json --markdown docs/ppc_clas_full_table.md \
  --trajectory-figure docs/ppc_clas_full_trajectories.png \
  --error-figure docs/ppc_clas_full_errors.png \
  --metric-figure docs/ppc_clas_full_comparison.png --min-epoch-coverage 0.92
```
+ README「### Moving CLAS PPP vs MRTKLIB」節と docs/ppc_clas_validation.md「Current verdict」を手動同期。テスト: `python3 -m pytest tests/test_ppc_clas_scorecard.py -q`。

### MRTKLIB v0.5.1 参照実走(per-epoch 比較用)
```bash
cd /tmp/mrtklib_ref
build/rnx2rtkp -k conf/benchmark/clas.toml -k conf/benchmark/<tokyo|nagoya>.toml \
  -ts <week> <tow_start-60> -te <week> <tow_end> \
  -o <out>.nmea <rover.obs絶対パス> <base.nav絶対パス> <L6ファイル>
# per-satellite トレースは -x 3(窓を絞ること。フル窓だと数GB)
```
- L6: /tmp/ppc_clas_full/<name>/l6/ または /media/sasaki/aiueo/rtklib_v2_ws_output/ppc_clas_full/<name>/l6/
- **GGA 時刻は UTC**(time_system=gpst 表記でも)→ `tow = day_offset + sod + 18`。検証法: 高速走行エポックで reference.csv と <1m 一致するのは +18s のときだけ。GGA quality 4 = FIX。
- week/tow: t1–t3 = week 2324、n1–n3 = week 2325。reference.csv 1行目に tow/week あり。

---

## 6. 主要パス・md5 台帳

| 何 | どこ |
|---|---|
| メインチェックアウト | ~/workspace/rtklib_v2_ws/gnssplusplus-library (develop 8a44ef2) |
| 実験ワークツリー | ~/workspace/rtklib_v2_ws/gnssplusplus-worktrees/clas-n3-kinchisq(ブランチ agent/clas-gate-experiments-20260727、未コミット変更あり、data→メインの data へのシンボリックリンク) |
| 正準6走行 .pos(公表の元) | SSD: rtklib_v2_ws_output/clas_outage_fix_sixrun/ |
| 実験出力 | SSD: rtklib_v2_ws_output/clas_gate_experiments/ |
| n3/n2/t2 調査一式(FINDINGS/REPORT.md, デバッグログ, MRTKLIB nmea/trace) | SSD: clas_n3_investigation/, clas_n3_kinchisq/, clas_n3_nb_inventory/, clas_n2_post351/, clas_t2_final_gap/ |
| SSR 展開済み CSV | SSD: rtklib_v2_ws_output/ppc_clas_full/<name>/ssr/<name>_expanded.csv |

md5(バイト同一検証の基準、正準 = clas_outage_fix_sixrun/):
- n2 = 01a2dff7544941f17aa6d20e660b11ab
- t2 = 01612c5af8c53ef40814dcb72f660a69
- n3 = a251f8a34b16cb2b3527ab021577b7d5
- n1 = 4cc36e9f95a4803f1961f43e51b20cdc, t1 = 6b1c689174d1097cdd268d39c6635cbf, t3 = dbd884dbea08227f00a3ade81159c8f3
- (#351 以前の n3 ベースライン = dd1fd422df58e5546e3057d761d6a3c8)

### デバッグタグ用語集(GNSS_PPP_DEBUG=1 の stderr)
`[PPP-RESAMB-DIRECT] fixed:/ratio reject:` = **最終 accept 層**(WLNL-RESAMB は per-satellite 層なので数えるな) / `[CLAS-KIN-RECOVERY]` = リセット後30エポック検疫の nb≥7+ratio フロア / `[CLAS-KIN-CHISQ]` = χ²ゲート(センチネル100注意) / `[CLAS-FLOATCNT]` = 15連続FLOATの全状態ワイプ / `[CLAS-KIN-MAXDIFFP]` = SPP乖離リセット / `[CLAS-HOLDCONT-DBG(-RESET)]` = #349 carve-out 診断(hold_valid/hold_age_nfix/streak/carve_out_fired) / `[CLAS-SLIP] reason=outage_sat` = outage スリップ(#351 後は激減が正常) / `[CLAS-POST-RESET-FLOOR]` = ノブA2 の棄却ログ

---

## 7. 罠(全部実際に踏んだ)

1. **並列ソルバは最大2本**。6本並列は swap 枯渇で OOM kill(3本即死)。安全は直列。killed ランは**出力0バイト**(書き出しは終了時バッファフラッシュ)— 45分走っていても 0 バイトはあり得る。
2. **pgrep/pkill 自己マッチ**: パターンは [b]racket 必須、かつ watcher の echo 文やコマンド本文に平文プロセス名を書くな(pkill と起動を同一複合コマンドに入れると自殺する)。
3. subagent は「monitor 待ち」で何時間も止まる。**ラン完了はディスク(md5/ファイルサイズ)で自分で確認して蹴る**こと。
4. worktree の scripts/ は本体と乖離し得る → apply_lever_arm 明示、可能なら採点はメインチェックアウトから。
5. MatchedEpoch に epoch_time は無い(.tow を使う)。scripts import はリポジトリルートから。
6. agent の集計数字は帰属込みのことがある(例:「エイジゲート殺し34」の第一次シグネチャは13)— 元ログで再カウントせよ。
7. tokyo_run3 は最重量(SSR 701MB、~2h)。ピーク RSS ~2.0GB で正常。

---

## 8. 残りキュー(ユーザー指定順の続き)

1. **(進行中)** ノブ A2+B の完成 → 6走行検証 → デフォルト flip PR(§3)。成功すれば >3m 全ランゼロ + t2 23% + n2 バースト解消。
2. n1 精度(0.450 vs 公表 1.105 で既に勝ち — 実質クローズ、ユーザーに確認)。
3. バンク済みリード: FLOATCNT タイマーの数え方(行政棄却を FLOAT に数えない)、posvar-gate 収束、[CLAS-FILTER2] G04 未検出スリップ、nb=8-vs-hold 摩擦、リカバリフロア nb≥7 の再校正(前提は反証済み)。
4. 長期バー: MRTKLIB v0.4.3 realtime 97.7% fix。

## 9. 運用ルール

- マージは**必ずユーザーの明示指示**(「マージ！」等)を待つ。PR 作成までは自律で可。
- 重い入出力は外付けSSD(/media/sasaki/aiueo/rtklib_v2_ws_output/)。リポジトリ内に .pos やログを置かない。
- コミット trailer に Co-Authored-By を付ける。docs/HANDOFF_PPC_CLAS_CODEX.md(本書)は**コミット禁止**(untracked のまま維持)。
