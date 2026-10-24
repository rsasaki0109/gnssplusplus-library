# Claude向け引き継ぎ: GNSS可視化とRTK FIX調査

## まず読む要約

ユーザーはREADMEの実データ可視化から、**「LOS衛星が多いのにRTK FIXしない理由を調べ、誤FIXを増やさず改善したい」**という調査に進んでいる。
安全なFIX復帰はまだ未解決。設定の緩和や局所的な拒否ルールを足すだけでは、全区間の誤FIXが増えた。
直近では、再現テストで証明できた信頼位置履歴の不具合2件を修正した。これは既定設定での誤FIX増加の原因ではなかった。

次の担当は、既存成果を維持して整数候補の誤選択・共分散・受理後の状態発展の調査を続ける。**FIX数だけで改善判定しない。**
GTはオフライン採点と可視化のカメラにだけ使い、ソルバーには渡さない。

## ブランチとPR

資料作成直前にGitHubで状態を確認した。作業開始時は再確認すること。

| 対象 | 状態・用途 |
| --- | --- |
| Repository | `rsasaki0109/gnssplusplus-library`、元の対象ブランチは `develop` |
| 作業場所 | `/workspace/gnssplusplus-library` |
| 現在のブランチ | `fix/rtk-doppler-consensus` |
| ソルバー最終修正 | `1f38e319a38007c224635dbf1a1239069c9d1fc7` |
| [PR #641](https://github.com/rsasaki0109/gnssplusplus-library/pull/641) | OPEN / DRAFT、base `develop`。FIX検査・履歴修正・棄却実験の記録。push済み |
| [PR #639](https://github.com/rsasaki0109/gnssplusplus-library/pull/639) | OPEN / DRAFT、`docs/gnss-hero-zenith`。天頂ビュー。head `5b65458fcf78971ee18fcfa788510e09582db1b3` |
| [PR #637](https://github.com/rsasaki0109/gnssplusplus-library/pull/637) | マージ済み。GT / RTKLIB / libgnss++を同じ3D空間に描くREADME GIF |

引き継ぎ資料を追加する前の作業ツリーはcleanだった。この資料は追加のドキュメントコミットになるので、上記「ソルバー最終修正」は資料自身のHEADとは異なる。
PR #639と#641は別作業。現在のブランチには未マージの天頂ビュー変更は入っていない。
現在、#641や#639をマージする依頼は受けていない。

主なコミット:

- `8fcdf6f9`: 有効なDoppler整合性検査をAR / hold-fix両経路に適用。
- `2e073cb2`, `6473615a`: 不採用実験のパッチ・評価を保存。パッチ空コンテキスト行の空白とLFを `.gitattributes` で保持。
- `1f38e319`: 同一時刻の履歴更新と棄却時の復元漏れを修正。

## 読む順序

1. [AGENTS.md](../AGENTS.md): CMake / coreテスト / per-concern TU / Conventional Commits。
2. [信頼位置履歴の修正](rtk_trusted_history_rollback.md): 最新の実装と影響範囲。
3. [履歴修正の全12回評価](urbannav_trusted_history_evaluation.json): 既定・実験設定8回とCV設定4回。
4. [FIX復帰調査](urbannav_doppler_consensus.md): 根拠、従来の不採用実験、限界。
5. 必要なら [README GIFの再現・出典](odaiba_hero.md)。

JSONは過去の実験時点のsource/binary/input/output hashを記録している。過去レポートのhashを現在のHEADに無条件で当てない。
最新評価の `execution_source_hashes` と `review_source_hashes` の違いは、実行後に古い説明コメントを訂正したため。
コメント訂正後に再ビルドし、実行ファイルが採点済みバイナリと完全一致することを確認している。

## 確認できた原因と保持している修正

### 古いFIX位置への固定が移動中の再FIXを阻害

既定値は `max_position_jump_m=5.0`、`max_fixed_anchor_age_s=0`（アンカーが失効しない）。
OdaibaのGIF区間では、現在位置から約415 m離れ、約179.1秒古いFIX位置とのジャンプで候補が拒否されていた。
GTでも約413 m移動している。単なる衛星数不足ではなかった。
区間内の201観測epoch中173 epochで整数FIX候補ができたが、`max_position_jump` で全て拒否された診断がある。
例: GPS週2032 / TOW 273700、full LAMBDA ratio約41.79、閾値3.0。

この問題を避けるためにアンカー期限だけを設定すると誤FIXが増えるため、**既定値は変えていない**。

### 有効なDoppler検査が省略されていた

以前は新しいFIXアンカーがあると検査を省略し、hold-fix経路にも同じ検査がなかった。
`validateFixedDopplerConsensus()` を共通化し、有効時は両経路で受理前に検査する。
既存軌跡が候補時刻に追いついていなければ `fixed_doppler_stale`、距離不整合なら `fixed_doppler_consensus`。

設定の既定値は0（無効）、moving-baseは従来どおり対象外。軌跡がない場合の初回FIXは依然として従来の初期化経路。
軌跡は受理済みFIXを起点にする**相対的な整合性検査**であり、独立した絶対位置や整数正解の保証ではない。

### 同じepochの再記録で速度履歴が潰れていた

`generateSolution()` は仮FLOATを `rememberSolution()` に記録する。その後、同時刻のFLOAT再記録やFIXへの置換が起こる。
以前は毎回「現在」を「前回」にシフトし、前後の信頼位置の時刻が同じになっていた。
`CV_PREDICT` は差分速度を作れず、ゼロ速度にフォールバックしていた。
現在は**新しい時刻のときだけ履歴を進める**。同一時刻では現在サンプルのみを置換する。

### 棄却時に「前回の信頼位置」が戻っていなかった

現在の信頼位置・時刻・有無は復元していたが、`prev_trusted_position_` / `prev_trusted_time_` / `has_prev_trusted_position_` は復元していなかった。
仮FLOAT生成後に棄却してSPPを返しても、速度用の履歴が書き換わっていた。
現在は両サンプルを保存し、FLOAT早期棄却・FIX検証棄却・FIX後段棄却の3経路で共通の `restoreTrustedState()` を使う。

新しい回帰テスト2件は修正前に失敗し、修正後に成功。
整数探索の既存のfull / subset / multi-frequency合成ケースでも、受理前のFLOAT state / covarianceが完全一致することを追加検査した。
これは**全てのオプションや状態の完全なトランザクション性を証明するものではない**。

## 最新の実データ結果

履歴修正の前後では、両データセットの既定設定と `age5_doppler3` 設定のPOSが全てbyte-identical。
`LEGACY` trust policyは前回サンプルを使わないため、この修正はその設定での誤FIX増加を説明も解消もしない。

| 現在の設定 | FIX数 | FIXの3D誤差 >0.5 / >2 / >3 m | 欠測数 | 全位置の水平P95 m |
| --- | ---: | ---: | ---: | ---: |
| Odaiba 既定 | 1298 | 0 / 0 / 0 | 1426 | 4.7772 |
| Shinjuku 既定 | 122 | 0 / 0 / 0 | 3234 | 8.3821 |
| Odaiba age5_doppler3 | 4256 | 157 / 3 / 0 | 1480 | 4.8963 |
| Shinjuku age5_doppler3 | 1949 | 13 / 1 / 1 | 3332 | 8.3432 |

`age5_doppler3` = `--max-fixed-anchor-age 5 --max-fixed-doppler-consensus 3`。推奨設定ではない。

履歴を実際に使う既存の `--float-trust-policy cv-predict` は次の結果。

| 対象 | 修正前 → 修正後の欠測 | 修正前 → 修正後の水平P95 m |
| --- | ---: | ---: |
| Odaiba CV | 1809 → 1827 | 11.3394 → 16.5589 |
| Shinjuku CV | 3156 → 3601 | 11.1764 → 9.7648 |

CVのFIX数は1298 / 122で不変、上記3閾値の誤FIXは前後とも0、正しかったFIXの喪失も0。
ただし欠測が両方で増え、Odaiba P95も悪化した。**履歴の不変条件を直したことと、CVモデルの実用性能向上を混同しない。CVの有効化は推奨していない。**

## 不採用案: 同じ調整を繰り返さないための記録

以下はいずれも全区間で不採用。設定を結果に合わせて再調整していない。

| 案 | 結果・保存場所 |
| --- | --- |
| アンカー期限5秒だけ | FIX増加とともに誤FIX増。ローカル `output/anchor-guard-validation/` |
| 適応ジャンプ速度30 m/s | 誤FIX増。短いdtを1秒に正規化する既存処理もあり、広い緩和になる |
| age5 + Doppler3 | 上の表。既定採用はNO_GO |
| さらに `--min-fix-satellites 4` | 初期比較のShinjukuで3D >3 m誤FIXが14件に増加。ローカルのanchor-guardレポート参照 |
| 位置出力なしでもDoppler追跡を維持 | Odaiba >0.5 / >2 mが157/3 → 173/13、欠測1480 → 2132。Shinjukuも悪化、単発の大誤差FIXは残った |
| 初回FIXを5候補連続で確認 | Shinjukuの対象単発FIXは止まったが、全体の >3 m誤FIXが1 → 14、>0.5 mが13 → 473。正しい既定FIXも4/122喪失 |

最後の2案は**実装・CLI・専用テストを本体から除去済み**。再現用の正確なパッチと評価JSONだけを保存。

- `docs/urbannav_doppler_output_gap_experiment.patch` / `..._evaluation.json`
- `docs/urbannav_doppler_bootstrap_experiment.patch` / `..._evaluation.json`

各パッチは独立に `8fcdf6f9` に適用するもの。現在のブランチに重ねて適用しない。
パッチのSHA-256は各実行manifestのsource diffと一致。
`--fixed-doppler-bootstrap-epochs` と評価スクリプトの `--after-bootstrap-epochs` はその実験パッチ内だけにあり、**現行本体には存在しない**。

## 時刻・データ・採点

- データは公開UrbanNav TokyoのOdaiba / Shinjuku。両方とも同じ収録系・Trimble機材系列で、既に調査に使用済み。未使用のholdoutや独立ハードウェア検証とは呼べない。
- 各ディレクトリに `rover_trimble.obs`, `base_trimble.obs`, `base.nav`, `reference.csv`。
- ローカル: `data/driving/Tokyo_Data/{Odaiba,Shinjuku}/`。
- 測量済み基地局ECEF: **`-3961904.9530 3348993.7578 3698211.7553` m**。ヘッダーの概略座標に置き換えない。
- 時刻はGPS週 / TOW。以下のTOWをUTCや日本時間と混同しない。
- Odaiba GT: 12410 epoch、週2032 / TOW 273375.1–274616。
- Shinjuku GT: 20950 epoch、週2032 / TOW 281275.1–283370。
- 週とTOWを1 msへ丸めて完全一致で採点。補間・nearest借用で欠測を埋めない。FIX status=4。
- 「誤FIX」は公開GTに対する3D位置誤差の閾値であり、整数ベクトル自体の誤りの直接証明ではない。
- 欠測、正しいFIXの喪失、共通出力集合での誤差、復帰時間、末尾のright-censored outageも残す。
- 並列ジョブの実時間は競合を含む。速度改善の主張には使わない。
- 採用条件の考え方は [fix_recovery_guard.md](fix_recovery_guard.md)。FIX数増加や既知の1点の抑止だけでは採用しない。

公開アーカイブは `https://dl.dropboxusercontent.com/s/isu1bugpgr8dpas/Tokyo_Data.zip`。
ZIP全体は約4.45 GBだが、HTTP Rangeで必要な4ファイルだけ取得してCRC / SHAを確認している。
追跡済みの `scripts/readme_hero/fetch_inputs.py` は **Odaiba専用**。
`--output-dir` だけをShinjukuにしても中身はOdaibaなので注意。
現在環境のShinjuku用スクリプトは `/workspace/scratch/gnss-public-data/fetch_shinjuku.py`。
別環境では元スクリプトのZIP member選択 `/Odaiba/` を `/Shinjuku/` に変えて再取得し、保存済み評価JSONの入力hashと照合する。

## ビルドと検証の再開

現在環境ではGCC 14.2 / C++20 / Release、CPU割当4、RAM約32 GiB。

| ローカル資産 | 場所 |
| --- | --- |
| Python/CMake venv | `/workspace/scratch/gnss-build-env`（numpy / Pillow / pybind11等あり） |
| Eigen 3.4.0 / GTest 1.17.0 install prefix | `/workspace/scratch/gnss-deps` |
| Eigen source | `/workspace/scratch/eigen-3.4.0` |
| GTest source | `/workspace/scratch/googletest` |
| 最新native binary | `build/apps/gnss_solve` |

```sh
cd /workspace/gnssplusplus-library
/workspace/scratch/gnss-build-env/bin/cmake -S . -B build \
  -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON \
  -DGNSSPP_BUILD_PYTHON_BINDINGS=OFF \
  -DCMAKE_PREFIX_PATH=/workspace/scratch/gnss-deps \
  -DPython3_EXECUTABLE=/workspace/scratch/gnss-build-env/bin/python3
/workspace/scratch/gnss-build-env/bin/cmake --build build -j4
CPLUS_INCLUDE_PATH=/workspace/scratch/gnss-deps/include/eigen3${CPLUS_INCLUDE_PATH:+:$CPLUS_INCLUDE_PATH} \
  /workspace/scratch/gnss-build-env/bin/ctest --test-dir build -L core -j4 --output-on-failure
```

最新変更でcore **100/100成功**。`/workspace/scratch/trust-history-core.log` に結果。
2件のPythonテストが `/usr/include/eigen3` を固定参照するため、上記include環境変数が必要。
GTSAM / OpenCV / ROS2は未導入、Python bindingsはビルドしていない。
無ラベルの全CTestには未ビルドbindingsや別のローカル研究成果を要求するものがある。まずcoreを使う。
`data/rover.obs` 等をUrbanNavへsymlinkして歴史的fixture用テストを実行しない。

既定・age5/Doppler3の8回比較:

```sh
/workspace/scratch/gnss-build-env/bin/python3 \
  scripts/analysis/evaluate_urbannav_fix_continuity.py \
  --before /workspace/scratch/gnss-solve-before-trust-history \
  --after /workspace/scratch/gnss-solve-after-trust-history \
  --data-root data/driving/Tokyo_Data \
  --output output/trust-history-new-comparison --jobs 4
```

出力先は空ディレクトリ必須。既存成果を上書きしない設計。
CVの4回比較は `/workspace/scratch/evaluate_trust_history_cv.py`。
別環境では評価JSON内のnative argvから再現できる。
baselineがない場合は別checkoutの `6473615a` を同じツールチェーンでビルドし、変更前binaryを退避する。
比較中に実行ファイル・入力を再ビルドや書換えで変更しない。

## バイナリと生ログの所在（同じクラウド環境のみ）

`/workspace/scratch`、`data/`、`output/` の大部分はgit管理外。別環境には自動で付いてこない。
追跡済みJSON / パッチ / コマンドを再現の根拠にする。

| バイナリ | 意味・SHA-256 |
| --- | --- |
| `gnss-solve-before-consensus` | 元のDoppler省略バグあり。SHAは初期評価JSON参照 |
| `gnss-solve-after-consensus`, `gnss-solve-before-trust-history` | gate修正済み、履歴修正前。`84566f414f86563111bf2807b8e5912b583ff3c7744175c1c3540a3d8a4a2a71` |
| `gnss-solve-after-trust-history` / 現在の `build/apps/gnss_solve` | 現行。`854dbf4f8fbd2e2c4f11be95e085ac45cdeb06432f8c8691fc8719caf6bc1f60` |
| `gnss-solve-independent-doppler` | 棄却実験専用。現在の本体ではない |
| `gnss-solve-bootstrap-confirmation` | 棄却実験専用。現在の本体ではない |

表内の `gnss-solve-*` は全て `/workspace/scratch/` 配下。

| ディレクトリ | 内容 |
| --- | --- |
| `output/anchor-guard-validation/` | 初期オプション比較、min4等、`README.md` / scores / provenance |
| `output/doppler-consensus-validation/` | gate修正8回、Odaiba / Shinjuku debug prefix |
| `output/doppler-observation-continuity-validation/` | 不採用のDoppler維持実験 |
| `output/doppler-bootstrap-validation/` | 不採用の5候補実験、Shinjuku debug CSV |
| `output/trust-history-validation/` | 最新修正の既定・age5比較8回 |
| `output/trust-history-cv-validation/` | 最新修正のCV比較4回 |
| `output/readme-hero/` | 元の2ソルバーPOS、`sky-ar-debug.csv`、`guard-age-doppler-debug.csv` 等 |

## 次に追うコードと検証点

| 場所 | 調べる対象 |
| --- | --- |
| `src/algorithms/rtk.cpp` | `generateSolution()`, `rememberSolution()`、候補生成と信頼状態の関係 |
| `src/algorithms/rtk_epoch.cpp` | FLOAT生成、AR候補、検証、棄却、hold-fix、snapshot / rollback |
| `src/algorithms/rtk_ar_resolve.cpp` | `resolveAmbiguities()`, subset選択、`finishAmbiguityResolution()` |
| `src/algorithms/rtk_ar_validate.cpp` | `validateFixedSolution()`, Doppler / history / residual gates、`tryHoldFix()` |
| `src/algorithms/rtk_fallback.cpp` | 再初期化、信頼喪失、CV seed、受理後の状態発展 |
| `src/algorithms/rtk_filter.cpp` / `rtk_update.cpp` | 更新後のFLOAT state / covariance、残差・重み |
| `tests/test_rtk_legacy.cpp` | `RTKTrustedHistoryTest.*`、AR fixtureでのFLOAT state / covariance不変条件 |

次の作業候補（**未確認の仮説であり、修正済みの事実ではない**）:

1. 大誤差FIXの最初の発生点で、採用subset・参照衛星・ratio・DD整数候補・Qb/Qab・float→fixed変位・hold由来かを記録する。
2. そのepochより前の状態で初めて差が出た箇所を特定し、整数選択そのものと、正当に受理された過去の候補が作る状態の差を分ける。
3. 同一epochの反復KF更新で共分散を引き継ぐ処理が、再線形化と情報の二重計上を区別できているかを線形toy caseで検証する。
   対象は `rtk_epoch.cpp` の `updateFilter()` 反復と `rtk_filter.cpp`。**この仮説は今回未検証・未修正。二重計上バグと断定しない。**
4. ログに現れない別状態の漏れを疑う場合も、受理済みhold更新など意図的な状態変化と区別し、再現テストを先に作る。

診断例: `--debug-epoch-log output/new-debug.csv --max-epochs 15740` をShinjukuの先頭から実行するとTOW 282847.6付近まで到達する。
元の問題epochはOdaiba 273936.1（新しいアンカーによるDoppler省略）、Shinjuku 282847.6（軌跡なしの初回FIX）。
棄却した5候補実験では別の大誤差がShinjuku 282745.7–282746.9と283306.7に出た。
CSVやPOSを途中から切り出してソルバーを開始すると初期化履歴が変わる。診断prefixは先頭から再実行し、元の全区間POSと数値行を照合する。

## 可視化を再開する場合の注意

ユーザーは実在の公開データ・PLATEAU建物・OSM地図でOSSらしさを示したい。架空の軌跡や見栄えのためのFIX化は求めていない。
現在マージ済みGIFはGT白破線、libgnss++緑、RTKLIB橙の3線を同じシーンに描き、カメラはGTに追従。
Odaiba週2032 / TOW 273700–273720の20秒は、両出力の完全一致時刻・coverage・native誤差等の条件から選んだ有利なshowcase区間で、全区間性能の代表値ではない。

PR #639の天頂ビューは未マージ。衛星32個のclip内unionを各GT時刻へ伝播し、RINEX観測が消えても衛星を消さない修正済み。
観測欠落はIDの `*` とRx数で別表示する。LOS/NLOSは2025年PLATEAU建物による合成遮蔽判定で、2018年の実測受信状態や位相品質のラベルではない。
モデル上LOSが多いことだけでRTK FIX可能とは判定できない。solverにこの幾何ラベルを与えた比較でもない。
描画コード・再現コマンドは該当ブランチの `scripts/readme_hero/` と `docs/odaiba_hero.md` にある。

## 運用メモ

- ユーザーには日本語で簡潔に進捗を伝える。既に許可された調査・修正で繰り返し確認を求めない。
- 新しい仮説のパラメータは実行前に固定し、Odaiba / Shinjukuの結果を見て閾値を追い込まない。
- この環境はproxy経由の制限付きネットワーク。GitHub API、公式Dropbox直リンク、PLATEAU、OSM等が許可されている。proxy / CA設定を保持し、資格情報を表示しない。
- `gh api` とgit pushは動作した。`gh auth status` の表示だけで再ログインを始めない。
- `gh pr edit` はProjects classicのGraphQLで失敗したことがある。PR本文はJSONファイルを作り、`gh api --method PATCH repos/rsasaki0109/gnssplusplus-library/pulls/641 --input <file>` で更新できる。
- 実験の失敗や性能上の限界を隠さない。修正済みの状態不変条件と、未解決の誤FIX原因を分けて報告する。
