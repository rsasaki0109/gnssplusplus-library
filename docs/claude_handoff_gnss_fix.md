# Claude向け引き継ぎ: GNSS可視化とRTK FIX調査

## まず読む要約

ユーザーはREADMEの実データ可視化から、**「LOS衛星が多いのにRTK FIXしない理由を調べ、誤FIXを増やさず改善したい」**という調査に進んでいる。
安全なFIX復帰はまだ未解決。設定の緩和や局所的な拒否ルールを足すだけでは、全区間の誤FIXが増えた。
直近では、再現テストで証明できた信頼位置履歴の不具合2件を修正した。これは既定設定での誤FIX増加の原因ではなかった。
その後、**反復KF更新が同じepochの観測を2回適用し、位置・DD整数の共分散を約半分に過小評価する**不具合を再現テストで確定した。
iterated EKF化（opt-in `--kf-relinearize-from-prior`）は全区間の採用基準を満たさず、**既定は不変**（[詳細](rtk_iterated_update_covariance.md)）。
続いて `applyHoldAmbiguity()` が条件付き更新ではなく上書きであることも再現した。RTKLIB式のKF hold（opt-in `--hold-kalman-update`）はhold由来の誤FIXを大きく減らすが、AR由来の誤FIXは残りNO_GO（[詳細](rtk_hold_ambiguity_update.md)）。
その後PPC-Datasetを追加取得し、CD系にKF holdを重ねたCDMHを凍結評価した。holdの偏り保持は大半解消したが、default比の採用条件は不合格でNO_GO（[詳細](rtk_cdmh_ppc_evaluation.md)）。

次の担当は、既存成果を維持して整数候補の誤選択・共分散・受理後の状態発展の調査を続ける。**FIX数だけで改善判定しない。**
GTはオフライン採点と可視化のカメラにだけ使い、ソルバーには渡さない。

## ブランチとPR

資料作成直前にGitHubで状態を確認した。作業開始時は再確認すること。

| 対象 | 状態・用途 |
| --- | --- |
| Repository | `rsasaki0109/gnssplusplus-library`、元の対象ブランチは `develop` |
| 作業場所 | `/workspace/gnssplusplus-library` |
| 現在のブランチ | `fix/rtk-doppler-consensus` |
| ソルバー最終修正 | `bd60dfd`（opt-in iterated EKF、既定不変）。その前の履歴修正は `1f38e319a38007c224635dbf1a1239069c9d1fc7` |
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

1. [AGENTS.md](https://github.com/rsasaki0109/gnssplusplus-library/blob/develop/AGENTS.md): CMake / coreテスト / per-concern TU / Conventional Commits。
1. [hold FIXのNIS上限に位相の当てはまり条件](rtk_hold_fix_phase_fit.md) / [JSON](hold_fix_phase_fit_evaluation.json): 最新。opt-in `--hold-fix-update-nis-max-phase-fit <af>`（既定0）を追加: `--hold-fix-max-update-nis-per-obs` の棄却を、hold位置のDD搬送波位相の当てはまり（整数に依らないcos平均の±0.10 m内最大、10ペア未満は不明扱い）が `af` 未満のときだけにする。閾値は位相監査の0.5/10ペア/±0.10 mと既存NIS値10で、調整なし（NISと位相不整合のANDは同じ10区間の分布を見て選んだと開示）。UHP（UH＋0.5）は位相確定の誤FIX 133→36をUHと同じに保ち、U比の正FIX喪失を923→548（−41%）、Shinjuku 0.91→0.18%、nagoya/run1 2.53→0.55%。ただし判定Lはtokyo/run1–3（2.09/2.21/1.22%）とnagoya/run3（9.32%）で不合格、Mはtokyo/run3のHp95のみ不合格で、opt-in保持・preset不変。東京の喪失は棄却の数百epoch後のAR FIX（0.1 m以内）の失敗。nagoya/run3はTOW 553961.2のholdが実は0.87 m誤り（下のNIS記録の「正しい」は誤りで訂正済み）、正しく棄却した後にhold経路が閉じ、Uが0.2 mに戻った後のhold 98件を失う。UHP2（＋keeps-hold-run）はnagoya/run3不変・Shinjukuの誤FIX増で不採用。
1. [hold FIXのNIS上限と `--preset urban` の欠測](rtk_hold_fix_nis.md) / [JSON](hold_fix_nis_evaluation.json): その前。`--preset urban`（=CDME、#661で追加、live/replayは#662）の本物の誤FIX 256件のうち183件はhold FIXで、うち107件はフロート更新のNIS/観測>10の直後。既存の `max_fixed_update_nis_per_observation` はARだけを検査し `tryHoldFix()` は素通り。opt-in `--hold-fix-max-update-nis-per-obs`（既定0）を追加し、値10（既存PPC実験の値）で凍結評価。監査後の誤FIX>0.5 mは10区間で256→142（−44.5%）、位相確定の誤FIX 133→36（Shinjuku 69→3）、どの区間も誤FIXは増えずRTKLIB比も10区間で優位。ただしFIXは1.0%減、PPC 5区間でurbanの正FIXを1.6–9.9%失い（判定L不合格）、欠測/Hp95も一部悪化（判定M）。凍結規則によりトレードオフをユーザー判断に回し、ユーザーはopt-in維持を選択。改良版 `--hold-fix-update-nis-keeps-hold-run`（NISだけで弾いたholdの後もhold経路を閉じない）はnagoya/run1・tokyo/run1–2の喪失を減らすが誤FIXは142→181に戻り、nagoya/run3は不変（NIS 1559の測定側の不整合でも旧holdは正しく、holdを弾くと不整合な更新がフロートに入り次のholdが1 m超で棄却される）。NISはholdが誤りか測定が悪いかを区別できない。どちらもopt-in保持。欠測増の原因: `gnss_solve` の運動学ポストフィルタ（高さ段差>max(1 m, 4 m/s·dt)を捨てる）がFIXを基準にするため、FIX連続区間後の高さ誤差数mのFLOAT/SPPを捨てている（意図どおり）。ただし偏ったFLOAT基準のせいで正しいFIXも捨てる（Shinjuku 71件など、未修正）。
1. [誤FIXラベルの搬送波位相監査](rtk_phase_audit.md) / [JSON](phase_audit_cdmeh_evaluation.json): その前（ソルバー変更なし、解析ツール `scripts/analysis/truth_dd/phase_audit.py` とテスト追加）。CDMEのShinjuku誤FIX 242件の最大区間は停車中（TOW 283198–283247）に参照軌跡が北へ0.41 mずれたもの: DD搬送波位相のambiguity functionは参照位置で−0.01–0.14、FIX位置で0.84–0.96。凍結規則で全FIX>0.5 mを判定し、Shinjuku CDMEは123件が`reference_inconsistent`、確定誤FIXは69件（うち64件はTOW 281650の1区間）。RTKLIBの>3 m誤FIX 112件は0件。PPCは参照の問題がほぼない（nagoya/run2のCDME 9件のみ）。監査後の誤FIX数ではCDMEがRTKLIBに10区間すべてで優位（Shinjuku 119/0/0 vs 128/112/112）。確定誤FIXの区間は旧holdの上書き: 浮動解が0.99 mずれ共分散trace 1.8e-4 m²のまま部分ARで正しくFIX→5回目でholdがSD整数だけ上書き→次epochでNIS 1→52、LAMBDA失敗、0.65 mずれたhold FIXを6.6秒受理。CDMEH（CDME+`--hold-kalman-update`）を凍結評価: Shinjuku確定誤FIX 69→3、tokyo/run3 28→3だが、PPC tokyo/run1が24→129（RTKLIB超え）、u-blox Shinjuku 10→57、CDME比の正FIX喪失1–15%でNO_GO。次は状態を整合させる狭いhold修正（再現テスト→凍結評価）。
1. [Doppler整合性の基準軌跡に期限](rtk_doppler_consensus_age.md) / [JSON](urbannav_doppler_consensus_age_evaluation.json): その前。RTKLIBを全衛星系（#657）にすると既定のlibgnss++はOdaiba全区間でFIX 1298 vs 5839と負ける。差の主因はDのロックアウト（RTKLIBだけ正FIXの2129 epoch中1888）。opt-in `--max-fixed-doppler-consensus-age`（既定0=従来）を追加し、CDME=CDM+5秒で凍結評価。RTKLIBより「FIX多く誤FIXも少ない」は10区間中9区間（Trimble Shinjukuのみ0.5–2 m誤FIX 242 vs 128で未達、>3 mはRTKLIB 776件 vs CDME 1件）。ただしCDM比では0.5–2 mの誤FIXと欠測が増え判定C不合格→opt-inのまま。Shinjukuの新しい誤FIXは全てD判定済みの「FIX連続区間のドリフト」（Dは毎FIXで再anchorするので見えない）。次はこのドリフト機構そのもの。
1. [PPC用RTK設定上のCD系](rtk_cd_ppc_profile.md) / [JSON](ppc_profile_cd_evaluation.json): その前（ソルバー変更なし、解析ツール3本追加）。リポジトリのPPC用RTK設定P（`--preset low-cost --ratio 2.4 ...`）にM / CDM / CDMHを重ねて凍結評価。P+MはPとbyte一致。P+CDMは>2 mの誤FIXを918→0にするが正FIXを58–89%失いNO_GO。主因はDoppler整合性検査（D）のロックアウト: 軌跡は受理FIXでしか再anchorされず、経過時間の上限がないため、3 mを超えて漂流すると以後の正しい候補（真値誤差1–16 cm）を全て棄却する（3区間で正9556件棄却・誤13件、最長2807 epoch≒9分）。素のargvのCDMでも同じ（tokyo/run2で3513 epoch）。漂流の原因はPPCのDoppler速度の裾（真値位置で解いてもp95 1.5–3.5 m/s、行ごとの外れ値除去なし）。最大残差の4σ除外で誤差p95は31–49%減るが5秒で3 mは超える。
1. [CDMHのPPC評価](rtk_cdmh_ppc_evaluation.md) / [JSON](ppc_cdmh_evaluation.json): その前（コード変更なし）。PPC-Dataset（SharePointはcookie jar必須、無しだと403だがネットワーク遮断ではない）を取得し、Tokyoと同じ素のargvで default / CDM / CDMH（CDM+`--hold-kalman-update`）を凍結評価。CDMはPPCでFIX 3560→7636、誤FIXは全て<2 mだがtokyo/run2で正FIX 6.26%喪失（旧holdの上書きが0.25–0.44 m偏った解を約24秒保持→その後もLAMBDA失敗）。CDMHで偏ったFIXは124→2 epoch、喪失6.26→2.28%（予測<2%は未達）、nagoya/run3の誤FIX 15→4。ただしdefault比の採用条件は12区間中11で不合格（誤FIX増6区間、喪失>1%が3区間、欠測増）でNO_GO・既定不変。注意: PPCはリポジトリのPPC RTKプロファイル（`--preset low-cost --ratio 2.4 ...`）ではないので既存PPC資料と数値比較しない。
1. [FIX連続区間のドリフト](rtk_fix_streak_drift.md) / [JSON](urbannav_fix_streak_drift.json): その前（挙動変更なし、debug列2つ追加）。CDの残る誤FIXは、1衛星の位相が1–2秒で0.4–1.5サイクル非整数にランプ／段差する区間（LLIなしも多い）にFIXが追従したもの。`postfix_residual_max_pair` は5件中3件で早期に原因衛星を示すが大きさ2–5 cmは正FIXのp95内。副次: 正しいFIXに数十〜160サイクル誤った整数のペアが混じる例（Shinjuku 117 epoch）。holdされるかは未確認。
1. [max position jumpの動き補正](rtk_max_position_jump_motion.md) / [JSON](urbannav_max_position_jump_motion_evaluation.json): その前。CDが失う正FIX（Odaiba 37）は全て動き不感の `max_position_jump`（5秒アンカー×5 m）。opt-in `--max-position-jump-motion-compensation`（Dと同じ伝播規則、新閾値なし）で30を回復。M単独は凍結ゲートの列挙項目を全Tokyoで満たす（u-blox Odaiba FIX 461→617、誤FIX 0）が「誤FIXの減少」条項は満たさず既定はメンテナ判断。CDRI（CD+R+I）は誤FIXが増えNO_GO。CDの残る誤FIX 10件はFIX連続区間末尾の0.07–0.15 m/epochのドリフト（Doppler再anchorで未検出）。
1. [長期lockの過信](rtk_long_lock_overconfidence.md) / [JSON](urbannav_long_lock_overconfidence.json): その前（挙動変更なし）。10 Hz真値DD（採点・解析専用）で位相誤差の振幅はモデル通り（6–9 mm）だが約3秒の相関と遅い成分、コードは中心はモデル通りで裾が重い（3σ超7%）。10 Hzの白色扱い（凍結予測の1 Hz間引き試験で棄却）、電離層、コードの裾のいずれも主因ではない。次はフィルタ内部（kinematic再seedによる位置/ambiguity相関の喪失、コード由来の初期化、参照衛星切替）。
1. [基地局補間の曲率](rtk_base_interpolation_curvature.md) / [JSON](urbannav_base_interpolation_evaluation.json): その前。`interpolateBaseEpoch()` がモデル距離まで線形補間し、1 Hz基地局の距離曲率（DD p95 1.2 cm）を落としていた不具合を、opt-in `--base-interp-curvature` として修正（**既定は不変**）。固定ゲートは雑音レベルの差（欠損+3/+4、u-blox Shinjuku P95 +1.2 cm、HK Harsh）で不合格。Qbの過信の主因ではない。
1. [heroクリップのFIX](rtk_hero_clip_fix.md) / [JSON](urbannav_hero_clip_fix_evaluation.json): その前。PR #639のGIFでLOSが多いのにFIXしない理由と、u-blox holdoutでの検証。
1. [整数一貫性の診断](rtk_ambiguity_consistency_diagnostics.md) / [JSON](urbannav_ambiguity_consistency_diagnostics.json): その前（挙動変更なし）。
1. [hold更新](rtk_hold_ambiguity_update.md) / [評価JSON](urbannav_hold_update_evaluation.json): その前の調査。
1. [反復更新と共分散](rtk_iterated_update_covariance.md) / [評価JSON](urbannav_iterated_update_evaluation.json): その前の調査。
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

### 反復KF更新が同じ観測を再適用していた（確定・既定は未修正）

`processRTKEpochInternal()` は kinematic で2回 `updateFilter()` を呼ぶ。2回目は更新後の事後状態・共分散に同じ行を再度Kalman更新していた。
雑音なし・初期値バイアスなしの合成シーンで、1回更新に対し位置分散とDD整数分散が **0.50–0.51倍**（再現テストは修正前に失敗）。
一方で2回目の再線形化自体は必要: 初期SPPが約4.7 mずれると1回目は18行中14行を3 m prefit gateで除外し、2回目で回復する。

opt-in `--kf-relinearize-from-prior` は、現在iterateでgate → `r0' = r_i + H(x_i - x0)` で事前値へ戻し、(x0, P0) に1回だけ適用する。
OFFは全8実行でbaselineとPOS byte-identical、ONは実験POSとbyte-identical。

全区間評価（判定規則は実行前に固定、閾値の再調整なし）はNO_GO:

| 対象 | before → after |
| --- | --- |
| Odaiba 既定 | FIX 1298→1298、誤FIX 0→0、欠測 1426→1515、水平P95 4.7772→5.1399 |
| Shinjuku 既定 | FIX 122→122、誤FIX 0→0、欠測 3234→3451、水平P95 8.3821→1.9650 |
| Odaiba age5_doppler3 | 誤FIX >0.5/2/3 m 157/3/0→103/3/0、欠測 1480→1562、0.5 m復帰P95 186.7→298.5 s |
| Shinjuku age5_doppler3 | 誤FIX 13/1/1→**516/125/15**、FIX 1949→3972、欠測 3332→3581 |

正しい共分散にすると、過小評価された共分散に合わせて働いていた既存gateを通る候補の集合が変わる。
既存の誤FIX（baseline age5_doppler3）の大半は少数ペア（Odaiba中央値6ペア、93%が≤6）のAR候補で、float→fixed変位が約0.9 m（正しいFIXは18ペア・0.01 m）。
**これは観測であり、検証済みgateではない**。この表から閾値を選ぶのは既に棄却した事後調整と同じ。

`RTKProcessor::fixed_baseline_` が未初期化（Eigen）で、メンバー追加後に `RTKHoldFixMinMatchedTest` がNaN比較で失敗した。ゼロ初期化した（POSは不変）。

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
| holdをKF疑似観測に（`--hold-kalman-update`、単独/relinearize併用） | hold由来誤FIXはOdaiba 74→7、Shinjuku(R) 234→98。ただしOdaiba age5でP95 4.90→6.23・欠測+98、Shinjuku age5で誤FIX 13→19・正FIX喪失5.8%、併用では>2/>3 m増。NO_GO、opt-in保持 |
| CDMH（CD + `--max-position-jump-motion-compensation` + `--hold-kalman-update`）の既定化 | PPC 6区間・Tokyo 4区間・HKで凍結評価。holdの偏りは解消方向（tokyo/run2喪失6.26→2.28%）だが、default比で誤FIX増6区間・喪失>1%が3区間・欠測増でNO_GO。[記録](rtk_cdmh_ppc_evaluation.md) |
| CDMEH（CDME + `--hold-kalman-update`） | Shinjukuの旧hold由来の誤FIXは消えるが、PPC tokyo/run1で誤FIX 24→129（>3 m 1→5）、u-blox Shinjuku 10→57、CDME比の正FIX喪失が10区間中9で>1%（最大14.5%）。NO_GO。[記録](rtk_phase_audit.md) |
| `--preset urban` に `--hold-fix-max-update-nis-per-obs 10`（＋`--hold-fix-update-nis-keeps-hold-run`）を追加 | 誤FIX −44.5%・RTKLIB比は維持だが、PPC 5区間で正FIX 1.6–9.9%喪失・欠測/Hp95悪化で凍結規則上はユーザー判断待ち。opt-in保持。[記録](rtk_hold_fix_nis.md) |
| さらに `--hold-fix-update-nis-max-phase-fit 0.5`（UHP、＋keeps-hold-runのUHP2） | 位相確定の誤FIXはUHと同じ36、正FIX喪失はUH比−41%だが、tokyo/run1–3・nagoya/run3で喪失>1%、tokyo/run3のHp95悪化でpreset不採用。UHP2は誤FIXが戻り不採用。opt-in保持。[記録](rtk_hold_fix_phase_fit.md) |
| CDME（CDM + `--max-fixed-doppler-consensus-age 5`）の推奨・既定化 | RTKLIB比は10区間中9区間で優位だが、CDM比で0.5–2 m誤FIXと欠測が増え判定C不合格。opt-in保持。[記録](rtk_doppler_consensus_age.md) |
| PPC用RTK設定PにM / CDM / CDMHを追加 | P+MはPとbyte一致、P+CDM/CDMHは>2 m誤FIX 0だが正FIX 58–90%喪失でNO_GO。Dのロックアウトが主因。[記録](rtk_cd_ppc_profile.md) |
| 反復KF更新をiterated EKFに（既定化） | 上の表。欠測増・Odaiba P95悪化・Shinjuku age5の誤FIX急増でNO_GO。opt-inとして保持、パッチは `docs/urbannav_iterated_update_experiment.patch`（`35654026` に適用） |

「位置出力なしでもDoppler追跡を維持」と「初回FIXを5候補連続で確認」の2案は**実装・CLI・専用テストを本体から除去済み**。再現用の正確なパッチと評価JSONだけを保存。

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
- 注意: `evaluate_urbannav_fix_continuity.py` の `baseline_correct_fix_lost` は**既定recipeのbaseline**に対する喪失。age5_doppler3等の同一recipe比較での喪失ではない（同一recipeではiterated EKFがOdaiba 15 / Shinjuku 309件を喪失）。採用判定では同一recipe比較も見る。
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

PPC-Dataset（開発データ、holdoutではない）は `data/PPC-Dataset/`（gitignore）に置く。取得は
`curl -sS -L -c cj.txt -b cj.txt -o PPC-Dataset.zip "<taroz/PPC-Dataset READMEのSharePointリンク>?download=1"`。
SharePointのリダイレクトがcookieを要求するため、cookie jarなしでは403になる（ネットワークポリシーの遮断ではない）。
ZIP sha256 `460b2174…b76658`（162247330 bytes）。`.git/` を除いて展開し、[JSON](ppc_cdmh_evaluation.json) の24ファイルhashと照合する。
基地局座標は各 `base.obs` の APPROX POSITION を使う（`--base-ecef` 省略時の `gnss_solve` と同じ値。`ppc-native-replay` も省略している）。

## 別環境での再構築（2026-10 実施）

`/workspace/scratch` がない環境では次で再現した（Ubuntu 24.04、GCC 13.3、CMake 3.28）。

```sh
apt-get install -y libeigen3-dev libgtest-dev pybind11-dev
pip install --break-system-packages --ignore-installed matplotlib   # 3件のPython coreテスト用
python3 scripts/readme_hero/fetch_inputs.py --manifest <scratch>/odaiba-manifest.json
sed 's#"/Odaiba/"#"/Shinjuku/"#' scripts/readme_hero/fetch_inputs.py > <scratch>/fetch_shinjuku.py
python3 <scratch>/fetch_shinjuku.py --output-dir data/driving/Tokyo_Data/Shinjuku --manifest <scratch>/shinjuku-manifest.json
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON -DGNSSPP_BUILD_PYTHON_BINDINGS=OFF
cmake --build build -j4 && ctest --test-dir build -L core -j4
```

8入力のSHA-256は保存済み評価JSONと一致。`35654026` のbinaryで既定・age5_doppler3の4 POSが保存hashとbyte-identical（GCC 14→13でも一致）。
system Eigenがあれば `CPLUS_INCLUDE_PATH` は不要。

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
3. ~~反復KF更新の二重計上~~ → **確定**（上記）。ただし修正単独では採用不可。
   - ~~`applyHoldAmbiguity()` の上書き~~ → **確定**、hold経路の誤basin伝播には効くが主因ではない（[hold更新](rtk_hold_ambiguity_update.md)）。
   - ~~R/HRでのAR候補の受理~~ → 診断済み（[整数一貫性](rtk_ambiguity_consistency_diagnostics.md)）。BSRは正誤とも≈1、full setの最良cost/pairは正しいFIXでも2–137（期待値≈1）で、Qb由来のratio/BSR/χ²では分離できない。整数から外れているのは長期lock（中央値49–198 epoch）のambiguity。誤FIXはfull setが4–10ペアしかないepoch。HRでは正しいFIXでも|z|中央値16–19（hold疑似観測の毎epoch再適用による二重計上の疑い、未検証）。
   - heroクリップ: 既定では179–199秒前・約400 m先の古いFIXアンカーで全候補が棄却。アンカーを失効させると、今度は `exceedsFixHistoryJump()` の静止前提0.2 m判定が10 Hzで2 m/s超の正しいFIXを3回に1回棄却（`.FFF.`）。opt-in `--fix-history-motion-compensation` を追加。CD（anchor5+Doppler3+10ambiguity下限+動き補正）はクリップで連続FIX（中央値2.1 cm）、u-blox holdoutで誤FIX 0だが、Trimbleで誤FIX 6/4件（0.5–2 m）と正FIX喪失2.9%で既定化は不可。
   - u-blox rover（`rover_ublox.obs`）は今回初めて使ったholdout。epoch時刻に+1〜3 msの定常オフセットがあるため、5 ms以内で照合し参照を補間して採点する。
   - ~~長期lockのambiguityが整数から外れる原因~~ → 一部切り分け（[基地局補間](rtk_base_interpolation_curvature.md)）。真値DD（1 Hz、採点専用）の持続的な整数サイクル段差はOdaibaで11時刻、うち9時刻はソルバーが同じ1秒にLLI/リセットを記録、未検出はC04 B2Iの2件のみで主因ではない。基地局補間がモデル距離まで線形補間して曲率を落とす不具合（基地局epochで0、中間で最大のため1 Hz解析には現れない）をopt-in `--base-interp-curvature` で修正したが、受理ARのcost/pair・|z|はほぼ不変（Odaiba 4.73→4.44）。
   - ~~残る長期lockの過信（マルチパスのfloat吸収か、ambiguity過程雑音か）~~ → 入力側は切り分け済み（[長期lockの過信](rtk_long_lock_overconfidence.md)）。10 Hz白色扱い・電離層・コードの裾はいずれも主因ではない。最大|z|ペアのlockは7–16秒で「長期」というより数相関時間。
   - 次: フィルタ内部（kinematicのSPP再seedで位置/ambiguity相関が失われる点、0.3 mモデルでのコード由来初期化、参照衛星切替）を、凍結予測→解析の順で検証する。hold再適用の二重計上は合成シーンで先に再現する。
   - `gnss_live`・`examples/rtk_positioning`・`holdBaseEpoch` は目標時刻の暦を独立に選ぶため、補間区間内の暦切替（数十cm）を拾いうる（未修正）。
   - ~~CDの正FIX喪失（tokyo/run2）とhold basin~~ → 旧holdの上書きが原因の一部（[CDMH](rtk_cdmh_ppc_evaluation.md)）。KF holdで大半解消するが採用条件は不合格。
   - ~~CD系をPPC RTKプロファイル上で評価~~ → NO_GO（[記録](rtk_cd_ppc_profile.md)）。Dの基準軌跡に経過時間・不確かさの上限がなくロックアウトする欠陥と、Doppler速度解が行ごとの外れ値に弱い欠陥の2つを特定（いずれも未修正）。
   - 次の候補: (a) Dの基準軌跡の失効条件（再現テスト→凍結評価。新しい閾値を要するのでTokyo/PPC/HK全てで評価）、(b) Doppler速度解のロバスト化（`gnss_solve` のFLOAT継続とfusionの既定経路にも効くので影響範囲が広い）、(c) PPCでCD系だけ欠測が1–12 epoch増える原因（53件中15件は `gnss_solve` のFIX後5秒のFLOATドリフト判定の窓内、残りは未特定）。
   - Doppler軌跡は受理FIXごとに再anchorするため、連続した誤basin（Shinjuku 283274–283289で145 epoch）は検出できない。
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
