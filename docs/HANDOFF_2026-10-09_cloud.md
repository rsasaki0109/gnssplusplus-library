# 引き継ぎ (2026-10-09) — クラウドセッション向け

develop = `c37979c3`。オープン PR は #549（HAS、ドラフト、以前から保留）のみ。
ローカル環境（外付け SSD、ローカルのメモ、ローカルの出力）はクラウドからは見えない。必要なデータは下の「データの入手」から取得する。

## 運用ルール（ユーザー指示）
- 実作業はサブエージェント（Sonnet）に任せ、オーケストレータは検証役に徹する。数値は自分で再採点し、md5 と「凍結 commit が比較より前か」を確認する。サブエージェントの主張は裏取りするまで信じない。
- PR 作成は提案して了承を得ればよい（「おすすめで」で OK）。**マージはユーザーが「#NNN マージ」と明示したときだけ**。
- 新機能は default OFF とし、OFF のときは出力を bit-identical に保つ。候補の判定は contract を commit で凍結してから比較を回す。結果を見たあとに閾値を調整しない。No-Go も docs と decision json に記録して PR にする。
- CLAS は「ベタ移植」が方針。MRTKLIB v0.5.1（github.com/h-shiono/MRTKLIB）を忠実に移植し、推測でチューニングせず、エポックごとの比較で設計する。6 run の fix%、FIX RMS2D、>3 m の 3 指標で判定する。
- ユーザーとのやり取りは日本語。

## 本日の成果（マージ済み）
| PR | 内容 |
|---|---|
| #562 | SPP cold-start lockout 修正。replay で receiver_position が無いと仰角マスクで永久に固まっていた。T2/T3/N1 の km 級位置誤差を解消 |
| #563 | online PVA 候補 v1 `velocity_consistency_v1`（No-Go：N1 で位置 gate がロックアウト） |
| #566 | 候補 v2（re-anchor を追加。No-Go：RTK フィルタ側の gate） |
| #564/#565 | スマホ（Nantes Mimir、Zenodo 12566912）の気圧高さ補助（No-Go：水平 P50 +5.5%、高さは大幅改善）と複数信号対応（事後評価の参考値） |
| #567 | CLAS 受信機潮汐を MRTKLIB パリティに（`GNSS_PPP_CLAS_RECEIVER_TIDE` ほか、default OFF。t3 の p68 が合格） |
| #568 | CLAS PAR 周波数ゲート（`GNSS_PPP_CLAS_PAR_FREQ_GATE`、default OFF）。07-18 の意図的逸脱の方が fix 率で +1.65pt なので維持 |
| #569 | `rtk.cpp` の報告共分散が FLOAT/FIX とも固定値 0.01·I だったバグを修正（`reported_covariance_mode`、default OFF）。PVA 候補 v3/v4 |

## 未完了（優先順）
1. **online PVA v4 の処理時間 gate を再測定する。**
   - v4 = `--candidate velocity_consistency_v4`。contract は `docs/online_pva_candidate_v5.md`、結果は `docs/online_pva_candidate_v5_results.md`。
   - 精度と復帰の gate は全合格（同時刻の candidate-none 比で 0/558）。
   - 記録済み control 比では `processing.p95_ms` が 5 件不合格。原因はローカルホストの高負荷（別セッションの LiDAR 処理）。
   - クラウドの静かなマシンなら、**同じマシンで control（`--candidate none` を develop で実行）と v4 の両方を回し**、`scripts/analysis/compare_online_pva.py --candidate-name velocity_consistency_v4 --contract docs/online_pva_candidate_v5.md` で判定する。
   - 規模は 6 run × {normal, gnss_outage 60–70 s, imu_gap 60–64 s}。replay 同士は同時 3 本まで。
   - Go の場合でも、**default を切り替えるかはユーザーに相談**する（開発データのみで holdout なし。Tokyo2 は欠落明けの約 30 秒が v3 より悪化）。
   - ローカルでも同じ再測定を自動待機させている。どちらか先に終わった方を使えばよい。
2. **online PVA の残課題**
   - Nagoya1 は後退スタートのため heading latch が 178° ずれ、姿勢誤差が 31°。rover_gap_reset が走行中に alignStatic をやり直す問題もある。
   - Tokyo2 の IMU 欠落で姿勢誤差 18°。
   - 収束した FLOAT の共分散がまだ 3〜6 倍楽観的。
3. **CLAS**（`docs/ppc_clas_full_table.md`）
   - hard gate は 6/6 PASS。
   - soft gate の p68 は t2/n1/n2 が未達。n1/n2 は比較エポック集合とバージョン差で、共通エポックで比べれば同等以上。t2 は 86 秒の誤 integer 区間で、浮動小数の 0.05 cycle 差による chaos。打ち止めを推奨。
   - n3 の TTFF（27 s vs 9 s）も未達。

## データの入手（クラウド）
- **PPC-Dataset**（Tokyo/Nagoya 各 3 run、約 800 MB）
  - github.com/taroz/PPC-Dataset の README にある Download Link（Chiba Tech SharePoint の匿名共有）から取得する。
  - 動作確認済みの直接エンドポイント（ブラウザの UA と cookie jar を付けて curl）：
    `https://chibakoudai-my.sharepoint.com/personal/66lsm6_chibatech_ac_jp/_layouts/15/download.aspx?share=ETmyNr1VrcpFjqxgjXJdgkQBkfTwnykhSPOVKClOUBNOMQ`
  - 置き場所は `data/PPC-Dataset/{tokyo,nagoya}/run{1,2,3}/`。
- **CLAS の L6** は QZSS archive（sys.qzss.go.jp/dod/archives/clas.html）から。手順は `docs/HANDOFF_PPC_CLAS_CODEX.md` を参照。
- **MRTKLIB v0.5.1 の参照**：github.com/h-shiono/MRTKLIB。claslib testdata の `clas_grid.def` と `igs14_L5copy.atx` が必要。`rnx2rtkp` の時刻指定は `-ts 2024/07/23 01:23:00` の形式。
- `reference.csv` はすでにアンテナ位置。採点に lever arm は使わない。

## 主要コマンド
- PVA：`python3 apps/gnss.py pva-evaluate --run-dir data/PPC-Dataset/tokyo/run1 --replay-binary build/apps/gnss_pva_replay --output-dir <new> --scenario normal --candidate <name>`
- 既定パリティ：`scripts/analysis/check_pva_default_parity.py`
- C++ テスト：`run_tests`（ローカルでは gtsam の都合で `LD_LIBRARY_PATH` の指定が必要だった）
