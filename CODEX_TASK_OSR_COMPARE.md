# Codex タスク: CLASLIB ssr2osr ビルド + OSR 数値比較

## 目的

CLASLIB と libgnss++ の OSR（観測空間補正）を同一衛星・同一エポックで数値比較し、
float 精度の差（libgnss++ 3.78m vs CLASLIB 0.002m）の原因を特定する。

## Task 1: CLASLIB ssr2osr を Linux でビルド

```bash
cd /media/sasaki/aiueo/ai_coding_ws/gnssplusplus_thesis_ws/data/clas/claslib/util/ssr2osr
cat makefile  # ビルド方法を確認
```

- ソース: `ssr2osr.c` + `../../src/*.c`
- makefile を確認して Linux 用に修正（gcc、パス修正）
- ビルドして `ssr2osr` バイナリを作成
- 設定ファイル `sample.conf` を Linux パスに修正

## Task 2: CLASLIB ssr2osr を 2019 データで実行

```bash
./ssr2osr -k <config> \
  ../../data/0627239Q.obs \
  ../../data/sept_2019239.nav \
  ../../data/2019239Q.l6 \
  -o /tmp/claslib_osr_dump.csv
```

出力にどのカラムがあるか確認。trop, iono, cbias, pbias, PRC, CPC 等。

## Task 3: libgnss++ の OSR をダンプ

libgnss++ の OSR 値は `GNSS_PPP_DEBUG=1` で `[OSR]` ログに出る。
2エポック目（tow=230421）の G14, G25, G26 の値:
- trop, rel, iono_l1, cbias0, pbias0, windup, orb_los, clk_corr, PRC0, CPC0

これを CLASLIB の同じエポック・同じ衛星の値と比較。

## Task 4: 差分分析

各補正項の差（メートル）を計算:
- trop: libgnss++ vs CLASLIB
- iono: libgnss++ vs CLASLIB
- cbias: libgnss++ vs CLASLIB
- pbias: libgnss++ vs CLASLIB
- orbit_correction: libgnss++ vs CLASLIB
- clock_correction: libgnss++ vs CLASLIB
- PRC: libgnss++ vs CLASLIB
- CPC: libgnss++ vs CLASLIB

差が最も大きい項目を特定して報告。

## 制約
- コードは変更しない（調査のみ）
- CLASLIB のソースは `/media/sasaki/aiueo/ai_coding_ws/gnssplusplus_thesis_ws/data/clas/claslib/src/`
- データは `/media/sasaki/aiueo/ai_coding_ws/gnssplusplus_thesis_ws/data/clas/claslib/data/`
- libgnss++ は `/media/sasaki/aiueo/ai_coding_ws/rtklib_v2_ws/gnssplusplus-library/`
