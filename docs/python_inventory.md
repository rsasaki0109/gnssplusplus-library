# Python棚卸しと言語比率の是正

対象: `gnssplusplus-library` (`output/`, `build*`, `__pycache__`除外)

## 1. 総括

| 言語 | 追跡ファイル数 | LOC | バイト |
|---|---|---|---|
| Python `.py` | 971 | ~339k | 15,243,768 |
| C++ `.cpp` | 308 | ~189k | 8,677,385 |
| C++ `.hpp`/`.h` | 187 | ~42k | 1,841,537 |
| C++ 合計 | 495 | ~231k | 10,518,922 |

## 2. 判明した事実（当初計画の訂正）

- CLI登録数は `apps/gnss.py` の `COMMANDS` 辞書で決まる。現在147コマンド。ファイル数を減らしてもCLI登録数は減らない（当初の「330→98」は誤り）。
- `apps/commands/benchmarks` 281ファイル中、未登録は200。ただし**完全な死にコードは0**。全て兄弟実験・tests・docsレコードから参照される。
- 主要な肥大要因は smartphone phase 実験家系:
  - `apps/commands/benchmarks/gnss_smartphone_phase*.py` 169ファイル (~5.40MB)
  - `tests/test_smartphone_phase*.py` 176ファイル (~0.93MB)
  - `scripts/analysis/**` (~0.80MB), `scripts/experiments/**` (~0.59MB)

## 3. 物理移設が不可である理由

phaseモジュールは凍結記録 (`docs/use_cases/records/*.json`) とモジュール内で
自身・兄弟モジュールの **ソースSHA256を検証** している。移設に伴いパス
ブートストラップ (`parents[3]` → `parents[2]` 等) や `apps/commands/benchmarks`
参照を書き換えると、ファイル内容が変わりハッシュ検証が失敗する
（例: `Phase199 evaluator hash changed`）。provenanceを保つにはバイト不変が必須
のため、物理移設は行わない。

## 4. 採用した是正: Linguist除外

`.gitattributes` で実験家系をGitHub Linguistの言語統計から除外:

```
apps/commands/benchmarks/gnss_smartphone_phase*.py linguist-vendored
tests/test_smartphone_phase*.py                   linguist-vendored
scripts/analysis/**    linguist-vendored
scripts/experiments/** linguist-vendored
```

結果（バイト）:

| | 値 |
|---|---|
| Python 集計対象 | 7,524,838 |
| Python 除外 | 7,718,930 |
| C++ 集計対象 | 10,518,922 |

**C++が第1言語（+約3.0MB差）**。ランタイム・テストには一切影響なし
（代表phaseテスト21件で `OK` を確認）。

## 5. 今後の運用

- 新規phase実験は `scripts/experiments/` 配下に置き、`COMMANDS` へ登録しない。
- 凍結対象モジュールは編集禁止。変更が必要なら新phaseとして追加し、
  旧ファイルはバイト不変のまま残す。
- `apps/commands/` の登録コマンドは現状147。追加はレビュー必須。
