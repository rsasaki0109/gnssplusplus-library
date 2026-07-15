# CLAS Brush-up Baseline (2026-07-14)

This note freezes the behavior and evidence contracts used by the CLAS
responsibility-splitting work. The brush-up phase must not change these
results unless a later, explicitly-scoped model PR updates the baseline.

## Source

- Commit: `1d27bc3` (`develop`)
- Worktree: clean before the baseline run

## Lightweight regression gates

```bash
python3 -m pytest -q \
  tests/test_clas_a4b_native_selfdiff.py \
  tests/test_clas_zd_component_diff.py \
  tests/test_clas_zd_component_summary.py \
  tests/test_claslib_osr_zd_export.py \
  tests/test_claslib_zd_component_export.py \
  tests/test_optional_clas_zd_component_diff.py \
  tests/test_cli_tools.py -k 'qzss_l6 or clas_ppp'
```

Result: `47 passed, 16 skipped, 208 deselected`.

```bash
./build/tests/run_tests --gtest_filter='*Clas*:*CLAS*'
```

Result: `36 passed` from six test suites.

## Public-data A4b evidence gate

```bash
python3 scripts/ci/run_clas_a4b_native_selfdiff.py
```

Result: `passed`.

| Metric | Baseline |
| --- | ---: |
| Native epochs | 300 |
| Native PPP solution rate | 100.0% |
| Native code-dump rows | 6550 |
| GPS L2W exact observation rows | 300 |
| GPS L2W observation fallback rows | 0 |
| GPS L2W exact bias rows | 300 |
| GPS L2W bias fallback rows | 0 |
| Self-diff common rows | 300 |
| Self-diff compared components | 16080 |
| Self-diff maximum absolute delta | 0.0 m |
| Self-diff unmatched rows | 0 |

Stable generated-artifact checksums for this source revision:

- `native_code_dump.csv`: `0d767bc50c9e170cbb6bad4d9c14716c20448c0b93434c64c4bcfd5af0689427`
- `selfdiff.json`: `11fccc68ef1d84668a145815034161d647d44dadd82ef4fca2bf270994d7bb7b`

The top-level CI summary embeds absolute artifact paths, so its own checksum is
not a portable regression contract. Compare its status and metrics instead.
