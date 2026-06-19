# CLAS Public Validation Datasets

> **Current `develop` status (2026-06-19)**: the current CLI no longer exposes
> the historical `--claslib-parity` entry point. Native CLAS runs use
> `gnss ppp --clas-osr` directly, or `gnss clas-ppp --profile clas`, which
> forwards `--clas-osr` after expanding QZSS L6/Compact SSR inputs. The iter55
> millimetre-level table below is retained as historical reference; the active
> default-flip decision remains the A5 STOP described in
> [clas_dd_filter_a5.md](clas_dd_filter_a5.md).

This page records the public CLAS datasets used for the native CLASNAT parity
checks.  It separates accepted regression gates from public exploratory runs
that are useful for robustness work but are not yet millimetre-level passes.

## Public Source

The current CLAS parity data comes from the public CLASLIB repository:

```text
https://github.com/QZSS-Strategy-Office/claslib.git
```

The local validation copy used during iter55 was:

```text

```

Its Git remote was confirmed as:

```text
origin  https://github.com/QZSS-Strategy-Office/claslib.git
```

The files below are from that repository's `data/` directory.

## Accepted Regression Gate

| Case | Public files | Window | Native mode | Acceptance status |
| --- | --- | --- | --- | --- |
| 2019-08-27, QZSS CLAS sample | `0627239Q.obs`, `sept_2019239.nav`, `2019239Q.l6` | 2019-08-27, 16:00 GPST window | Historical iter55 `--claslib-parity` native CLASNAT path | Historical accepted gate. Iter55 measured `0.00556 m` RMS 3D against the CLASLIB trajectory over the last 100 epochs of a 2000-epoch run, and `0.01014 m` RMS 3D over the last 100 epochs of a 300-epoch smoke window. |

This is the dataset behind the historical `--claslib-parity` mm-level
regression. It is public because the input files are shipped in the public
`QZSS-Strategy-Office/claslib` repository, not because they are private local
capture files.

The same 2000-epoch iter55 run measured `0.00251 m` RMS 3D against the fixed
ECEF reference `(-3957235.3717, 3310368.2257, 3737529.7179)` over its last 100
epochs.

## Public Exploratory Runs

| Case | Public files | What was checked | Result |
| --- | --- | --- | --- |
| 2018-11-25, CLASLIB bundled sample | `0161329A.obs`, `tskc2018329.nav`, `2018328X_329A.l6` | CLASLIB reference generation and 300-epoch native CLASNAT run | CLASLIB reference solved, but native CLASNAT stayed in fallback status for the checked window. Direct trajectory diff against CLASLIB was about `1.336 m` RMS 3D over the last 100 aligned epochs, so this is not an accepted `<20 mm` gate. |
| 2019-12-15, CLASLIB bundled ST12 sample | `0627349AB.bnx`, `sept_2019349.nav`, `2019349B.l6` | CLASLIB reference from BINEX, then `convbin` to RINEX 3.02 with a GNSS signal mask before native CLASNAT | CLASLIB reference solved, but native CLASNAT did not reach the mm gate in the checked window. This remains an input-format and robustness probe, not an accuracy gate. |

No additional bundled CLASLIB public sample reached `<20 mm` native CLASNAT RMS
in iter55.  Those probes are still useful because they confirm that the
additional samples are public and reproducible, and they expose the next
robustness gaps without weakening the accepted 2019-08-27 gate.

## Reproduce The Current Native CLAS Run

Clone CLASLIB and point the commands at its public `data/` directory:

```bash
git clone https://github.com/QZSS-Strategy-Office/claslib.git /tmp/claslib
DATA=/tmp/claslib/data
```

Build the current native PPP binary:

```bash
cmake -S . -B build
cmake --build build --target gnss_ppp -j4
```

Run the current CLAS wrapper. `--profile clas` expands the raw QZSS L6 file and
forwards the resulting sampled corrections to `gnss ppp --clas-osr`:

```bash
python3 apps/gnss.py clas-ppp \
  --profile clas \
  --obs "$DATA/0627239Q.obs" \
  --nav "$DATA/sept_2019239.nav" \
  --qzss-l6 "$DATA/2019239Q.l6" \
  --qzss-gps-week 2068 \
  --out /tmp/gnsspp_2019239.pos \
  --summary-json /tmp/gnsspp_2019239_summary.json \
  --max-epochs 300
```

`gnss clas-ppp` also forwards `--antex <antennas.atx>` and
`--receiver-antenna-type <type>` to the underlying `gnss ppp` run for
receiver/satellite antenna parity probes. The public `0627239Q.obs` header does
not declare a receiver antenna type, so A4b receiver-antenna probes should pass
the native ANTEX antenna key explicitly, for example
`--receiver-antenna-type "TRM59800.80     NONE"`. The CLASLIB ANTEX file uses
CRLF line endings, so the native ANTEX loader trims trailing carriage returns
before matching labels. Set `GNSS_PPP_CLAS_RX_ANTENNA=1` to materialize those
receiver terms into the CLAS OSR `receiver_antenna_m` component and the
aggregate `PRC`/`CPC` terms. The default path leaves them unchanged while A4b
parity is measured.

For the A4b GPS L2W identity probe, enable the diagnostic gates and write a
native zero-difference code component dump:

```bash
GNSS_PPP_CLAS_DD_FILTER=1 \
GNSS_PPP_CLAS_CODE_ROW_PARITY=bias \
GNSS_PPP_CLAS_RX_ANTENNA=1 \
GNSS_PPP_CLAS_CODE_DUMP=/tmp/clas_a4b_native_code_dump.csv \
python3 apps/gnss.py clas-ppp \
  --profile clas \
  --obs "$DATA/0627239Q.obs" \
  --nav "$DATA/sept_2019239.nav" \
  --qzss-l6 "$DATA/2019239Q.l6" \
  --qzss-gps-week 2068 \
  --antex "$DATA/igs14_L5copy.atx" \
  --receiver-antenna-type "TRM59800.80     NONE" \
  --compact-code-bias-composition-policy base-only-if-present \
  --compact-code-bias-bank-policy latest-preceding-bank \
  --compact-bias-row-materialization selected-satellite-base-extend \
  --out /tmp/gnsspp_clas_a4b_native.pos \
  --summary-json /tmp/gnsspp_clas_a4b_native_summary.json \
  --max-epochs 300
```

Self-diff the dump to inspect identity provenance before comparing against a
CLASLIB-side component dump:

```bash
python3 scripts/analysis/clas_zd_component_diff.py \
  /tmp/clas_a4b_native_code_dump.csv \
  /tmp/clas_a4b_native_code_dump.csv \
  --base-label native \
  --candidate-label native \
  --row-type code \
  --json-out /tmp/clas_a4b_native_code_selfdiff.json \
  --details-csv /tmp/clas_a4b_native_code_selfdiff.csv

python3 - <<'PY'
import json
payload = json.load(open("/tmp/clas_a4b_native_code_selfdiff.json"))
print(json.dumps(payload["identity_provenance"], indent=2, sort_keys=True))
PY
```

Disposable CLASLIB dumps usually use numeric `sys`/`prn`/RTKLIB `code` fields
instead of native `G14`/`C2W`-style row keys. Normalize those dumps before using
them as the oracle side of the component diff:

```bash
python3 scripts/analysis/claslib_zd_component_export.py \
  /tmp/claslib_zd_dump.csv \
  --output /tmp/claslib_zd_dump.normalized.csv \
  --stage-label post

python3 scripts/analysis/clas_zd_component_diff.py \
  /tmp/claslib_zd_dump.normalized.csv \
  /tmp/clas_a4b_native_code_dump.csv \
  --base-label claslib \
  --candidate-label native \
  --stage post \
  --row-type code \
  --sat G14 \
  --freq 1 \
  --rinex-code C2W \
  --duplicate-policy mean \
  --json-out /tmp/clas_a4b_native_vs_claslib_code.json \
  --details-csv /tmp/clas_a4b_native_vs_claslib_code.csv
```

On the 300-epoch 2019 sample smoke, current `develop` produces 5,400 native code
rows. The GPS L2W slice has 300 rows, all with exact bias identity and exact
observation matches, and zero observation-family or code-bias fallback rows.
Use `base-only-if-present` with the latest preceding base bank for this A4b
probe: the subtype-6 network code-bias rows are replacement rows for the
subtype-4 base values, not additive deltas. On the same 300-epoch window, the
direct network policy produced periodic G14/C2W `code_bias_m` outliers such as
`-0.70 m` where CLASLIB reports `+0.76 m`. The A4b policy above restores those
rows to the base value and reduces the G14/C2W `code_bias_m` diff from
`mean_abs=0.1795 m, rms=0.4000 m, max_abs=1.4600 m` to
`mean_abs=0.1319 m, rms=0.3029 m, max_abs=0.7800 m`.

The row materialization option also extends the latest base code-bias bank onto
phase-only subtype-6 network rows. Without it, the 300-epoch G14/C2W smoke has
49/300 native rows with `code_bias_present=0` and `code_bias_m=0`; with it, the
same slice has 0/300 missing code-bias rows. On the short CLASLIB ZD oracle
window bundled with this probe, that phase-only-row extension reduces the
G14/C2W `code_bias_m` diff from `mean_abs=0.0764 m, rms=0.2295 m,
max_abs=0.7600 m` to `mean_abs=0.0091 m, rms=0.0135 m, max_abs=0.0200 m`, and
the aggregate `PRC` diff from `mean_abs=0.0915 m, rms=0.2369 m,
max_abs=0.7819 m` to `mean_abs=0.0243 m, rms=0.0266 m, max_abs=0.0419 m`. The
gated receiver-antenna materialization keeps the exact GPS L2W row on CLASLIB's
receiver-antenna slot; on the same 300-epoch G14/C2W slice this reduces
`receiver_antenna_m` from `mean_abs=0.0249 m` to `mean_abs=0.0021 m`. The
remaining top-row deltas are then dominated by residual and atmosphere terms
rather than the 30-second code-bias sign flip, phase-only code-bias gaps, or
receiver-antenna slot mismatch.

The diff JSON also includes `top_row_component_breakdowns`, which groups all
component deltas for the same ZD key. Use that field to decide whether the next
GPS L2W A4b slice is dominated by `receiver_antenna_m`, `prc_m`, atmosphere, or
another component before changing the gated correction model.

CI can regenerate the native side of this A4b probe without a pre-staged native
CSV:

```bash
python3 scripts/ci/run_clas_a4b_native_selfdiff.py
```

The wrapper sparse-checks out the public CLASLIB `data/` directory at the pinned
`GNSSPP_CLAS_A4B_CLASLIB_REF`, runs the native command above, and self-diffs the
`G14/C2W` code rows. It emits
`ci_clas_a4b_native_selfdiff_summary.json`, `native_code_dump.csv`,
`native_summary.json`, and `selfdiff.{json,csv}`. The summary uses
`status: passed` only when the native run reaches the configured epoch count,
the GPS L2W rows preserve exact observation/bias identity, fallback counts are
zero, and the self-diff has no unmatched rows or component deltas. Set
`GNSSPP_CLAS_A4B_DATA_ROOT` to use a pre-existing CLASLIB data checkout, or
`GNSSPP_CLAS_A4B_AUTO_FETCH=0` to record `blocked_infrastructure` instead of
fetching public data. CI treats that blocked state as a failure by default; set
`GNSSPP_CLAS_A4B_FAIL_ON_BLOCKED=0` only for diagnostic-only local runs.

Build with CLASLIB linked only when you need oracle-backed unit tests:

```bash
cmake -S . -B build-claslib \
  -DCLASLIB_PARITY_LINK=ON \
  -DCLASLIB_ROOT_DIR=/tmp/claslib
cmake --build build-claslib --target run_tests -j4
```

Run the oracle helper parity tests when that optional build is available:

```bash
./build-claslib/tests/run_tests --gtest_filter='*ClasnatParity*'
```

Expected helper coverage after iter55 is 17 tests, including `filter`,
`lambda`, `tropmodel`, and `stec_grid_data`.
