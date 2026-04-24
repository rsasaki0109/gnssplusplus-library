# CLAS Public Validation Datasets

> **Implementation status on `develop`**: the native CLASNAT path that produced
> these numbers is fully implemented on branch
> [`codex/ship-of-theseus-20260418`](https://github.com/rsasaki0109/gnssplusplus-library/tree/codex/ship-of-theseus-20260418).
> That branch has no common ancestor with `develop` and needs an
> "Allow unrelated histories" merge or an incremental re-port.

This page records the public CLAS datasets used for the native CLASNAT parity
checks.  It separates accepted regression gates from public exploratory runs
that are useful for robustness work but are not yet millimetre-level passes.

## Public Source

The current CLAS parity data comes from the public CLASLIB repository:

```text
https://github.com/QZSS-Strategy-Office/claslib.git
```

For local validation, set `CLASLIB_ROOT` to a checkout of that repository.

Its Git remote should be:

```text
origin  https://github.com/QZSS-Strategy-Office/claslib.git
```

The files below are from that repository's `data/` directory.

## Accepted Regression Gate

| Case | Public files | Window | Native mode | Acceptance status |
| --- | --- | --- | --- | --- |
| 2019-08-27, QZSS CLAS sample | `0627239Q.obs`, `sept_2019239.nav`, `2019239Q.l6` | 2019-08-27, 16:00 GPST window | `gnss_ppp --claslib-parity` default native CLASNAT path | Accepted gate. Iter55 measured `0.00556 m` RMS 3D against the CLASLIB trajectory over the last 100 epochs of a 2000-epoch run, and `0.01014 m` RMS 3D over the last 100 epochs of a 300-epoch smoke window. |

This is the dataset behind the current `--claslib-parity` mm-level regression.
It is public because the input files are shipped in the public
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

## Reproduce The Main Public Gate

Clone CLASLIB and point the commands at its public `data/` directory:

```bash
git clone https://github.com/QZSS-Strategy-Office/claslib.git /tmp/claslib
DATA=/tmp/claslib/data
```

Build with CLASLIB linked when you want the oracle-backed unit tests and bridge
available:

```bash
cmake -S . -B build \
  -DCLASLIB_PARITY_LINK=ON \
  -DCLASLIB_ROOT_DIR=/tmp/claslib
cmake --build build --target gnss_ppp -j4
cmake --build build --target run_tests -j4
```

Run the native CLASNAT parity path:

```bash
./build/apps/gnss_ppp \
  --claslib-parity \
  --obs "$DATA/0627239Q.obs" \
  --nav "$DATA/sept_2019239.nav" \
  --ssr "$DATA/2019239Q.l6" \
  --out /tmp/gnsspp_2019239.pos \
  --summary-json /tmp/gnsspp_2019239_summary.json \
  --ref-x -3957235.3717 \
  --ref-y 3310368.2257 \
  --ref-z 3737529.7179 \
  --max-epochs 300 \
  --quiet
```

Run the oracle helper parity tests:

```bash
./build/tests/run_tests --gtest_filter='*ClasnatParity*'
```

Expected helper coverage after iter55 is 17 tests, including `filter`,
`lambda`, `tropmodel`, and `stec_grid_data`.
