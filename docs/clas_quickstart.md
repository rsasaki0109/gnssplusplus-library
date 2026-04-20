# CLAS Quick Start

This page is the shortest practical entrypoint for running the current
`CLAS/MADOCA` path in `libgnss++`.

Use this page when you want to:

- run a parity-oriented CLAS PPP solve,
- inspect `L6 / Compact SSR` rows,
- compare `libgnss++` output against a `CLASLIB` answer.

If you want internal boundaries and module ownership, use
[CLAS API & Flow](clas.md) instead.

## 1. Get the public CLAS sample data

The current public parity gate uses sample data shipped in the public CLASLIB
repository:

```bash
git clone https://github.com/QZSS-Strategy-Office/claslib.git /tmp/claslib
DATA=/tmp/claslib/data
```

The accepted 2019 case uses:

- `$DATA/0627239Q.obs`
- `$DATA/sept_2019239.nav`
- `$DATA/2019239Q.l6`

## 2. Build the solver

```bash
cmake -S . -B build \
  -DCLASLIB_PARITY_LINK=ON \
  -DCLASLIB_ROOT_DIR=/tmp/claslib
cmake --build build --target gnss_ppp -j4
cmake --build build --target run_tests -j4
```

`CLASLIB_PARITY_LINK=ON` is only needed for CLASLIB bridge/oracle tests.  The
native CLASNAT runtime path itself remains native.

## 3. Run the native CLAS parity preset

The thinnest entrypoint is the parity wrapper:

```bash
GNSS_PPP=./build/apps/gnss_ppp \
./scripts/run_gnss_ppp_claslib_parity.sh \
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

What this gives you:

- `/tmp/gnsspp_2019239.pos`
- `/tmp/gnsspp_2019239_summary.json`
- debug stderr if `GNSS_PPP_DEBUG=1`

`--claslib-parity` now uses the native CLASNAT transcription path by default.
`--ported-clasnat` is still accepted but redundant.  Add
`--legacy-strict-parity` or `--no-ported-clasnat` only when intentionally
re-running the older strict CLAS OSR path for regression comparison.

## 4. Verify the oracle-backed helper parity tests

```bash
./build/tests/run_tests --gtest_filter='*ClasnatParity*'
```

The iter55 helper contract is 17 passing tests, including native parity for
`filter`, `lambda`, `tropmodel`, and `stec_grid_data`.

## 5. Compare against a CLASLIB answer

If you already have a `CLASLIB` reference `.pos`, compare it with:

```bash
./scripts/benchmark_fair_vs_claslib.sh /tmp/gnsspp_2019239.pos \
  <claslib_answer.pos> \
  -3957235.3717 3310368.2257 3737529.7179
```

This produces benchmark JSONs for:

- `libgnss++` vs truth/reference
- `CLASLIB` vs truth/reference
- direct trajectory diff

The public accepted dataset list and exploratory public probes are maintained
on [CLAS Public Validation Datasets](clas_validated_datasets.md).

## 6. Inspect raw Compact SSR or expanded rows

To inspect the transport side before PPP:

```bash
python3 apps/gnss.py qzss-l6-info --help
```

Typical use cases:

- raw `QZSS L6` inspection
- expanded sampled row generation
- compact merge-policy experiments

When the question is "did the correction rows themselves already go wrong?",
start here before reading PPP logs.

## 7. Turn on debug only when the boundary is unclear

Use:

```bash
GNSS_PPP_DEBUG=1
```

Then read the tags in this order:

1. `[SSR-BIAS-SEL]`
2. `[OSR]`
3. `[CLAS-PHASE-ROW]`
4. `[CLAS-AMB-SEED]`
5. `[CLAS-AMB-OBS]`
6. `[PPP-WLNL-COMP]`
7. `[CLAS-WLNL-FIX]`

The full map lives on [CLAS Debug Tag Playbook](clas_debug_playbook.md).

## 8. Know which knobs are stable

The most important stable CLAS-facing knobs on `gnss ppp` are:

- `--claslib-parity`
- `--ported-clasnat`
- `--legacy-strict-parity`
- `--no-ported-clasnat`
- `--clas-epoch-policy`
- `--clas-osr-application`
- `--clas-phase-continuity`
- `--clas-phase-bias-values`
- `--clas-phase-bias-reference-time`
- `--clas-ssr-timing`
- `--clas-expanded-values`
- `--clas-subtype12-values`
- `--clas-residual-sampling`
- `--clas-atmos-selection`
- `--clas-atmos-stale-after-seconds`

The broader compact merge/composition policy surface is documented on
[CLAS Compact SSR Policies](clas_compact_ssr_policies.md).

## 9. When to use which page

- Use this page for the first run: [CLAS Quick Start](clas_quickstart.md)
- Use boundary docs next: [CLAS API & Flow](clas.md)
- Use public data status next: [CLAS Public Validation Datasets](clas_validated_datasets.md)
- Use policy docs when `Compact SSR` rows look wrong: [CLAS Compact SSR Policies](clas_compact_ssr_policies.md)
- Use artifact docs when comparing parity runs: [CLAS Parity Datasets & Artifacts](clas_parity_artifacts.md)
- Use stderr tag triage when logs are noisy: [CLAS Debug Tag Playbook](clas_debug_playbook.md)
- Use issue-level status when parity is still unresolved: [CLAS Parity Blockers](clas_parity_blockers.md)
