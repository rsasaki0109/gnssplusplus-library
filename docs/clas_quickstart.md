# CLAS Quick Start

This page is the shortest practical entrypoint for running the current
`CLAS/MADOCA` path in `libgnss++`.

Use this page when you want to:

- run a parity-oriented CLAS PPP solve,
- inspect `L6 / Compact SSR` rows,
- compare `libgnss++` output against a `CLASLIB` answer.

If you want internal boundaries and module ownership, use
[CLAS API & Flow](clas.md) instead.

## 1. Build the solver

```bash
cmake -S . -B build
cmake --build build --target gnss_ppp -j4
```

## 2. Run the strict CLAS parity preset

The thinnest entrypoint is the parity wrapper:

```bash
GNSS_PPP=./build/gnss_ppp \
./scripts/run_gnss_ppp_claslib_parity.sh \
  --obs <rover.obs> \
  --nav <broadcast.nav> \
  --ssr <corrections.l6-or-expanded.csv> \
  --out parity.pos \
  --summary-json parity_summary.json \
  --ref-x <ecef_x> \
  --ref-y <ecef_y> \
  --ref-z <ecef_z>
```

What this gives you:

- `parity.pos`
- `parity_summary.json`
- debug stderr if `GNSS_PPP_DEBUG=1`

## 3. Inspect raw Compact SSR or expanded rows

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

## 4. Compare against a CLASLIB answer

If you already have a `CLASLIB` reference `.pos`, compare it with:

```bash
./scripts/benchmark_fair_vs_claslib.sh parity.pos \
  <claslib_answer.pos> \
  <ref_x> <ref_y> <ref_z>
```

This produces benchmark JSONs for:

- `libgnss++` vs truth/reference
- `CLASLIB` vs truth/reference
- direct trajectory diff

## 5. Turn on debug only when the boundary is unclear

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

## 6. Know which knobs are stable

The most important stable CLAS-facing knobs on `gnss ppp` are:

- `--claslib-parity`
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

## 7. When to use which page

- Use this page for the first run: [CLAS Quick Start](clas_quickstart.md)
- Use boundary docs next: [CLAS API & Flow](clas.md)
- Use policy docs when `Compact SSR` rows look wrong: [CLAS Compact SSR Policies](clas_compact_ssr_policies.md)
- Use artifact docs when comparing parity runs: [CLAS Parity Datasets & Artifacts](clas_parity_artifacts.md)
- Use stderr tag triage when logs are noisy: [CLAS Debug Tag Playbook](clas_debug_playbook.md)
- Use issue-level status when parity is still unresolved: [CLAS Parity Blockers](clas_parity_blockers.md)
