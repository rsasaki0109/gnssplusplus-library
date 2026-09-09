# Scripts

The scripts directory contains reproducible research, benchmark, analysis, and
CI entrypoints. Generated files belong under `output/`, which is ignored by
Git; source code and small deterministic fixtures belong in the repository.

## Layout

- `ci/`: CI wrappers, optional gates, artifact validation, and release matrices.
- `analysis/`: analysis, evaluation, row-level diff, summary, and parity tools.
- `experiments/`: reproducible experiment drivers and their small example inputs.
  - `experiments/ppp_ar/`: PPP-AR policy sweeps and lane-level fixtures.
  - `experiments/ppc/`: PPC benchmark drivers, offline selectors, replays, and reports.
- `generate_*.py`: figures, scorecards, reports, and machine-readable artifacts.
- `run_*.py`: reproducible experiment and benchmark drivers.
- `convert_*.py`: format conversion utilities.
- `dump_*.m`: MATLAB/Taroz inspection helpers kept with the relevant workflow.

Prefer adding a new script to the existing role-based group instead of creating
another top-level category. Shared application code belongs under
`apps/commands/support/`; shared standalone RTK geometry helpers belong under
`tools/`.

## Kaggle token configuration

`configure_kaggle_token.sh` stores a Kaggle access token at
`${KAGGLE_CONFIG_DIR:-$HOME/.kaggle}/access_token` without accepting a token as
a command-line argument or making a network request. In a terminal, use the
hidden double-entry prompt:

```bash
scripts/configure_kaggle_token.sh
scripts/configure_kaggle_token.sh --check
```

For automation, opt in explicitly to stdin. Do not put a literal token in a
shell command or commit it to a file:

```bash
printf '%s\n' "$KAGGLE_API_TOKEN" | scripts/configure_kaggle_token.sh --stdin
scripts/configure_kaggle_token.sh --check
```

`--force` is required to replace an existing token in non-interactive mode;
interactive mode asks for confirmation unless `--force` is supplied. The
script rejects empty values, extra lines, whitespace, and control characters,
uses mode `700` for the directory and `600` for the token file, and installs
the file through a same-directory atomic rename. `--check` validates only
existence, non-zero size, and permissions; it never authenticates over the
network.

The static checks are:

```bash
bash -n scripts/configure_kaggle_token.sh
command -v shellcheck >/dev/null && shellcheck scripts/configure_kaggle_token.sh || true
```
