# Scripts

The scripts directory contains reproducible research, benchmark, analysis, and
CI entrypoints. Generated files belong under `output/`, which is ignored by
Git; source code and small deterministic fixtures belong in the repository.

## Layout

- `ci/`: CI wrappers, optional gates, artifact validation, and release matrices.
- `analysis/`: focused row-level diff, summary, and parity analysis tools.
- `analyze_*.py`: read-only analysis of existing solver or experiment output.
- `evaluate_*.py`: policy, integrity, or holdout evaluation.
- `generate_*.py`: figures, scorecards, reports, and machine-readable artifacts.
- `run_*.py`: reproducible experiment and benchmark drivers.
- `convert_*.py`: format conversion utilities.
- `dump_*.m`: MATLAB/Taroz inspection helpers kept with the relevant workflow.

Prefer adding a new script to the existing role-based group instead of creating
another top-level category. Shared application code belongs under
`apps/commands/support/`; shared standalone RTK geometry helpers belong under
`tools/`.
