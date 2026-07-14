# Configuration files

Checked-in configuration files are grouped by purpose:

- `examples/`: small runtime examples for interactive commands.
- `signoff/`: reusable acceptance criteria for operational sign-off workflows.
- `benchmarks/`: reproducible experiment, parity, and benchmark profiles.

Files ending in `.example.*` are templates. Copy them before adding local paths
or credentials. Benchmark profiles without the `.example` marker are pinned
reproduction inputs and should only change with their documented acceptance
criteria.

Camera fixtures and other non-configuration assets live under `test_data/`.
