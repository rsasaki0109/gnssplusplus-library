# Configuration files

Checked-in configuration files are grouped by purpose:

- `examples/`: small runtime examples for interactive commands.
- `signoff/`: reusable acceptance criteria for operational sign-off workflows.
- `benchmarks/`: reproducible experiment, parity, and benchmark profiles.

Files ending in `.example.*` are templates. Copy them before adding local paths
or credentials. Benchmark profiles without the `.example` marker are pinned
reproduction inputs and should only change with their documented acceptance
criteria.

`examples/fuse.example.toml` is the compact configuration surface for
`gnss fuse --config`. Command-line values override its `[gnss_fuse]` defaults.
`examples/solve.example.toml` provides the same pattern for `gnss solve` and
its `[gnss_solve]` table.

Camera fixtures and other non-configuration assets live under `test_data/`.
