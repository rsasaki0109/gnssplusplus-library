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
`examples/ppp_static.toml` and `examples/clas_kinematic.toml` are ready-to-run
`gnss_ppp` profiles for static PPP and CLAS PPP-RTK kinematic positioning.

`examples/station.example.toml` is the operational entry point for a
long-running rover/base RTK session. It uses `[station]` plus
`[station.receiver]`, validates local inputs and the `gnss_live` binary, then
creates a timestamped run directory containing `run.json`, `status.json`, the resolved
receiver config, the live log, and the solution file. Use:

```bash
gnss station check --config configs/examples/station.example.toml
gnss station start --config configs/examples/station.example.toml
gnss station status --config configs/examples/station.example.toml --wait-seconds 5
gnss station stop --config configs/examples/station.example.toml
```

The status command resolves the most recent run through `latest.json`. URI
endpoints such as NTRIP, TCP, and serial inputs are checked as endpoints;
ordinary paths are resolved relative to the TOML file. Credentials are
redacted in JSON output and run manifests, while the generated private
receiver config remains readable only by the launching user where supported.

Camera fixtures and other non-configuration assets live under `tests/fixtures/`.
