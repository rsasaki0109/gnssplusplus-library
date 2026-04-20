# Contributing

## Branch and PR workflow

- Do not push new feature work directly to `develop`.
- Start from the latest `develop` and use a topic branch:
  - `feature/<topic>`
  - `fix/<topic>`
  - `docs/<topic>`
- Keep one user-visible value per PR.
- Every PR should add or tighten at least one regression, sign-off, or
  dogfooding check.

## PR size rule

- 1 PR = 1 feature or 1 operational improvement
- 1 PR = 1 clear benchmark/sign-off/test story
- Do not mix protocol ingestion, solver tuning, and docs-only cleanup in the
  same PR

## Required checks

Run the smallest relevant set and the broad regression set:

- `python3 tests/test_cli_tools.py`
- `python3 tests/test_benchmark_scripts.py`
- `python3 tests/test_packaging.py`
- `python3 tests/test_python_bindings.py -v`
- `python3 tests/test_ros2_node.py`
- `ctest --test-dir build --output-on-failure`

If the PR touches workflow or docs pipeline files, also run:

- `bash scripts/ci/run_hygiene.sh`
- `bash scripts/ci/run_cppcheck.sh`
- `python3 -m mkdocs build --strict`

CI lanes are split as follows:

- `bash scripts/ci/run_core_tests.sh` for the cross-platform gate
- `bash scripts/ci/run_extended_tests.sh` for Linux-only heavy checks such as web, packaging, and ROS2 surfaces
- `bash scripts/ci/run_optional_tests.sh` for broader Linux-only integration suites such as the full CLI and benchmark script regressions
- `bash scripts/ci/run_optional_rtk_signoffs.sh` for dataset-gated RTK sign-offs with JSON/log artifacts under `output/ci_optional_rtk_signoffs*`
- `bash scripts/ci/generate_dashboard_artifacts.sh` for the dashboard/manifest artifact path used in CI

Docs-only changes keep CI on the lightweight path: hygiene here, plus the separate Docs workflow.

## External code references

When taking ideas from external repos:

- mention the source repo in the PR body,
- describe what was adopted at the code/design level,
- add a local regression proving the imported idea still works here.

Current reference repos:

- `tomojitakasu/RTKLIB`
- `rtklibexplorer/RTKLIB`
- `JAXA-SNU/MALIB`
- `QZSS-Strategy-Office/madocalib`
- `QZSS-Strategy-Office/claslib`
- `commaai/laika`
- `tomojitakasu/PocketSDR`
- `globsky/SignalSim`
