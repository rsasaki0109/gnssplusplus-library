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

## External code references

When taking ideas from external repos:

- mention the source repo in the PR body,
- describe what was adopted at the code/design level,
- add a local regression proving the imported idea still works here.

Current reference repos:

- `tomojitakasu/RTKLIB`
- `rtklibexplorer/RTKLIB`
- `JAXA-SNU/MALIB`
- `commaai/laika`
