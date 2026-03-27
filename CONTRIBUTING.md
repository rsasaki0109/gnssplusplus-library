# Contributing

## Branch And PR Workflow

- Do not push new feature work directly to `develop`.
- Start from the latest `develop` and use a topic branch:
  - `feature/<topic>`
  - `fix/<topic>`
  - `docs/<topic>`
- Keep one user-visible value per PR.
- Every PR should add or tighten at least one regression, sign-off, or dogfooding check.

## PR Size Rule

Use this rule when splitting work:

- 1 PR = 1 feature or 1 operational improvement
- 1 PR = 1 clear benchmark/sign-off/test story
- Do not mix protocol ingestion, solver tuning, and docs-only cleanup in the same PR

Good examples:

- `feature/laika-fetch-products`
- `feature/laika-ppp-pipeline`
- `feature/rtklibexplorer-low-cost-tuning`
- `feature/rtklibexplorer-stream-ux`
- `feature/ppc-signoff-expansion`

Bad examples:

- `feature/external-repos`
- `feature/ppp-and-stream-and-docs`

## Required Checks

Before opening a PR, run the smallest relevant set and the broad regression set.

Minimum relevant checks:

- `python3 tests/test_cli_tools.py`
- `python3 tests/test_benchmark_scripts.py`
- `python3 tests/test_packaging.py`
- `python3 tests/test_python_bindings.py -v`
- `python3 tests/test_ros2_node.py`
- `ctest --test-dir build --output-on-failure`

If the PR touches only a subset, include the focused command you used in the PR body.

## External Code References

External repos are references, not vendored dependencies. When taking ideas from them:

- mention the source repo in the PR body
- describe what was adopted at the code/design level
- add a local regression proving the imported idea is still working here

Current reference repos used for code/design comparison:

- `tomojitakasu/RTKLIB`
- `rtklibexplorer/RTKLIB`
- `JAXA-SNU/MALIB`
- `commaai/laika`

## Sign-off Rule

If a command has a `*-signoff` variant, prefer using it in PR validation.

Examples:

- `gnss short-baseline-signoff`
- `gnss rtk-kinematic-signoff`
- `gnss ppp-static-signoff`
- `gnss ppp-kinematic-signoff`
- `gnss odaiba-benchmark --require-*`

Sign-off commands are the preferred quality gates for solver changes.
