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
- Use the `Development slice` issue template before starting larger work. It
  should state the one user-visible value, the one regression/sign-off
  improvement, scope boundaries, and explicit non-goals.
- Fill the PR template's behavior-boundary fields when runtime defaults,
  env/config gates, artifact schemas, datasets, or credentials are involved.

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
- `bash scripts/ci/run_optional_rtk_signoffs.sh` for dataset-gated RTK,
  PPC coverage-matrix schema smoke, SCORPION, and long PPC taroz FGO sign-offs
  with JSON/log artifacts under `output/ci_optional_rtk_signoffs*`; the summary
  uses `summary_schema: ci_optional_rtk_signoffs.v1` and a passed step fails the
  lane if any declared output artifact is missing
- `bash scripts/ci/run_optional_clas_zd_component_diff.sh`,
  `bash scripts/ci/run_optional_madoca_materialization_diff.sh`, and
  `bash scripts/ci/run_optional_madoca_residual_component_diff.sh` for
  oracle/native diff artifacts; they report `blocked_infrastructure` with
  next actions when their CSV inputs are unavailable, require a summary/log
  artifact in CI, and fail a passed diff command if the declared JSON/CSV
  artifacts are missing
- `python3 scripts/ci/run_clas_a4b_native_selfdiff.py` for the public CLAS A4b
  native-side evidence bundle; it sparse-fetches CLASLIB public data unless
  `GNSSPP_CLAS_A4B_DATA_ROOT` is supplied, generates the native code dump, and
  self-diffs the `G14/C2W` rows before any oracle-backed model change
- `bash scripts/ci/generate_dashboard_artifacts.sh` plus
  `python3 scripts/ci/validate_artifact_manifest_contract.py output/artifact_manifest.json`
  for the dashboard/manifest artifact path used in CI; the validator requires
  schema version `artifact_manifest.v1`, matching bundle counts, root-confined
  artifact links, non-empty local files, and parseable JSON/CSV/PNG payloads

Docs-only changes keep CI on the lightweight path: hygiene here, plus the separate Docs workflow.

Keep taroz FGO validation split by cost:

- Light unit coverage belongs in CTest, such as `python_ppc_taroz_amb_pdc_smoke_tests`
  dry-run checks, dogfood argument/summary smoke tests, and focused C++ factor
  tests.
- Optional artifact parity tests also live in CTest. They must skip cleanly
  when local `output/dogfood/...` MATLAB-oracle artifacts are absent, and they
  become strict checks after a developer regenerates those artifacts.
- Heavy local checks include
  `python3 apps/gnss.py taroz-oracle-suite --native-bin-dir build/apps`
  for the consolidated taroz MATLAB-oracle gate,
  `python3 apps/gnss.py taroz-pc-dogfood --generate-matlab-dump`
  for the PC ambiguity mode,
  `python3 apps/gnss.py taroz-observable-dogfood --mode pos-pdc --generate-matlab-dump`
  for single-receiver observable modes,
  `python3 apps/gnss.py taroz-pos-vel-amb-pdc-dogfood --generate-matlab-dump`
  when MATLAB and the taroz checkout are available,
  `python3 apps/gnss.py ppc-taroz-amb-pdc-smoke --max-epochs 200 --generate-spp-seed`,
  `python3 apps/gnss.py ppc-taroz-amb-pdc-smoke --run nagoya/run3 --max-epochs 1000 --generate-spp-seed`,
  and optional MATLAB/taroz parity tests when their external artifacts are present.
- Taroz final-output contract tests should be updated with every new dogfood
  output surface. At minimum, tie the final CSV/`.pos` rows to summary counts,
  graph value counts, optimizer cost fields, GPS week/TOW ordering, and finite
  state columns.
- External PPC-Dataset runs are dataset-gated and should not be required for the
  cross-platform CI lane unless a dedicated optional job provides the data.

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
