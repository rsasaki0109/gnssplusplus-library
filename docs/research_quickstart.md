# Research Quick Start

This guide is for GNSS researchers who want a reproducible experiment surface
instead of a one-off positioning run.

The target workflow is:

1. pin a dataset and command line,
2. emit a solution plus summary JSON,
3. compare metrics against another solver, profile, or receiver path,
4. archive the exact artifacts.

## Build once

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
```

Confirm the dispatcher can see the command set:

```bash
python3 apps/gnss.py --help
```

## Reproduce a bounded RTK run

The bundled PPC-Dataset subset includes rover observations, base observations,
broadcast navigation, IMU logs, and reference truth. Start with a bounded run
while iterating:

```bash
mkdir -p output/research

python3 apps/gnss.py ppc-rtk-signoff \
  --dataset-root data/PPC-Dataset \
  --city tokyo \
  --run run1 \
  --preset low-cost \
  --ratio 2.4 \
  --max-epochs 1000 \
  --out output/research/ppc_tokyo_run1_rtk.pos \
  --summary-json output/research/ppc_tokyo_run1_rtk.json
```

The `.pos` file is the trajectory. The `.json` file is the experiment record:
it captures metrics, solver settings, wall time, reference matching, and
threshold results.

## Add explicit thresholds

When promoting an experiment result, add requirements so the command fails if a
future change weakens the claim:

```bash
python3 apps/gnss.py ppc-rtk-signoff \
  --dataset-root data/PPC-Dataset \
  --city tokyo \
  --run run1 \
  --preset low-cost \
  --ratio 2.4 \
  --summary-json output/research/ppc_tokyo_run1_gate.json \
  --require-positioning-rate-min 85 \
  --require-fix-rate-min 50 \
  --require-realtime-factor-min 1
```

Keep threshold sets in `configs/*.toml` once they become long-lived. For
example:

```bash
python3 apps/gnss.py ppc-rtk-signoff \
  --config-toml configs/ppc_rtk_signoff.example.toml
```

## Compare solver profiles

Use the same input data and vary one policy at a time:

```bash
python3 apps/gnss.py ppc-rtk-signoff \
  --dataset-root data/PPC-Dataset \
  --city tokyo \
  --run run1 \
  --preset low-cost \
  --ratio 3.0 \
  --summary-json output/research/tokyo_run1_ratio3.json

python3 apps/gnss.py ppc-rtk-signoff \
  --dataset-root data/PPC-Dataset \
  --city tokyo \
  --run run1 \
  --preset low-cost \
  --ratio 2.4 \
  --summary-json output/research/tokyo_run1_ratio2p4.json
```

Use [RTK Tuning Gates](rtk_tuning_gates.md) for the current PPC guard and
ambiguity-resolution controls, and [Benchmarks](benchmarks.md) for the checked
public summary tables.

## Python inspection

The Python package exposes RINEX inspection, file-based solves, coordinate
helpers, and `.pos` statistics. After building, point `PYTHONPATH` at the build
and source package directories:

```bash
PYTHONPATH="$PWD/build/python:$PWD/python" python3 - <<'PY'
from libgnsspp import load_solution, read_rinex_header

header = read_rinex_header("data/PPC-Dataset/tokyo/run1/rover.obs")
print(header["marker_name"], header["receiver_type"])

solution = load_solution("output/research/ppc_tokyo_run1_rtk.pos")
stats = solution.statistics()
print("epochs", solution.size())
print("availability", stats.availability_rate)
print("fix_rate", stats.fix_rate)
print("last_status", solution.last_solution().status_name)
PY
```

For notebooks, keep the command execution outside the notebook or write the
exact command into the notebook output. The important point is that every
figure can be traced back to a `.pos`, `.json`, and command line.

## Experiment lane

PPP-AR and CLAS policy sweeps live under `experiments/ppp_ar/` and use shared
TOML inputs:

```bash
python3 experiments/ppp_ar/run_experiments.py \
  --config experiments/ppp_ar/input.example.toml
```

Use the experiment lane when comparing strategy arms. Promote only the winning
behavior into the stable CLI after it has a sign-off or regression.

## Artifact contract

Research runs should leave this minimum artifact set:

| Artifact | Purpose |
|---|---|
| `solution.pos` | Solver output over time |
| `summary.json` | Metrics, settings, thresholds, provenance |
| `comparison.csv` | Epoch-level or segment-level deltas when comparing solvers |
| `plot.png` or `kml` | Human inspection |
| command text or TOML config | Reproduction path |

Prefer one artifact directory per experiment, for example
`output/research/<dataset>/<profile>/`.

## Debugging runtime and gates

For a reproducible robotics-style runtime/debug run, use the full smoke profile:

```bash
python3 apps/gnss.py robotics-smoke \
  --profile full \
  --out-dir output/research/robotics_full
```

The resulting summary JSON includes the exact dispatched command,
`robotics_smoke_profile`, `robotics_smoke_status`,
`robotics_smoke_failure_reasons`, and `robotics_smoke_thresholds`. This makes
failed gates inspectable without scraping terminal logs.

Open the same artifact in the web UI:

```bash
python3 apps/gnss.py web --port 8085 --root .
```

The robotics panel links the summary, `.pos`, reference, run directory,
rover/base/nav inputs, threshold comparisons, and failure reasons. Use that
view when comparing machines, compiler settings, RTK profiles, or dataset
subsets.

## Where to go next

- [Dataset Gallery](dataset_gallery.md) lists the public dataset lanes.
- [Validation](validation.md) lists the sign-off commands and threshold style.
- [Reference Analyses](references/index.md) records upstream comparison notes.
