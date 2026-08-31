# Native smartphone FGO optimizer-stop experiment

This is a development-only No-Go experiment. The native-FGO v1 graph,
initialization, Huber loss, measurement selection, noise values, and output
format were frozen. The only candidate change was optimizer stopping:

| lane | max iterations | relative cost threshold | absolute cost threshold |
| --- | ---: | ---: | ---: |
| `baseline8` | 8 | 0 | 0 |
| `candidate50` | 50 | `1e-6` | 0 |

The candidate values follow the installed GTSAM
`LevenbergMarquardtParams::CeresDefaults` (`maxIterations=50`,
`relativeErrorTol=1e-6`, `absoluteErrorTol=0`). Gradient and step tolerances
were not added because the Eigen backend does not expose them. The split was
selected from archive central-directory metadata before payload materialization;
the truth-use inventory is
`docs/use_cases/records/smartphone_r5_gsdc2023_native_fgo_optimizer_truth_use_inventory_v1.json`.

## Reproduction

The freeze and its manifest are:

- `docs/use_cases/records/smartphone_r5_gsdc2023_native_fgo_optimizer_stop_freeze_v1.json`
- `docs/use_cases/records/smartphone_r5_gsdc2023_native_fgo_optimizer_stop_freeze_v1_manifest.json`

Run only the frozen train truth-free stage with:

```sh
env PYTHONHASHSEED=0 \
  LD_LIBRARY_PATH=/home/sasaki/.local/lib:/opt/ros/jazzy/lib \
  python3 apps/commands/benchmarks/gnss_smartphone_native_fgo_optimizer_stop_eval.py \
  truth-free-run --output-root output/smartphone-r5/native-fgo-optimizer-stop-v1
```

The route artifacts are sealed in
`output/smartphone-r5/native-fgo-optimizer-stop-v1`. The first scoring attempt
exposed an orchestration-only temporary-directory path in the manifests. The
recovery normalized manifest paths without touching solver bytes, and the
scorer accepted the first epoch emitted by the frozen `--skip-epochs 0`
command. The recovery command was:

```sh
env PYTHONHASHSEED=0 python3 \
  apps/commands/benchmarks/gnss_smartphone_native_fgo_optimizer_stop_eval.py \
  repair-truth-free-manifests --output-root output/smartphone-r5/native-fgo-optimizer-stop-v1
```

The final train score was then run through the explicit recovery mode. It
materialized three declared train truth members; the first was reused after
the recorded scorer failure, so the total truth-file read count was four
(three in the final evaluation plus one prior recovery read). No validation or
future holdout truth was materialized or opened.

```sh
env PYTHONHASHSEED=0 \
  LD_LIBRARY_PATH=/home/sasaki/.local/lib:/opt/ros/jazzy/lib \
  python3 apps/commands/benchmarks/gnss_smartphone_native_fgo_optimizer_stop_eval.py \
  train-score-recovery --output-root output/smartphone-r5/native-fgo-optimizer-stop-v1
```

## Result

The machine-readable result is
`docs/use_cases/records/smartphone_r5_gsdc2023_native_fgo_optimizer_stop_result.json`.
The candidate lowered aggregate H-P95 from 5.60945 m to 5.45829 m and the
four-diagnostic mean from 4.27561 m to 4.21703 m, while availability and truth
coverage were unchanged. It regressed aggregate H-P50 from 2.94741 m to
2.97757 m, regressed H-P50 on every route, and failed route-wise diagnostic or
vertical non-regression. It is therefore No-Go; the fresh validation and
future holdout remain sealed.

The candidate reached 29, 50, and 34 iterations on the three routes (two
reported convergence), with approximately 1.19x, 1.68x, and 1.35x FGO wall
time. Production RTK/SPP defaults were not changed. The optimizer-stop tests
cover the immutable lane declaration, iteration bound, scoped atomic-path
repair, deterministic commands, and byte identity when a lane converges by
eight iterations.
