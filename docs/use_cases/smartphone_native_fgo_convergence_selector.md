# Native smartphone FGO convergence selector

This is a development-only, truth-free selector experiment. It runs the
unchanged native-FGO v1 graph twice on each frozen train identity: baseline8
(`max_iterations=8`, stopping disabled) and candidate50 (`max_iterations=50`,
relative cost threshold `1e-6`). Candidate50 is selected only when the solver
reports convergence before its bound, the finite cost trace is monotonic, all
device epoch keys and required factors are present, no forbidden SD/DD/carrier
factor is present, and every adjacent output transition is at most 70 m/s.
Otherwise the selector atomically publishes a byte-exact baseline8 copy.

## Reproduction

The pre-materialization freeze is
`docs/use_cases/records/smartphone_r5_gsdc2023_native_fgo_convergence_selector_freeze_v1.json`
(`dd94186ee2f1509d1c45bcae60de4c40d8cd51a8c5298d2dbb51b36e6cc7801a`), with
manifest hash
`4211c038911be86d7711b828e4689e8089cccf7bce494161b170d6207f2feb1e`.
Run the truth-free phase first:

```sh
PYTHONHASHSEED=0 LD_LIBRARY_PATH=/home/sasaki/.local/lib:/opt/ros/jazzy/lib \
python3 apps/commands/benchmarks/gnss_smartphone_native_fgo_convergence_selector_eval.py \
  truth-free-run --output-root output/smartphone-r5/native-fgo-convergence-selector-v1
```

Only after the truth-free manifest is sealed may the fixed train truth score be
run:

```sh
PYTHONHASHSEED=0 LD_LIBRARY_PATH=/home/sasaki/.local/lib:/opt/ros/jazzy/lib \
python3 apps/commands/benchmarks/gnss_smartphone_native_fgo_convergence_selector_eval.py \
  train-score --output-root output/smartphone-r5/native-fgo-convergence-selector-v1
```

The resulting evidence is
`output/smartphone-r5/native-fgo-convergence-selector-v1/train_evaluation.json`
(`c7b18e1f15a70a9a729200811336b1e1731bc9f13f62996d49a47220a464de6a`) and its
truth-free manifest
(`c00ba9f2d98a4ecb77ee6348b257304430433197c2c89871489011dc468a652a`).

## Result and data boundary

The two exact selected phone identities were unused before the freeze, but the
routes have earlier evidence for other phones; that route-overlap risk was
declared before materialization. The selector selected baseline8 on
`2021-01-04-21-50-us-ca-e1highway280driveroutea/pixel5` because candidate50
ran to 50 iterations, did not converge, and had a non-monotonic cost trace.
It selected candidate50 on
`2021-07-14-20-50-us-ca-mtv-e/sm-g988b` after a 10-iteration converged,
monotonic trace. Both selected outputs were finite, complete, and within the
70 m/s bound.

The frozen promotion gate requires every route and the aggregate to strictly
improve all four implemented diagnostic variants and their mean while keeping
availability/key coverage and vertical P95 safe. The train result is
`no-go-train-gate`: the fallback route is equal by construction and the
candidate route is equal to baseline in all four diagnostic variants. The
aggregate selector is therefore equal to baseline (H WGS84 P50 `1.719311 m`,
P95 `2.761730 m`, diagnostic mean `2.242728 m`, availability `1.0`), so no
fresh validation or future holdout was materialized/opened. No code or
parameter changes were made after scoring, and production RTK/SPP defaults
remain unchanged. The machine-readable result is
`docs/use_cases/records/smartphone_r5_gsdc2023_native_fgo_convergence_selector_result.json`.

The candidate runtime multipliers were reported as `5.9375x` and `1.2636x`
for the two routes; the fixed runtime ceiling is a score veto only when
exceeded. No Kaggle token, leaderboard value, external mutation, or old
holdout was used.
