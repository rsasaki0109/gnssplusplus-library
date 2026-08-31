# Native smartphone FGO carrier compatibility selector

This is a development-only experiment. The previously sealed all-device
float-carrier lane remains No-Go and is not modified. This lane addresses a
demonstrated adapter contract defect: Android `AccumulatedDeltaRangeState=16`
has no ADR-valid bit, but the old RINEX adapter omitted both the carrier and a
loss-of-lock marker. With the native `> 1500 ms` arc-gap rule (and a two-second
boundary accepted by the old adapter), unrelated carrier arcs could be joined.
The high 214--254 m carrier residuals were therefore an arc-boundary problem,
not a carrier unit or sign conversion problem. A truth-free derivative audit
confirmed ADR and pseudorange-rate correlation above 0.999999 on the affected
routes.

The compatibility wrapper leaves the original adapter, source hashes, and
production RTK/SPP defaults untouched. It rewrites only the lane-local RINEX
carrier field: invalid/reset/cycle-slip rows, hardware-clock transitions, and
gaps over 1500 ms are blanked and marked LLI=1; the next valid row is also
marked to start a new arc. Code, Doppler, SNR, satellite geometry, and all
other fields are preserved byte-for-byte. Raw and RINEX epoch/key sets must
match, otherwise the wrapper fails closed.

## Frozen selector contract

The exact native Eigen v1 pseudorange + ordinary TDCP + position/clock motion
graph is the baseline (`baseline8`). The candidate adds only undifferenced
float carrier factors, with ADR-valid observations, no base, no SD/DD factors,
and no integer fixing (`carrier_float50`). It uses the fixed 50-iteration bound
and relative cost threshold `1e-6`; all other graph/noise parameters are the
v1 values.

The selector uses no truth, device-model label, or leaderboard feature. A route
can select the candidate only if it has finite positive carrier factors and
ambiguity states, a maximum valid arc of at least 5 s, finite monotone cost
trace and genuine convergence, finite graph/ambiguity conditioning proxies,
finite output with no transition over 70 m/s, and both:

* prefit ADR-rate RMS no greater than `2 * (0.01 m * 4 / sin(10 degrees)) =
  0.4607016386514907 m/s`;
* postfit carrier RMS no greater than
  `0.01 m * 4 / sin(10 degrees) = 0.23035081932574535 m`.

The native summary does not export a covariance matrix, so the manifest calls
the finite graph-factor/ambiguity-state ratios *conditioning proxies* rather
than claiming a covariance estimate. Any failed check selects an atomic,
byte-exact copy of the baseline output.

## Reproduction

The role inventory and freeze must be verified before generation:

```sh
PYTHONHASHSEED=0 python3 apps/commands/benchmarks/gnss_smartphone_native_fgo_carrier_compatibility_eval.py verify-freeze
PYTHONHASHSEED=0 python3 apps/commands/benchmarks/gnss_smartphone_native_fgo_carrier_compatibility_eval.py truth-free-run
```

The truth-free run uses only the eight already materialized development/train
identities in `...carrier_compatibility_role_inventory_v1.json`: three
optimizer-stop routes, two convergence-selector routes, and three structural
development smokes. It contains pixel4, pixel5, and sm-g988b families and
reports leave-one-route-group-out and leave-one-phone-family-out folds after
development truth is explicitly opened. Validation, all prior holdouts, all
test identities, Kaggle scores, and tokens are excluded.

After the truth-free manifest and every route hash are sealed, the permitted
development-only evaluation is:

```sh
PYTHONHASHSEED=0 python3 apps/commands/benchmarks/gnss_smartphone_native_fgo_carrier_compatibility_eval.py train-score
```

This command materializes/reads only the eight declared development
`ground_truth.csv` members, once for this phase. It never opens a validation,
holdout, or test truth member. A strict all-four-diagnostic and mean
route/fold/aggregate gate is required for a development-only promotion. On a
failure, the selector is sealed No-Go and no new validation asset is reused;
on success, a genuinely new validation asset is still required. There is no
Kaggle submission step.

Focused contract tests are in
`tests/test_smartphone_native_fgo_carrier_compatibility_eval.py`. Release
commands should use the installed Eigen/GTSAM loader path:

```sh
LD_LIBRARY_PATH=/home/sasaki/.local/lib:/opt/ros/jazzy/lib ctest --test-dir build -j1 --output-on-failure
git diff --check
```
