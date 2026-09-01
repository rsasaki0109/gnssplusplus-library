# Native smartphone FGO float carrier research lane

This is a development-only experiment. It keeps the native Eigen graph used by
the frozen smartphone v1 lane (undifferenced pseudorange, ordinary TDCP, and
position/receiver-clock motion), and enables the existing undifferenced
carrier-phase factors. Every `(constellation, PRN, signal, continuous arc)`
gets its own real-valued ambiguity state. No base, single/double difference,
ambiguity-between, or integer-fixing factor is permitted.

The adapter is the source of the carrier observations. It requires finite
Android ADR with the valid state bit, uses the existing Galileo E1 Hatch-30
truth-free path, and resets an arc for invalid/reset/cycle-slip/missing ADR,
epoch gap, or hardware-clock discontinuity. The FGO command additionally uses
`--reject-rover-carrier-lli`, a 2 s maximum gap, existing 0.01 m carrier sigma,
1000 m ambiguity-prior sigma, and the existing 4-sigma Huber threshold. These
are frozen defaults, not values selected against truth.

## Reproduction

Verify the freeze before any input work:

```sh
python3 apps/commands/benchmarks/gnss_smartphone_native_fgo_float_carrier_eval.py verify-freeze
```

The only executed smoke is truth-free and reuses prior truth-free materialized
inputs. It runs at most 30 epochs per route and publishes atomic diagnostics:

```sh
PYTHONHASHSEED=0 \
LD_LIBRARY_PATH=/home/sasaki/.local/lib:/opt/ros/jazzy/lib \
python3 apps/commands/benchmarks/gnss_smartphone_native_fgo_float_carrier_eval.py structural-smoke
```

The sealed freeze and smoke decision are recorded in
`docs/use_cases/records/smartphone_r5_gsdc2023_native_fgo_float_carrier_freeze_v1.json`
and
`docs/use_cases/records/smartphone_r5_gsdc2023_native_fgo_float_carrier_structural_smoke_no_go_v1.json`.

## Result and data boundary

All three smoke windows built nonzero pseudorange, ordinary TDCP, motion,
carrier-phase, and ambiguity-state populations, with zero forbidden factors
and zero fixing. None converged within the frozen eight-iteration v1 bound;
two also had large carrier residual RMS. The candidate is therefore No-Go and
was not scored against train truth. Increasing the iteration bound or changing
the carrier/ambiguity/arc parameters would be a new experiment and is not
allowed under this sealed result.

The archive contains 41 train identities. After excluding every known truth
use, prior train/validation/holdout role, and prior truth-free materialization,
only one exact identity remains. Consequently the requested new three-identity
device-diverse train split cannot be formed without reusing a prior split. The
lane stops before truth and leaves validation and holdout sealed. The exact v1
output remains the only permitted fallback; production RTK/SPP defaults are
unchanged.

Upstream context is recorded, without copying external code, in
`smartphone_r5_gsdc2023_published_solution_reproducibility_audit.json`: the
official ION first-place abstract supports tightly coupled GNSS/INS and time
adjustment, while the third-place abstract mentions raw pseudorange, Doppler,
carrier phase, and TDCP validation. The local implementation audit confirms
that the required float no-base path already exists in `src/algorithms` and is
activated only by the explicit CLI switches above.

The referenced reproduction audit is historical context only. Its earlier
MAT-related experiment is classified as a rejected external experiment by
`records/smartphone_r5_gsdc2023_native_only_cleanup_v1.json`; no MAT artifact
is accepted by the current native lane.
