# Experiments

Experimental workflows and evaluation results.

## PPP Ambiguity Resolution

See `experiments/ppp_ar/` for PPP-AR experiment configurations and lane-level results.

Keep PPP-AR, CLAS, and MADOCA strategy sweeps in the experiment lane until the
behavior is stable enough to promote into a normal CLI or solver default. The
lane is allowed to try competing policies, broad strategy catalogs, and
reference-oracle comparisons; the stable solver path is not.

Use `experiments/ppp_ar/run_experiments.py` for CLAS PPP policy sweeps:

```bash
python3 experiments/ppp_ar/run_experiments.py \
  --config-toml experiments/ppp_ar/input.example.toml \
  --output-json output/ppp_ar_experiments/results.json \
  --markdown-out output/ppp_ar_experiments/results.md
```

Use `experiments/ppp_ar/suite.example.toml` when a result is being evaluated
for promotion. A single-case win is a diagnostic result, not a promotion
candidate.

## Promotion Rules

An experiment result can move toward a stable CLI/profile/default only when the
PR carries both the behavior change and the regression artifact that protects
it. At minimum, the PR must show:

- the same input schema across all strategy arms: `obs`, `nav`, one correction
  source, `reference_ecef`, `max_epochs`, mode, and output directory layout
- a baseline or stable-control arm in the same run
- shared output metrics: `wall_time_s`, `ppp_solution_rate_pct`,
  `ppp_fixed_solutions`, `fallback_solutions`,
  `clas_hybrid_fallback_epochs`, `mean_3d_error_m`,
  `median_3d_error_m`, `p95_3d_error_m`, `max_3d_error_m`,
  `readability_score`, and `extensibility_score`
- `promotion_gate_checks`, `promotion_gate_pass`, `promotion_stage`, and
  `promotion_rationale` in the result JSON
- a suite-level comparison when multiple public or oracle-backed cases exist
- an explicit regression budget for solution rate, fixed count, P95/max error,
  runtime, and any oracle delta being claimed
- a follow-up stable sign-off or schema test before the behavior becomes
  default-on

Promotion should be staged:

| Stage | Meaning | Allowed next step |
|---|---|---|
| `exploratory` | The arm does not repeatedly beat the control or has unclear tradeoffs. | Keep it under `experiments/` or delete it. |
| `extended_trial` | The arm passes in most cases but not all. | Add cases, diagnostics, or a narrower env/config-gated trial. |
| `trial_candidate` | The arm passes the shared gates across the suite. | Wire an opt-in CLI/profile flag with a dedicated sign-off. |
| `promotion_candidate` | The arm passes the suite and improves solver-specific outcomes such as fixed epochs. | Consider default-on only after the stable sign-off passes. |
| `stable_control` | The comparison baseline. | Keep stable and use as the reference arm. |

Do not promote behavior directly from an experiment result when:

- the result depends on reference truth for a runtime decision
- the strategy changes protocol decoding, correction expansion, and solver
  behavior in one PR
- the decoder lacks subtype-level parity or deterministic unsupported-subtype
  handling
- the result improves fixed-only epochs while all-epoch RMS, P95, or max error
  regress outside the stated budget
- the experiment needs external credentials but has no local fixture or
  optional CI skip reason
- the artifact schema changes without a reader or schema-compatibility test

The experiment lane may keep broad comparison tables. Stable PRs should stay
small: one user-visible behavior, one sign-off or schema improvement, and one
documented regression budget.
