# Phase515 — weighted nuisance-projected scalar information

Added scalarProjectedInformation to pseudorange_position_information.hpp.
It whitens a scalar measurement coefficient and arbitrary nuisance columns by
row sigma, removes the numerically independent nuisance space using thin SVD,
and returns the remaining squared norm. No inverse of a singular normal matrix,
priors, damping, robust reweighting or trajectory access. Existing position
information routine is unchanged. This is a diagnostic, not a solver feature.

Two new tests compare against explicit weighted least-squares elimination,
check sigma scaling, duplicate columns, complete absorption, absent nuisance,
empty rows and invalid dimensions/sigma. Fresh standalone build exited 0;
9/9 tests passed (2 new, 7 existing). Full CTest and raw-run integration remain
outstanding. No raw observability measurements or accuracy claims yet.

```sh
c++ -std=c++17 -O0 -Iinclude -I/usr/include/eigen3 tests/test_pseudorange_position_information.cpp -lgtest -lgtest_main -lpthread -o /tmp/phase515_scalar_information
/tmp/phase515_scalar_information
```

Current-state integration evidence:

- sourceClockComponentFor delegates to raw_p_seed::c7ClockComponentFor.
  The actual clock Jacobian assigns base slot 0 and selected component to one
  (not addition: GPS L1 remains one). Use this exact basis in the diagnostic.
- fgo_problems.cpp computes residual_ionosphere_coefficient from SPP seed
  elevation and row frequency even with the option disabled.
- The app later overwrites problem.epochs positions from GNSS-first results
  in memory. It does not recompute those coefficients at that handoff.
- The diagnostic should run after final main row selection and report its
  linearization convention explicitly: current position Jacobian versus the
  stored SPP-derived coefficient. Recomputing a coefficient is a separate
  model choice, not silently equivalent to the existing factor metadata.

Next wire this diagnostic to the same-run main problem, preserving factors,
weights and defaults, and verify output invariance before interpreting aggregate
information. No saved solution is authorized as diagnostic input. The overall
.782/LB objective remains unmet and active.
