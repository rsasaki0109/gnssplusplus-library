# Phase173 Phase171 raw-P seed routing fix

Status: bounded implementation and launch-free validation complete.  No real
raw route was rerun in Phase173.

## Proven failure and fix

The single Phase172 H invocation was admitted through the Phase171 CLI, read
the pinned GNSS/IMU/nav inputs, built a retained problem, and then failed with
`retained-epoch-source-identity-not-found`.  Its sealed summary showed
`adapter_input_epochs=0`, `raw_result_present=false`, `retained_epochs=2294`,
four retained pseudorange factors, and 4799 TDCP factors; neither the raw-P
seed solve nor GNSS-first/main FGO was attempted.  The unchanged identity
guard was correct.

The source cause was the Phase165 adapter initialization condition in
`apps/native/gnss_fgo_imu_no_base.cpp`: Phase171 was excluded by
`native_phase165_raw_p_no_doppler_graph &&
!native_phase171_raw_p_no_doppler_imu_main`.  Phase173 removes only that
exclusion, so Phase171 executes `raw_p_seed::solve` and
`adaptSameRunNoDopplerSeeds` once.  The existing later Phase171 branch still
skips only the terminal Phase165 GNSS-only optimize/return; the retained-key
join and all identity/finite guards remain unchanged.

## Validation

`tests/test_smartphone_phase173_phase171_raw_seed_routing.py` is deliberately
launch-free and reads only source/manifest metadata.  Its `3/3` tests assert
that Phase171 enters shared raw-P initialization, suppresses only the terminal
Phase165 solve, and requires the pinned Phase172 selector combination.  These
are static routing assertions, not runtime raw-data or end-to-end proof.

The affected executable rebuilt successfully:

```text
cmake --build build --target gnss_fgo_imu_no_base -j2  PASS
```

The previously verified bounded native Phase93/101/135/138/164/165/167/171
regression remains green (`28/28` in the local filter; the expanded native
filter was independently verified `64/64`).  The full Phase171 argv was also
validated against `--help` and a nonexistent-input admission smoke; it reaches
raw GNSS opening rather than the Phase143 affine gate.  No payload was opened
by these validation tests, and no solver was run.

Phase172 remains an immutable failed diagnostic.  A future real H run requires
a new binary/input/argv pin and a separate one-shot authorization/result; this
fix record makes no accuracy, convergence, or leaderboard claim.
