# Phase 40 direct raw-Doppler WLS handoff

Phase 40 evaluates a truth-free, native opt-in Android initialization path:

```text
--native-direct-doppler-wls-handoff
```

The candidate consumes the existing in-process
`FGOProblem::doppler_velocity_wls_estimates`, which is built from raw Doppler
rows, broadcast satellite state, and the original raw SPP seed.  Those ECEF
velocity estimates are converted to ENU at the first original raw SPP seed and
are used only for the IMU velocity/heading seed.  The GNSS-first optimizer is
not run, and no GNSS-first position, velocity, or receiver-clock state is
copied.  Original raw SPP positions remain the position seeds.

The pre-implementation freeze is
[the Phase 40 freeze record](records/smartphone_r5_phase40_direct_doppler_wls_handoff_freeze_v1.json)
(SHA-256
`544089e32a7f7c0ca96ee6cbbfd7634145aebe288fe32a947ef1f67d5c07f52e`, frozen
at `bd22998`).  Its fixed contract reports direct-valid, propagated-valid, and
rejected estimates for every problem epoch.  Only the existing bounded linear
interpolation and one-sided edge hold (maximum 1.0 s) are permitted; every
estimate must be finite, have velocity norm `<=70 m/s`, and have clock-rate
absolute value `<=2000 m/s`.  Any incomplete or rejected sequence fails
closed.  TDCP remains the ordinary base graph contract and must have positive,
equal built/inserted counts on a passing run.  Raw UTC keys remain exact and
coordinate propagation is forbidden.

The source and focused-test push was `4b60601`.

## Structural result

The fixed target probe used only the frozen raw inputs and the base flags:

```bash
python3 apps/commands/benchmarks/gnss_smartphone_phase40_direct_doppler_wls_handoff.py \
  run-matrix --output-root output/smartphone-r5/phase40-direct-doppler-wls-handoff-v3
```

It failed closed on the first MTV-h candidate run, so the remaining route
repeats and flag-off controls were not started.  The truth-free diagnostics
were:

* 4,724 corrected undifferenced-Doppler factors;
* 1,181 of 1,325 epochs had at least four rows, while 144 had insufficient rows;
* direct-valid `0`, propagated-valid `0`, rejected `1,325`;
* non-finite `0`, over-70-m/s `1,181`, over-2000-m/s clock-rate `577`;
* first solved reason `physical-gate`, with 4 rows, velocity norm
  `8919.7537472980548 m/s`, and clock-rate absolute value
  `7314.3850372387406 m/s`.

Thus the candidate is a structural No-Go under the frozen physical bounds.
No output was published, no GNSS-first optimizer was attempted, and no
position/clock values were copied.  Production defaults and the Phase 31
champion remain unchanged.  The machine-readable records are the
[Phase 40 No-Go result](records/smartphone_r5_phase40_direct_doppler_wls_handoff_result_v1.json)
and [failure manifest](records/smartphone_r5_phase40_direct_doppler_wls_handoff_structural_failure_manifest_v1.json).

The Phase 40 focused Python test (4 tests), Phase 40 CTest, and Phase 39
regression CTest passed.  The full 142-test CTest run with the runtime library
path had four unrelated failures: three historical Phase 35/37/38 tests still
pin the pre-Phase 40 binary SHA, and `python_cli_tests` had serial pseudo-
terminal fixture failures.  Those historical pins and fixtures were not
rewritten as part of Phase 40; the result record contains the exact test
summary.

The historical flag-off SHA records remain authoritative in the Phase 39
freeze; they are documented in the Phase 40 result without changing the
freeze.  In particular, the corrected Phase 37 mtv-u submission SHA is
`524769cdf67aa857eefaafdadf943dd76586dda6a53e8b6d1df4a707d7699f71`.
Existing pre-Phase 40 binary SHA pins are historical and were not rewritten.
Any Doppler measurement-contract correction belongs to a separately frozen
Phase 41; Phase 40 does not relax a gate or change the measurement model.
