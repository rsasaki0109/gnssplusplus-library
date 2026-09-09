# Phase514 — residual code ionosphere versus band-clock nuisance

Reviewed Phase12 score record: old Pixel7pro development recipe improved by
about 2.3 cm versus its Phase11 reference. This is neither the current H recipe
nor proof of generalization. Phase396 already establishes that legacy residual
ionosphere only changes P factors, not ordinary TDCP. Do not redo that option
as a claimed joint carrier/code correction.

Current backend inspection adds a concrete integration boundary: Phase171
rejects use_residual_ionosphere_states, and the source-clock pseudorange branch
constructs its dedicated factor then continues before the legacy ionosphere
branch. Removing the guard alone would therefore not connect the desired state
to those source-clock code factors. No guard or production factor changed.

Added two synthetic controls to tests/test_dual_frequency_tdcp_observability.cpp,
using the real residual_ionosphere::signalCoefficient helper:

- At equal satellite elevation, one vertical residual column is in the span
  of independent band-clock columns (six rows, joint rank two, not three).
- With elevation diversity the projected residual column remains nonzero and
  joint rank becomes three. Adding group-constant biases leaves that projection
  unchanged: this is not a way to estimate missing satellite ISC.

Scope: fixed receiver position, equal weights, two independent band clocks,
single epoch. It does NOT establish full C7/position/IMU observability, physical
validity, robust-weight conditioning or actual smartphone precision. Full C7
uses a different but potentially equivalent clock basis for these two groups;
this test intentionally does not claim to execute the production clock factor.

Fresh standalone build and run exited 0, all 13 tests passed (two new, eleven
existing). Test file was already registered in tests/CMakeLists.txt. Full CTest
and production native trajectory runs were not performed.

```sh
c++ -std=c++17 -O0 -Iinclude -I/usr/include/eigen3 tests/test_dual_frequency_tdcp_observability.cpp -lgtest -lgtest_main -lpthread -o /tmp/phase514_ionosphere_observability
/tmp/phase514_ionosphere_observability
```

Next: measure the retained code-row ionosphere column after projecting out
actual position and C7 clock columns, with native same-run raw seeds only.
This discriminates useful local measurement information from a prior-driven
extra degree of freedom before a new source-clock factor integration. Do not
tune ionosphere priors against H while this structural evidence is missing.
No MAT/truth/saved trajectory input or new accuracy claim. Goal remains unmet.
