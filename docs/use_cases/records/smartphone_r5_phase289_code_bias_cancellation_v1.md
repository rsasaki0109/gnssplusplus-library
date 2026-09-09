# Phase289 explicit code-bias cancellation boundary

Starting HEAD `3da7ac6`; root-only source inspection and synthetic test.

Native rover (`fgo_problems.cpp:614-646`) subtracts group delay unless
Phase126 is enabled. Native base (`base_pseudorange_compensation.cpp:511-532`)
does the same unless source_complete is enabled. Their legacy operators
(`fgo_internal.hpp:340` and base source line 50) have matching constellation
branches: GPS/QZSS TGD, BeiDou primary/secondary TGD by signal, Galileo shared
selection helper. Matching formulas do not prove matching ephemerides,
signals, times or effective corrections on real data.

At fixed retained observations, geometry and seeds, let W be the base
smoothing/interpolation operator, b_r the rover explicit bias and b_b the
base bias sequence. The difference from omitting explicit bias on both sides
is `W(b_b) - b_r`, provided finite support and W weights are unchanged.
Identical constant biases cancel. Bias transitions within the base smoothing
window, different ephemeris selection, or different signal conventions need
not cancel. Upstream admission and seed changes remain outside this identity.

Added `SharedCodeBiasCancelsOnlyWhenSmoothedBaseBiasMatches` to the existing
base compensation suite. It calls production centeredMovingMean and
subtractCorrection on synthetic values. A shared constant 6 m cancels;
base bias samples 3/6/9 m and rover bias 9 m leave -3 m; changing only the
base policy leaves -9 m. This is not evidence that an H navigation transition
actually occurs, nor a full base/rover builder test or an accuracy result.

Next integration must choose one coherent paired code-bias policy and cover
both builders, without assuming same-formula cancellation. Source clock
state evaluation and geometry still require paired inspection. No production
code or historical recipe is changed. No raw, candidate, truth, MAT, station
table or Kaggle/token payload was opened in this turn.

Validation: gnss_run_tests target built successfully with -j4; all 18
BasePseudorangeCompensationTest tests passed, including the new case.
These tests may create/read their synthetic RINEX fixtures; the no-raw-read
statement above concerns real dataset payloads. Full CTest was not run.
