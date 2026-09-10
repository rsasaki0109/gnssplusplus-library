# Phase457 — native seed contamination mechanism and RAIM control

Added `RawPSeedTest.ContaminatedCodeSeedMaskSensitivityControl` to the
existing native raw-P test fixture. Sixteen synthetic GPS broadcast orbits,
one epoch, one +300 m code error, fifteen clean code rows. Actual native
`raw_p_seed::solve` is used; no Kaggle files, MAT, saved solutions or truth
files are read. The clean synthetic solve has position error below 0.1 m.

To isolate influence, this control disables atmospheric corrections,
elevation rejection, variance/elevation weighting, and iterative outlier
detection. Huber defaults are unchanged (3 sigma, minimum factor 0.05).
The solve budget is 100 iterations at 1e-4 m convergence. These settings and
the generated geometry are not the phone pipeline or an accuracy benchmark.
RAIM is explicitly tested OFF and ON, since it is ON by default in SPP.

Clean-row residuals are independently synthesized at each estimated receiver
position/clock and compared to the uncorrupted observations. Apply the native
centered-P acceptance primitive with an ideal clean center of zero and the
existing L1 threshold of 20 m. This is not the complete FGO builder's global
median/elevation/correction pipeline and must not be described as such.

| Huber | RAIM | Used rows | Position error m | Clock error m | Clean rows exceeding 20 m |
|---|---|---:|---:|---:|---:|
| OFF | OFF | 16 | 126.103 | 71.6766 | 12 / 15 |
| ON | OFF | 16 | 16.4223 | 9.3343 | 4 / 15 |
| OFF | ON | 15 | < 1e-7 | < 1e-7 | 0 / 15 |
| ON | ON | 15 | < 1e-7 | < 1e-7 | 0 / 15 |

The initial RAIM-disabled experiment establishes that contaminated seeds can
push clean rows past a residual threshold, and Huber can reduce that effect.
The RAIM-enabled control materially changes the next action: existing native
RAIM already handles this single gross outlier. It would be misleading to
promote a robust seed candidate from the disabled-safeguard result alone.
No production configuration change or new raw replay is justified here.

Next useful evidence must address a failure mode not already covered by
native RAIM: e.g. multiple contaminated observations or weak geometry under
the actual seed quality settings, with controls keeping existing safeguards
enabled. Do not tune noise, geometry, or contamination until Huber looks good.
Use a predeclared small synthetic case set and preserve negative results.
No conclusion about the cause of LAX epochs 851/852 follows from this test.

Standalone C++ test build and the four-case mechanism test passed. The final
RawPSeedTest suite rerun, including RAIM assertions, passed all 27 tests.
Full CTest/PPC remains unrun.
The performance goal is still active and unmet.
