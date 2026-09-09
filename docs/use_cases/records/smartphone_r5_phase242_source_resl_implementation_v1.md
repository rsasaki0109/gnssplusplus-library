# Phase242: independent source resL observable

Added default-off `use_source_tdcp_resl_observable` and native CLI
`--native-source-tdcp-resl-observable`. CLI admission is Pixel5 Phase171
ECEF-D with explicit UTC fallback, excluding historical Phase117/118/120.
Metre sigma and main Doppler/motion are compatible. Public build/optimize
reject the new selector combined with historical Phase120.

Only ordinary TDCP carrier preparation uses the existing source-resL
helper (raw carrier plus satellite clock). Existing corrected carrier is
retained for pair admission, including the code-phase jump gate. No P/D,
sigma, robust k, geometry, raw admission or time mapping change.
Summary includes `source_tdcp_resl_observable_requested`; this is request
telemetry, not an independently measured graph insertion counter.

Code-path inspection: main configuration is assigned before building the
problem; `gnss_first_config = config` and `gnss_first_problem = problem`
propagate the selection and prepared TDCP factors into the initial stage.
This is static evidence, not yet a real-route two-stage verification.

Validation:

- Build session 74878 completed both native executable and C++ tests.
- Initial focused tests: 7/8 passed. New atmosphere-change assertion failed
  with historical, non-surface-validated ECEF fixture positions.
- Changed only the new fixture to equatorial points at approximately 100 m
  height so the atmosphere model can be active; kept the assertion intact.
- Rebuild session 28745 completed. All 8 focused C++ tests passed, including
  temporal sign oracle, new builder pair/key/sigma preservation, observable
  agreement with existing resL switch, and mixed-selector rejection.
- Both source-resL and metre-sigma Python CLI suites: 16 tests passed.

No full CTest claim. No raw route solve or accuracy evaluation yet.
No MAT or saved positioning inputs. Next freeze an isolated raw H run
against Phase234/235 with k=4 and the new observable selector enabled.
