# Phase471 — actual TRYLAMBDA diagnostic fault injection

Extended scripts/native_lm_trace_audit.cpp to instantiate the actual native
optimizer wrapper in TRYLAMBDA mode, not just feed synthetic SUMMARY text.
A two-component state/one-row valid derivative probe solved successfully
both with lambda=1e-30 and with zero lambda. Both diagnostic tests correctly
failed their requirement to observe a linear failure; neither reproduced
the target path. Preserve these negative outcomes rather than claiming rank
deficiency necessarily throws in this solver.

Final diagnostic probe deliberately supplies a NaN Jacobian while retaining
a finite nonlinear residual. Lambda initial/upper bound zero stops the
failed trial immediately. This is fault injection only, never a production
sensor factor or a physical GNSS rank/conditioning simulation.

Observed linked-native output: failed-lambda count=1 min=0 max=0,
nearby_key=unavailable; wrapper failure counter=1; final cost=0.5. Exit 0.
This verifies the corrected Phase96/TR YLAMBDA aggregation emits a failure
range using the actual optimizer path. The original SUMMARY parser control
still reports two attempts, one linear failure. No accuracy implication.

The main native executable has not been rebuilt since the Phase470 correction.
No new raw replay was launched in this phase. Next rebuild, freeze fresh pins
and rerun a single baseline to check output identity and actual lambda ranges.
No truth, MAT, saved position input or production setting change. Goal unmet.
