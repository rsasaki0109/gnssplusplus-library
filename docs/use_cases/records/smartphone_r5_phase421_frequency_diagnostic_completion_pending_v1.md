# Phase421 — frequency diagnostics implementation, build pending

After Phase416/417 completed and their original pins were verified, added a
backend diagnostic counter incremented at each actual frequency prior insertion.
The app serializes this count and the existing backend TDCP RMS. Enabled runs
fail before output when prior/state counts disagree, either RMS is nonfinite,
or app/backend RMS differs by more than 1e-7 m. Graph construction/noise/solve
settings are otherwise unchanged; this is not a new model candidate.

Expanded the existing test to compile the actual app guard as well as its
correction-report code. Matched diagnostics pass; missing prior, RMS mismatch,
NaN backend RMS and infinite app RMS reject; OFF does not apply the guard.
Python unittest passed (1 test, multiple compiled C++ cases).
Native rebuild and fresh backend test compilation were started, not yet
verified complete. Fresh backend assertions include one prior per paired
state. Do not link old FGOResult-layout test objects against the new libraries.

Before scoring: finish build/tests, freeze new diagnostic-only raw replays in
new output directories, require exact Phase416/417 candidate hashes and the
new diagnostic checks. Never repin or retry the completed historical runs.
No truth evaluation has been performed.
