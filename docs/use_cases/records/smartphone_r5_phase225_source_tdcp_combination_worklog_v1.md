# Phase225: source TDCP mapping admission with ECEF-D and main motion

Primary agent, no subagents. Removed the CLI Phase184/ECEF-D exclusion;
changed FGOProcessor admission to allow the dedicated ECEF-D staging lane
alongside the existing no-D staging and Phase171 main lanes. Other-lane
rejection and Phase117/118 conflict guards remain. Defaults, Type mapping,
factor equations and noise sigmas are unchanged.

Added a source-k=true case to the existing synthetic handoff with ECEF-D,
dedicated main Doppler and Pose3 motion enabled. Existing assertions check
selected k=0.2 in staging/main, factor counts, finite clock exports and
convergence. The fixture sets main use_robust_loss=false: it proves admission
and diagnostic selection, not the main robust weighting's numerical effect.
The production factor continues consuming the resolved threshold; raw-data
accuracy for this combination is still unmeasured.

CLI admission is tested with/without source k, stopping at the downstream
missing raw-recipe guard; this is not a successful raw solve.

Verification: initial build 75621 completed, but fgo.cpp changed during that
build. A subsequent build 9094 recompiled fgo.cpp and relinked both requested
targets, exit 0; only this fresh build was used for the reported runtime tests.
Focused C++ tests 10/10 passed. Regression selection 85/85 passed (two
FGOTdcpRobustK tests overlap the focused selection; 93 unique C++ tests).
CLI/source-contract tests 18/18 passed. This is not full CTest.

Before a raw experiment, strengthen the synthetic combined case to exercise
robust loss enabled as in production, then freeze source/binary/input pins.
The intended source preset affects both GNSS-first and main TDCP likelihoods;
GNSS-first results, seeds and stop speed gates need not remain identical to
Phase222. Do not assert otherwise or tune k to H truth. No raw run, MAT,
truth/candidate payload read, or evaluation occurred in this phase.
