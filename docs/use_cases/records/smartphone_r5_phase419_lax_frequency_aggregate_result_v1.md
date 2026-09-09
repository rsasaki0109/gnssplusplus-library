# Phase419 — LAX-T enabled frequency-state aggregate result

Phase417 completed once, exit 0, 213.989313286962 s. Frozen manifest SHA256:
`372642594ee1b1729d097b97872225ef19647ffab4b8a964d500dbe2e7da2fab`.
Current source, test, algorithm, executable and raw input pins passed the
post-run verifier before any pinned implementation changes.

Both stages converged. Main frequency states: 5624; wrapped scalar TDCP
factors: 11248; total TDCP: 24964, unchanged from baseline. Residual/cost
finite checks and grouped accounting passed. Output contains 1466 identifiers
in exactly the expected raw UTC order, including the historical first epoch.
Candidate SHA256:
`58ecf000b36df8df7bd884158115b7565d040c1ddde42536d576b9f1631bd4e3`.
Corrected app TDCP RMS: 0.02073382918960137 m;
reconstructed TDCP Huber cost: 111218.39821103004.
These are fitted residual diagnostics, not positioning accuracy and not the
whole graph objective (the added state priors are not part of this TDCP sum).

Truth reads: 0; accuracy evaluations: 0. Phase418 backend-RMS/prior-accounting
verification gaps remain. Do not treat these aggregate checks as approval to
score, promote, submit, or claim improvement. Phase416 H remained live when
this result was recorded; it must be polled, not restarted.
