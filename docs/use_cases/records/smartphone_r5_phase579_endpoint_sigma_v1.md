# Phase579: raw ADR endpoint uncertainty candidate

Acceptance is horizontal `(P50+P95)/2 <= 1.0 m`; byte equality is not
an accuracy requirement. H is reused development data, not held-out or LB.

Phase578 completed successfully in 171.67971666599624 s with verified pins.
Its output SHA256 was
`4a0c4ef8822c72aa9134368cd57fe769817895e63b5dbe925783a9c8d091fa8e`,
identical to the existing H baseline (previous evaluation: 1.0769180514591334 m).
No new truth read was needed for that identity check.

The new opt-in `--native-tdcp-adr-endpoint-sigma` uses
`hypot(previous ADR uncertainty, current ADR uncertainty)` in metres for
each admitted TDCP, in GNSS-first and main. Existing admission logic and
scalar Huber loss remain. Missing/invalid admitted endpoint uncertainties
fail explicitly. There is no fitted multiplier, floor, truth-derived bias,
MAT input, or saved positioning input. Default is OFF.

This is a diagonal marginal-variance experiment, not temporal whitening.
Android uncertainty is not assumed calibrated merely because it is available.
Five focused covariance/helper tests passed. Native build and raw candidate
execution must complete before claiming an accuracy result.

Runner: `scripts/run_phase579_endpoint_sigma.py`. The runner freezes source,
binary and raw-input hashes before inference and checks them afterward;
it does not access truth or score the result.
