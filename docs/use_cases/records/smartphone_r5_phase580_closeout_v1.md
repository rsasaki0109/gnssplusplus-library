# Smartphone raw-only native FGO closeout — 2026-09-09

The user explicitly accepts approximately 1.077 m and requests closeout
and merge. This supersedes the earlier <=1 m acceptance requirement, not
the meaning of the metric or the no-MAT inference contract.

Accepted H development baseline: 1.0769180514591334 m, horizontal
`(P50+P95)/2`, 3,139 matched predictions. Phase578 reproduced the baseline
output with verified pins; its SHA256 is
`4a0c4ef8822c72aa9134368cd57fe769817895e63b5dbe925783a9c8d091fa8e`.
Inputs are raw Android GNSS, IMU and broadcast navigation. GNSS-first
positions are generated within the same native process and handed off in
memory. Neither MAT nor saved positioning results are inference inputs.

This is reused development data. Other development routes remain worse:
A 1.361335 m, U 1.305960 m, LAX 3.102603 m. No fleet-wide 1.077 m,
sub-metre generalization, 0.782-class native performance or leaderboard
placement is claimed. Historical MAT-based scores are unrelated to this
accepted baseline.

Experimental joint ionosphere, Doppler rotation-rate, frequency residual
states and ADR-endpoint sigma remain default-off and are not promoted by
this closeout. Phase579 is an experimental raw run, not the accepted
solution. Historical negative experiments and their audit records remain
available for reproducibility.

Local closeout checks so far: native executable build succeeded; endpoint
covariance/helper tests 5/5 passed; Python runner syntax and git diff
whitespace checks passed. Full current-tree CTest has not been claimed.
PR #491 targets develop; remote CI and merge must be checked separately.

Publication packaging: the 173,215,107-byte Phase97 structural diagnostic
JSON exceeds GitHub's 100 MB limit and is omitted from the publication
tree. The original file and full research history remain in the local
research branch (blob `35636510746b92fd89f21c7f7163e928e1905952`).
Historical links to that raw diagnostic require the local research archive.
Publication is a consolidated commit on the existing remote PR head;
the original local research history is not rewritten or deleted.
