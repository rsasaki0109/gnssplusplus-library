# Phase505 — verified identity intersection, no baseline output change

Native PID 3801727 / session 44009 is terminal, exit 0 in
411.62053016899154 seconds. verify_phase505_identity_shadow.py passed against
current source/binary pins. Graph and GNSS-first summaries equal Phase503;
stage termination checks pass. Output hash equals baseline byte-for-byte:
4a0c4ef8822c72aa9134368cd57fe769817895e63b5dbe925783a9c8d091fa8e.

Exact same-row candidate counts: 426 raw, 420 geometry/quality eligible,
243 baseline residual passes, 243 with retained problem epoch. Zero factors
added. Raw labels survive native processing sufficiently to reproduce this
intersection; no saved candidate/position input, truth read or scoring.

This seals the diagnostic, not an accuracy improvement. A main-only
readmission experiment would require full factor provenance from the ordinary
builder and must not alter the baseline median or GNSS-first. Candidate labels
are not clean-code certificates. Keep all selection defaults unchanged and
freeze any later opt-in factor experiment before evaluation. Overall goal
remains unmet; no active job from this diagnostic run remains.
