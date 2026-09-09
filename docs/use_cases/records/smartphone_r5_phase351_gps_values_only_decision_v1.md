# Phase351 decision: GPS-only numerical correction not promoted

Frozen Phase350 completed once in 267.7103130900068 s; 81591 P factors,
3139 exact evaluation rows, converged GNSS-first and main, full-model
retained-identity check passed. One frozen evaluation read truth once and
computed one score; no interpolation or offset reapplication.

H development route score: 1.7792513752546433 m (P50
1.6076153473111645, P95 1.9508874031981218). Operational baseline:
1.0769392017393964 m; degradation +0.7023121735152469 m.
Same-support zero-values Phase330: 1.1370801747544546 m.
Full-values Phase327: 2.2517127747561947 m.

GPS correction values alone therefore produce substantial degradation
relative to zero values in this paired/support-controlled recipe. The
regression cannot be attributed solely to non-GPS numerical corrections.
The intermediate score is not an additive attribution across constellations
in a nonlinear robust graph, nor proof that the base reference coordinates
are wrong. No GPS-only setting promoted; production baseline unchanged.
Next investigation should focus on GPS correction/reference consistency,
not repeat propagation audits or assume dropping non-GPS values fixes it.
These are reused H development results, not independent validation or LB
evidence. Raw-only pipeline maintained; 0.782/LB objective unachieved.
