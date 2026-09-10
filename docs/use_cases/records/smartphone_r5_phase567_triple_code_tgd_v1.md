# Phase567: isolate known TGD contribution before bias attribution

Specification evidence:
[IS-GPS-200N](https://archive.gps.gov/technical/icwg/IS-GPS-200N.pdf),
20.3.3.3.3.2 (printed p100), gives satellite clock corrections with -TGD
for L1 C/A and -gamma12*TGD for L2 P(Y).
[IS-GPS-705J](https://www.gps.gov/sites/default/files/2025-07/IS-GPS-705J.pdf),
20.3.3.3.1.2.1, gives L5 -TGD plus signal-specific ISC.
Hence the TGD-only raw code delays are c*TGD, gamma12*c*TGD, c*TGD.
Applying Phase566 closure yields (1-gamma15)*c*TGD. This derivation does
not establish zero ISC, L5X receiver combination semantics, or full LNAV/CNAV
clock compatibility. Do not apply the L2 P(Y) expression to C2X.

Added optional raw navigation argument to the native diagnostic. For C2W
triples only, select a valid healthy broadcast message at raw receive time
with the existing source ephemeris selector. Keep matched-support raw,
TGD-component, and raw-minus-TGD-only aggregates. No orbit/position estimate,
truth, MAT or saved solution is used. Four standalone tests passed, including
per-signal TGD delays versus analytic closure for positive/zero/negative TGD.

One H base/nav run, executable `/tmp/phase567_triple_audit`, exit 0:

| Aggregate across five satellite streams | Median absolute stream center (m) | Median within-stream MAD (m) |
|---|---:|---:|
| Matched raw closure | 2.228671036535534 | 0.22513222476768302 |
| TGD component | 0.6644515120403226 | 0 |
| Raw minus TGD only | 0.88962032692617465 | 0.22513222476768302 |

All 17,500 rows supported, zero missing/unhealthy TGD rows; 3,500 epochs.
Medians across streams are nonlinear: do not subtract table medians to
infer an explained fraction. Constant TGD per stream leaves within-stream
MAD unchanged. Smaller aggregate closure is not evidence of positioning
improvement or a receiver-only residual bias. Unavailable ISC, code tracking,
multipath and noise remain possible components; do not fit a correction table.

Raw inputs are the previously pinned H base.obs (Phase565) and H brdc.nav
(Phase511); no new products downloaded. Production FGO remains unchanged.
Full CTest unrun, no accuracy evaluation/submission. The standalone diagnostic
is not a performance candidate. This closes the TGD-only attribution step;
repeating closure/centering sweeps cannot identify the missing physical terms.
