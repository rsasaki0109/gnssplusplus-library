# Phase365 — H/U development support comparison

Primary-agent work. U is historically scored development data (Phase250),
not a new holdout. No new truth read, candidate input, score or solver run.

The Phase364 native raw diagnostic on Phase249's pinned U device_gnss.csv
completed successfully: 1102 epochs, 18023 finite adjacent carrier pairs,
zero >20000-cycle jumps, zero nonadjacent identity pairs within 1.5 s, zero
shared L-D mask rejections. Thus the two Phase363 hypotheses have no observed
support in either H or U parser populations.

Read-only aggregate summaries from Phase234 H and Phase249 U show:

| Aggregate | H | U |
| --- | ---: | ---: |
| Problem epochs | 3140 | 1102 |
| P factors | 101916 | 32475 |
| D factors | 66685 | 12496 |
| TDCP candidate pairs | 71499 | 17906 |
| TDCP factors | 69270 | 17289 |
| Code-phase jump rejections | 2229 | 617 |
| Gap / loss-of-lock rejections | 0 / 0 | 0 / 0 |
| Median arc length (epochs) | 5 | 4 |
| Stop epochs | 988 | 111 |

The code-phase gate removes about 3% of candidate pairs on both routes;
unlike the absent 20000-cycle event, it has measurable support. This does
not prove those rejections are wrong. A noisy pseudorange change and an
actual carrier slip can both trigger a code-minus-carrier gate.

Next discriminating audit: for the rejected pairs, count independently
valid raw Doppler/carrier consistency and tracking-state evidence using
same-run raw/model quantities. Do not remove the gate or tune its 10 m
threshold from these aggregate counts. If a changed admission hypothesis
is supported, synthetic noisy-code versus true-slip controls and both H/U
development comparisons are needed before promotion. Do not infer
out-of-sample accuracy from either route or from residual fit alone.
