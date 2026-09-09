# Phase489 — H NHC development accuracy: effectively unchanged

One evaluation of the frozen Phase488 candidate completed exit 0. Evaluator
and candidate/structural hashes were frozen before truth access; an exclusive
attempt marker prevents repeat evaluation. Existing Phase203 -> Phase199 ->
Phase189 exact-key scoring kernel reused unchanged. No inference invocation,
raw GNSS/IMU/nav reads, MAT input, threshold sweep or submission during scoring.

3139/3139 exact keys matched, both domain coverages 1.0, finite metrics:

- P50: 0.8397272149184783 m
- P95: 1.312923572583866 m
- (P50+P95)/2: 1.076325393751172 m
- Historical operational baseline: 1.0769392017393964 m
- Difference: -0.0006138079882243019 m (about 0.6 mm)

This is a reused development route, not heldout or leaderboard evidence.
The candidate includes both the tested lambda floor and NHC whereas the
historical scored operational baseline lacks the floor; the floor-only H
output was not separately truth-scored. Do not attribute the entire small
difference exclusively to NHC. The floor-only output previously differed from
the operational baseline by at most approximately 0.305 mm per key.

The candidate does not materially close the gap to .782 m. Keep default off;
do not tune NHC strength/gates on this near-tie. A broad improvement claim
requires frozen transfer experiments and genuinely unused evaluation data.
Next investigate whether NHC changes velocity/attitude but has little leverage
on position through the existing IMU/motion graph, using raw-only diagnostics
or synthetic observability controls rather than truth-derived corrections.
Overall requested goal remains unmet.
