# Phase453 — verified signed rejection result and next diagnostic boundary

Phase451 is complete, not waiting: launcher return code 0, one native
invocation, 221.39416547806468 seconds, completed 2026-09-08T18:42:41.926563Z.
Revalidated with `python3 scripts/verify_native_imu_bias_diagnostic.py 451`.
Manifest SHA256:
`8f15c4569168078d12b64c92376dac02573b76fb9ab7166836f10f5c8baa308b`.
The verifier checked pinned sources, binary and raw inputs, convergence,
diagnostic consistency, and exact baseline output identity. Candidate SHA256:
`d2c619121951d1a322ab39e15fcf16dbf7a6d61d2c651d03a4f68ea357c61e7d`.
No truth was read and no accuracy evaluation was performed.

| Native epoch | Before / after centered P mask | Rejected negative / positive | Maximum absolute centered residual (m) |
|---|---|---|---|
| 851 | 8 / 1 | 7 / 0 | 60.2731280799772 |
| 852 | 12 / 6 | 5 / 1 | 94.06206996513174 |

No rejected zero/nonfinite values in these two epochs. Both remain the only
single-epoch nominal clock-projected P rank-deficient epochs out of 1466;
this is not a full temporal FGO observability or error diagnosis.

The diagnostic verifier regression suite passed 5 tests using
`python3 -m unittest discover -s tests -p test_native_imu_bias_diagnostic.py`.
This is a narrow verification, not the full CTest/PPC gate.

## Interpretation and next measurement

The negative imbalance is established; its cause is not. Do not restore
observations or relax thresholds from these aggregate counts. Phase452 raw
P-D statistics are a different population and cannot be joined by epoch alone.

Code inspection also fixes an important sign convention before any comparison:
`pseudorangeDopplerDifference` returns integrated Doppler range change minus
the code range change. A negative *change* in code error therefore contributes
positively to this proxy (assuming consistent Doppler). A static negative code
offset need not produce any P-D jump. Thus comparing its sign directly with
the absolute centered seed residual is not a valid agreement test.

Next diagnostic must join exact raw epoch identity, satellite and signal to
each pre-mask P factor in the same native run, then summarize retained and
rejected populations separately. Preserve source epoch identity rather than
assuming retained-vector indices equal raw indices. Include missing matches
and both incoming/outgoing availability; do not interpret missing pairs as
zero. Account for the fact that `applyAdjacentMasks` can mask both endpoints
of a bad pair, so only examining the incoming pair is incomplete. Emit
aggregate counts/ranges only, never serialize positioning states as inputs.
Keep current masks and thresholds unchanged while separating temporal code
inconsistency from seed geometry/clock effects. No further truth scoring is
justified by the present diagnostic-only change.

The native raw-only performance goal remains unmet and active. No new
submission or promotion is supported by this result.
