# Phase391 — H raw replay and cross-route robust-tail comparison

One native invocation completed, exit 0, 281.14163485507015 seconds.
Manifest SHA256 `c0b61fbcae60a05c705175a17ee7ca6421b9263a670d4404184dc32766ed1f2e`.
`scripts/verify_phase391_tdcp_diagnostic.py` passed. H candidate hash is
`e1e148c41fc8b87e444bd845b4a96eb8e7a56137ec91851b5c0b5d6cba64af29`,
byte-identical to Phase234. No saved positions used for inference, no truth
read and no new accuracy score. Both native stages converged.

| Ordinary TDCP postfit reconstruction | H Phase391 | LAX-T Phase390 |
| --- | ---: | ---: |
| Finite residuals | 69270 | 24964 |
| Huber-tail count | 8273 | 4259 |
| Huber-tail fraction | 0.11943121120254079 | 0.1706056721679218 |
| Huber cost sum | 292874.9206331389 | 153446.7752896991 |
| Quadratic cost sum | 774735.5311744515 | 525907.3166542781 |
| Huber/quadratic ratio | 0.3780321268977538 | 0.2917753193963875 |

LAX-T has about 5.12 percentage points more residuals in the robust tail.
Robustification is active on both routes. Different observation populations,
motion and geometry prevent attributing the positioning difference to that
tail fraction. Do not tune Huber thresholds against the development scores.
LAX-T alone uses the sparse GNSS-first admission repair; this is not a
controlled swap of identical graph support between routes.

Added `tests/test_native_tdcp_huber_reconstruction.py`: extracts the actual
production helper, compiles it with the real FGO configuration header, and
executes boundary, sign, disabled-loss, zero-threshold, nonfinite-input and
conflicting-selector assertions. Passed (one unittest, multiple C++ checks).
This validates the scalar formula, not complete factor/residual parity.

Next investigate TDCP tail support by signal/arc and its relationship to raw
measurement validity within the same native run. Prefer observable-backed
model corrections over route-specific thresholds. No performance improvement
or leaderboard claim follows from these diagnostics; objective remains unmet.
