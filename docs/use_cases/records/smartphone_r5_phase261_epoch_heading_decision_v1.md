# Phase261 epoch heading decision

Phase260 completed once in 293.27471878402866 seconds, using raw GNSS,
IMU and navigation only. All 3,140 per-epoch heading seeds were inserted.
The complete GNSS-first aggregate summary equals Phase234; this does not
independently compare every internal state. Published output has 3,139
exact epochs, with no interpolation, hold or unresolved epochs.

Twenty metadata/evaluator tests passed before commit 304ec41 froze the
evaluation. Candidate and H truth were then read once each for one score.
Exact full-domain, finite Earth position, and over-70-m/s gates passed.

- Reference score: 1.0769392017393964 m.
- Epoch-heading score: 1.0770393548563864 m.
- Delta: +0.00010015311698996499 m (about +0.10 mm).
- P50: 0.8369180202653327 m; P95: 1.31716068944744 m.

No improvement established; retain the default/reference initialization.
The very small difference does not establish statistical equivalence or
reproducibility across builds/routes. Native main final costs are also
close (319301.96368482074 versus 319301.9783692179), despite different
initial costs. This supports deprioritizing heading-only initialization as
the explanation for the approximately 0.295 m gap to 0.782 on H.

H remains repeatedly used development data, not heldout or leaderboard
evidence. No submission or MAT/saved-position solver input was used.

Next inspect the remaining bias initialization and first-state prior
differences identified in Phase257, separating initial Values from actual
graph constraints. A source zero-bias seed and a native finite bias prior
are not interchangeable changes. Audit which priors are active in the
Phase171 graph before proposing another raw experiment. Do not sweep
heading smoothing windows against H truth.
