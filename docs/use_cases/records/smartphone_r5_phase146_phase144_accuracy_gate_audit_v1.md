# Phase146 Phase144 accuracy gate audit

This is the launch-free plan for one offline accuracy evaluation of the
immutable Phase144 raw-result opaque outputs.  The candidate is explicitly
`Phase144 raw result opaque_solution`; Phase141 and Phase142 candidate files
are forbidden.  Phase145 revalidated the Phase144 native summaries and kept
the historical Phase144 result immutable.

The evaluator delegates parsing, exact `(phone, UnixTimeMillis)` joining,
warm-up exclusion, Haversine distance, linear P50/P95, route scalar,
continuity speed, and fixed MTV-A then LAX-T macro aggregation to the pinned
Phase142 evaluator and Phase82/Phase76 implementation.  The Pixel5 offset is
already applied once by the Phase144 structural run; this lane applies no
coordinate transform (`pixel5_offset_reapplication: 0`).

The pinned gates are strict full alignment and finite/Earth-valid candidate
rows, prediction-domain coverage exactly 1.0, zero transitions over 70 m/s,
two routes in order with one read and one score each, and macro strictly less
than 0.782 m.  Equality fails.  Result records contain only route metrics,
aggregate metrics, gates, and read accounting; no coordinate rows.

Before authorization this plan reads only sealed JSON/source metadata.  A
separate authorization commit is required before exactly two candidate and two
truth payload reads.  No solver, raw GNSS/IMU/navigation/base, MAT/PDC,
precomputed coordinate, Kaggle, rerun, repair, fallback, or submission lane is
permitted.  Passing this two-route validation is not leaderboard proof and does
not authorize submission.
