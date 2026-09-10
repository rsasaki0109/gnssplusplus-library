# Phase478 — evaluation-only output separation

Compared frozen Phase472 baseline and Phase476 candidate CSVs by exact
(phone, UnixTimeMillis) identity, verifying both SHA256 values before reading.
This comparison is an evaluation-only script, never a source of inference
states. No truth file was accessed. All 1466 identities match uniquely.
No interpolation, first-row projection or missing-row filling.

Spherical haversine separation (radius 6371008.8 m):

- Mean 0.00008378432197802165 m.
- Median 0.0000381267647747684 m.
- P95 0.0002729539663139396 m (linear percentile).
- Maximum 0.001295046261605395 m.

Selected graph counts match: 91242 factors, 7330 values, 1465 IMU intervals,
63 stop pose factors, 69 stop velocity factors and 69 stop epochs. Matching
counts alone do not prove every factor value is identical; native candidate
changes only the LM lower-bound selector by design and changes its solve path.

On the same aligned evaluation keys and distance metric, the triangle
inequality bounds each error difference by the maximum output separation.
The corresponding P50/P95 mean score therefore cannot differ by more than
about 0.00130 m. This is a mathematical bound, not a new accuracy evaluation,
and does not transfer to other routes or unseen data. The candidate cannot
close the large LAX-to-0.782 gap through this numerical change alone.

Keep default OFF until H/U numerical replication and controlled runtime
checks. No lambda sweep or new LAX truth evaluation is warranted for this
millimetre-scale comparison. Next extend the same fixed floor to H/U without
changing solver types or reading truth, then return to measurement/model
improvements needed for the actual accuracy target. No leaderboard claim.
