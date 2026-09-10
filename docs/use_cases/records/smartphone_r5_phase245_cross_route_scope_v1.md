# Phase245: broader validation scope

Metadata/path inspection only; no truth CSV opened or new solver run.

Do not describe pe1 as fresh: Phase34 quality-anchor validation result
explicitly records one truth read and scores for
2023-05-09-21-32-us-ca-mtv-pe1/pixel5. Later Phase44 metadata saying it
remains sealed is insufficient and contradicted by this earlier result.

The exact future xe1 Pixel5 identity has reserved-holdout references, but
this inspection does not prove global non-use. Other xe1 dates/phones have
materialized truth paths. Preserve the reserved route until a route-group
and cross-phone history audit is complete; do not infer independence from
absence of one local truth path.

For immediate broader *development* validation, choose MTV-U
2023-03-08-21-34-us-ca-mtv-u/pixel5. Phase37 raw GNSS/IMU/nav paths exist;
Phase44 already labels this a development route. Native source quality
metadata identifies U as Street, matching H's current motion setting.
LAX-T is also materialized but Highway; source motion-preset generality is
not yet implemented, so do not silently claim a Highway source preset.

Next freeze the Phase234/235 reference recipe on U with source resL OFF,
source metre sigma ON and k=4. Update raw input pins and route-dependent
expected counts; do not carry H's 3139/66685 count gates into U. Do not
change estimator parameters after viewing U accuracy. This is a transfer
diagnostic on previously used development data, not heldout validation or
leaderboard proof. No precomputed trajectories become inference inputs.
