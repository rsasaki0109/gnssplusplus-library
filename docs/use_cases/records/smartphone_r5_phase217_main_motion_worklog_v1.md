# Phase217 main Pose3 XXVV integration — verification log

Primary-agent work, no raw run or truth access. Phase216 factor tests passed
previously; the new full-graph integration below is not yet validated.

Added default-off `--native-phase217-main-pose3-motion`, config wiring,
GNSS-first disable and requested/enabled/factor/gap-skip summary telemetry.
The backend inserts the tested Pose3 XXVV factor for positive intervals
below 1.5 s, using fixed Street sigma 0.05 m. It rejects nonfinite/nonpositive
time intervals and skips gaps >=1.5 s for this factor only. Initial priors,
IMU integration and existing factors remain unchanged. The option is a
Pixel5 Street experimental preset, not automatic setting.Type inference.

Requires Phase171 IMU main, zero finite lever arm, legacy position-motion
off, and Phase201/205/209/213 options off. CLI additionally requires the
ECEF-D initialization lane and explicit UTC fallback. Dedicated main
Doppler is deliberately off in this isolated experiment.

Extended actual handoff test for convergence and one inserted motion factor,
default-off zero counts, and rejection of main Doppler coexistence. Added
seven CLI rejection cases with synthetic route names and no raw paths.
Time-gap skip and other backend guards still need dedicated coverage beyond
the current normal-interval handoff fixture; do not claim exhaustive tests.

Build session **40454** is running for the app and C++ tests. Do not launch
raw comparison before a successful build and fresh test execution.

Verification update: the fresh app built successfully. Phase217/213/209/205
CLI rejection suites passed **22/22** (pytest session 35120), with no raw
input paths. C++ test compilation continues in session 40454; actual
main-motion handoff/convergence verification is still pending.

Final update (supersedes pending statements above): build session 40454
completed with exit zero. Fresh focused C++ tests passed **23/23**, including
actual Phase217 main handoff convergence, factor counts and main-D rejection.
The curated related regression filter passed **85/85** (Android IMU/time,
Phase194/167/143/164/165/101/93/135, raw seeds, TDCP robust-k, upstream stops).
Total C++ **108/108**, plus fresh CLI **22/22**; not a full CTest claim.
No raw Phase217 run or accuracy evaluation has occurred. Next freeze one
Phase198-best-plus-Phase217-only experiment: expect 3139 extra XXVV factors,
185699 total factors and 15700 values, with unchanged GNSS-first and IMU
configuration. Validate actual counts and gap skips before scoring.
