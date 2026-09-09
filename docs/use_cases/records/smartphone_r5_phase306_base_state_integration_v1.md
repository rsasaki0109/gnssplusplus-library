# Phase306 opt-in epoch states inside base correction model

Starting HEAD 3109a6ca; root-only implementation. Config adds default-off
use_source_epoch_states, requiring source_complete equations. The model
builds the composed satellite state map once per epoch, then reuses states
for that satellite's observation rows. Ephemeris metadata lookup uses the
same explicit receive-time selector. No legacy propagation fallback occurs.
Diagnostics expose requested and state count. These library fields are not
yet serialized by the FGO CLI, which does not enable this feature.

Existing candidate-stream transaction remains: build begins by clearing old
streams and publishes only after successful completion. New synthetic tests
cover legacy-equation rejection, a positive three-epoch source-model build,
and later missing navigation clearing previously successful output. The
positive fixture uses SBAS with native GPS_L1CA frequency metadata and no
atmosphere, isolating integration rather than proving real atmospheric parity.

Remaining work: paired rover preparation, CLI configuration/telemetry and
raw base correction/support qualification. No accuracy claim. This is an
opt-in behavior change, not a pure solver refactor; no real-data byte parity
is claimed for the unselected path from synthetic tests alone.

No real raw/nav/truth/candidate/MAT/station-table/network/Kaggle/token read.
Only synthetic test fixtures. No positioning solver or score execution.

Validation: gnss_run_tests target built successfully; all 33 base-compensation
tests passed, including both new integration tests. No full CTest/raw parity.
