# Smartphone R5 Phase165 native raw-P mixed-system no-Doppler graph

Status: implemented and source-tested; default-off; no real route execution.

## Contract

Phase165 adds the opt-in
`--native-phase165-raw-p-no-doppler-graph` lane to
`apps/native/gnss_fgo_imu_no_base.cpp`.  The lane performs one same-invocation
raw-P solve, adapts its typed position/velocity/C0/D records, copies only the
finite position and clock into the in-memory `ObservationData` epochs, and
sets `use_spp_seed=false` before `FGOProcessor::buildPseudorangeProblem`.
The retained problem is joined back to the adapter by the exact
`(time, raw_source_index, raw_utc_time_millis)` identity.  It then runs the
dedicated GNSS-only Point3/V/C7/D graph and returns before any IMU or main
output path.  No seed file, trajectory, truth, MAT, or coordinate output is
read or published.

The lane removes only valid measured observations whose source system/signal
has no official C7 slot, and reports their count.  Supported rows are retained
through the normal corrected measurement builder, including supported mixed
constellation/frequency rows and valid TDCP rows.  Unsupported rows are not
folded into C0 or silently assigned an ISB.

## C7 and adapter semantics

`raw_p_seed::c7ClockComponentFor` in
`src/algorithms/raw_p_seed.cpp` is the single source mapping used by the raw
adapter and the GTSAM backend (`src/algorithms/fgo_gtsam_internal.hpp`):

`GPS L1/L5 -> C0/C4`, `GLONASS L1 -> C1`, `Galileo E1/E5A -> C2/C5`, and
`BeiDou B1/B2A -> C3/C6`; unsupported systems/signals return `-1`.

The raw adapter continues to certify native GPS-reference C0 when supported
non-GPS rows are present.  Absent C1..C6 entries remain unavailable in the
typed seed; the graph initializes those nuisance states numerically and
estimates observed slots from P factors.  They are not measured ISB values.
The backend strict raw graph gate now accepts any nonnegative supported C7
component and rejects an unsupported retained factor.

## Synthetic verification

The final current-source filter ran 51/51 tests:

* 26 raw-P stage tests, including rank/time/clock groups, bootstrap,
  collect-all, and no-fill policies;
* 4 same-run adapter tests, including GPS C0 with supported mixed rows,
  unsupported-slot accounting without C0 folding, and wrong epoch identity;
* 2 Phase93, 6 Phase101, 8 Phase135, 3 Phase164, and 2 Phase165 GTSAM
  tests.

Phase165 tests cover known-position recovery with GPS/GLONASS/Galileo/BeiDou
rows, an absent C6 slot with explicit weak gauge, and fail-closed unsupported
retained C7 factors.  The first fixture run correctly recovered the position
but failed only because the test expected one absent slot while its fixture
actually covered all seven C components (8/9 focused tests).  The fixture was
corrected to make C6 genuinely absent; the final run passed without relaxing
any numerical tolerance.  Existing Phase164/Phase93/Phase101/Phase135 tests
remain green.

`cmake --build build --target gnss_fgo_imu_no_base -j2` passed.  The rebuilt
binary exposes the Phase165 selector; a synthetic CLI admission check rejects
Phase165 combined with prep-only Phase159 before any input read.

No raw GNSS/IMU/nav/base payload, truth, MAT, Kaggle, or real solver route was
read or executed in Phase165.  This commit proves wiring and synthetic graph
admission only; it is not accuracy or leaderboard evidence.
