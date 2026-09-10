# Phase335: H base receive-time and clock audit

Read the H raw base header and epoch marker lines for metadata only. Header
time system is GPS; first/last calendar times are 2021-08-24 20:29:58 and
21:28:17 in that system. There are 3500 epochs and no receiver-clock-offset
fields (hence no nonzero values). No coordinate or observation values were
printed or exported. This was a read-only metadata scan, not a new frozen
solver experiment, truth read, candidate read or calibration.

Native RINEX calendar parsing forms GPS week/TOW without adding leap seconds.
Added a synthetic RINEX3 test for the header's GPS calendar: week 2172,
TOW 246598, no receiver clock bias. This is appropriate for this explicitly
GPS-tagged file; it does not validate UTC-tagged RINEX or other time systems.

Inspected local MatRTKLIB Gsat: its optional receiver-clock argument defaults
to zero and the source base/rover calls do not supply that argument. Native
shared-state selection likewise does not subtract a receiver clock. The native
final broadcast clock is polynomial plus relativity, with no explicit TGD.
Added a synthetic GPS ephemeris test that changes TGD/TGD-secondary and proves
satellite position, clock and drift are unaffected. This prevents assuming a
hidden double group-delay correction at that layer; it does not prove full
orbit/relativity/reference-clock parity against an independent implementation.

## Verification and storage issue

Normal `gnss_run_tests` build failed writing assembly under `/tmp`: root
filesystem was nearly full (48 MB available at diagnosis). The old binary's
38 passing tests do NOT cover the additions. No user data was deleted.
Compiled the focused test translation unit with existing built libraries,
using `/dev/shm/gnss-phase335-compile.cJV54L` for compiler temporaries and
`/tmp/gnss-phase335-tests.6dSdyV/base_tests` for its small executable. That fresh
binary passed all 40 base-compensation tests, including both additions.
This is a focused alternate build, not successful normal build/full CTest.
Root space afterward was about 43 MB; future full builds need a storage plan.

No obvious GPS-vs-UTC or optional receiver-clock omission explains the H base
residual in these checks. Station antenna-reference interpretation and full
independent clock/orbit comparison remain unresolved. Do not change base
coordinates or tune offsets from H truth. Production solver configuration and
operational score remain unchanged; 0.782/LB objectives remain unachieved.

## Normal-build recovery (2026-09-08)

Rechecked the clean worktree and storage: root had about 46 MB free. The
existing generated `build/tests/run_tests` occupied 300,144,112 bytes. Moved
only that rebuildable executable to
`/dev/shm/gnss-test-build-recovery.BJvQt9/run_tests.before` (volatile backup,
lost on reboot), without changing raw data, results or positioning binaries.
With `TMPDIR=/dev/shm/gnss-test-build-recovery.BJvQt9`, the ordinary
`cmake --build build --target gnss_run_tests -j2` completed successfully,
including recompiling the changed base-compensation translation unit and
linking the normal test executable back at its original path.

The newly built normal executable passed all 40
`BasePseudorangeCompensationTest.*` tests and the one
`FGOSourceRoverStateTest.*` integration test. The paired-epoch CLI Python
suite passed 11 tests against the existing positioning executable. These
are focused checks, not full CTest, independent orbit/clock parity, a new
accuracy evaluation, or the real-data byte-identity refactor gate.

Root space remained about 45 MB after linking: this recovered this build,
not sustained storage capacity. No files were deleted. Future larger build
or experiment artifacts still require a storage plan. Next scientific work
remains an independent broadcast orbit/clock comparison before interpreting
apparent base residual geometry as a coordinate error or applying a change
to the production recipe. No truth or candidate files were read here.
