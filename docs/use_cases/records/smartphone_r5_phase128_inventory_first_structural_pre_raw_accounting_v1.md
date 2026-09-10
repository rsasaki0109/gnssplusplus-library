# Phase128 inventory-first structural pre-raw accounting

- Execution label: `Luna Max`
- Status: `sealed-pre-raw-no-payload-activity`
- Contract audit commit: `c2557ac7b99f2fd2827233900fbe88a10a9b1695`
- Contract freeze commit: `751dbfff4fe21c1d9642a27c016fcab2cd317afa`
- Implementation commit: `50357e8f3eabbb2eca672b00a16eb2ce32cc2f16`
- Runner/manifest commit: `0bc9b1a53b0c798ae6d331504ba4f48c93eb2ee1`
- Manifest SHA-256: `8be91bc2e80a7228dd66a3facae1e719e59b6e356de209b5101488ca1bf02ac6`
- Target binary SHA-256: `d823b031d865bde973d3bd6ffcf2957288bccf29d66bf89ade3dea11d108bfe0`

This seal covers only launch-free qualification.  The validator checked the
static audit/freeze/manifest, implementation source pins, and built target;
the synthetic inventory gate was exercised entirely in memory.  It did not
materialize or open a route payload.  Raw phone GNSS, raw phone IMU,
broadcast-navigation, and raw-base RINEX/header/hash reads are zero.  Native
solver launches, solution-row/coordinate access, truth/MAT/PDC/precomputed
coordinate/accuracy/Kaggle access, reruns, fallbacks, and raw-content copying
are also zero.

Qualification results:

- Phase128 parser/provenance Python tests: 8/8 passed.
- Phase128 inventory-first structural Python tests: 10/10 passed.
- CMake-registered Phase128 Python lanes: 2/2 passed.
- `gnss_run_tests` target build: passed.
- Full C++ gtest binary: 1,160 tests; 1,102 passed, 58 skipped, 0 failed.
- The historical Phase127 source-hash mismatch remains fail-closed evidence;
  its old validator/result was not rewritten.

The frozen route matrix is exactly MTV-A followed by LAX-T, one attempt per
route.  It remains unauthorized.  After a separately committed independent
authorization, each route must first inventory the permitted raw phone GNSS,
raw phone IMU, broadcast nav, and sealed raw-base RINEX once.  The inventory
must distinguish header absent/valid-empty/entries/malformed, preserve
per-record canonical nav reject reasons, use typed satellite keys and native
GPST query/toe, normalize only `data[10] > 128` by subtracting 256, enforce
FCN `[-7,6]` and `|query-toe| <= 1800 s`, and certify every retained rover and
base GLONASS row.  Any missing, malformed, conflicting, tied, out-of-time,
out-of-range, nonfinite, or partial condition seals that route with solver
count zero.  Only a complete route may enter the pinned Phase126 compound
structural recipe with Phase127/128 and Phase118 selectors.

This seal does not authorize raw reads, solver execution, solution inspection,
truth/accuracy evaluation, publication, or Kaggle.  The next boundary is an
independent authorization that pins this seal, the manifest/runner,
implementation, target binary, and route input path/hash metadata.  The
historical Phase127 hash artifact must remain unchanged.
