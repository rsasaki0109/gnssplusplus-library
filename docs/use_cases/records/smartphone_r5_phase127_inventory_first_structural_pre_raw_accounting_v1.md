# Phase127 inventory-first structural pre-raw accounting

- Execution label: `Luna Max`
- Status: `sealed-pre-raw-no-payload-activity`
- Runner/manifest commit: `93ee772e595aac01a5de4e7357becd88581f70ec`
- Freeze commit: `5c3e66fc4b62d86b52f8e9ee8aa1f26f4cb2a8a4`
- Implementation commit: `f41d082e5170fe5dcbebbb4526c7d513f9512b60`
- Manifest SHA-256: `56f7190bb7cc71d36e4eb829713a05cdeca74dfd73840c2b202c9c89499eae1d`
- Target binary SHA-256: `653797970fbe65f199fc98dfe47ddd57ec26da66fadc4df40dff930a6d1c436a`

This seal covers the launch-free qualification boundary only.  The validator
read the audit/freeze/manifest and static implementation sources and hashed
the already-built target.  It did not materialize or open a route payload.
The raw phone GNSS, raw phone IMU, broadcast navigation, and sealed base RINEX
read counts are all zero.  Native solver launches, solution-row access,
coordinate interpretation, truth/MAT/accuracy, PDC, precomputed-coordinate,
Kaggle, rerun, fallback, and raw-content-copy counts are also zero.

The exact future matrix is MTV-A followed by LAX-T, one admitted attempt per
route.  It remains unauthorized.  After a separately committed authorization,
the runner must first inventory the raw base header and broadcast GLONASS
ephemerides at each exact rover/base query time.  Header-primary FCN, selected
time-valid `geph.frq`, `|query-toe| <= 1800 s`, FCN `[-7, 6]`, duplicate/conflict
ledger, and 100% rover/base retained-row coverage are prerequisites.  Any
failure seals that route as inventory-fail-closed with solver count zero; no
repair, fallback, or rerun is allowed.

Qualification results:

- New Phase127 focused Python tests: 11/11 passed.
- CMake-registered Phase127 focused test: 1/1 passed.
- Target build `gnss_run_tests`: passed.
- Full C++ `ctest -R '^run_tests$'`: 1/1 passed.
- The sealed Phase126 Python check remains unchanged and reports its expected
  historical app-source SHA mismatch (8 tests passed, one hash error): old
  expected `1c9340...` versus current Phase127 app `af2d74...`.  This is
  retained as historical evidence and was not rewritten.

No raw structural run, solution inspection, truth evaluation, accuracy
calculation, or Kaggle action is authorized by this artifact.  The next
permitted boundary is an independent authorization commit pinning the route
input paths/hashes, this pre-raw seal, the manifest, runner, implementation,
and target binary.  Only then may the inventory stage read permitted raw
inputs; Stage 2 may launch the pinned Phase126 structural solver only after a
route passes the complete inventory gate.
