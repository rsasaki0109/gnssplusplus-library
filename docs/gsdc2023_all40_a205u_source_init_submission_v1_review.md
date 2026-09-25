# Native submission review

Artifact: `E:\rtklib_v2_ws_output\gsdc_native\all40_a205u_source_init_submission_v1\submission.csv`
SHA-256: `152ead0ed97cca96e6b2e645434ec36ef588be0a5a13b73a0317fffda61a8206`

All 40 drives and 71,936 official keys come from converged native states. No exported coordinate interpolation/hold or sample coordinate was used. User-authorized submission **56479759** completed with official **Public 1.698 m / Private 1.333 m**. It remains above the parity target. [Submission evidence](use_cases/records/gsdc2023_all40_official_submission_v1.json) records the upload receipt and authenticated server scores. The downloaded server payload has the exact same SHA-256 as the reviewed local CSV.

| Phone | Drives | Official keys | Missing | Interpolated/held | Median runtime (s) | Maximum runtime (s) |
|---|---:|---:|---:|---:|---:|---:|
| mi8 | 4 | 7115 | 0 | 0 | 731.0 | 1721.3 |
| pixel4 | 2 | 3070 | 0 | 0 | 369.1 | 485.5 |
| pixel4xl | 1 | 1190 | 0 | 0 | 1230.5 | 1230.5 |
| pixel5 | 17 | 34407 | 0 | 0 | 559.6 | 9073.6 |
| pixel6pro | 3 | 3754 | 0 | 0 | 239.0 | 2020.2 |
| pixel7pro | 3 | 5749 | 0 | 0 | 599.9 | 868.8 |
| samsunga32 | 1 | 1829 | 0 | 0 | 1097.6 | 1097.6 |
| samsunga325g | 2 | 3284 | 0 | 0 | 202.5 | 251.2 |
| sm-a205u | 1 | 1699 | 0 | 0 | 252.2 | 252.2 |
| sm-a325f | 1 | 1782 | 0 | 0 | 755.6 | 755.6 |
| sm-a505u | 1 | 2384 | 0 | 0 | 414.0 | 414.0 |
| sm-g988b | 2 | 2312 | 0 | 0 | 308.8 | 319.1 |
| sm-s908b | 1 | 1728 | 0 | 0 | 1790.3 | 1790.3 |
| xiaomimi8 | 1 | 1633 | 0 | 0 | 604.0 | 604.0 |

Runtime reflects shared machine load. Test per-drive P50/P95 remain unknown; the table reports coverage and execution time. The machine-readable record groups devices from the same route and retains per-drive runtime and output provenance. The original assembly manifest retains its pre-submission state; the linked submission record is authoritative for the current official score.
