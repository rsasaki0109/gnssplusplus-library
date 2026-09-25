# A325G source-initialization all40 submission candidate

Status: locally audited, not submitted. Official score unknown.

Candidate: `E:/rtklib_v2_ws_output/gsdc_native/all40_a205u_a325g_source_init_submission_v1/submission.csv`

SHA-256: `119279101c2cf3bf50b8b64f2b195cb1ba8e7939b903cc5a8477cc41799e9125`

The candidate contains 71,936 official keys across all 40 test drives. Every position is from the native C++ estimator; no sample coordinates, output interpolation or coordinate hold are used. Input hashes, run arguments, executable and native output provenance were rechecked during assembly.

Relative to official submission 56479759 (Public 1.698 m / Private 1.333 m), only the two samsunga325g drives change: 1,423 May12 rows and 1,861 June22 rows. The other 38 drives / 68,652 rows are identical. The executable and all other inference options are unchanged; these two drives enable source velocity-derived attitude and zero-bias initialization.

The motivating exposed train A325G comparison improved local score from 11.929479401 to 1.317772064 m. Test accuracy cannot be inferred from that improvement. Test position changes reach 10.03 m on May12 and 55.48 m on June22; these are differences, not verified error reductions. GNSS-first summaries and seed metadata match each test control. No test ground truth was read.

The reviewed CSV remains immutable. A new external submission requires explicit user instruction under the active goal. This candidate alone does not establish taroz parity, and no official score is claimed.

Detailed provenance: `use_cases/records/gsdc2023_all40_a325g_candidate_review.json` and the candidate directory's `manifest.json` / `review.json`.
