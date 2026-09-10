# Phase288 base ordering and copy-boundary qualification

Starting HEAD: `783be45`. Root-only work; no subagent.

## Current evidence

The H raw-base member recorded in Phase287 exists now. One complete binary
hash read found 11,854,125 bytes and SHA-256
`4d3e37cbe0347fa56216db54ede9e0f30731885f337f1653ab5a86afb2bb2150`.
A separate bounded header read reached END OF HEADER, found Earth-valid
approximate XYZ, and a present, exactly zero H/E/N antenna delta. No INTERVAL
field was reported by that scan. No coordinates were printed. This confirms
identity and removes a nonzero-delta ambiguity, not survey accuracy or
equivalence to the source station table. Observation rows were hashed, not
parsed or interpreted.

`fgo_gnss_imu.m:63-94` excludes residual outliers before subtracting base
correction. Native `fgo_problems.cpp:1130-1164` likewise masks before the CLI
base transaction (`gnss_fgo_imu_no_base.cpp:11814` onward). Moving correction
ahead of this mask is therefore not justified by source parity.

The ECEF-D stage copies the corrected main problem at CLI line 12346, builds
a temporary raw problem, and transfers only D rows from it. It does not
replace P with the temporary uncorrected P rows. The alternative velocity-only
handoff branch does rebuild the entire problem; that branch must not be
confused with the operational ECEF-D branch.

The miss-mask helper marks every retained row as corrected and rejects a
subsequent application before callback lookup. The new synthetic
`UpstreamBaseHandoffTest.CopiedCorrectedRowsRejectSecondApplication` exercises
the actual helper followed by an FGOProblem value copy: 12 m is subtracted
once, sigma is preserved, and repeating on the copy rejects without lookup
or measurement mutation. This tests the primitive copy boundary, not a full
CLI run or source-equivalent base model.

## Remaining integration work

Preserve this mask-before-correction and corrected-P-copy ordering. Audit
base/rover satellite clock and code-bias conventions together before adding
a new default-off current-graph selector. The existing Phase126 declaration
of station-reference verification is a contract choice, not independent
proof of station-table parity. Do not simply relax its route guard.

No solver trajectory, candidate or truth payload, station-table payload,
MAT, Kaggle or token access; no accuracy calculation. Raw base was opened
twice as described above. Production solver code is unchanged.

Validation: `cmake --build build --target gnss_run_tests -j4` exited 0.
The new handoff test plus the existing upstream residual, metre-TDCP and
observable-preprocessing suites passed all 12 tests. This is focused
synthetic validation, not full CTest or real-data accuracy evidence.
