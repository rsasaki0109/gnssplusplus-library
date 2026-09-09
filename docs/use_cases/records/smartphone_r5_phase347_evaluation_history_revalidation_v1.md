# Phase347: revalidate evaluation roles before further performance experiments

Inspected existing record metadata and raw-input path names only. Phase37
materialized raw GNSS/IMU/nav bundles exist for Pixel5 MTV-H, MTV-U and LAX-T.
These paths are not evidence of fresh holdout status. H is heavily reused
development; U already has development accuracy history. Do not relabel
either as independent validation when moving away from H diagnostics.

Phase24's route-group split originally reserved
`2023-05-09-21-32-us-ca-mtv-pe1/pixel5` for fresh validation. The authoritative
Phase34 validation result now reports status `validation-pass`, one validation
truth open and development-only promotion. That exact evaluation is historical
validation evidence, not permission to call a new run on it fresh/untouched.
The result explicitly does not establish native 0.782-class performance.

Phase24 reserved `2023-05-16-19-54-us-ca-mtv-xe1/pixel5` as future holdout.
Phase34 reports zero opens then. This limited review does not establish its
present never-opened status across all later records. Do not access that
truth on the basis of the old counter; a current comprehensive route-group
truth-use inventory and frozen candidate/evaluation protocol are required
before any claim of fresh holdout evidence.

No archive payload, ground truth, candidate CSV or new raw observation was
read in this turn. No new split was declared, no holdout was opened and no
performance experiment was launched. This review prevents recycling historical
validation as independent evidence. The active goal remains the raw-only native
pipeline and independently supported performance, not accumulating H-only
diagnostic passes. Root storage remains very constrained (about 44 MB at
the preceding check); existing tmpfs diagnostic builds do not solve space
requirements for a new multi-route experiment.
