# Phase343: avoid repeating already evaluated re-masking

Revalidated existing implementation and aggregate evaluation records before
starting a new initialization experiment. The source .m GNSS/IMU stage uses
GNSS-first or previous-pass estimates; its MAT storage mechanism is forbidden
here, while same-process raw-derived state handoff is already implemented.
Native initial admission uses SPP seed residuals. Phases280-284 already
implemented retained pre-mask pools and same-run GNSS-first re-selection.

The authoritative Phase286 aggregate result reports H development score
1.0781431256597123 m versus operational 1.0769392017393964 m, delta
+0.0012039239203158747 m, not improved. Only the JSON evaluation record was
read in this review; neither the candidate nor truth payload was reopened.
Do not rerun this same base-off re-masking experiment as a new hypothesis.

Phase284 also documents a necessary mutation guard: accepted factors must
still match the original pool's corrected P, sigma, clock group and satellite
position before re-selection. Base modes are rejected at CLI admission.
Removing that guard would risk replacing corrected factors with pre-correction
pool entries and is not an implementation of source-compatible re-selection.

Any future paired base/re-masking experiment needs a deliberately composed
transaction: select using the intended pre-base residuals, apply base correction
to the selected rows once, preserve support bookkeeping, and test that no
uncorrected rows are restored. Current evidence does not yet justify choosing
that experiment over resolving the base-reference discrepancy. No new solver
change, raw run, calibration, score or promotion in this review.
