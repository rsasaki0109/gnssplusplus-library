# Phase244 source resL decision

Frozen evaluator commit c4ee439; 13 selected Python tests passed before
truth read. Candidate/truth each read once, one score calculation, native
reruns zero. All 3139 rows matched exactly and were finite/Earth-valid;
no interpolation, hold, offset reapplication, or over-70-m/s violation.

Score 1.0916048864786838 m versus Phase235 1.0769392017393964 m:
delta +0.014665684739287421 m. P50 0.8894868610236794 m;
P95 1.2937229119336884 m. P95 improved but the prespecified scalar
worsened, so do not promote Phase243 as the best H recipe.

Retain Phase234/235 reference, source metre sigma with k=4 and legacy
atmosphere-corrected TDCP observable. The new source-resL selector remains
default-off and available for equation comparisons. This finding does not
prove source resL is generally inferior; other model interactions and
route generalization remain untested. No full source-parity claim.

H is reused training/development, not heldout or leaderboard evidence.
The 0.782 m target remains unmet. Next prioritize broader raw-only route
validation of the current reference and audit route eligibility/truth
history before calling any result independent. Avoid another H-only
threshold sweep. Source affine/nonlinear geometry remains an open model
difference, but needs an isolated synthetic derivation before route tuning.
