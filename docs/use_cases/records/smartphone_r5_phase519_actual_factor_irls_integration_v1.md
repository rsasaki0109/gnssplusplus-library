# Phase519 — actual initial factor IRLS information

Backend diagnostic now observes each actual inserted source-clock Pose3 code
factor in Phase171 main. Calls NoiseModelFactor::unwhitenedError(initial,H)
and weight(initial), retaining the actual six pose and seven C7 Jacobian
columns, stored ionosphere coefficient and makeNoise-compatible safe sigma.
No hand-reconstruction from stale SPP residual or scalar clock. No graph or
initial Values modifications. Weights and rows are captured at insertion;
the aggregate is emitted before optimization.

For each epoch, project the coefficient against these 13 nuisance columns
with nominal sigma, then sigma/sqrt(actual weight). Report both median absolute
information values, valid epoch/row counts and downweighted count. This is
initial-state Gaussian/IRLS local information, not an optimized-state posterior
or exact robust Hessian. Compared to Phase516, pose rotational columns are
also present; compare nominal/IRLS within this same diagnostic, not blindly
equate different nuisance models. Invalid rows are counted, never silently
declared a complete measurement set; require zero invalid before interpretation.

Native target build session 54323 completed with exit 0. Runner
scripts/run_phase519_ionosphere_monitor.py passes syntax checking and freezes
source/input/binary hashes, baseline output hash and one zero-invalid monitor.
Raw H launched as PID 3846318, session 68532, observed live at record creation.
Output directory: output/smartphone-r5/phase519-h-ionosphere-monitor-v1/mtv-h/.
Completion, graph invariance and output equality are still UNPROVEN. Poll this
same handle; do not restart on observation timeout. No truth/MAT/saved solution
used for inference. Goal remains active and unmet.
