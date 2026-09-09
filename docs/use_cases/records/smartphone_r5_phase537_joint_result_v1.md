# Phase537 — first joint candidate: modest development gain, large runtime cost

Phase536 session 53166 exited 0 after 2311.175235774019 s. Native process
3898717 is terminal. Candidate output SHA256:
4d83d222cb4c8ce6ca7943d55043d7287eea04d049c477566ea24d20c712ff69.
Structure verifier passed source/binary/raw pins, identical GNSS-first/epoch
coverage, +3140 factors and values, full code/TDCP binding counts, finite
state diagnostics and both lambda-floor stage contracts.

Residual vertical L1 state maximum 0.476393311182 m, RMS 0.173409018385 m;
maximum code correction 2.21087736979 m and TDCP correction 0.164310707747 m.
No clipping/nonfinite values; these finite meter-scale values were reviewed
before scoring, not asserted to prove atmospheric truth.

Phase537 evaluator froze output hashes then scored the single frozen candidate
and the matched Phase535 baseline. Exact-key coverage 3139/3139, finite, no
over-70-m/s events. H development score (P50+P95)/2:
- matched baseline 1.0769180514591334 m
- joint candidate 1.0595361786357798 m
- delta -0.017381872823353683 m (1.74 cm improvement)
- candidate P50 0.8331784224265367 m; P95 1.2858939348450227 m

This is a previously explored training/development route, not heldout or LB
evidence. No solver/truth tuning during evaluation; two truth payload reads
are explicitly accounted for across paired scoring calls. No MAT or saved
positioning inputs in inference.

Runtime increased from 180.879793 s to 2311.175236 s (~12.8x), while main
iterations rose only 43 to 56 (~1.30x). Extra iterations alone do not explain
runtime; factor linearization/elimination cost is a next diagnostic target.
Do not claim a profiled cause yet. Main final cost 300951.5506837501 versus
baseline 319301.97126389534; differing models make cost no accuracy proof.

Not promoted; 0.782 goal remains unmet. Next use identical frozen priors for
U/LAX transfer and investigate numerical runtime without truth-dependent
parameter changes. No active native job remains from this experiment.
