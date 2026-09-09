# Phase532 — staged joint graph replacement and temporal priors

Added joint_ionosphere_graph_plan.hpp. Validates provided epoch keys and exact
factor-index bindings, stages code/TDCP wrappers, zero initial scalar states,
segment anchor priors and zero-mean random-walk BetweenFactor<double> edges.
Input graph/Values remain const throughout validation/planning. Returned
replacements overwrite original indices; priors are the only appended factors.
Reject duplicate replacement indices, invalid epoch order, key collisions in
existing values OR graph references, bad coefficient/noise/time configuration.

Two actual GTSAM tests added to registered code-ionosphere suite: three states
across two segments produce two anchors plus one walk; replacing one original
code factor and adding priors preserves zero-state cost without duplication.
Invalid bindings leave the original graph/Values unchanged. Fresh standalone
/tmp/phase532_joint_graph_plan passed 5/5 (two new, three existing). No full
CTest, production native rebuild or raw run this phase.

Boundary: caller must supply exact code/TDCP identities, complete desired
coverage and physically justified prior parameters. This helper validates
indices but cannot infer whether a referenced scalar factor is the intended
observation. It permits prior-only epochs; fixture tests use them deliberately,
not as evidence of GNSS observability. Gap-reset temporal priors do not silently
drop measurement edges across gaps. A future lane must define that policy.

Next wire a dedicated default-off main-only selector to collect actual inserted
factor indices and apply the plan after graph construction. GNSS-first must
remain unchanged. Existing legacy ionosphere guards remain intact. No physical
prior setting frozen yet, no truth/MAT/saved positioning input, no accuracy
claim. Overall .782/LB goal remains active and unmet.
