# Phase465 — actual C7 temporal constraint semantics

Read native construction and cached original gsdc2023/fgo_gnss.m source.
Lines 100-101 use Diagonal.Sigmas with [sigma_motion_clk; zeros(6,1)], and
the jump variant [Inf; zeros(6,1)]. Native ordinary-edge construction matches.

Existing native comments incorrectly called six zero sigmas zero-information
entries. Linked GTSAM documentation and native_clock_noise_audit.cpp confirm
they create a Constrained model. With sigma0=1 and all residuals=1:
isConstrained=1, whitened tail norm=sqrt(6), squared distance=6001. This
distance includes a constrained penalty, not an ordinary variance estimate.

Corrected comments only in fgo_gtsam_internal.hpp/fgo_gtsam_backend.cpp.
No model or parameter change. Six non-C0 clock components already have
temporal equality constraints on eligible edges. Adding smoothing on the
premise that they are unconstrained would be misguided.

Next audit source jump behavior: original jump noise removes C0 information
but retains six constraints; native decisions can skip an entire edge.
Confirm branch activity in the actual recipe before proposing changes.
No real-data defect or accuracy improvement is established by this finding.
No raw solve, truth, MAT payload or submission. Goal remains unmet.
