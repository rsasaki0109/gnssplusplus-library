# Phase398 — preserve scalar robust loss and residual-atmosphere semantics

Code inspection and synthetic tests only; no raw replay, truth, saved
position input, MAT, score or production graph change.

Current builder (`fgo_problems.cpp`, corrected_carrier and tdcp_carrier)
uses raw carrier + satellite clock - model troposphere + model ionosphere
in the tested baseline, then differences endpoints. Therefore a new ionosphere
variable would represent the change in remaining slant delay, not total delay.
For y_corrected = g-alpha*delta(I_true-I_model), prediction must be
g-alpha*deltaI_residual. Adding total ionosphere would double-correct the model.
The source-resL alternative bypasses atmosphere; it is a different convention
and must not be silently mixed into this experiment.

Added three algebraic regression controls (fresh standalone suite 11/11):

- Joint radial Huber on a two-row vector changes the baseline even with zero
  nuisance variance: residuals (4,4), k=4 give independent cost 16, but smaller
  radial-Huber cost. Thus it is not a one-parameter extension preserving the
  existing scalar-Huber factors.
- Gaussian nuisance elimination and robust nuisance optimization have different
  optima in a concrete convex fixture. Do not claim the Phase397 covariance
  primitive marginalizes the existing robust objective.
- Corrected-carrier synthetic equations require residual, not total,
  frequency-scaled ionosphere change at both frequencies.

Design choice for the next prototype: retain independent scalar Huber
measurement rows and introduce a shared explicit residual slant-change
nuisance for an exactly matched satellite/endpoint dual-frequency pair, with
one Gaussian prior. No extra transformed observations. Unmatched rows remain
unchanged. A finite fixed prior is required, with a separately justified
physical scale; the synthetic values are not production recommendations.
At disabled mode no new keys or altered factors may appear. Pairing, Jacobian,
clock units, reset handling and same-run provenance require native tests before
any raw accuracy experiment. This is a research design, not validated accuracy.
