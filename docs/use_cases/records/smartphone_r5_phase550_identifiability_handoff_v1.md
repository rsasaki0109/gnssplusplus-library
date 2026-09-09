# Phase550 — identifiability controls registered; stop shared-state tuning

Registered the constant-mode, geometry-free closure and free-position/C7-span
controls in tests/CMakeLists.txt. Recompiled the three standalone translation
units together as /tmp/phase550_identifiability: 9/9 tests passed. Full CTest
has not been built/run. Registration changes the current source hash; older
experiment manifests are historical pins, not to be rewritten to match.

Phase549 free-position test uses 84 synthetic observations, a full-rank
3-position + 7-independent-clock-group design, and an independent ionosphere
column. Constructed non-atmospheric error orthogonal to the baseline design
leaves baseline position unchanged, but the augmented design fits it exactly
with nonzero position error. This establishes a local Gaussian counterexample,
not a causal diagnosis of the real robust temporal graph.

Decision: do not continue anchor/density sweeps or route-specific switches for
the rejected joint model. Closure statistics do not identify its constant
mode. Any revived atmospheric model needs an independent constraint and an
explicit hardware-bias ambiguity treatment, not residual reduction alone.

Next performance work should audit the existing native P/DC observation
model against the permitted source algorithm: clock/signal corrections and
their code-versus-carrier consistency before adding free nuisance states.
Use current raw-only default-off baseline and synthetic invariants, avoid
reopening ruled-out TGD/ISC guesses, Hatch/readmission/floor sweeps, or truth
parameter fitting. No native runtime change or new accuracy claim this step.
