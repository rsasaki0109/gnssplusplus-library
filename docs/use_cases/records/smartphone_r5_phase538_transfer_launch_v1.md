# Phase538 — fixed-prior transfer launch

H's preregistered joint parameters are unchanged: 3 m anchor, 0.02 m/sqrt(s)
walk density, 1.5 s gap. run_phase538_joint_transfer.py uses the existing raw
U/LAX recipe manifests and the exact Phase536 binary/source pins, adding only
the joint selector. Input byte sizes/hashes are verified before each launch.
Output hashes and prior summary metadata are evaluation-only comparisons;
saved positions are not supplied to the native executable.

U launched native PID 3932176, session 81310; verified live after launch.
Completion and accuracy remain pending. LAX has not yet launched. No tuning
using H truth, new scoring, rebuild or submission in this step. The runtime
launcher checks graph increments, complete joint binding counts, epoch and
GNSS-first identity and convergence; detailed solved-magnitude review remains
required before evaluation. No claim of heldout validation for these routes.
