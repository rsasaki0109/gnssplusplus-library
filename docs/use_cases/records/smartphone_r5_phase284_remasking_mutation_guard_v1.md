# Phase284: reject post-pool correction mutation

Inspection found later base-compensation code can mutate corrected P rows
in the CLI (around 11979-12006). Base modes are already rejected by the
new CLI selector. Added a second defense in the pure reselector: every old
accepted key must match its retained pool row's corrected P, sigma, clock
group and corrected satellite position exactly. Differences throw before
selection can replace the main vector. This prevents these fields being
silently reverted by copying the older pool.

Tests mutate measurement, sigma and satellite position independently and
require rejection. Build 50863 completed both test and native CLI targets
with exit 0. Five selected C++ tests and fourteen CLI tests passed. No full
CTest, real-data byte-parity, or full arbitrary-field equality claim.

The next frozen raw experiment must use the operational Phase234 argv plus
only --native-pseudorange-remasking, with no relative height, prior omission,
source resL or affine options. Require unchanged GNSS-first aggregate
handoff, positive recovered rows, old=unchanged+removed,
new=unchanged+recovered, exact output coverage, finite convergence and no
fallback. Original builder counts describe pre-remasking selection; compare
actual final P count with the new count. No threshold changes after outcome.

No raw run, truth read, MAT payload, saved positioning input or submission
occurred during this implementation/test step. Accuracy remains unproven.
