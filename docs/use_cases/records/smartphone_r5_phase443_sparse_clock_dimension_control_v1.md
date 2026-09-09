# Phase443 — clock dimension versus pseudorange count

While Phase442 remains live, checked the dimensional bound relevant to its
pending topology output. With m whitened P rows and rank r clock Jacobian,
clock-projected position rank is at most min(3,m-r). Thus four P rows do not
provide three unconstrained position directions if two independent clock
combinations must be estimated from those rows alone.

Synthetic NumPy control: position Jacobian rows (1,0,0), (0,1,0), (0,0,1),
(-1,0,0); clock rows (1,0), (1,0), (1,1), (1,1). SVD projection verified
clock rank 2 and projected position rank 2, matching upper bound 4-2=2.
No raw or saved positioning data, truth, solver changes or scoring.

This is a dimension control, NOT a finding that either deficient LAX-T epoch
has this topology. Pending native logs will supply actual epoch/P/clock counts.
Incoming/outgoing TDCP counts identify edges but not independent geometric
constraints or their weights. Full FGO includes temporal clock and IMU terms;
do not translate the single-epoch rank bound into full-graph failure.

Phase442 session 59478 / native PID 3628580 was verified live. Preserve its
source pins and await completion; do not change masks or add priors on this
synthetic evidence alone.
