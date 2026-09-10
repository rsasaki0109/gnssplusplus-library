# Phase379 — insufficient per-epoch P support blocks raw staging

One frozen diagnostic invocation, PID 3485424, returned 1 in
2.471742880064994 seconds. Manifest SHA256:
`bf07a8e7af19e0631eaf9ce2959f8be6e9398cee36f5462675fb727c3e5aabb4`.
The rebuilt backend emits: invalid seed or insufficient P support;
fewer_than_four_P=1. Main/raw/GNSS-first counts all 1466, solution count 0,
iteration count 0, C0/D factor count 0, termination attempted false.

At least one retained epoch has fewer than four P factors, causing the
raw-staging admission loop to reject the entire graph before optimization.
The compound condition may have additional failing seed predicates; this
diagnostic proves the P-support predicate fails, not that all others pass.

Next examine sparse-epoch observability in the coupled Point3/V/C7/D graph,
and distinguish valid raw-derived seeds from actual seed failures. A
per-epoch standalone four-P requirement is stronger than a joint graph's
observability requirement, but removing it without checking clock-component
gauges and temporal constraints is unsafe. Require synthetic sparse epochs
with connected and disconnected clocks before an opt-in admission change.
Do not delete timestamps, fill with saved positions or bypass seed validity.
No truth read or accuracy score; LAX-T comparison remains incomplete.
