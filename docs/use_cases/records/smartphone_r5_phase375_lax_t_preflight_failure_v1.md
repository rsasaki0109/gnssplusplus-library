# Phase374/375 — LAX-T retained-epoch alignment failure

Both frozen raw invocations terminated with return code 1, once each:
baseline PID 3477471, 3.191394624998793 s; variant PID 3477610,
3.4560344689525664 s. Both report GNSS-first initialization unavailable
because retained epoch counts are not identical, then fail closed.

No retry, truth read, scoring, interpolation, saved-state substitution or
submission. These failed runs do not compare positioning accuracy and do
not support accepting or rejecting the code-gate alternative on LAX-T.

Next inspect the raw epoch retention and GNSS-first/main alignment contracts
for this route. Preserve these failed attempts. Any repair must align
native raw-derived epoch identity without using saved trajectories or
truth, and needs synthetic missing-epoch controls before a separately
frozen run. Do not bypass the alignment gate or call the CLI-only preflight
a successful solver validation. The goal remains active and unmet.
