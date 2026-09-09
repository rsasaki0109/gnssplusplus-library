# Phase578: user-approved accuracy acceptance gate

User explicitly allows non-identical positioning output if accuracy is at
most 1 m. Apply this as (P50+P95)/2 horizontal error in metres <=1.0, using
the established metric. Byte identity becomes informational, not an accuracy
acceptance requirement. Retain raw-input/source/binary provenance, finite
solutions, explicit coverage/domain checks, and no inference leakage.

Do not reinterpret the new threshold as a proved result: current H baseline
~1.0769m and A~1.3613m fail it. A route passing does not prove all routes or
leaderboard performance. The broader .782-class/LB objective remains an
aspiration, not achieved by relaxing this intermediate acceptance gate.

Interrupted build12146 confirmed terminal exit0, complete native executable
rebuilt with Observation layout and ADR uncertainty metadata. Focused native
Android loader tests20/20 passed in Phase576. No factor/weight change yet.

`run_phase578_adr_metadata.py` freezes current H raw-only command, sources,
binary and raw pins before launch. Exact output hash comparison informational
only; no accuracy read by runner. Completion must precede separate evaluation.
No score/submission claimed by this record.
