# Phase376 — alignment failure scope correction

The generic retained-count error compares main keys, GNSS-first keys and
GNSS-first solution times. It does not prove the main and GNSS-first
problem epoch counts differ: missing solver output can produce the same
message. Phase374/375 emitted no native summary, so the failing vector
cannot be identified from those sealed stderr messages alone.

Added aggregate counts to the future failure log at the CLI handoff error
site. No coordinates or raw epoch series are exported; no alignment gate
is bypassed. This edit has not yet been built or exercised. Next compile
and test telemetry, then use a separately frozen diagnostic execution to
identify the actual mismatch before selecting a retention repair. Do not
automatically retry Phase374/375 or change retention based solely on the
generic error. No new solver/truth/accuracy run occurred this turn.
