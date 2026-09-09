# Phase547 — measurement-time dual-band carrier audit

Corrected audit_raw_dual_frequency_carrier.py to compute elapsed time from
integer TimeNanos differences plus TimeOffsetNanos differences, not UTC row
labels. Retains UTC adjacency/gap and hardware-discontinuity guards, and
rejects nonpositive/nonfinite or oversized measurement-time intervals.
Eight Python tests pass, including raw/UTC interval mismatch, backwards raw
clock and changing offsets. Native inference unchanged.

Raw-only H/U/LAX audit succeeded; no truth, positioning or enriched satellite
fields read. GPS/Galileo accepted paired increments and absolute P95 m/s:
- H: 12878 / 7795; 0.0246307 / 0.0241942.
- U: 1978 / 2771; 0.0290772 / 0.0314388.
- LAX: 2503 / 3229; 0.0313576 / 0.0292184.
LAX Galileo has one increment above 1 m/s (max 1.18335); valid Android flags
alone do not certify an uncontaminated arc. Counts are diagnostic pairs, not
the native accepted TDCP family and not independent samples.

These are differences of raw L1/L5 carrier increments, not vertical residual
ionosphere estimates. They contain clock/hardware/multipath effects and no
absolute arc ambiguity information. They cannot validate the constant U
state or justify integrating an arbitrary zero into an absolute correction.

Phase546's six synthetic closure controls passed: common geometry/ionosphere
cancel, differential code changes remain, but constant bias/common code error
are invisible and phase slips/differential clocks confound interpretation.
Do not promote the closure to a clean-code admission test on that basis.
Next requirement is paired raw code construction with the existing native
clock/time validity contract, not using CSV enriched pseudorange fields.
