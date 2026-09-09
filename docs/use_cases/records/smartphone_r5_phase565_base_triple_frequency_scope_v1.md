# Phase565: additional raw frequency support for independent bias diagnosis

Prior base correction experiments are not promising candidates to repeat:
Phase351 GPS-only values degraded positioning; Phase354 centering improved
that regression but remained worse than base-off. Phase334 L1/L5 neutral
combination retained residual structure, so ionosphere alone is insufficient
as an explanation. Do not change correction magnitudes from H truth.

Inspected permitted `correct_pseudorange.m`: reference residuals are formed
before its later antenna-offset operation. This observation is not new parity
proof or evidence that the raw RINEX header coordinates are surveyed correctly.

New read-only raw inventory via `scripts/audit_base_triple_frequency.py`:
H base SHA `4d3e37cbe0347fa56216db54ede9e0f30731885f337f1653ab5a86afb2bb2150`,
3,500 epochs / 30,214 GPS rows. Simultaneous finite nonzero code+carrier on
L1C/L2W/L5X: 17,433 rows across 5 satellites; L1C/L2X/L5X: 7,000 across 2.
Counts overlap and must not be added as unique observations. No loss-of-lock
or elevation screening, no continuity claim. Parser deliberately refuses
multiline GPS headers/short rows; it is a narrow inventory, not an alternative
production RINEX loader. No positioning estimates or correction tables emitted.

This establishes available L2 redundancy, previously unused in the L1/L5
residual test. Next construct a fixed triple-frequency combination cancelling
common geometry and first-order inverse-frequency-squared terms; derive and
test coefficients before raw evaluation, retain ambiguity/receiver hardware
limitations. Use it to distinguish non-dispersive/non-common structure, not
to label fitted offsets as physical ionosphere. Native loader and slip gates
are required before any inference integration. No new accuracy read/run,
no promotion, full goal remains unmet.
