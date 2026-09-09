# Phase498 — existing temporal mask cannot localize an error by itself

Compiled and ran scripts/native_code_mask_localization_audit.cpp against
current actual applyAdjacentMasks implementation, exit 0. Synthetic GPS L1,
1-second intervals, zero range rate and constant geometric range:

- Code errors [0,100,0] m: all three rows masked (one bad, two clean).
- Code errors [100,100,100] m: zero rows masked (persistent offset invisible).
- Code errors [0,0,0] m: zero rows masked.

Existing rule masks both endpoints of each discrepant P-D edge; two edges
around an impulse remove both clean neighbors. This is conservative source
behavior, not an implementation bug or evidence these exact cases dominate H.
No production mask changes, raw route replay or truth evaluation occurred.

Three-epoch edge signs may support a localization diagnostic, but cannot
prove the centre sample alone is wrong: alternating or persistent offsets
can give ambiguous evidence, and Doppler may share device errors. Do not
readmit endpoints solely because two residuals have opposite signs. Next
quantify possible collateral removal with exact raw identities and require
additional support before an admission experiment. Preserve existing slip,
clock-discontinuity and timing gates; no threshold sweep. Goal unmet.
