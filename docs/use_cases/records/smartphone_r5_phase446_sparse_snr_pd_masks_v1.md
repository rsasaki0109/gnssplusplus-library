# Phase446 — raw SNR and adjacent P-D mask attribution

Extended the native raw epoch audit with actual upstream::applyAdjacentMasks
on all raw-loaded Pixel5 epochs, 1.5 s default continuity, and the active
20 dB-Hz SNR floor. Compiled/linked successfully; no FGO/native inference,
navigation, saved positions or truth input. Native FGO's input-domain identity
must still be checked for exact downstream attribution.

| Epoch | Raw/selected | SNR below 20 | Adjacent P-D masked | Union |
|---|---:|---:|---:|---:|
| 850 | 17 | 4 | 3 | 7 |
| 851 | 14 | 2 | 3 | 5 |
| 852 | 18 | 2 | 3 | 5 |
| 853 | 20 | 1 | 4 | 5 |

For deficient epochs 851/852, these two raw-domain checks leave 9/13 rows,
while the final native main graph contains only 1/6 P factors. Thus these
checks alone do not account for all missing P support. Further signal,
geometry, elevation and centered seed-residual screening remain relevant.
Do not label the remaining 8/7 rows as any specific mask without tracing it.
This is overlapping predicate accounting, not exclusive rejection order.

No support for bypassing the SNR or P-D masks; no parameter or model changes,
no additional accuracy evaluation. Next inspect downstream geometry and
centered P-residual admission with same-run raw-derived states.
