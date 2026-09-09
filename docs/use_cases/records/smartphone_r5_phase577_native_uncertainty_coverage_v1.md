# Phase577: native raw ADR uncertainty coverage

Built `scripts/native_adr_uncertainty_audit.cpp` directly with the current
Android reader, observation and types sources, no old ABI objects/libraries.
Pixel5 device mapping; require raw Android clock; enriched pseudorange
verification disabled. No nav, IMU, saved positions, truth, MAT or optimizer.
One raw pass per H/U/LAX/A completed successfully (session80069 exit0).

| Route | Epochs | Native rows | Metadata present | Carrier valid | Valid carrier missing sigma | Masked carrier with metadata | Valid-carrier median sigma m |
|---|---:|---:|---:|---:|---:|---:|---:|
| H | 3140 | 109747 | 90310 | 77462 | 0 | 12848 | .0028421844273806814 |
| U | 1102 | 35810 | 27230 | 20552 | 0 | 6678 | .0032108332879374254 |
| LAX | 1466 | 51243 | 39623 | 29857 | 0 | 9766 | .002930522447223262 |
| A | 2159 | 84268 | 68386 | 54572 | 0 | 13814 | .0024208662868967453 |

Native masks/mapping reduce H/A carrier counts versus the broader Phase574
raw ADR-bit inventory; A median correspondingly differs. These are native
loader counts, not final admitted TDCP rows. Missing uncertainty does not
block the sampled native valid carriers, but it must remain handled in the
general API. Uncertainty metadata neither restores masked carrier validity
nor proves receiver calibration or independent endpoint noise.

Binary `/tmp/phase577_native_adr_audit` SHA
5d02072f8a805c464004122fe9a59ed2a0f69c5821c4b5ab341b934c9030e0bc;
source SHA d9af7197dddee5735f489f05a2a7401f58ca99c02bac80fe76c0b8b0951738af.
Raw input paths are existing Phase37 H/U/LAX and Phase25 A. No new accuracy
evaluation or runtime FGO integration. Full native build12146 remains live,
last around97% compiling FGO problems/base compensation. Disk ~1.2GB free.
Resume it; then verify unchanged full positioning output before covariance
candidate integration. Full CTest unrun; goal remains unmet.
