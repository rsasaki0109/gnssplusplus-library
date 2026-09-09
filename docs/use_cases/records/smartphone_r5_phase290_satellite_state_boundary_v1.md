# Phase290 satellite state and geometry boundary

Source inspection at HEAD `7024bef`, clean starting tree, root-only.
Previous turn added a passing code-bias cancellation test (progress).

## Confirmed current behavior

- Base source lines 287-321 and rover fgo_problems lines 540-569 both
  evaluate at epoch minus the row pseudorange/c, subtract the returned
  satellite clock bias, evaluate again, and select ephemeris at the updated
  time. Both operate inside the observation-row loop. Neither is missing
  that second evaluation.
- Legacy base and rover both rotate satellite position once using geometric
  travel time. NavigationData::calculateGeometry (navigation.cpp:332) is a
  plain Euclidean distance plus local azimuth/elevation calculation; it does
  not add another Sagnac term. Do not fix a nonexistent double correction.
- Source-complete base instead uses Phase126 first-order geodistWithSagnac;
  the rover builder still uses rotated satellite coordinates. This is not
  bit-equivalence, even though both account for Earth rotation.
- Broadcast clock bias includes relativity (navigation_internal.hpp:915).
  The polynomial-only intermediate drift is superseded by clock finite
  differencing in navigation.cpp:51 when those evaluations succeed. It is
  not sufficient evidence for a missing-relativistic-drift bug.

## Newly narrowed source-parity uncertainty

Gsat.m:172 calls satposs; the MEX wrapper src/mex/satposs.c returns m-by-nsat
state arrays and calls RTKLIB with nsat observations per epoch. Thus source
state output is satellite-level, not separate per frequency. Native state
preparation is per signal row and uses that row's pseudorange. This can
produce different transmit times across bands; magnitude and accuracy effect
are not established here.

The cached MatRTKLIB submodule status reports RTKLIB commit
`159e150d4a54e6b7b15d81128289b8559523ca81` as uninitialized. Local searches
did not find its ephemeris.c. The exact source pseudorange selection and
initial clock evaluation policy therefore remain unverified; do not assume
first-L1, average bands, or substitute another RTKLIB revision from memory.

Next concrete step: obtain/read that pinned source as algorithm code (not
data), resolve the satellite-level pseudorange selection, and decide whether
current paired native state preparation is retained or a coherent shared
source-state adapter is required. Preserve legacy admission and avoid a
base-only geometry/clock change. This uncertainty does not block continued
source work, and does not authorize claiming source completeness.

No production change, solver/build/test invocation, real raw/truth/candidate
payload read, MAT, station table, network request or Kaggle/token access.
No accuracy improvement is claimed. Only source and existing audit text were
read, plus local git submodule metadata.
