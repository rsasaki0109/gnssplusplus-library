# Phase551 — default code/carrier correction path audit

Inspected fgo_problems.cpp's main observation preparation and the companion
receiver preparation in fgo_internal.hpp. Both default paths use:
P_corrected = P_raw + satellite_clock_m - ionosphere - troposphere - group_delay;
L_corrected = L_raw + satellite_clock_m + ionosphere - troposphere.
Main code and carrier share the same per-row satellite state and atmospheric
values, with ionosphere frequency scaling from the selected row frequency.
No default-path opposite-sign inconsistency was found in this inspected scope.

The main source stores both Earth-rotation-corrected satellite positions and
unrotated source positions for P/carrier, carrying both endpoint versions into
TDCP. This alone does not prove backend geometry chooses the correct version;
trace the actual C7/Pose3 factor constructors next before asserting no double
Earth-rotation correction. Do not change group delay based on this algebra:
the prior Phase509–513 ISC/base-cancellation audit still applies.

Optional source-resL selector handling differs between preparation helpers,
but it is OFF in the evaluated default recipe, so that difference is not
evidence explaining H/U/LAX baseline performance. No runtime modification,
new truth read or accuracy evaluation in this audit.
