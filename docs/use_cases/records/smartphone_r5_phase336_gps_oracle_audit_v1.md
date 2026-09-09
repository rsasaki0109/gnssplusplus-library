# Phase336: independent GPS broadcast propagation diagnostic

Before the raw-navigation pass, fix the comparison to all valid GPS messages
parsed by native RINEX from H `brdc.nav`, evaluated at toe minus 3600 seconds,
toe, and toe plus 3600 seconds. Compare native propagation with separately
compiled MADOCA RTKLIB `eph2pos` using the same mapped broadcast elements.
Report sample count and maximum position norm/clock-in-metres differences
only. Fixed diagnostic tolerance: 0.001 m for each. No receiver positions,
MAT files, candidate files or truth. This does not test independent RINEX
parsing, message selection, transmission-time correction, other constellations,
or exact pinned MALIB parity. It is not an inference dependency.

MADOCA source revision: `0089f7dc97e8e2ba283a40be2edf4b73a140df6c`.
SHA256 inputs and implementation:

- brdc.nav: `147d948f0eba3bf09e295e7f67fbe8db60c25e236bc3c7d958dc933483f10909`
- ephemeris.c: `a80049dc0d17f5c82a990bebbf7c58d187c270007c8953e8e8b00455b6dceeb6`
- rtkcmn.c: `259419b4f8099f5893e43f95b24e83a064cb0d62efef28b675e57c35e0912e75`
- rtklib.h: `dc3a2b4bf0287b6cc25abeda102f0ef73272f1000fe9b26cda4efd2714e837bb`
- scripts/native_gps_broadcast_oracle_audit.cpp: `f913fc64bb9524ec7c0cb9d361c76c3153630240059223db14f5f60a70cc2604`
- diagnostic executable: `13eec34c5a9d28b5a6194f498db1676cef3863ef0d34c74299c5a9b784f7d407`

Built external C files with gcc `-O2 -ffunction-sections -fdata-sections`;
C++ diagnostic with `-std=c++17 -O1`, native built libraries and
`-Wl,--gc-sections`. Temporary objects/executable are under
`/dev/shm/gnss-test-build-recovery.BJvQt9` because root storage is nearly full.
An initial link missing native solver dependencies failed; the complete link
succeeded. No scientific run occurred during that failed build.

## Result

The one raw-navigation diagnostic pass exited 0 with 1200 comparisons
(400 valid GPS ephemerides at three offsets). Maximum position difference
was `2.4990007029843962e-08` m; maximum clock difference expressed in metres
was `1.4220308998803659e-09` m. Both are below the preregistered millimetre
threshold. GPS broadcast propagation with matched elements at these epochs
does not exhibit a metre-scale disagreement with this independent source.
This narrows the next investigation to observation/time/reference handling
and other measurement-model terms; it does not validate receiver coordinates
or explain the base-correction regression. Production code and score unchanged.
