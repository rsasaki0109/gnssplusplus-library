# Phase298 H raw-base header applicability

Starting HEAD `35d69a9`. Root-only. One bounded header-only read of the
Phase63 H base.obs reached END OF HEADER. Version is 3.03, despite the
historical archive member name containing rnx2. RINEX2 implementation is
therefore not a prerequisite for this H input. Current hash was verified
in Phase288; this turn did not rehash or interpret observation rows.

Header codes:
- GPS: C1C/L1C/S1C, C2X/L2X/S2X, C2W/L2W/S2W, C5X/L5X/S5X.
- GLONASS: C1C/L1C/S1C, C2P/L2P/S2P.
- Galileo: C1X/L1X/S1X, C5X/L5X/S5X, C7X/L7X/S7X, C8X/L8X/S8X.

The GPS L2 W/X competition is concrete source-header evidence, not inferred
from a score. Added a synthetic reader fixture with that declared competition:
X appears first, W wins for both P and carrier; in the next epoch W is absent
while X remains, and no X fallback is emitted. Numbers are synthetic, not
copied observation values. Existing production flag remains default off.

Next: use the opt-in reader in a frozen truth-free base inventory to measure
retained code/slot support, then connect the paired satellite-state adapter.
No need to delay H for RINEX2. Source state clock/ephemeris selection and
FGO integration remain incomplete; this does not establish an accuracy gain.

Real input access: one raw-base header read only. No phone/nav/truth/candidate,
MAT, station table, network or Kaggle/token read; no solver or score. Tests
create/read synthetic RINEX fixtures.

Validation: gnss_run_tests built successfully; all 60 tests matching
BasePseudorangeCompensationTest.*:*Rinex*:*RINEX* passed. No full CTest or
real-data reader inventory yet.
