# Phase303 explicit default broadcast selector

Starting HEAD ab23cfdc; root-only. Reviewed seleph/selgeph/selseph and
MAXDTOE constants in pinned MALIB 159e150d4a54e6b7b15d81128289b8559523ca81
ephemeris.c and rtklib.h through in-memory HTTPS.

New source_ephemeris_selection.hpp implements IODE-agnostic nearest-toe
selection with last equal-age record winning. Limits in seconds: GPS/QZSS/
NavIC 7201, Galileo 14400, BeiDou 21601, GLONASS 1800, SBAS 360.
Galileo defaults require bit 9 I/NAV and strictly positive age. No environment
override or shared NavigationData selection behavior is changed.

Input order remains authoritative, and returned pointers borrow that vector.
Caller must preserve lifetime and source ordering. Native invalid records
are skipped and malformed matching toe is rejected; these are explicit
native admission checks. Health remains separate, as in source selection.
Navigation reader ordering/deduplication parity is not established here.

Synthetic tests cover all seven systems at/in excess of age limits, last
duplicate-age tie, and Galileo equal-toe/FNAV rejection plus INAV admission.
Selection has not yet been joined to the epoch pipeline or enabled in FGO.

No real raw/nav/truth/candidate/MAT/station-table/Kaggle/token input. Only
algorithm sources fetched. Tests use synthetic fixtures; no score or solver.

Validation: gnss_run_tests target built; all 30 base-compensation tests passed.
No full CTest or real-data/source-order parity claimed.
