# Phase292 satellite transmission pseudorange slots

Starting HEAD `8c3eaa3`; root-only. Source-selection primitive added to
source_transmission_clock.hpp; production paths remain unchanged.

Local MatRTKLIB src/mex/obs2obs.c:450 explicitly orders slots as
L1,L2,L5,L6,L7,L8,L9. compile.m:18 defines NFREQ=7. Missing fields are zero
initialized; NaN is converted to zero when building obsd_t. Combined with
the pinned satposs evidence in Phase291, selection is first nonzero slot,
not first arriving observation, smallest SignalType or unconditional L1.

The new pure selector accepts seven slots from one satellite/epoch and
returns slot provenance plus metres. It preserves finite negative values
at this source-selection layer. Selected infinity fails closed (a deliberate
stricter native contract than passing it to propagation); all missing returns
no selection. No averaging or fabricated pseudorange is performed.

Synthetic tests cover L1 precedence, NaN/zero fallback through L2/L5,
negative finite selection, selected infinity rejection, all absent and L9.
Input masks and signal-to-slot assignment are caller responsibilities, not
proven by this primitive. Native Observation grouping, duplicate tracking
code policy, ephemeris selection and final state generation remain to be
connected and tested before any accuracy run.

No real raw/truth/candidate, MAT, station table or network/Kaggle access.
Existing base tests use synthetic RINEX fixtures. No new score.

Validation: gnss_run_tests target build exited 0; all 20
BasePseudorangeCompensationTest cases passed. Not full CTest or solver parity.
