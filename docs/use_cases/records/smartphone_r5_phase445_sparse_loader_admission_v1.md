# Phase445 — sparse epoch loss occurs after native raw loading

Added `scripts/native_raw_epoch_admission_audit.cpp`, compiled against current
native loader/libraries, and ran once on raw LAX-T device_gnss.csv with epoch
range 850–853. Clock-only parsing, no nav, solver, saved positions or truth.
Initial link lacked solver-library dependencies; corrected static-library
grouping and linked successfully. Diagnostic executable is
`/tmp/gnss_raw_epoch_admission`.

Native loader selects every raw row in these epochs: 17/17, 14/14, 18/18,
20/20 respectively. Empty loader reason is the selected-row convention.
Thus the 14→1 and 18→6 P-factor reductions at 851/852 occur downstream of
loader selection, not through unsupported-signal or basic raw timing removal.
This does not identify which downstream preprocessing/geometry mask rejected
each observation, nor show that rejected rows are usable.

Independent raw-field aggregate check found no zero receiver time, transmit
time below 1e10 ns, or BiasUncertaintyNanos over 1e4 in these four epochs.
MultipathIndicator is 0 throughout (do not equate this with proven no multipath).
CN0 minima are 18.0–18.5 dB-Hz; no CN0-based causal rejection claim was made.
Next inspect native downstream P admission masks at these same raw identities.
No new accuracy evaluation or parameter change.
