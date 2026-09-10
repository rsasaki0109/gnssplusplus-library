# Phase233: previous-endpoint and selector-isolation validation

Primary agent. Extended the source-metre builder fixture with previous SNR
40/current SNR 60 for every satellite. Band p85=60, expected previous-endpoint
sigma=0.02 m; current-endpoint selection would give 0.002 m. All four generated
factors meet the former value. Added builder and optimizeProblem rejection
tests for each Phase117/118/120 mixture and corresponding CLI conflict cases.

Changed summary unit wording to distinguish the selected source metre sigma
from the historical wavelength-scaled helper. Configuration is set before
main problem construction and copied into gnss_first_config at app line
12013 (line numbers may shift). The ECEF-D branch rebuilds its stage problem
using gnss_first_processor, so both builders inherit the selector. This is
source-flow evidence, not a newly executed full staged dynamic-sigma solve.
More precisely, gnss_first_problem starts as a copy of the main problem;
the temporary ECEF-D rebuild transfers only Doppler rows. The authoritative
TDCP factors (including their new sigma) are copied from the main builder,
not replaced by the temporary rebuild's TDCP factors.

Build session 63895 completed exit 0. Fresh C++ tests 14/14 passed; fresh CLI
tests 22/22 passed. git diff --check passed. No full CTest claim. No raw solve,
truth/candidate payload access, MAT read, or score calculation in this phase.

Next freeze raw experiment from Phase222 best plus source metre sigma only,
retaining k=4 for a single-factor-family comparison. Both stage and main sigma
change, so initialization and stop counts may change. Report actual TDCP sigma
distribution from factor construction/summary; do not call this a full source
replica since SNR population and residual correction parity remain unresolved.
