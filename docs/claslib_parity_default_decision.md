# CLASLIB parity profile default decision

Decision date: 2026-07-15

Decision: **keep the typed CLASLIB parity profile opt-in**. Do not enable the
complete profile by default in the 0.1 release.

## Evidence

The canonical 3580-epoch run passes the canonical scorecard. It matches all
3580 solution epochs, has no native-only FIX epochs, and reports 14.005 mm full
trajectory 3D RMS and 4.753 mm P95. GPS, QZSS, and Galileo each have complete
ZD row coverage and pass the 10 mm PRC/CPC RMS gates. Its scorecard is
`output/clas_p7_iono_gm_canonical_3580/canonical_scorecard.json`.

The independent public 2018-11-25 TSKB sample does not pass. On the final
400-second window, all 400 matched epochs are mutually FIX, but the native to
CLASLIB trajectory delta is 58.417 mm 3D RMS and the final delta is 63.534 mm.
The filter-state comparison reports 43.098 mm component-wise float-position
RMS and only 53.4% common diagnostic-state coverage. The solution and state
reports are:

- `output/clas_p7_multidataset_2018329_native_700/oracle_native_solution_diff.json`
- `output/clas_p7_multidataset_2018329_native_700/oracle_native_state_diff.json`

Safety checks pass independently: the public MADOCA materialization self-diff
is exact across 64,693 rows, the non-CLAS static PPP gate-off/gate-on outputs
are byte-identical, and the CLAS white-noise typed-profile OFF/ON outputs are
byte-identical. These checks show containment, but they cannot override the
failed second-dataset accuracy gate.

## Reconsideration gate

Default-on may be reconsidered only after the 2018 sample and at least one
additional public day/site pass the complete canonical scorecard, including
100% solution and constellation ZD coverage, DD measurement gates, filter-state
gates, FIX/FLOAT publication gates, and the 20 mm full-trajectory RMS gate.
MADOCA and non-CLAS safety outputs must remain bit-exact.

## Rollback trigger

If the profile is enabled by default later, immediately return it to opt-in
when any pinned public scorecard fails, a native-only FIX is observed, solution
or ZD coverage falls below 100%, full-trajectory RMS exceeds 20 mm, or any
MADOCA/non-CLAS safety hash changes. Keep the exact-0 profile kill switch until
two releases after the last such failure.
