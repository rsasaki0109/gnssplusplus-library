# Phase235 metre-domain dynamic sigma development result

Primary agent. Evaluation frozen at 4facbf7 before truth access; 13 tests
passed. One candidate read, one truth read, one score; no native rerun.

H scalar improved from 1.2680574850266653 to 1.0769392017393964 m,
delta -0.19111828328726888 m (about 15.1%). P50 worsened from
0.7706427231288419 to 0.8369083196573837 m; P95 improved from
1.7654722469244886 to 1.3169700838214091 m. All 3139 rows match exactly,
finite/Earth-valid, zero over-70-m/s transitions. No interpolation, hold,
offset reapplication or dropped truth rows.

Phase234 is the new H development best by the frozen scalar, not proof of
generalization or leaderboard rank. The sigma change affects both stage and
main; attribution cannot isolate initialization from the final likelihood.
Historical Phase117 wavelength-scaled sigma was not used. Native solver used
raw inputs only, no MAT or saved positioning input. Production defaults remain
unchanged. The 0.782 target is still unmet.

Next test the source-derived joint sigma + Huber k=0.2 configuration as a
separately frozen ablation: the previous k-only experiment used a different
sigma and cannot answer this interaction. Do not perform a truth-guided sweep.
Broader-route validation and SNR population/resL correction audits remain
required before claiming source parity or leaderboard performance.
