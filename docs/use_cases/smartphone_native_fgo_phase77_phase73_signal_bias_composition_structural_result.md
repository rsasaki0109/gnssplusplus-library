# Phase77 structural result: signal-bias composition no-go

Phase77 evaluated a fresh Phase73 finite-base-pc miss-mask control and the
same recipe with the existing `--native-signal-bias-states` option on all four
Pixel5 routes. The candidate was repeated twice per route. All 12 native
invocations converged, produced exact prediction-domain keys, finite
coordinates, finite signal-bias states, finite TDCP residuals, deterministic
repeat artifacts, and zero over-70-m/s rows.

The frozen composition gate failed on all four routes. Enabling the existing
signal-bias option also enables the native multi-frequency signal-eligibility
path. Consequently, the base-pc miss-mask was applied to a different adopted
pseudorange population rather than preserving the sealed Phase73 population:

| route | Phase73 control original→retained | signal-bias candidate original→retained | states / factors |
| --- | ---: | ---: | ---: |
| MTV-a | 61,754 → 46,556 | 83,612 → 59,915 | 2 / 13,359 |
| MTV-h | 73,286 → 51,890 | 108,722 → 66,769 | 1 / 14,879 |
| LAX-t | 34,319 → 33,552 | 50,706 → 45,095 | 2 / 11,543 |
| MTV-u | 24,395 → 22,369 | 35,391 → 29,881 | 1 / 7,512 |

Candidate repeats were byte-identical. The control matched the Phase43
prediction-key reference, while candidate prediction-domain coverage remained
exact. The decisive failure is the predeclared Phase73 miss-mask telemetry
identity: candidate original/retained/drop counts and correction statistics
changed on every route. This is a structural incompatibility of the holistic
composition, not a relaxed coverage threshold or a presentation-only issue.

No C++ source was changed and no development truth was read. Phase43 remains
the champion; Phase73, Phase51, Phase58, and the Phase77 signal-bias option
remain experimental. Accuracy, validation, Kaggle, MAT, WLS, and solver-after-
truth actions are not authorized. The next separately frozen source factor is
the official raw-IMU stop velocity/pose factor family, whose native telemetry
is currently zero.

Machine-readable details, exact artifact hashes, and read accounting are in
the [sealed result record](records/smartphone_r5_phase77_phase73_signal_bias_composition_structural_result_v1.json),
[freeze](records/smartphone_r5_phase77_phase73_signal_bias_composition_structural_freeze_v1.json),
[manifest](records/smartphone_r5_phase77_phase73_signal_bias_composition_structural_manifest_v1.json),
and [structural contract](smartphone_native_fgo_phase77_phase73_signal_bias_composition.md).
