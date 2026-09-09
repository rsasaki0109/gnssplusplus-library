# Phase543 — LAX evaluation alignment, frozen before candidate scoring

Inspected project_phase385_lax_t_evaluation.py and its identical Phase424
successor. Existing LAX contract verifies all 1466 native output UTC keys
against sorted unique raw GNSS UTC keys, then removes exactly the first output
row from BOTH baseline and candidate for evaluation only, preserving every
retained CSV byte. It does not select rows based on residuals or truth values.
Truth metadata in Phase386 contains 1465 rows, SHA256
29e0861dd1ecb8865c10adab69396d98ed96618e8877b09d04aa8d671edf79e8.

Use this same first-epoch projection for Phase538 LAX paired evaluation after
successful numeric verification; retain full original inference outputs.
Freeze hashes of originals, raw key input, projection code and projected
outputs before scoring. Projected files are evaluation artifacts only and
must never be fed to any native positioning run. No interpolation or fill.

No truth payload read or candidate score in this inspection. LAX native PID
3941214 / session 38740 verified live at about 4m22s. Next await terminal
status, verify_phase538_joint_transfer.py lax, review magnitudes, then build
the frozen paired evaluation using the established Phase203 metric kernel.
