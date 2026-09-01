# Phase 63: settings-integrity recovery result

The Phase63 recovery fixed the observed `settings_train.csv` SHA from the
Phase62 fail-closed record and opened the archive once. It read the settings
member once and streamed each of the four declared base RINEX members exactly
once. All central sizes/CRCs, materialized sizes/SHA-256 values, and the
deterministic course-window overlaps were recorded.

The preflight is `no-go-base-header-contract`: every member has a finite
`APPROX POSITION XYZ` and a finite observed epoch interval, but all four have a
blank `MARKER NAME`, no `INTERVAL` header, and an actual RINEX version `3.03`
while `settings_train.csv` declares `V2`. No station identity or header
interval is inferred from `Base1`, filenames, or external files. Therefore
the raw base source is not authorized for native correction, and no accuracy
or 0.782 evaluation was run.

Forbidden inputs remained unopened: handset GNSS/IMU, truth, MAT, navigation,
WLS/Sv coordinates, solver, Kaggle/token data, `base_position.csv`, and
`base_offset.csv`. The Phase62 integrity-failure record remains immutable.

See the [freeze](records/smartphone_r5_phase63_settings_integrity_recovery_freeze_v1.json), [manifest](records/smartphone_r5_phase63_settings_integrity_recovery_manifest_v1.json), and [result](records/smartphone_r5_phase63_settings_integrity_recovery_result_v1.json).
