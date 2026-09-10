# Phase75 accuracy recovery integrity failure

Phase75 was a scorer-only recovery of the Phase74 truth-header defect. It
correctly reached the first declared truth read, but then failed closed on an
evaluator reference to a nonexistent `PHASE43_CONTROL` export from the sealed
Phase74 helper. No accuracy score was observed and no native or solver process
ran. The initial attempt read two candidate and two control artifacts plus the
MTV-a truth once; it must not reread that truth in Phase75.

The initial evaluator and manifest hashes are preserved in the failure record.
Phase76 uses a new freeze, evaluator, manifest, output root, and one truth read
per route. Its control identity check is sourced directly from the inherited
Phase74 freeze route pins, with no helper constant dependency.

See the [failure record](records/smartphone_r5_phase75_phase74_accuracy_integrity_recovery_failure_v1.json),
[Phase75 freeze](records/smartphone_r5_phase75_phase74_accuracy_integrity_recovery_freeze_v1.json),
and [Phase75 manifest](records/smartphone_r5_phase75_phase74_accuracy_integrity_recovery_manifest_v1.json).
