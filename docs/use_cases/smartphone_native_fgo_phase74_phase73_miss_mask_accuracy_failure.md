# Phase74 accuracy scorer integrity failure

Phase74 did not produce an accuracy score. Its scorer was sealed after the
Phase74 freeze and manifest, but stopped fail-closed on the first route while
reading the existing Phase44 development truth once. The scorer required an
exact three-column truth header. The sealed Phase44/59 source contract instead
uses CSV `DictReader`: `UnixTimeMillis`, `LatitudeDegrees`, and
`LongitudeDegrees` are required, while `phone` and additional columns may be
present. This is an evaluator schema bug, not a physical or accuracy result.

The failed attempt read two candidate/control artifacts and one truth for
MTV-a, launched no native or solver process, and retained no truth payload. It
must not reread that truth under the Phase74 contract. The immutable Phase74
freeze, evaluator, and manifest remain unchanged; a future recovery needs a
new freeze, manifest, output root, and one truth read per route using the
sealed DictReader field semantics. No validation, holdout, MATLAB, WLS,
precomputed-coordinate, archive, or Kaggle access occurred.

See the [failure record](records/smartphone_r5_phase74_phase73_miss_mask_accuracy_failure_v1.json),
[accuracy freeze](records/smartphone_r5_phase74_phase73_miss_mask_accuracy_freeze_v1.json),
and [evaluator manifest](records/smartphone_r5_phase74_phase73_miss_mask_accuracy_manifest_v1.json).
