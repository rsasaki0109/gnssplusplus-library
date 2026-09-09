# Phase248 first native epoch output

Default output behavior remains warmup-excluded. New optional
`--android-include-first-native-epoch` requires Phase171 ECEF-D, raw UTC
keys, all epochs and no skips. It passes an explicit boolean to the raw
alignment helper, starting output at raw index zero instead of one.
The first epoch must have an actual same-run native solution within the
existing time tolerance; no interpolation or edge hold can replace it.
Summary warmup_epoch_excluded now reflects the selected policy.

No solver factors or estimator parameters changed. Later-epoch historical
alignment behavior is unchanged; actual-run interpolation/hold counts must
still be checked before accuracy evaluation. Full raw-domain output is
not automatically the same as a competition's requested output domain.

Build 19041 completed tests; AndroidRawGnssTest suite 17/17 passed.
Build 66538 completed the native executable; new CLI suite 5/5 passed.
Not full CTest. No new raw solve or truth read during this implementation.
Next freeze a new U run with this output flag, retaining the estimator.
Phase246/247 artifacts and failed attempt remain immutable.
