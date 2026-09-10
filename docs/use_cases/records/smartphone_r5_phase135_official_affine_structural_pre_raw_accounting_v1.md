# Phase135 official affine structural pre-raw seal

This record seals the launch-free qualification boundary for the Phase135
official affine measurement-family candidate.  It does not authorize a raw
run.  No phone GNSS/IMU, broadcast navigation, raw-base RINEX, truth, MAT,
PDC, precomputed coordinate, solver, accuracy, or Kaggle payload was read.

## Pinned source and contract

The source-parity audit is `01b7b68a37c668422c7ca396cfe56edab706f819` and its
official Doppler range-rate correction is `f9a1fc9403e707243a30d9e06aa8ea59493cdc5`.
The original Phase135 implementation is `48b13f098a75da87e52bcb15f66e3434c950baf7`;
the structural freeze is `bd1ef39ba9a7a29c210d953a7edc1e60011fb7bd`, with
freeze file SHA-256
`005c90c8be2fa9cdfabdb777589064bc8ece22c5efa6872bf732cb329db872ce`.

The launch-free validator, manifest, and synthetic tests are sealed in
`be2f83721df24912f29cb59ae1c26a342d8701b5`.  Their manifest pins the target
binary SHA-256
`bcd9a1a896c09248cd1ff3d1a4b1cd2eb9d1eebf0559073c4bad9158188c529f`.

## Fixed recipe

MTV-A then LAX-T are the only routes, one possible run each.  Phase135
official affine P/D/ordinary-TDCP families and Phase118 official Huber-k are
enabled.  Phase117/120 and the Phase126–134 compound chain are disabled.  The
Phase107 native raw-base compensation/source miss-mask path remains enabled;
additional-frequency-band preservation remains disabled.  C7/D meter-state
mapping, fixed TDCP sigma 0.03 m, Highway Huber threshold 0.8 sigma,
MULTIFRONTAL_QR/EliminateQR, IMU/LM/filter settings, and the Pixel5 final
offset are unchanged.

The corrected Doppler contract uses the official factor LOS
`-e=(receiver-satellite)/range`, receiver ECEF velocity, satellite clock drift
in metres/second, and one explicit first-order Sagnac term.  Phase135 requires
all admitted P/D/TDCP rows to use the affine branch transactionally, with no
legacy-family mixing and an exact Pose3–X bridge.  A later authorized result
must expose only opaque solution hash/row metadata, never coordinate rows.

## Qualification and accounting

The validator's placeholder command snapshots and freeze/manifest pins pass;
the structural synthetic suite is 5/5, the existing Phase135 source suite is
4/4, Python compilation passes, and the target build passes.  The C++ suite
ran 1183 tests: 1125 passed, 58 pre-existing fixture/environment skips, and 0
failed.  These are source/build tests only; no native data process was
launched.

All pre-raw counters are zero: raw payload reads, native command/solver
invocations, solution-row/coordinate reads, truth/accuracy, MAT/PDC/
precomputed-coordinate, Kaggle/token access, and reruns/fallbacks/repairs/
sweeps.  Only tracked source, sealed metadata, and the compiled target hash
were read.

## Authorization boundary

An independent authorization commit must pin the complete manifest, runner,
freeze/correction commits, test qualification, and target binary hash before
any raw materialization.  After authorization, the runner may inspect only the
two allowed raw phone streams, broadcast navigation, and sealed raw-base
RINEX, in the fixed route order.  Inventory or structural failure is
fail-closed and forbids solver launch; no fallback, repair, retry, rerun, or
sweep is permitted.  Structural success still does not authorize truth,
accuracy, publication, or Kaggle evaluation.
