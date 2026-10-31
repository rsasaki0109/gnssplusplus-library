# Holdout results: the validated-only KF hold in `--navi776-tc`

Contract: [navi776_kf_hold_holdout.md](navi776_kf_hold_holdout.md), frozen at
commit `cc1fe32` before any estimator ran on UrbanNav Hong Kong
Medium-Urban-1.

## Decision: inconclusive

The control fixes 1.49% of the epochs, below the contract's 5% floor, so the
run cannot separate the arms. Per the contract:

- the `--navi776-tc` preset is unchanged;
- `--hold-kalman-update --hold-kalman-update-validated-only` stays an opt-in;
- the run is not repeated, and no other data is added to compensate;
- Medium-Urban-1 is now development data.

Nothing was changed after seeing the result.

## Numbers

Nine runs per arm, means (the nine runs agree exactly). After the frozen
antenna offset:

| Arm | FIX (% of epochs) | Correct FIX at 0.5 m | Wrong FIX (% of FIX) | Correct FIX at 0.25 m |
| --- | ---: | ---: | ---: | ---: |
| Control (`--navi776-tc`) | 1.49 | 0.50 | 66.67 | 0.00 |
| Candidate (+ validated-only KF hold) | 1.49 | 0.50 | 66.67 | 0.00 |

About ten epochs are fixed in each arm, and the two arms give the same result.

## Executed procedure

1. The contract, the converter's `HKMediumUrban1` run and the spread script's
   `--truth-offset-body` were committed as `cc1fe32`.
2. `gnss_fuse` was built from a clean worktree of `cc1fe32` (Release, Ninja).
3. On development data (HK Deep-Urban-1, unperturbed lever arm), that binary
   reproduces the comparison runs byte for byte, for both the control and the
   candidate.
4. The converted Medium-Urban-1 files matched the SHA256 in the contract.
5. The two commands of the contract were run once, with the frozen worktree's
   spread script and that binary, at 2026-10-31 06:33 UTC.
