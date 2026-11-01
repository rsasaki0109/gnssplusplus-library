# Frozen holdout results: the fused `--out` of `gnss fuse` on SmartPNT-POS

The [contract](smartpnt_fuse_output_holdout.md) was committed before any
estimator ran on its data: locally as `9fdf587` at 23:48:54 UTC, on `7a94af1`
(the head of #694), 25 s before the first run. #694 was then squash-merged as
`fcbdc6f`, whose tree is identical to `7a94af1`, and the commit was replayed
onto `develop` unchanged as `0694856` (`git range-diff` reports no change).
These are the results of its one run.

- **(a) `--independent-output-filter` under `--navi776-tc`: pass.**
- **(b) `--consider-before-heading-latch` without tight coupling: fail.**

## Run

- **When:** 2026-10-31, 23:49–23:54 UTC, the eight commands of the
  contract, once each.
- **Binary:** SHA256 `4d30dbd7…2ffc`, the one the contract names.
- **Data:** the converted files of the contract, unchanged.
- **Every arm** exited 0 and wrote 8,332 fused epochs of 8,334 rover epochs
  on Data11, and 20,320 of 20,320 on Data12. The heading aligned and
  converged in every arm, and in the independent output filter of the (a)
  candidates.
- **RTK streams:** `cmp` finds them identical between control and candidate
  for both candidates on both runs.

## Results

Fused `--out` 3D error p50 / p90 (m), control → candidate, scored by
`scripts/analysis/score_fused_output.py`. Both arms match the same number of
epochs on each run: 8,272 on Data11 and 20,260 on Data12.

| Candidate | Run | Control | Candidate | p50 | p90 | Run outcome |
| --- | --- | ---: | ---: | --- | --- | --- |
| (a) | Data11 | 181 / 790 | 0.790 / 4.42 | lower | lower | better |
| (a) | Data12 | 93.7 / 1,542 | 0.145 / 10.2 | lower | lower | better |
| (b) | Data11 | 1.064 / 4.80 | 1.402 / 33.6 | higher | higher | worse |
| (b) | Data12 | 0.1425 / 6.218 | 0.1416 / 6.184 | equal | equal | equal |

- **(a) passes.** It is better on both runs, and its RTK streams are
  identical.
- **(b) fails.** It is worse on Data11 (p50 +32%, p90 seven times higher),
  and equal on Data12.

## Reported, not in the rule

The RTK stream of each mode is the same in both of its arms. 3D p50 / p90 (m),
then FIX rate, correct FIX (3D below 0.5 m, % of epochs) and wrong FIX (% of
FIX):

| Mode | Run | RTK p50 / p90 | FIX | Correct | Wrong |
| --- | --- | ---: | ---: | ---: | ---: |
| `--navi776-tc` (a) | Data11 | 0.655 / 4.05 | 46.50% | 42.03% | 9.62% |
| `--navi776-tc` (a) | Data12 | 0.069 / 9.25 | 61.67% | 59.12% | 4.13% |
| no tight coupling (b) | Data11 | 0.985 / 4.15 | 42.54% | 38.15% | 10.32% |
| no tight coupling (b) | Data12 | 0.078 / 6.65 | 59.66% | 56.32% | 5.59% |

- **The (a) pass was expected.** The control is the default `--out` under
  `--navi776-tc`, whose drift is the
  [known issue](imu_fusion.md#independent-output-filter) behind the flag.
- **What the holdout adds** is that the candidate's output is usable on new
  data. Against the plain `gnss fuse` output (the (b) control) it is:
  - better on Data11: 0.790 / 4.42 m against 1.064 / 4.80 m;
  - level at p50 and worse at p90 on Data12: 0.145 / 10.2 m against
    0.143 / 6.22 m. Its RTK stream has the same longer tail (p90 9.25 m
    against 6.65 m).

## After scoring: why (b) is worse on Data11

Both runs are development data from here on. This look came after the
decision and does not change it.

- **Same inputs, different heading.** On Data11 the car stands still for
  630 s (TOW 459712–460342). Both arms latch the heading at TOW 460377, from
  the same course (98.8°). Two diagnostic reruns of both arms reproduce the
  scored outputs byte for byte: one with `LIBGNSS_DEBUG_HEADING=1`, one with
  `--attitude-csv`.
- **Error against the truth heading, median per 300 s window:**
  - **Control:** under 2° in every window from TOW 460600 to 463200.
  - **Candidate:** 82–144° until TOW 462400. Its roll and pitch error is
    1.1–2.0°, against 0.14–0.21° for the control in the same windows.
  - **Convergence:** the candidate converges by TOW 462700. The two outputs
    are identical after TOW 464113.
- **Not established.** Data12 also starts with a long stand-still (658 s),
  and there the arms are equal. So the stand-still alone does not explain
  Data11. The cause is not investigated further here.

## What follows

- **(a):** a separate PR may propose that `--navi776-tc` turns on
  `--independent-output-filter` when `--base` is given. It will carry this
  record, the development evidence and the usual tests. Merging it remains
  the maintainer's decision.
- **(b):** nothing changes. `--consider-before-heading-latch` stays an
  opt-in.
- The runs are not repeated, and no other data is added to compensate.
  SmartPNT-POS Data11 and Data12 are development data from now on.
