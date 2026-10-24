# UrbanNav FIX continuity investigation

The enabled Doppler continuity gate now checks both ambiguity-resolution and
held-integer FIX candidates before acceptance. A fresh last-FIX position no
longer bypasses it. An existing Doppler track must be propagated to the current
epoch; a stale, future-dated or nonfinite reference cannot authorize a candidate.
Rejection happens before hold integers, the held baseline or trusted output
positions are updated.

`--max-fixed-doppler-consensus` still defaults to zero (disabled), and moving-base
operation bypasses it. The track is initialized from an accepted FIX and
reanchored by later accepted fixes. With no track, the first candidate bootstraps
it using the other existing validation gates. Long gaps can invalidate the track
and require another bootstrap. This is a relative-motion consistency check, not
an independent absolute-position estimate or an integer-integrity guarantee.

## Why the previous comparison was counterintuitive

In the Odaiba hero interval, the default solver repeatedly rejected integer
candidates against a FIX position about 179 seconds old. The rover had moved
about 413 m. The nonexpiring reference and 5 m absolute jump limit prevented
reacquisition even when the ratio test passed. The geometric LOS overlay does
not establish carrier observation quality or integer correctness.

Expiring that anchor alone admits incorrect fixes. The earlier experiment used
`--max-fixed-anchor-age 5 --max-fixed-doppler-consensus 3`. At TOW 273936.1,
its last-FIX anchor was only 0.2 s old, so the conditional Doppler check was
skipped; the accepted FIX had about 4.67 m 3D reference error. The hold fallback
also lacked the check entirely. Both acceptance paths now use the same gate.
The default anchor lifetime and jump limits are unchanged.

## Reproduction

Use the public UrbanNav Tokyo archive with the original Odaiba and Shinjuku
Trimble rover/base observations, broadcast navigation and reference CSV files.
Both drives use the same surveyed base ECEF coordinates
`(-3961904.9530, 3348993.7578, 3698211.7553)` m. The README hero input-fetching
instructions describe the official archive. These drives share a collection and
receiver family; they do not establish generalization to independent hardware.
Shinjuku was already used in the preceding option investigation.

Preserve the unmodified Release executable before rebuilding the patch, then run:

```sh
python scripts/analysis/evaluate_urbannav_fix_continuity.py \
  --before /path/to/baseline/gnss_solve \
  --after build/apps/gnss_solve \
  --data-root data/driving/Tokyo_Data \
  --output output/doppler-consensus-validation --jobs 2
```

The script runs both complete drives with each executable, using the default
settings and the frozen `--max-fixed-anchor-age 5
--max-fixed-doppler-consensus 3` experiment. Other solver settings match:
kinematic mode, GLONASS AR off and the surveyed base position above. Reference
coordinates are passed only to the scorer. Matching uses exact GPS week/TOW
rounded to 1 ms; missing output is retained, without interpolation. FIX errors
refer to reference position error, not proof that an integer vector is wrong.

The report records input/executable/POS hashes, native argv, return codes,
contended wall times, matched/missing epochs, horizontal P95, reference errors
above 0.5/2/3 m, baseline-correct FIX losses and recovery outages. Terminal
outages remain right-censored. Runtime measurements include concurrent build
and diagnostic work and cannot support a speed comparison.

## Decision: no default adoption

The [complete evaluation](urbannav_doppler_consensus_evaluation.json) retains all
eight full runs. Default POS files are byte-identical between binaries for both
drives. The previous opt-in results were reproduced before scoring the patch.
No thresholds were retuned after examining the results.

| Drive | Recipe | FIX epochs | FIX >0.5 m / >2 m / >3 m | Missing epochs | All-position horizontal P95 m |
| --- | --- | ---: | ---: | ---: | ---: |
| Odaiba | Default, both binaries | 1,298 | 0 / 0 / 0 | 1,426 | 4.7772 |
| Odaiba | Previous age5/doppler3 | 3,669 | 53 / 1 / 1 | 1,471 | 4.8933 |
| Odaiba | Patched age5/doppler3 | 4,256 | 157 / 3 / 0 | 1,480 | 4.8963 |
| Shinjuku | Default, both binaries | 122 | 0 / 0 / 0 | 3,234 | 8.3821 |
| Shinjuku | Previous age5/doppler3 | 1,949 | 13 / 1 / 1 | 3,332 | 8.3432 |
| Shinjuku | Patched age5/doppler3 | 1,949 | 13 / 1 / 1 | 3,332 | 8.3432 |

The Odaiba gross error disappears, but smaller wrong FIX populations and missing
outputs increase. Shinjuku's opt-in POS is byte-identical between binaries.
Baseline-correct FIX loss is zero at 0.5 m and 2 m, but that does not offset the
new errors or missing positions. Common-position-population errors and completed
recovery/censoring statistics are retained in the JSON rather than comparing
only successful reacquisitions. The patch fixes gate coverage; it does not
establish safe reacquisition or justify changing defaults.

Two diagnostic prefix replays corroborate the mechanism; every numeric POS row
in each prefix matches its corresponding full run. At Odaiba TOW 273936.1, the
patched candidate has a 0.2 s-old FIX anchor, is 4.255276 m from the propagated
Doppler track, and is rejected with `fixed_doppler_consensus` (FLOAT output).
Earlier gate decisions change filter history, so this is not the identical
candidate vector produced by the previous binary.

At Shinjuku TOW 282847.6, the accepted gross-error FIX has no Doppler consensus
distance: there is no continuity track to check. The preceding output epoch is
282827.9, about 19.7 s earlier. The FIX bootstraps a new track using the existing
other gates. A gate that compares against an existing track cannot validate that
track's bootstrap position. This needs separate current-observation and
reinitialization validation; simply expiring the old FIX reference or enabling
the continuity option does not solve it.

## Tests

Four native regression tests cover the fresh-anchor bypass, motion-consistent
reacquisition after expiry, stale/future/nonfinite tracks, disabled/bootstrap/
moving-base behavior and rejection before hold/trusted-state mutation. The old
library reproduces the bypass failures. The complete Release build succeeds.
The core CTest lane initially passes 98/100 tests; the two helper-compilation
tests with a hardcoded system Eigen location both pass when rerun with
`CPLUS_INCLUDE_PATH=/workspace/scratch/gnss-deps/include/eigen3`. No test source
was changed for that environment issue. Optional GTSAM, OpenCV, ROS2 and Python
bindings were not built.
