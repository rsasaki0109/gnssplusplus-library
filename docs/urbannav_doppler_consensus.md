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

## Initial gate-coverage evaluation: no default adoption

The [complete evaluation](urbannav_doppler_consensus_evaluation.json) retains all
eight full runs for the initial gate-coverage patch at `8fcdf6f9`. Default POS
files are byte-identical between binaries for both
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

## Discarded experiment: Doppler during missing position output

RTK/SPP output loss does not necessarily mean that rover Doppler observations
are missing. A separate experiment continued solving Doppler velocity from the
propagated track even when the position solution was invalid. A synthetic
Doppler-only scene reproduced the old dependency; both new mechanism tests and
all 100 core CTest entries passed for that experiment.

However, the [eight full replays](urbannav_doppler_output_gap_evaluation.json)
failed adoption. With the same age5/doppler3 settings, Odaiba FIX errors above
0.5/2 m changed from 157/3 to 173/13, and missing output rose from 1,480 to
2,132. Shinjuku changed from 13/1 to 23/2, with missing output rising from 3,332
to 3,400; the gross-error FIX at TOW 282847.6 remained. Default outputs were
byte-identical. These outcomes do not establish that maintaining an unbounded
relative track is safe.

The runtime/test changes were **discarded**. The exact
[experimental patch](urbannav_doppler_output_gap_experiment.patch) is retained
for reproduction on `8fcdf6f9`; its SHA-256 matches the source diff captured by
the replay manifest. It is not part of the production solver. No threshold was
retuned to rescue this experiment.

## Discarded experiment: confirm bootstrap over five epochs

A second independent experiment added
`--fixed-doppler-bootstrap-epochs 5` to the original age5/doppler3 recipe.
The count was fixed before replay and was not retuned. This experimental
option is **not present in the production CLI**. Apply its archived
[source patch](urbannav_doppler_bootstrap_experiment.patch) independently on
a clean source checkout of `8fcdf6f9` to reproduce it. The patch also extends
the evaluator, which runs it with `--after-bootstrap-epochs 5`.

With no trusted track, five consecutive otherwise valid candidate epochs had
to agree with the first tentative position propagated by fresh measured Doppler
velocity. Pending candidates kept the ordinary FLOAT output. Their fixed
coordinates and integers were not promoted to trusted state. The tentative seed
was not moved toward later candidates. A missing candidate, rejected final
candidate, position discontinuity, stale/missing velocity or time discontinuity
broke the evidence. AR and hold attempts at the same timestamp could not count
twice. Reset/config changes cleared it. Gap/velocity-age bounds remained the
existing 2 s limits. At 10 Hz, five candidates span at least 0.4 s. Initial
acquisition also required confirmation.

The [eight full replays](urbannav_doppler_bootstrap_evaluation.json) reject this
experiment as well:

| Drive | Recipe | FIX epochs | FIX >0.5 m / >2 m / >3 m | Missing epochs | Horizontal P95 m |
| --- | --- | ---: | ---: | ---: | ---: |
| Odaiba | Gate coverage only | 4,256 | 157 / 3 / 0 | 1,480 | 4.8963 |
| Odaiba | Five-epoch bootstrap | 4,250 | 157 / 3 / 0 | 1,482 | 4.8964 |
| Shinjuku | Gate coverage only | 1,949 | 13 / 1 / 1 | 3,332 | 8.3432 |
| Shinjuku | Five-epoch bootstrap | 4,074 | 473 / 22 / 14 | 3,366 | 8.3075 |

At Shinjuku TOW 282847.6 the candidate is now pending with streak 1; status is
FLOAT, `final_fixed_applied=0`, and the reason is
`fixed_doppler_bootstrap_pending`. Every numeric POS row in a 15,740-epoch
diagnostic prefix matches the full replay. However, new gross-error FIX epochs
occur at 282745.7–282746.9 and 283306.7. Confirming repeated relative positions
does not establish that their absolute ambiguity basin is correct. Changed
acceptance history affects later estimation; rejecting one known bad epoch is
not a global improvement.

Four previously correct default FIX epochs are lost on each drive, including
4/122 (3.28%) on Shinjuku, beyond the 1% budget. Missing positions also increase.
Default POS files remain byte-identical before/after on both drives. Recovery,
right-censored outages, exact source/binary/input/output hashes and argv are
retained in the report. The archived patch's hash equals the source diff in its
replay manifest.

The mechanism tests and all 100 core CTest entries passed, but the full-data
accuracy gate failed. The runtime/CLI/helper/test changes were **discarded**.
The retained gate-coverage fix and reproducible rejected experiments are
accompanied by a separate [trusted-history repair](rtk_trusted_history_rollback.md).
Neither experiment justifies changing defaults or claiming safe
reacquisition. Further work needs to explain incorrect ambiguity selection and
state evolution across candidate rejection; another continuity threshold or
confirmation count cannot be justified by these results.

## Tests for retained gate coverage

Four native regression tests cover the fresh-anchor bypass, motion-consistent
reacquisition after expiry, stale/future/nonfinite tracks, disabled/bootstrap/
moving-base behavior and rejection before hold/trusted-state mutation. The old
library reproduces the bypass failures. The complete Release build succeeds.
At restoration checkpoint `6473615a`, after discarding both experiments,
all 100 core CTest entries pass with
`CPLUS_INCLUDE_PATH=/workspace/scratch/gnss-deps/include/eigen3` for the two
helper-compilation tests with a hardcoded system Eigen location. No test source
was changed for that environment issue. That checkpoint's executable is byte-identical
to the gate-coverage binary used before these experiments. Optional GTSAM,
OpenCV, ROS2 and Python bindings were not built.
