# CD wrong fixes: one satellite's carrier ramps inside a FIX streak

This note follows the [max position jump note](rtk_max_position_jump_motion.md).
The opt-in CD recipe keeps 10 wrong FIX epochs (0.5–0.9 m) on the Trimble
drives: 6 on Odaiba and 4 on Shinjuku. This round finds their mechanism and
adds two debug columns. **Solver behaviour is unchanged.** With and without
the change, the default and CD POS are byte-identical on both drives. Record:
[JSON](urbannav_fix_streak_drift.json).

The reference trajectory is used only for labels and for the analysis-only
truth DD of [scripts/analysis/truth_dd](rtk_long_lock_overconfidence.md).

## Mechanism

Each wrong epoch ends a FIX streak whose error grows smoothly. On Shinjuku
at TOW 281297:

- the error grows 0.02 → 0.07 → 0.10 → 0.17 → 0.24 → 0.32 m over 13 fixed
  pairs, with the same reference satellites throughout;
- the AR ratio falls 287 → 98 → 28 → 19 → 6.8 → 4.0;
- hold fixes take over at 0.41–0.62 m;
- the next FLOAT is back at 3 cm, so the drift is in the FIX, not in the
  reference.

In every episode, the truth DD shows **one satellite's carrier ramping or
stepping by a non-integer 0.4–1.5 cycle over 1–2 s**:

| Episode | Satellite | Ramp | LLI |
| --- | --- | --- | --- |
| Shinjuku 281297 | G25 L1C | −1.45 | one flag |
| Shinjuku 282324 | G24 L1C / L2W | −0.57 / −0.45 | flagged |
| Shinjuku 283170 | E05 L1X / L5X | −0.85 / −0.63 | one flag |
| Odaiba 273748 | E09 L5X / L1X (C24 −0.28) | +0.53 / +0.43 | none |
| Odaiba 273918 | J02 L2X | −0.60 | none |

A non-integer ramp is not a cycle slip that the detectors can flag. The
float follows it, LAMBDA keeps returning the nearby integers, and every
epoch-to-epoch step (0.07–0.15 m) stays below the motion-compensated 0.2 m
history check. The Doppler consensus re-anchors at every accepted FIX.

## New telemetry

`--debug-epoch-log` gains two columns from the same pass as
`postfix_residual_rms`:

- `postfix_residual_max_abs_m`: the largest |post-fix DD phase residual| over
  the fixed pairs;
- `postfix_residual_max_pair`: that pair, as `REF-SAT/Lf`.

`RTKPostFixResidualTest.ReportsLargestPairResidualAlongsideRms` checks them on
the noise-free scene: the residual is below 1 cm, named after a GPS pair, and
at least the RMS.

On the episodes:

- **The culprit is often visible early.** The largest pair names the culprit
  in 3 of 5 episodes before the error reaches 0.5 m. On Shinjuku 281297 it is
  G12-G25 from 0.10 m error on.
- **Its size is ordinary.** The culprit's residual is only 2–5 cm because the
  fixed solution follows it. That is inside the correct AR fixes' p95 of
  0.050 m (Odaiba) and 0.091 m (Shinjuku). A magnitude threshold cannot
  separate them; a persistence test (same pair, growing residual) might, but
  it needs its own frozen evaluation.
- **Hold fixes compute no post-fix residual**, so the hold epochs at the end
  of an episode are not covered.

## Side finding: grossly wrong integers in correct fixes

Some correct AR fixes (cm error) carry a pair whose post-fix residual is
metres:

- Shinjuku: 117 epochs, for example E09-E05/L2 at 40.68 m (about 160 E5a
  cycles) for 6 s from TOW 282806. The E05 E5a truth DD drifts 3 cycles in
  22 s after an LLI at 282788.
- Odaiba: 4 epochs, G20-G13/L1 at 7.8 m (41 cycles).

The position is unaffected, so that ambiguity is effectively decoupled.
However, the opt-in `max_postfix_residual_rms` check would reject these
correct fixes, because their RMS is metres. Whether such integers are later
held has not been checked; that is the next question.

No rule is proposed in this round.
