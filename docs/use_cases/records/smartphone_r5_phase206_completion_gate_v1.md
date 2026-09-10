# Phase206 completion and evaluation gate

The primary agent owns this run; the delegated agent is interrupted at the
user's request. This is a pending-run gate, not a completion or accuracy claim.

## Execution evidence

- Frozen manifest: `smartphone_r5_phase206_h_native_phase205_bias_density_manifest_v1.json`
  (SHA256 `fc441e5540bb682216fae81227e0e7d59a63afbe6c0530a64310764e0704b0bf`).
- Launcher: `scripts/run_phase206_bias_density.py`; native PID 2843927,
  launcher session 33814. Revalidate the process or session before waiting.
- The launcher writes its initial record to `launcher_status.json` and its
  terminal record to **`launcher_completed.json`** in the same output directory.
  The initial record deliberately remains unchanged with a null return code.
  It is not authoritative evidence of continued execution after completion.
- The frozen manifest's metadata field list is not fully implemented by the
  launcher: session and log paths are not embedded in the terminal record.
  Preserve the frozen manifest; the result must identify the terminal record,
  supply session/log references separately, and disclose this discrepancy.
- Do not retry or replace this candidate if the process fails. Preserve logs
  and record failure. No scoring before successful structural validation.

## Before scoring

1. Require terminal return code zero and matching manifest hash/native PID.
   Hash the terminal record, summary, logs, and opaque candidate. Count CSV
   records without interpreting coordinate values; expect 3139 published rows.
2. Require the summary's Phase205 density diagnostic to be requested/enabled,
   with 3139 intervals and finite positive scale bounds. Record observed
   inclusive sample count and scale range rather than assuming a constant 53.
3. Require Phase201 schedule OFF, Phase194 measurement-noise selector OFF,
   and effective Phase197 offset -20 ms. Compare the GNSS-first, epoch,
   initialization, measurement-noise, UTC-offset and output-contract summary
   objects against Phase198. Report discrepancies instead of silently relaxing
   the gate. Record convergence, termination trace, factor counts and stops.
4. Seal a structural result without truth or accuracy access. Candidate bytes
   read for hashing/counting are not coordinate interpretation; report this
   distinction rather than claiming that the candidate was never opened.
5. Freeze a separate evaluator manifest with the new candidate/structural
   result hashes before scoring. Reuse the unchanged Phase189 metric kernel
   through existing score-only entry points, not historical run-specific
   authorization or evaluation entry points. Pin every transitive dependency.
6. Verify exact-key join, no interpolation/hold/fill, no offset reapplication,
   and nonfinite rejection using synthetic tests. Authorize one evaluation
   against the already-used H development truth; do not run the solver again.

## Decision boundary

While Phase206 was running, the primary agent reran the unchanged Phase203
evaluator tests: `python3 -m pytest -q
tests/test_smartphone_phase203_phase202_h_accuracy.py` passed 10/10. These
exercise synthetic coordinates and frozen metadata, not the Phase206 candidate.
An additional read-only invocation of `verify_pre_truth()` with `Path.open`
instrumented to reject non-`.py`/`.json` access observed 12 metadata opens and
zero payload opens. This is instrumentation of that Python entry point only,
not an OS-wide I/O audit or verification of a future Phase207 wrapper.

Compare `(P50 + P95) / 2` against Phase199's 1.2751561666667786 m.
An H improvement is development evidence only, not heldout generalization,
0.782-class achievement, or leaderboard rank. Keep the option default-off
pending evidence. The full raw-only native FGO objective remains active.
