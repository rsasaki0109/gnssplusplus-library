# Phase410/411 — disabled-state raw replay passed

Both one-shot raw replays completed exit 0. Verification scripts passed source
pins, completion manifest hashes, converged stages, baseline candidate byte
identity, zero frequency-state/factor counts and grouped TDCP consistency.

- H Phase410: 330.393027661019 s, candidate SHA256
  `e1e148c41fc8b87e444bd845b4a96eb8e7a56137ec91851b5c0b5d6cba64af29`,
  identical to Phase234.
- LAX-T Phase411: 258.1538502329495 s, candidate SHA256
  `d2c619121951d1a322ab39e15fcf16dbf7a6d61d2c651d03a4f68ea357c61e7d`,
  identical to Phase382.

No truth or scoring, no MAT, and no saved coordinate input to inference.
Candidate bytes were only hashed for output verification. This validates OFF
behavior for these two recipes, not all routes, PPC or a global refactor gate.
All replay sessions completed; no live jobs at this checkpoint.

While waiting, added raw timing preflight `validate_tdcp_frequency_raw_timing.py`
and four tests (passed). Inspected H 77748 and LAX-T 42370 GPS/Galileo rows
pass zero-offset and cross-signal clock consistency checks. It is a validation
tool, not an inference input. Before CLI exposure, enforce equivalent timing
requirements in native loading so arbitrary callers cannot bypass them.

Phase412 experimental prior/design remains fixed. Next implement native input
guard and narrow CLI opt-in, then freeze enabled runs and pre-truth structural
checks. No accuracy improvement is established; goal remains active.
