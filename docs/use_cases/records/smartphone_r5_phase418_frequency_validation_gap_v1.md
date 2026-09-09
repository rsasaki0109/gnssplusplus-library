# Phase418 — enabled-run validation scope

Phase416 H and Phase417 LAX-T launched once from frozen raw-only manifests.
No truth evaluation has occurred. Do not restart either run merely on a poll
timeout; use launcher completion metadata and live process/session authority.

Added `scripts/verify_native_frequency_experiment.py` for post-run source,
binary and raw pin checks, convergence, positive frequency-state count,
2:1 wrapped-factor/state accounting, unchanged admitted TDCP counts,
finite residual/Huber aggregates, and exact ordered output identifiers against
raw UTC keys. The raw-key comparison was checked on completed OFF baselines:
H 3139 and LAX-T 1466 keys match. Coordinates are not interpreted by this check.

Outstanding Phase412 prerequisite: the app summary serializes its corrected
TDCP RMS, but not `result.diagnostics.tdcp_residual_rms_m` from the backend.
Synthetic tests previously compared these; this is not direct evidence of
agreement for the enabled raw runs. The aggregate verifier explicitly leaves
that check pending. Prior-count accounting also currently relies on backend
construction and tests rather than independently serialized raw-run counts.
Do not present successful aggregate checks as satisfying these missing gates,
and do not open truth before resolving the prospective verification contract.
Preserve frozen run source pins while both current runs are being verified.
