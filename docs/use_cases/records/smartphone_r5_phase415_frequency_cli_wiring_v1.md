# Phase415 — experimental native frequency-state CLI

Added default-OFF `--native-tdcp-frequency-residual-states` to the native app.
It enables the native raw timing guard and main FGO residual-state option with
the Phase412 prospective fixed 0.01 m prior. GNSS-first explicitly clears the
option and prior. Existing diagnostic state/factor counts and identity-checked
in-memory correction handoff remain in use; no saved trajectory input is added.

CLI admission requires the raw clock-only, UTC-key, all-epoch Phase171 H/LAX-T
development recipe with source TDCP metre sigma, Phase197 timing, Phase213
Doppler and Phase217 motion. It rejects base compensation, code-gate removal,
Cauchy P, gyro initialization, heading seeds and Phase120 atmosphere bypass.
Backend validity checks still apply. This admission check is not a substitute
for pinning the complete executable argv and source in an execution manifest.

Validation: native app build passed; fresh executable CLI suite 30/30 passed,
including H/LAX-T admission to deliberately missing raw input, incompatible
ablation rejection, absent source sigma and unlisted route rejection.
`git diff --check` passed. No enabled raw run or new truth evaluation yet;
accuracy and leaderboard goals remain unproven. Full CTest was not run.

Next: freeze enabled execution manifests and exclusive output paths for H and
LAX-T, run raw-only inference, and check convergence, output domain and factor
accounting before a separately frozen evaluation.
