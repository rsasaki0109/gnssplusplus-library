# Phase304 in-memory epoch state composition

Starting HEAD d885254b; root-only. Added source_epoch_states.hpp connecting
native code-provenance selection, explicit receive-time broadcast selection,
and selected-message transmission/propagation. Uses existing nav vectors
without invoking the environment-dependent shared selector. Results own state
values and expose selected slot/range, toe and health; no borrowed pointers
escape. Input objects are const and no files are accessed.

Strict contract: missing/admission-failed navigation, duplicate slots or
propagation error throws before a result is returned. This is not source
per-satellite local-miss parity. Callers must explicitly decide local-miss
policy before FGO integration; do not silently catch and substitute states.
Health is exposed for caller admission, not a healthy-state guarantee.
Native orbit equations, nav record ordering and upstream quality/tracking
admission still require qualification. No FGO CLI enables this path yet.

Synthetic composition test covers two bands yielding one SBAS state,
selected-range provenance, missing second-satellite navigation and duplicate
rejection. SBAS uses a synthetic native signal enum placeholder; code/system
provenance governs selection. This does not prove Android parser behavior.

Next: truth-free raw base/nav diagnostic of this composed path with explicit
failure taxonomy, then paired rover preparation. No raw/truth/candidate/MAT,
station-table, network or Kaggle/token input here. No score or trajectory run.

Validation: gnss_run_tests target built; all 31 base-compensation tests passed.
Existing synthetic fixture I/O only, not full CTest or raw-state parity.
