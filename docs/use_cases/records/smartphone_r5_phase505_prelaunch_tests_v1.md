# Phase505 prelaunch — candidate boundary tests, rebuild still active

Current code-edge gtests compiled directly and all five passed. Added explicit
outside-edge clock-break and >1.5 s gap cases: only the unsupported side is
removed. Copy/move/reordered-subset test preserves original raw epoch identity
and diagnostic label, while applyAdjacentMasks returns the identical P mask.

Negative control adds a constant 75 m code offset to every sample of the
five-epoch impulse fixture: codeEdgeCandidates returns exactly the same set.
Temporal support cannot distinguish common persistent code error from clean
neighbors. This is a deliberate limitation test, not a mask correction or a
claim that real candidate rows have this offset. No automatic readmission.

run_phase505_code_monitor.py is prepared but has NOT launched. It derives
Phase503 baseline argv, pins the new observation header, candidate helper and
tests plus current sources/binary, and asserts output identity on completion.
Build session 5222 is still producing dependent RTK/PPP objects, confirmed
live by fresh output. Do not run an old binary or start a duplicate build.
No real-data intersection counts or new truth evaluation yet.
