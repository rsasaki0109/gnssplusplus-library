# Phase568: broaden current raw-only baseline to MTV-A development

Phase567 closed the TGD-only diagnostic, not positioning improvement.
Now test the current baseline beyond H/U/LAX without another H-only tweak.
MTV-A (2021-03-16-18-59 / Pixel5) is already evaluated development data
(Phase146), never fresh holdout. Phase347's prior validation/holdout history
remains in force; no reserved-route truth is opened.

Raw GNSS/IMU/nav from Phase25 exist and hashes/byte lengths match Phase144
authorization metadata. No base file or previous solution enters inference.
Use the exact binary qualified by Phase563 H disabled replay, and its same
baseline argv except dataset ID, three raw input paths, and output paths.
No rotation-rate, joint-ionosphere, sparse staging, or newly added route
switch. Fix all commands/pins before launch. No coordinate/output repair.

Runner: `scripts/run_phase568_mtv_a_transfer.py`.
Manifest/logs: `output/smartphone-r5/phase568-mtv-a-transfer-v1/`.
Native job launched: session 75280, PID 3990586. Completion not yet verified;
resume same session or check actual process state, do not restart blindly.
After completion inspect graph, same-run handoff, retained raw epoch domain,
and pins before a separately frozen evaluation. If the route fails, record
that failure rather than selecting a new option from its truth error.

No accuracy read, submission, fresh split claim or goal completion this step.
