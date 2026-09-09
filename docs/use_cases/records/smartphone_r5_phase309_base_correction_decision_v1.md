# Phase309 source-scoped H base correction succeeds

Frozen at b982463b before raw read. Source/binary/base/nav hashes verified;
one hash scan and one native reader pass per real input, one model build.
No truth, phone, candidate, MAT, station-table, saved-state or positioning
input; correction streams remain in process and only aggregate output exits.

Build true, empty failure: 3500 epochs, 68697 satellite states, 112050
correction signal rows, 74777 excluded source-frequency rows, 38 streams.
112050 + 74777 = Phase299/307's 186827 input P rows. Epoch count matches
the prior independent marker inventory. No duplicate protection was relaxed.
Source-specified frequency admission avoids Phase307's failure; the older
failed run is preserved and not reclassified.

This proves this raw base correction configuration builds; it does not prove
rover correction support, source orbit/atmosphere identity, station survey
accuracy, or an improved trajectory. Header approximate XYZ/zero antenna
delta is still the explicit raw-only reference convention. No code/threshold
choice was based on a new positioning score.

Next qualify matching against raw smartphone code/epoch support and pair
rover satellite state/code-bias conventions before FGO integration. Existing
operational baseline stays unchanged. Diagnostic compiled successfully;
previous 33 focused tests were not rerun here. No full CTest, Kaggle/token
access, accuracy calculation or submission.
