# Phase299 H native base tracking inventory

Frozen before raw execution at c87594b. Source/binary/base hashes verified.
One whole-file hash/epoch-marker scan and two native reader passes; no
navigation, phone, truth, candidate, MAT or saved positioning input.

Both modes consumed 3500 epochs, matching the independent marker count.
Default reader emitted 193827 P rows; source header filter emitted 186827.
The entire 7000-row difference is GPS C2X. Every other emitted code count,
including 22478 C2W rows, is unchanged. No coordinate values were printed.

This establishes a real admission difference, not an accuracy gain. It does
not prove all dropped rows were eligible rover correction matches. Both
passes preserved additional native bands, so these counts are not the
operational no-base FGO graph or a reproduction of historical base recipes.

Next: integrate paired satellite-state preparation with this explicit input
contract; retain code-miss accounting and do not restore C2X solely to raise
coverage. Qualify actual base/rover correction matching before a solver run.
The source filter remains default off and no FGO CLI has been changed.

Diagnostic compiled successfully against the current native libraries. The
preceding 60 focused reader/base tests passed; this turn did not rerun them.
Native diagnostic exit codes were both zero. No accuracy calculation or
Kaggle/token access occurred. Goal remains incomplete.
