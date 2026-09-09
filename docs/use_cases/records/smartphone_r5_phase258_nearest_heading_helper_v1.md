# Phase258 nearest heading fill helper

Added default-off VelocityHeadingConfig.nearest_fill_interior. Historical
linear interior filling remains the default and its existing tests remain
unchanged. The option uses nearest valid sample index, with the later
sample selected on equal distance; that tie convention is explicit but
has not been verified against MATLAB execution. Do not claim full parity.

Build session 49908 completed; all 12 FusionInitializationTest tests passed,
including the new non-tied interior-neighbor test and legacy control.
Only comments/indentation were edited in the helper after its compilation;
the tested executable contains the same executable statements. No full
CTest claim and no raw-data accuracy evaluation.

The helper is not yet selected by the native smartphone CLI. Next connect
an exact same-run, per-epoch attitude seed sequence to the Phase171 graph,
with validation and diagnostics. This helper alone cannot improve the
current front-only heading initialization identified in Phase257.
