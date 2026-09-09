# Phase380 — native sparse P synthetic recovery

Fresh standalone test object linked with rebuilt native libraries. Five
FGOGtsamPhase171EcefDopplerGraphTest tests passed, including the strengthened
sparse test: remove every P row at the second of two epochs, perturb that
epoch's position seed by (3,-2,1) m, retain synthetic Doppler/TDCP/motion/
clock data, enable allow_native_raw_p_sparse_epochs. Recovered position
is within 1 mm of its synthetic expected value, with complete exports.
Clock discontinuity still rejects the graph. Default sparse rejection,
ordinary dense graph and invalid-Doppler controls passed as well.

This is a two-epoch synthetic control, not proof of arbitrary sparse-graph
observability or real-data performance. No raw/truth/MAT/saved-position
input or scoring occurred. The opt-in remains unconnected to the CLI.

After the passing run, the sparse option's scope guard was moved outside
the raw-staging branch so unsupported backend graph modes cannot silently
ignore it. That final guard edit is not yet rebuilt/tested. Before CLI
integration, rebuild and add unsupported-mode and all-sparse/no-anchor
controls. Preserve default behavior and all seed/clock/finite-data guards.
