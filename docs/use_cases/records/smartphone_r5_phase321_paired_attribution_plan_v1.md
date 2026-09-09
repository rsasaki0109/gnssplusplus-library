# Phase321: truth-free attrition audit and preregistered state-only ablation

Read only the existing Phase319 aggregate summary and algorithm source, not
the candidate, truth, or raw payloads. The post-admission miss-mask taxonomy
shows 18929 of 20453 excluded P factors were BeiDou B1I: all its P factors
were removed because no base stream exists. GLONASS missing-stream losses
were 234. Callback-unavailable losses were GPS L1 186, Galileo E1 547 and
Galileo E5a 557. All six per-signal count-conservation checks passed.
This identifies a major geometry/information change but does not prove its
causal contribution to the 2.2515 m score. Doppler/TDCP survival and clock
constraints mean this is not equivalent to removing a constellation entirely.

Before any further evaluation, define the next comparison: Phase234's exact
operational flags plus `--native-rover-epoch-states`, with base correction,
base miss mask, paired flag, and Phase126 zero-group-delay policy all OFF.
No parameter optimization or correction scaling. This isolates the combined
receive-time selection/shared-frequency transmission/forward-difference state
implementation from the base correction, code-bias and missing-P changes.
It does not separately identify the three internal state changes.

The CLI switch is default off, restricted initially to the raw H Phase171
all-epoch UTC recipe, rejects base/paired mixes, and is serialized in both
provenance locations. Existing baseline and paired modes are retained.
`gnss_fgo_imu_no_base` built successfully; all seven executable CLI tests
passed, including state-only admission to absent raw ingress and rejection
of a paired/base mix. No native raw route run or evaluation occurred here.

Interpretation fixed before running: a regression here would establish that
the state-only change can degrade this development route; it would not prove
base correction is harmless. A comparable or improved result would focus
the next investigation on the remaining coupled base/bias/mask changes; it
would not prove which of them is responsible. No score sweep, repeat scoring,
or truth-derived parameter selection is planned. Freeze run and evaluator
artifacts separately before their respective payload access.

The 0.782-class and leaderboard objective remains open. H is development data,
so neither this comparison nor the earlier result establishes generalization.
