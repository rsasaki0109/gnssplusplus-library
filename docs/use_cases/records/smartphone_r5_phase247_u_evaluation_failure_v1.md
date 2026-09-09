# Phase247 terminal evaluation failure

Frozen commit 6bb2c7e; 13 metadata/synthetic tests passed. One authorized
evaluation attempt terminated with exit 1. No score/result was produced.
The attempt claim remains; do not retry or overwrite this experiment.

Candidate and truth payloads were each read once by Phase189 before its
call into Phase74 rejected missing truth key 1678311290448. Preparation
correctly identified the 1101/1102 coverage difference but missed that
Phase189 passes expected_missing=None, requiring no missing truth keys.
The inherited tests did not cover this unequal-domain case. This was an
evaluation-preparation error, not a native solver failure.

Read-only raw CSV inspection after failure confirmed the first Raw UTC key
is 1678311290448. Thus the missing truth key is the excluded warmup epoch.
No coordinate interpretation, interpolation, truth trimming or second
score attempt was performed. Phase247 filenames inherit h_accuracy, but
their frozen route and candidate are U; they are not H results.

Next investigate publishing the native solution for the first epoch rather
than relaxing full-coverage scoring. The native graph contains 1102 epochs
but the current raw-output policy excludes one. Preserve H's existing
truth-domain behavior; any output-policy change needs explicit raw-key
tests and a new frozen run. Goal remains incomplete, including coverage.
