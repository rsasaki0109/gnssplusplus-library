# Phase348: recover root capacity from generated test debug information

Current storage inspection found root had 43 MB available. Another mounted
data disk had only 4.2 GB available and was not used. The generated normal
test executable was 300288552 bytes, although its text/data/BSS total was
about 16 MB. No raw data or evaluation artifacts were cleanup targets.

Copied the exact current test executable, preserving mode/timestamps, to
`/dev/shm/gnss-test-build-recovery.BJvQt9/run_tests.phase335.full`.
Used `objcopy --strip-debug` on that backup to create a separate executable
of 20061984 bytes. It passed all 40 base-compensation tests and the one
source-rover-state integration test before replacing `build/tests/run_tests`.
The same 41 tests passed again from the normal path afterward. This is not
full CTest or a solver output byte-identity gate.

Only generated test debug information was removed from the root filesystem;
the full executable remains recoverable from the tmpfs backup until reboot,
and the target is rebuildable from source afterward. The older Phase335
recovery backup also remains untouched. No positioning binary, raw input,
truth, candidate or evaluation result was changed/deleted.

Root available space increased to 311 MB. This permits modest existing-input
experiments but is not a sustained multi-route materialization solution.
Re-linking the normal tests with debug info will consume that space again;
use the existing focused/tmpfs compile approach where appropriate. No
accuracy improvement is claimed by this infrastructure recovery.
