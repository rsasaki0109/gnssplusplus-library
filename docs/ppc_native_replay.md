# PPC native replay from raw data

`gnss ppc-native-replay` rebuilds the selected native applications and generates
a fresh baseline from PPC RINEX and IMU inputs. It does not require historical
solver outputs. RTK, coupled RTK and fused antenna trajectories are scored
separately; their names and populations are retained in the manifest.

This source-tree workflow complements the historical `reproduce ppc-goal`
score-only lane. It does not reconstruct that lane's incompletely recorded
solver configuration or claim its published selected-tier scores.

## Run the evaluation

Configure a build from the current checkout first. Pass an explicit dataset
root and a new output directory. The replay command builds its own targets,
records the resulting binary hashes and fails if the source, binary or inputs
change while it is running.

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DGNSSPP_BUILD_PYTHON_BINDINGS=OFF
python3 apps/gnss.py ppc-native-replay --dataset-root /datasets/PPC-Dataset \
  --build-dir build --output-dir output/ppc-native-full
```

The default evaluates all six existing development runs with no epoch cap.
It runs native processes sequentially. `--paths rtk` or `--paths fusion` limits
the solver paths; `--runs tokyo/run1` selects a development run. On Windows use
`--build-config Release`, and add `--runtime-dir <GTSAM DLL directory>` when
the configured build links GTSAM. This workflow does not require GTSAM.

For a bounded connectivity check:

```bash
python3 apps/gnss.py ppc-native-replay --dataset-root /datasets/PPC-Dataset \
  --build-dir build --runs tokyo/run1 --max-epochs 120 \
  --output-dir output/ppc-native-smoke
```

A positive epoch cap is explicitly labelled `smoke`. The fresh baseline uses
`low-cost`, ratio 2.4, 18 subset drops, SNR weighting and disabled AR filtering.
The fusion route additionally enables `navi776-tc` and uses the documented
city-specific antenna lever arms. The actual arrays passed to each executable
are the authoritative recipe, stored in `manifest.json`.

## Inspect and repeat

With both paths selected, the output contains `source.tar.gz`, build and native
logs, saved CMake caches, RTK debug epochs, three POS streams per run, their score
summaries, and a versioned manifest. In-tree output must be Git-ignored; use
`output/`.
The archive includes edited and untracked source files that are not ignored
by Git. Extracting it into an empty directory reconstructs the source used
by the run. Input data remain external and are identified by SHA-256.

`state: passed` proves the requested native runs and scoring commands succeeded
and the recorded inputs stayed unchanged. It does not prove an accuracy target.
Runtime is recorded per native process; coupled RTK and fused outputs share
one fusion process, so their recorded runtime must not be summed twice.
Reference CSVs are hashed and read by the offline scorer, never passed to the
native inference command. These batch outputs do not certify online causality.

Repeat with a second empty directory and `--compare-to`:

```bash
python3 apps/gnss.py ppc-native-replay --dataset-root /datasets/PPC-Dataset \
  --build-dir build --output-dir output/ppc-native-repeat \
  --compare-to output/ppc-native-full/manifest.json
```

The comparison requires the same source contents, inputs, binaries, runtime
libraries, selected runs and epoch cap. It checks every emitted POS data field,
including positions and statuses, excluding comments and whitespace. A changed
population or solution fails the command and is retained in the manifest.
Source edits between the two runs require a new baseline.
