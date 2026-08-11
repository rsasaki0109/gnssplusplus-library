# Self-contained offline demo

The demo is the shortest way to verify a fresh checkout can run a positioning
pipeline without a dataset download, network service, or credentials. After
the normal native build, run one command from the repository root:

```bash
python3 apps/gnss.py demo
```

The command runs the built `gnss_ppp` executable against the three tracked
files in `demo/fixtures/` and writes:

```text
output/self-contained-demo/demo_solution.pos
output/self-contained-demo/demo_solution.kml
output/self-contained-demo/demo_summary.json
```

The `.pos` file is the libgnss++ RTKLIB-compatible position format, the KML is
the solver's track output, and the JSON combines the native PPP counters with
the demo's `self-contained-demo.v1` provenance block. A successful run has
eight processed and eight valid PPP solutions. The output directory can be
changed with `--output-dir PATH`.

## Native checkout

If this checkout has not been built yet:

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --target gnss_ppp --parallel 2
python3 apps/gnss.py demo
```

The demo command only opens local files. It does not call `fetch-products`,
NTRIP, HTTP, or any other network/credential source. A missing binary is
reported with the build command rather than silently falling back to a fake
solution.

## Docker

Build the existing runtime image, then run the same command with the output
directory mounted for easy inspection:

```bash
docker build -t libgnsspp:demo .
docker run --rm \
  -v "$PWD/output:/workspace/output" \
  libgnsspp:demo demo --output-dir /workspace/output/self-contained-demo
```

The image installs the same fixture under
`/opt/libgnsspp/share/libgnsspp/demo`; the container run itself needs no
network or credentials. The image build may use the package mirrors required
by the repository's existing `Dockerfile` when the base image/dependencies
are not already cached.

## Fixture provenance and limits

`demo/fixtures/` is a project-authored, deterministic fixture: six GPS
satellites are placed at known positions around a receiver at 35°N, 139°E,
45 m ellipsoidal height; pseudorange and carrier-phase values are computed
from geometric range, and two precise-product epochs are provided. The
equations and formatting are shared with the existing
`build_synthetic_ppp_inputs` test helper in `tests/cli_cases/_support.py`.

The three inputs are small, tracked text files distributed under this
repository's MIT license. No third-party raw observation or precise-product
file is bundled. Because the measurements are synthetic, this demo proves
CLI/build/artifact plumbing only; it is not evidence of field accuracy,
real-world satellite geometry, or RTK fix performance.
