# Install with pip

```bash
pip install gnsspp
```

installs the whole command suite plus the Python bindings. The PyPI
project is `gnsspp`; the Python module keeps its name (`import libgnsspp`). PyPI carries
prebuilt Linux wheels (x86_64, aarch64; CPython 3.10 to 3.14, from the first
release published by the workflow below); elsewhere pip builds from the sdist
with [scikit-build-core](https://scikit-build-core.readthedocs.io/). The
development tree:

```bash
pip install "git+https://github.com/rsasaki0109/gnssplusplus-library.git"
```

From a source checkout:

```bash
pip install .
```

## Requirements for source builds

- A C++20 compiler.
- Python 3.10 or newer.
- pip fetches pybind11, and CMake/Ninja when the system has no suitable
  version, into the isolated build environment.
- Eigen 3: the system copy is used when CMake finds it (for example
  `libeigen3-dev`). Otherwise the build downloads the Eigen 3.4.0 headers
  (SHA-256 checked) from gitlab.com.

The build takes several minutes. `CMAKE_BUILD_PARALLEL_LEVEL=8 pip install ...`
compiles in parallel.

## What gets installed

| Item | Use |
| --- | --- |
| `gnss` command | Same dispatcher as `apps/gnss.py`: `gnss --help`, `gnss demo`, `gnss spp`, `gnss solve`, `gnss ppp`, `gnss fgo`, ... |
| `libgnsspp` module | `import libgnsspp` (RINEX reading, solution loading, coordinate helpers, file-based `solve_*` wrappers) |
| `numpy`, `matplotlib` | Runtime dependencies of the Python commands (same set as the Debian package) |

The wheel holds the regular `cmake --install` tree (`bin/`, `share/`,
`configs/`, `tools/`, `scripts/`) inside the `libgnsspp` package directory, so
the `gnss` dispatcher finds its native tools and data exactly as in an
installed prefix. The C++ SDK (static libraries, headers, CMake and pkg-config
files) is left out; use a CMake build for C++ integration.

Optional extras used by some analysis and plotting commands:

```bash
pip install "gnsspp[full] @ git+https://github.com/rsasaki0109/gnssplusplus-library.git"
```

## Build settings

The wheel is built with `BUILD_TESTING=OFF`, `GNSSPP_BUILD_EXAMPLES=OFF`, and
without the optional GTSAM, CHOLMOD, OpenCV and ROS 2 integrations, so it has
no shared-library dependencies beyond the C/C++ runtime. Override any CMake
option with `--config-settings`, for example:

```bash
pip install . --config-settings=cmake.define.GNSSPP_USE_CHOLMOD=ON
```

To build distributable files:

```bash
python -m pip install build
python -m build            # dist/gnsspp-<version>.tar.gz and .whl
```

## Publishing to PyPI

`.github/workflows/pypi.yml` builds the sdist and manylinux_2_28 wheels
(x86_64 and aarch64, CPython 3.10 to 3.14). Each wheel is installed and tested
(`import libgnsspp`, `gnss --help`, `gnss demo`) before upload. Pushing a
`vMAJOR.MINOR.PATCH` tag that matches the `CMakeLists.txt` project version
publishes to PyPI through
[Trusted Publishing](https://docs.pypi.org/trusted-publishers/), so no API
token is stored in the repository. macOS and Windows users install from the
sdist, which compiles locally.

One-time setup by a maintainer:

1. On PyPI, open *Your projects → Publishing → Add a new pending publisher*
   and enter: PyPI project name `gnsspp`, owner `rsasaki0109`, repository
   `gnssplusplus-library`, workflow `pypi.yml`, environment `pypi`.
2. Optionally do the same on [TestPyPI](https://test.pypi.org/) with
   environment `testpypi`.
3. In the GitHub repository settings, create the environments `pypi` and
   `testpypi` (adding required reviewers to `pypi` gates every upload).

Dry runs: *Actions → PyPI → Run workflow* builds and tests everything; choose
`testpypi` to also upload there, then try it with

```bash
pip install --index-url https://test.pypi.org/simple/ \
    --extra-index-url https://pypi.org/simple/ gnsspp
```

A version can be uploaded to PyPI only once, so bump the `CMakeLists.txt`
version before tagging again. To build a wheel locally the same way CI does
(requires Docker):

```bash
pipx run cibuildwheel --platform linux --only cp313-manylinux_x86_64
```
