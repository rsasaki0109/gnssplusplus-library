# Install with pip

`pip` builds libgnss++ from source with
[scikit-build-core](https://scikit-build-core.readthedocs.io/) and installs the
whole command suite plus the Python bindings:

```bash
pip install "git+https://github.com/rsasaki0109/gnssplusplus-library.git"
```

From a source checkout:

```bash
pip install .
```

## Requirements

- A C++20 compiler.
- Python 3.10 or newer.
- CMake, Ninja and pybind11 are installed into the isolated build environment
  by pip; a system CMake is not needed.
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
pip install "libgnsspp[full] @ git+https://github.com/rsasaki0109/gnssplusplus-library.git"
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
python -m build            # dist/libgnsspp-<version>.tar.gz and .whl
```
