## Repository Layout

```
legacy/v1/          # Previous gnss++ implementation retained for reference
include/rtklib2/    # Public headers for the new core
src/rtklib2/        # Library implementation
tests/              # Unit & integration tests (ctest)
docs/               # Architecture notes, ADRs, specifications
cmake/              # Packaging helpers and toolchain snippets
scripts/            # Utilities and data processing aides
```

## Getting Started

```bash
git clone git@github.com:rsasaki0109/gnssplusplus-library.git
cd gnssplusplus-library
git checkout v2

cmake -S . -B build -DRTKLIB2_BUILD_TESTS=ON -DRTKLIB2_BUILD_EXAMPLES=OFF
cmake --build build
ctest --test-dir build --output-on-failure
```

### CMake Options

| Option | Default | Description |
| --- | --- | --- |
| `RTKLIB2_BUILD_TESTS` | `ON` | Build the sanity and future unit/integration tests |
| `RTKLIB2_BUILD_EXAMPLES` | `ON` | Build example applications once available |
| `RTKLIB2_ENABLE_WARNINGS` | `ON` | Enable strict compiler warnings |

The project targets **C++20** and requires **CMake 3.24+**. On GNU/Clang
toolchains we recommend enabling LTO and sanitizers through presets (coming
soon) for development builds.

## Documentation

- Architecture overview: `docs/architecture/overview.md`
- Decision log (ADR): `docs/adr/`
- Roadmap & milestones: `docs/roadmap.md`

Documentation will evolve as modules transition from skeletons to fully fledged
implementations. Contributions that improve or extend the docs are highly
encouraged.

## Contributing

1. Fork the repository and create a feature branch off `v2`.
2. Follow the coding style enforced by `clang-format` (configuration coming
   soon) and document public APIs.
3. Ensure `cmake --build build` and `ctest` pass locally.
4. Submit a pull request describing the motivation, design, and testing.

Please see `CONTRIBUTING.md` for additional guidelines. Bug reports and feature
requests are welcome via GitHub issues.

## License

MIT License – see `LICENSE` for details. RTKLIB v2 builds upon the heritage of
the original RTKLIB project by T. Takasu and the community; attribution is
maintained within the legacy code retained under `legacy/v1`.
# RTKLIB v2

A modernized, modular GNSS positioning framework that re-imagines the original
RTKLIB codebase with contemporary C++20, clean architecture, and first-class
developer experience.

## Project Goals

- **Composable architecture** – clearly defined module boundaries for ingestion,
  solvers, services, and telemetry.
- **Real-time readiness** – asynchronous pipelines, deterministic scheduling,
  and observability hooks baked in.
- **Extensibility** – plugin-friendly abstractions for new constellations,
  correction sources, and estimation algorithms.
- **Polyglot access** – C++ API first, with Python bindings and gRPC/REST
  gateways on the roadmap.
