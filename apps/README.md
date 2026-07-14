# Applications

The application layer is organized by role:

- `gnss.py` is the stable source-tree dispatcher used by documentation, tests,
  and development workflows.
- `native/` contains C++ command implementations built by CMake. Their target
  names and installed executable names remain unchanged.
- top-level `gnss_*.py` files are user-facing Python command entrypoints.
- `support/` contains importable Python implementation modules shared by those
  entrypoints. It is installed as a package beside the command scripts.
- `compat/` contains legacy executable names that forward to maintained tools.

Add user-facing commands to the `COMMANDS` registry in `gnss.py`. Native
commands must also be declared in `CMakeLists.txt`; installed Python commands
must be listed in its `install(PROGRAMS ...)` block.
