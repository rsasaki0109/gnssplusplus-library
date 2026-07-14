# Applications

The application layer is organized by role:

- `gnss.py` is the stable source-tree dispatcher used by documentation, tests,
  and development workflows.
- `native/` contains C++ command implementations built by CMake. Their target
  names and installed executable names remain unchanged.
- top-level `gnss_*.py` files implement Python commands and their shared
  modules. They remain together because several commands are also importable
  modules.
- `compat/` contains legacy executable names that forward to maintained tools.

Add user-facing commands to the `COMMANDS` registry in `gnss.py`. Native
commands must also be declared in `CMakeLists.txt`; installed Python commands
must be listed in its `install(PROGRAMS ...)` block.
