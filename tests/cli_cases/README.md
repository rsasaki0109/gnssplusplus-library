# CLI regression cases

`tests/test_cli_tools.py` remains the compatibility runner and defines the
single concrete `CLIToolsTest`. The test methods live in domain mixins here;
the filenames intentionally do not begin with `test_`, so unittest discovery
does not load the mixin modules as independent suites.

## Domain ownership

- `diagnostics_data.py`: doctor, bag, robotics, RINEX, products, IONEX, and DCB checks (14)
- `analysis_visuals.py`: SPP/visibility/statistics/plot/track/report rendering (11)
- `cli_surface.py`: help, benchmark, SmartLoc, and built ROS2 node checks (13)
- `ppp_processing.py`: synthetic and sampled PPP/SSR/CLAS processing checks (24)
- `stream_protocols.py`: stream relay, file protocol decoders, serial decoders, and conversion (32)
- `qzss_l6_decode.py`: QZSS L6 frame and compact correction decoding (21)
- `qzss_l6_policy.py`: QZSS L6 bias, atmosphere, phase, and row-policy checks (25)
- `signoffs.py`: RTK, PPP, product, and PPC signoff workflows (29)
- `clas_ppp.py`: direct CLAS PPP correction profiles and policy checks (15)
- `runtime_receiver.py`: replay, live, moving-base, web, artifact, and receiver lifecycle checks (28)

All methods retain their original `CLIToolsTest` names and source bodies.
Shared imports, constants, fixture builders, class datasets, and helper
methods are provided by `_support.py`; the compatibility runner re-exports
the public support names for existing callers.
