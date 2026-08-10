#!/usr/bin/env python3
"""CLI regression tests for the dispatcher-backed native tools."""

from __future__ import annotations

if __package__:
    from .cli_cases import _support
    from .cli_cases._support import *  # noqa: F401,F403
    from .cli_cases.analysis_visuals import AnalysisVisualsCases
    from .cli_cases.clas_ppp import CLASPPPCases
    from .cli_cases.cli_surface import CLISurfaceCases
    from .cli_cases.diagnostics_data import DiagnosticsDataCases
    from .cli_cases.ppp_processing import PPPProcessingCases
    from .cli_cases.qzss_l6_decode import QZSSL6DecodeCases
    from .cli_cases.qzss_l6_policy import QZSSL6PolicyCases
    from .cli_cases.runtime_receiver import RuntimeReceiverCases
    from .cli_cases.signoffs import SignoffCases
    from .cli_cases.stream_protocols import StreamProtocolCases
else:
    from cli_cases import _support
    from cli_cases._support import *  # noqa: F401,F403
    from cli_cases.analysis_visuals import AnalysisVisualsCases
    from cli_cases.clas_ppp import CLASPPPCases
    from cli_cases.cli_surface import CLISurfaceCases
    from cli_cases.diagnostics_data import DiagnosticsDataCases
    from cli_cases.ppp_processing import PPPProcessingCases
    from cli_cases.qzss_l6_decode import QZSSL6DecodeCases
    from cli_cases.qzss_l6_policy import QZSSL6PolicyCases
    from cli_cases.runtime_receiver import RuntimeReceiverCases
    from cli_cases.signoffs import SignoffCases
    from cli_cases.stream_protocols import StreamProtocolCases


class CLIToolsTest(
    _support._CLIToolsShared,
    DiagnosticsDataCases,
    AnalysisVisualsCases,
    CLISurfaceCases,
    PPPProcessingCases,
    StreamProtocolCases,
    QZSSL6DecodeCases,
    QZSSL6PolicyCases,
    SignoffCases,
    CLASPPPCases,
    RuntimeReceiverCases,
):
    """Compatibility facade combining all CLI regression domains."""


if __name__ == "__main__":
    unittest.main()
