"""Unit tests for ``apps/gnss_dd_residuals_html``."""

from __future__ import annotations

import sys
import tempfile
import unittest
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]
APPS_DIR = ROOT_DIR / "apps"
sys.path.insert(0, str(APPS_DIR))

import gnss_dd_residuals_html as html_mod  # noqa: E402


class TestHtmlValue(unittest.TestCase):
    def test_none_becomes_na(self):
        self.assertEqual(html_mod.html_value(None), "n/a")

    def test_escapes_special_characters(self):
        self.assertEqual(html_mod.html_value("<script>"), "&lt;script&gt;")

    def test_passes_through_numbers(self):
        self.assertEqual(html_mod.html_value(42), "42")


class TestMetricCell(unittest.TestCase):
    def test_wraps_label_and_value(self):
        out = html_mod.metric_cell("rows", 10)
        self.assertIn("rows", out)
        self.assertIn(">10<", out)
        self.assertIn("metric-label", out)
        self.assertIn("metric-value", out)


class TestStatsTable(unittest.TestCase):
    def test_empty_rows_renders_placeholder(self):
        out = html_mod.stats_table("By Kind", [], ["kind"])
        self.assertIn("No rows.", out)

    def test_includes_label_keys_in_header(self):
        rows = [
            {
                "kind": "phase",
                "rows": 10,
                "epochs": 5,
                "suppressed_rows": 0,
                "rms_residual_m": 0.01,
                "p95_abs_residual_m": 0.02,
                "max_abs_residual_m": 0.03,
                "rms_normalized_residual": 0.5,
                "p95_abs_normalized_residual": 1.0,
                "max_abs_normalized_residual": 1.5,
            }
        ]
        out = html_mod.stats_table("By Kind", rows, ["kind"])
        self.assertIn("<th>kind</th>", out)
        self.assertIn("phase", out)


class TestNiceMax(unittest.TestCase):
    def test_returns_one_for_empty_input(self):
        self.assertEqual(html_mod.nice_max([]), 1.0)

    def test_returns_one_for_nonpositive_maximum(self):
        self.assertEqual(html_mod.nice_max([0.0, -1.0]), 1.0)

    def test_rounds_up_to_125_decade_step(self):
        # 1.7 → next nice step is 2.0; 4.3 → 5.0; 7.8 → 10.0
        self.assertEqual(html_mod.nice_max([1.7]), 2.0)
        self.assertEqual(html_mod.nice_max([4.3]), 5.0)
        self.assertEqual(html_mod.nice_max([7.8]), 10.0)

    def test_handles_subunit_inputs(self):
        # 0.03 → 0.05 (next 5×10^-2 step)
        self.assertAlmostEqual(html_mod.nice_max([0.03]), 0.05)


class TestSvgRenderingNoData(unittest.TestCase):
    """Each SVG renderer must degrade gracefully on empty input."""

    def test_line_chart_empty(self):
        out = html_mod.svg_line_chart("Empty", [], [("a", "A", "#000")])
        self.assertIn("No chartable rows.", out)

    def test_bar_chart_empty(self):
        out = html_mod.svg_bar_chart("Empty", [], "value_key")
        self.assertIn("No rows.", out)

    def test_residual_scatter_empty(self):
        out = html_mod.svg_residual_scatter("Empty", [], "phase")
        self.assertIn("No phase rows.", out)

    def test_pair_heatmap_empty(self):
        out = html_mod.svg_pair_heatmap("Empty", [], [], "phase")
        self.assertIn("No phase rows.", out)


class TestSvgLineChartRendering(unittest.TestCase):
    def test_renders_polyline_for_finite_series(self):
        series = [
            {"epoch_index": 1, "tow": 1.0, "value": 0.1},
            {"epoch_index": 2, "tow": 2.0, "value": 0.2},
            {"epoch_index": 3, "tow": 3.0, "value": 0.3},
        ]
        out = html_mod.svg_line_chart(
            "T", series, [("value", "v", "#0f766e")]
        )
        self.assertIn("<polyline", out)
        self.assertIn("v", out)


class TestSvgResidualScatterRendering(unittest.TestCase):
    def test_renders_circles_for_kind_match(self):
        points = [
            {
                "epoch_index": 1,
                "tow": 0.0,
                "kind": "phase",
                "frequency_index": 0,
                "frequency_label": "freq0 primary (L1/E1/B1)",
                "pair": "G01-G05",
                "residual_m": 0.01,
                "abs_residual_m": 0.01,
                "suppressed_by_outlier_threshold": 0,
            }
        ]
        out = html_mod.svg_residual_scatter("Phase", points, "phase")
        self.assertIn("<circle", out)


class TestWriteHtmlReport(unittest.TestCase):
    def test_writes_self_contained_document(self):
        summary = {
            "rows": 0,
            "coverage": {
                "rows": 0,
                "epochs": 0,
                "first_epoch_index": None,
                "last_epoch_index": None,
                "first_tow": None,
                "last_tow": None,
                "satellites": 0,
                "satellite_pairs": 0,
                "pair_frequency_kind_tracks": 0,
                "frequency_labels": [],
            },
            "overall": {
                "rows": 0,
                "suppressed_rows": 0,
                "max_abs_residual_m": None,
                "worst_row": None,
            },
            "by_kind": {"phase": {"p95_abs_residual_m": None, "p95_abs_normalized_residual": None},
                        "code": {"p95_abs_residual_m": None, "p95_abs_normalized_residual": None}},
            "by_frequency": [],
            "pair_tracks": [],
            "top_pairs": [],
            "top_normalized_pairs": [],
            "time_series": [],
            "residual_points": [],
        }
        with tempfile.TemporaryDirectory() as tmpdir:
            out_path = Path(tmpdir) / "report.html"
            html_mod.write_html_report(out_path, summary)
            content = out_path.read_text(encoding="utf-8")
        # Must be a complete document with no external dependencies.
        self.assertIn("<!doctype html>", content)
        self.assertIn("<title>DD Residual Report</title>", content)
        self.assertNotIn("<link ", content)  # no external stylesheets
        self.assertNotIn("<script", content)  # no external scripts


if __name__ == "__main__":
    unittest.main()
