"""Road-height map builder (scripts/experiments/road_height/road_height_maps.py)."""

from dataclasses import dataclass
import importlib.util
import math
from pathlib import Path
import unittest

import numpy as np

try:
    import scipy.spatial  # noqa: F401

    HAVE_SCIPY = True
except ImportError:
    HAVE_SCIPY = False

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location(
    "road_height_maps", ROOT / "scripts/experiments/road_height/road_height_maps.py"
)
maps = importlib.util.module_from_spec(spec)
spec.loader.exec_module(maps)


@dataclass
class Epoch:
    status: int
    height_m: float


class DemPngTest(unittest.TestCase):
    def test_decodes_positive_negative_and_missing_heights(self):
        def rgb(value):
            value %= 2 ** 24
            return [value >> 16, (value >> 8) & 0xFF, value & 0xFF]

        pixels = np.array([[rgb(3825), rgb(-250), rgb(2 ** 23)]], dtype=np.uint8)
        height = maps.decode_dem_png(pixels)
        self.assertAlmostEqual(height[0, 0], 38.25)
        self.assertAlmostEqual(height[0, 1], -2.50)
        self.assertTrue(math.isnan(height[0, 2]))

    def test_tile_index_contains_the_point(self):
        lat, lon = 35.169, 136.892
        x, y = maps.dem_tile(lat, lon)
        n = 2 ** 15
        west, east = x / n * 360 - 180, (x + 1) / n * 360 - 180
        north = math.degrees(math.atan(math.sinh(math.pi * (1 - 2 * y / n))))
        south = math.degrees(math.atan(math.sinh(math.pi * (1 - 2 * (y + 1) / n))))
        self.assertTrue(west <= lon < east and south < lat <= north)


class PlateauRasterTest(unittest.TestCase):
    def test_overlapping_levels_spread_and_single_levels_do_not(self):
        ground = np.array([[0.0, 0.0, 3.0], [10.0, 0.0, 3.0], [0.0, 10.0, 3.0]])
        bridge = np.array([[0.0, 0.0, 9.0], [4.0, 0.0, 9.0], [0.0, 4.0, 9.0]])
        nodes = maps.rasterize_road(np.array([ground, bridge]))
        self.assertEqual(nodes[(1, 1)], [3.0, 9.0])  # under the bridge: two levels
        self.assertEqual(nodes[(7, 1)], [3.0])
        self.assertNotIn((9, 9), nodes)  # outside the ground triangle

    def test_tileset_leaves_returns_only_leaf_content(self):
        tileset = {"root": {"content": {"uri": "data/root.b3dm"}, "children": [
            {"content": {"uri": "data/a.b3dm"}},
            {"children": [{"content": {"uri": "data/b.b3dm"}}]},
        ]}}
        self.assertEqual(maps.tileset_leaves(tileset), ["data/a.b3dm", "data/b.b3dm"])


class OffsetAndSigmaTest(unittest.TestCase):
    @unittest.skipUnless(HAVE_SCIPY, "scipy is needed for the nearest-node search")
    def test_uses_fix_epochs_on_the_map_only(self):
        map_xy = np.array([[0.0, 0.0], [10.0, 0.0]])
        map_height = np.array([40.0, 41.0])
        sols = [Epoch(4, 41.5), Epoch(4, 42.6), Epoch(4, 41.4), Epoch(3, 50.0), Epoch(4, 90.0)]
        track = np.array([[0.2, 0.0, 0.0], [10.0, 0.5, 0.0], [0.0, -0.3, 0.0], [0.0, 0.0, 0.0], [5.0, 0.0, 0.0]])
        offset, sigma, n_fix = maps.offset_and_sigma(sols, track, map_xy, map_height)
        # FLOAT epoch and the FIX 5 m from any node are ignored: residuals 1.5, 1.6, 1.4.
        self.assertEqual(n_fix, 3)
        self.assertAlmostEqual(offset, 1.5)
        self.assertAlmostEqual(sigma, 1.4826 * 0.1)


if __name__ == "__main__":
    unittest.main()
