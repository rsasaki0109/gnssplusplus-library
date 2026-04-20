#!/usr/bin/env python3
"""Smoke tests for the optional ROS2 solution publisher node."""

from __future__ import annotations

import os
import subprocess
import tempfile
import unittest
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]
BINARY_DIR = Path(os.environ.get("GNSSPP_BINARY_DIR", ROOT_DIR / "build"))


def find_node_binary() -> Path:
    candidates = [
        BINARY_DIR / "ros2" / "gnss_solution_node",
        BINARY_DIR / "gnss_solution_node",
    ]
    for candidate in candidates:
        if candidate.exists():
            return candidate
    hits = list(BINARY_DIR.rglob("gnss_solution_node"))
    if not hits:
        raise FileNotFoundError("gnss_solution_node not found in build tree")
    return hits[0]


class Ros2NodeSmokeTest(unittest.TestCase):
    def test_solution_node_publishes_and_exits(self) -> None:
        node_binary = find_node_binary()
        with tempfile.TemporaryDirectory(prefix="gnss_ros2_node_") as temp_dir:
            solution_path = Path(temp_dir) / "sample.pos"
            solution_path.write_text(
                "% synthetic solution\n"
                "1316 518400.0 -3978242.0 3382841.0 3649903.0 35.0 139.0 10.0 4 9 1.0\n"
                "1316 518430.0 -3978243.0 3382840.0 3649902.0 35.0 139.0 10.0 6 10 2.5\n",
                encoding="ascii",
            )

            result = subprocess.run(
                [
                    str(node_binary),
                    "--ros-args",
                    "-p",
                    f"solution_file:={solution_path}",
                    "-p",
                    "publish_period_ms:=1",
                    "-p",
                    "max_messages:=2",
                    "-p",
                    "status_topic:=/custom/status",
                ],
                check=False,
                capture_output=True,
                text=True,
                timeout=10,
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            combined = result.stdout + result.stderr
            self.assertIn("published_messages=2", combined)
            self.assertIn("published_fixed=1", combined)
            self.assertIn("published_ppp_fixed=1", combined)
            self.assertIn("published_path_points=2", combined)


if __name__ == "__main__":
    unittest.main()
