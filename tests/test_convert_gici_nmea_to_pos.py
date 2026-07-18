import sys
import tempfile
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

import convert_gici_nmea_to_pos as converter  # noqa: E402


class ConvertGiciNmeaToPosTests(unittest.TestCase):
    def test_ppc_gga_time_height_and_fix_mapping(self) -> None:
        nmea = "\n".join(
            [
                "$GPRMC,040431.200,A,3540.0589034,N,13947.4597532,E,7.58,198.52,230724,0.0,E,R,V*63",
                "$GPGGA,040431.200,3540.0589034,N,13947.4597532,E,5,37,1.0,2.444,M,37.551,M,22377.0,0000*71",
                "$GPRMC,040431.600,A,3540.0595418,N,13947.4590947,E,7.68,193.97,230724,0.0,E,R,V*68",
                "$GPGGA,040431.600,3540.0595418,N,13947.4590947,E,4,31,1.0,2.398,M,37.551,M,22377.0,0000*7C",
            ]
        )
        with tempfile.TemporaryDirectory() as temp_dir:
            source = Path(temp_dir) / "gici.nmea"
            output = Path(temp_dir) / "gici.pos"
            source.write_text(nmea + "\n", encoding="ascii")

            self.assertEqual(converter.convert(source, output, leap_seconds=18), 2)
            rows = [line.split() for line in output.read_text().splitlines() if not line.startswith("%")]

        self.assertEqual((int(rows[0][0]), float(rows[0][1])), (2324, 187489.2))
        self.assertAlmostEqual(float(rows[0][5]), 35.66764839, places=9)
        self.assertAlmostEqual(float(rows[0][6]), 139.7909958867, places=9)
        self.assertAlmostEqual(float(rows[0][7]), 39.995, places=6)
        self.assertEqual((int(rows[0][8]), int(rows[0][9])), (2, 37))
        self.assertEqual(int(rows[1][8]), 1)

    def test_gga_requires_a_dated_rmc(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            source = Path(temp_dir) / "gici.nmea"
            output = Path(temp_dir) / "gici.pos"
            source.write_text(
                "$GPGGA,040431.200,3540.0589034,N,13947.4597532,E,4,37,1.0,2.444,M,37.551,M,0.0,0000*00\n",
                encoding="ascii",
            )
            with self.assertRaisesRegex(ValueError, "GGA before dated RMC"):
                converter.convert(source, output, leap_seconds=18)


if __name__ == "__main__":
    unittest.main()
