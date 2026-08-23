"""Tests for the fail-closed application SINEX coordinate reader."""

from __future__ import annotations

from datetime import datetime, timezone
from pathlib import Path
import sys
import tempfile
import unittest


ROOT_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT_DIR / "apps" / "commands"))

from support.gnss_sinex import read_station_estimate  # noqa: E402


SINEX = """%=SNX 2.02 TST 24:010:00000 TST 24:001:00000 24:007:00000 P 1 2 S
+SOLUTION/EPOCHS
*CODE PT SOLN T DATA_START__ DATA_END____ MEAN_EPOCH__
 TSK2  A   11 P 24:001:00000 24:007:00000 24:004:00000
 TSKB  A   12 P 24:001:00000 24:006:86399 24:003:86399
-SOLUTION/EPOCHS
+SOLUTION/APRIORI
     1 STAX   TSK2  A   11 24:003:43200 m 2 -1.00000000000000e+00 1.0e-03
     2 STAY   TSK2  A   11 24:003:43200 m 2 -2.00000000000000e+00 1.0e-03
     3 STAZ   TSK2  A   11 24:003:43200 m 2 -3.00000000000000e+00 1.0e-03
-SOLUTION/APRIORI
+SOLUTION/ESTIMATE
  1300 STAX   TSK2  A   11 24:003:43200 m 2 -3.95718496826790e+06 6.23438e-04
  1301 STAY   TSK2  A   11 24:003:43200 m 2  3.31023100877522e+06 5.54014e-04
  1302 STAZ   TSK2  A   11 24:003:43200 m 2  3.73770381925572e+06 5.60238e-04
  1303 STAX   TSKB  A   12 24:003:43200 m 2 -3.95720008745802e+06 5.38638e-04
  1304 STAY   TSKB  A   12 24:003:43200 m 2  3.31019900772871e+06 4.85017e-04
  1305 STAZ   TSKB  A   12 24:003:43200 m 2  3.73771149330339e+06 5.06776e-04
-SOLUTION/ESTIMATE
%ENDSNX
"""


class SinexTest(unittest.TestCase):
    def write(self, text: str = SINEX) -> Path:
        directory = Path(tempfile.mkdtemp(prefix="sinex_test_"))
        path = directory / "truth.snx"
        path.write_text(text, encoding="ascii")
        return path

    def test_reads_estimate_not_apriori(self) -> None:
        result = read_station_estimate(
            self.write(), "tsk2", datetime(2024, 1, 1, tzinfo=timezone.utc)
        )
        self.assertEqual(result.solution_id, "11")
        self.assertEqual(result.ecef_m, (-3957184.9682679, 3310231.00877522, 3737703.81925572))
        self.assertEqual(result.estimate_epoch, datetime(2024, 1, 3, 12, tzinfo=timezone.utc))

    def test_rejects_epoch_outside_validity(self) -> None:
        with self.assertRaisesRegex(ValueError, "found 0"):
            read_station_estimate(
                self.write(), "TSK2", datetime(2024, 1, 8, tzinfo=timezone.utc)
            )

    def test_rejects_duplicate_valid_solution(self) -> None:
        duplicate = SINEX.replace(
            " TSKB  A   12", " TSK2  A   12", 1
        )
        with self.assertRaisesRegex(ValueError, "found 2"):
            read_station_estimate(
                self.write(duplicate), "TSK2", datetime(2024, 1, 1, tzinfo=timezone.utc)
            )

    def test_rejects_incomplete_estimate(self) -> None:
        broken = SINEX.replace(
            "  1302 STAZ   TSK2  A   11 24:003:43200 m 2  3.73770381925572e+06 5.60238e-04\n",
            "",
        )
        with self.assertRaisesRegex(ValueError, "incomplete XYZ"):
            read_station_estimate(
                self.write(broken), "TSK2", datetime(2024, 1, 1, tzinfo=timezone.utc)
            )


if __name__ == "__main__":
    unittest.main()
