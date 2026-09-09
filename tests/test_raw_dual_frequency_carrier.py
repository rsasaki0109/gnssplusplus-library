import csv
import importlib.util
import io
from pathlib import Path
import unittest

spec = importlib.util.spec_from_file_location('dual', Path(__file__).resolve().parents[1] /
                                            'scripts/audit_raw_dual_frequency_carrier.py')
dual = importlib.util.module_from_spec(spec)
spec.loader.exec_module(dual)


class Payload:
    def __init__(self, rows):
        stream = io.StringIO()
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
        self.text = stream.getvalue()

    def open(self, mode='r', **kwargs):
        return io.BytesIO(self.text.encode()) if mode == 'rb' else io.StringIO(self.text)


def pair(epoch, l1, l5, state=1):
    return [dict(MessageType='Raw', ConstellationType=1, Svid=1,
                 utcTimeMillis=epoch, CarrierFrequencyHz=freq,
                 AccumulatedDeltaRangeState=state, AccumulatedDeltaRangeMeters=adr,
                 TimeNanos=epoch*1000000, TimeOffsetNanos=0,
                 HardwareClockDiscontinuityCount=0)
            for freq, adr in [(1575420000, l1), (1176450000, l5)]]


class DualFrequencyTest(unittest.TestCase):
    def test_common_change_cancels(self):
        r = dual.audit(Payload(pair(1000, 100, 20)+pair(2000, 105, 25)))
        self.assertEqual(r['groups']['1']['abs_max_mps'], 0)

    def test_differential_change_remains(self):
        r = dual.audit(Payload(pair(1000, 100, 20)+pair(2000, 106, 25)))
        self.assertEqual(r['groups']['1']['signed_median_mps'], 1)

    def test_slip_resets_and_never_bridges(self):
        r = dual.audit(Payload(pair(1000, 100, 20)+pair(1500, 102, 22, 5)+
                              pair(2000, 104, 24)))
        self.assertEqual(r['groups'], {})
        self.assertEqual(r['counts']['invalid_adr_pair'], 1)

    def test_clock_reset_and_offset_mismatch(self):
        rows = pair(1000, 100, 20)+pair(2000, 104, 24)
        rows[-1]['HardwareClockDiscontinuityCount'] = 1
        rows[-2]['HardwareClockDiscontinuityCount'] = 1
        self.assertEqual(dual.audit(Payload(rows))['groups'], {})
        rows = pair(1000, 100, 20)+pair(2000, 104, 24)
        rows[-1]['TimeOffsetNanos'] = 100
        self.assertEqual(dual.audit(Payload(rows))['groups'], {})

    def test_duplicate_fails(self):
        rows = pair(1000, 100, 20)
        with self.assertRaises(ValueError):
            dual.audit(Payload(rows+[rows[0]]))

    def test_raw_clock_interval_controls_rate(self):
        rows = pair(1000, 100, 20)+pair(2000, 106, 25)
        for row in rows[2:]:
            row['TimeNanos'] = 1500000000
        r = dual.audit(Payload(rows))
        self.assertEqual(r['groups']['1']['signed_median_mps'], 2)

    def test_nonmonotonic_raw_clock_rejected_despite_utc_progress(self):
        rows = pair(1000, 100, 20)+pair(2000, 106, 25)
        for row in rows[2:]:
            row['TimeNanos'] = 900000000
        self.assertEqual(dual.audit(Payload(rows))['groups'], {})

    def test_offset_change_is_part_of_interval(self):
        rows = pair(1000, 100, 20)+pair(2000, 106, 25)
        for row in rows[2:]:
            row['TimeOffsetNanos'] = -500000000
        self.assertEqual(dual.audit(Payload(rows))['groups']['1']['signed_median_mps'], 2)


if __name__ == '__main__':
    unittest.main()
