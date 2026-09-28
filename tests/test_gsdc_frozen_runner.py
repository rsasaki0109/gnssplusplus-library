"""Runner lifecycle checks with simulated children; no GNSS inference."""
import contextlib
import io
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts/analysis'))
import run_gsdc_frozen_pairs as runner


class FrozenRunnerTest(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.binary = self.root / 'simulated-binary'
        self.binary.write_text('not executable; Popen is mocked')
        self.input = self.root / 'input'
        self.input.write_text('frozen input')
        self.calls = []

    def plan(self, arms):
        commands = {}
        for arm in arms:
            folder = self.root / arm
            commands[arm] = [str(self.binary), '--out', str(folder / 'solution.csv'),
                             '--summary-json', str(folder / 'summary.json')]
        data = dict(binary=str(self.binary), binary_sha256=runner.sha(self.binary),
                    runs={'simulated/phone': dict(argv=commands,
                        inputs_sha256={str(self.input): runner.sha(self.input)})})
        if len(arms) == 3:
            data['arm_order'] = arms
        path = self.root / 'plan.json'
        path.write_text(json.dumps(data))
        return path

    def run_plan(self, path, fail_arm=None):
        def child(command, **kwargs):
            folder = Path(command[command.index('--out') + 1]).parent
            self.calls.append(folder.name)
            (folder / 'solution.csv').write_text('simulated')
            (folder / 'summary.json').write_text('{}')
            class Process:
                pid = 123
                def wait(self):
                    return 1 if folder.name == fail_arm else 0
            return Process()
        with patch.object(sys, 'argv', ['runner', str(path), '--workers', '1']), \
             patch.object(runner.subprocess, 'Popen', side_effect=child), \
             contextlib.redirect_stdout(io.StringIO()):
            runner.main()

    def test_legacy_pair_and_exclusive_restart(self):
        path = self.plan(['control', 'candidate'])
        self.run_plan(path)
        self.assertEqual(self.calls, ['control', 'candidate'])
        self.assertTrue(json.loads((self.root / 'execution.done.json').read_text())['native_execution_complete'])
        with self.assertRaises(FileExistsError):
            self.run_plan(path)
        self.assertEqual(len(self.calls), 2)

    def test_three_arms_complete_in_order(self):
        arms = ['control', 'clock_only', 'candidate']
        self.run_plan(self.plan(arms))
        self.assertEqual(self.calls, arms)
        self.assertTrue(json.loads((self.root / 'execution.done.json').read_text())['native_execution_complete'])

    def test_clock_failure_prevents_candidate(self):
        path = self.plan(['control', 'clock_only', 'candidate'])
        with self.assertRaises(SystemExit):
            self.run_plan(path, fail_arm='clock_only')
        self.assertEqual(self.calls, ['control', 'clock_only'])
        self.assertFalse((self.root / 'candidate').exists())
        self.assertFalse(json.loads((self.root / 'execution.done.json').read_text())['native_execution_complete'])

    def test_changed_input_prevents_launch(self):
        path = self.plan(['control', 'candidate'])
        self.input.write_text('changed')
        with self.assertRaises(SystemExit):
            self.run_plan(path)
        self.assertEqual(self.calls, [])

    def test_undeclared_third_arm_rejected_before_start(self):
        path = self.plan(['control', 'clock_only', 'candidate'])
        data = json.loads(path.read_text())
        del data['arm_order']
        path.write_text(json.dumps(data))
        with self.assertRaises(AssertionError):
            self.run_plan(path)
        self.assertFalse((self.root / 'execution.started.json').exists())


if __name__ == '__main__':
    unittest.main()
