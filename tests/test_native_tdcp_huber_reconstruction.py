"""Compile and exercise the actual app's cost helper without running GNSS."""
import os
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


class NativeTdcpHuberReconstructionTest(unittest.TestCase):
    def test_production_helper_boundaries_and_invalid_input(self):
        source = (ROOT / 'apps/native/gnss_fgo_imu_no_base.cpp').read_text()
        start = source.index('double phase116HuberCost(', source.index('return report;',
                             source.index('TdcpRuntimeReport evaluateTdcpRuntime(')))
        end = source.index('\nPhase116CarrierTdcpReport evaluatePhase116CarrierTdcp(', start)
        helper = source[start:end]
        code = r'''
#include <libgnss++/algorithms/fgo.hpp>
#include <cmath>
#include <limits>
#include <cassert>
''' + helper + r'''
int main() {
    libgnss::FGOProcessor::FGOConfig config;
    config.use_robust_loss = true;
    config.tdcp_huber_threshold_sigma = 4.0;
    bool tail = false;
    for (double sign : {-1.0, 1.0}) {
        assert(phase116HuberCost(sign * 2.0, config, tail) == 2.0 && !tail);
        assert(phase116HuberCost(sign * 4.0, config, tail) == 8.0 && !tail);
        assert(phase116HuberCost(sign * 10.0, config, tail) == 32.0 && tail);
    }
    assert(phase116HuberCost(0.0, config, tail) == 0.0 && !tail);
    config.use_robust_loss = false;
    assert(phase116HuberCost(10.0, config, tail) == 50.0 && !tail);
    config.use_robust_loss = true;
    config.tdcp_huber_threshold_sigma = 0.0;
    assert(phase116HuberCost(10.0, config, tail) == 50.0 && !tail);
    assert(std::isnan(phase116HuberCost(
        std::numeric_limits<double>::infinity(), config, tail)) && !tail);
    config.use_official_tdcp_huber_k = true;
    config.use_native_phase184_source_tdcp_huber_k = true;
    assert(std::isnan(phase116HuberCost(1.0, config, tail)) && !tail);
}
'''
        with tempfile.TemporaryDirectory(prefix='native-huber-test-') as directory:
            executable = Path(directory) / 'check'
            subprocess.run([os.environ.get('CXX', 'c++'), '-std=c++17', '-O0',
                            '-I' + str(ROOT / 'include'), '-I/usr/include/eigen3',
                            '-x', 'c++', '-', '-o', str(executable)],
                           input=code, text=True, check=True, capture_output=True)
            subprocess.run([str(executable)], check=True, capture_output=True)


if __name__ == '__main__':
    unittest.main()
