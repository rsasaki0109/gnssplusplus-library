"""Compile the real app diagnostic function against synthetic in-memory data."""
import os
from pathlib import Path
import re
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


class CorrectionDiagnosticsTest(unittest.TestCase):
    def test_actual_runtime_report_validates_correction_handoff(self):
        source = (ROOT / 'apps/native/gnss_fgo_imu_no_base.cpp').read_text()
        constants = '\n'.join(re.findall(
            r'constexpr double kNativeTdcp(?:MaxGapS|SigmaM|CodePhaseJumpThresholdM) = [^;]+;', source))
        self.assertEqual(len(constants.splitlines()), 3)
        start = source.index('struct TdcpRuntimeReport {')
        report = source[start:source.index('// Phase116 is deliberately', start)]
        start = source.index('double phase116HuberCost(')
        code_functions = source[start:source.index('\nPhase116CarrierTdcpReport evaluatePhase116CarrierTdcp(', start)]
        start = source.index('    if (options.native_tdcp_frequency_residual_states &&',
                             source.index('    const TdcpRuntimeReport tdcp_report = evaluateTdcpRuntime('))
        guard = source[start:source.index('    if (options.native_pdc_imu_tdcp &&', start)]
        code = r'''
#include <libgnss++/algorithms/fgo.hpp>
#include <cmath>
#include <limits>
#include <cassert>
#include <stdexcept>
#include <iostream>
''' + constants + report + code_functions + r'''
int checkGuard(bool enabled, const libgnss::FGOProcessor::FGOResult& result,
               const TdcpRuntimeReport& tdcp_report) {
    struct { bool native_tdcp_frequency_residual_states; } options{enabled};
''' + guard + r'''
    return 0;
}
int main() {
    using Processor=libgnss::FGOProcessor;
    Processor::FGOProblem problem;
    Processor::TimeDifferencedCarrierFactor factor;
    factor.previous_epoch_index=0; factor.current_epoch_index=1;
    factor.satellite={libgnss::GNSSSystem::GPS,1};
    factor.delta_carrier_m=-.02;
    problem.tdcp_factors.push_back(factor);
    Processor::FGOResult result;
    result.solution.solutions.resize(2);
    for (auto& s: result.solution.solutions) {
        s.position_ecef=libgnss::Vector3d(1.,2.,3.); s.receiver_clock_bias=0.;
    }
    Processor::FGOConfig config;
    auto baseline=evaluateTdcpRuntime(problem,result,config,true);
    assert(std::abs(baseline.residual_rms_m-.02)<1e-12);
    config.use_native_tdcp_frequency_residual_states=true;
    const auto rejects=[&]() {
        try { (void)evaluateTdcpRuntime(problem,result,config,true); }
        catch (const std::invalid_argument&) { return true; }
        return false;
    };
    assert(rejects()); // missing export
    Processor::FGOResult::TdcpFrequencyCorrection correction;
    correction.current_epoch_index=1; correction.satellite=factor.satellite;
    correction.alpha_slant_change_m=.01;
    result.tdcp_frequency_corrections.push_back(correction);
    const auto corrected=evaluateTdcpRuntime(problem,result,config,true);
    assert(std::abs(corrected.residual_rms_m-.01)<1e-12);
    assert(corrected.finite_residuals==1);
    assert(corrected.signal_aggregates.size()==1);
    result.diagnostics.tdcp_frequency_residual_states=1;
    result.diagnostics.tdcp_frequency_residual_priors=1;
    result.diagnostics.tdcp_residual_rms_m=corrected.residual_rms_m;
    assert(checkGuard(true,result,corrected)==0);
    result.diagnostics.tdcp_frequency_residual_priors=0;
    assert(checkGuard(true,result,corrected)==1);
    assert(checkGuard(false,result,corrected)==0);
    result.diagnostics.tdcp_frequency_residual_priors=1;
    result.diagnostics.tdcp_residual_rms_m+=2e-7;
    assert(checkGuard(true,result,corrected)==1);
    result.diagnostics.tdcp_residual_rms_m=std::numeric_limits<double>::quiet_NaN();
    assert(checkGuard(true,result,corrected)==1);
    result.diagnostics.tdcp_residual_rms_m=corrected.residual_rms_m;
    auto invalid_report=corrected;
    invalid_report.residual_rms_m=std::numeric_limits<double>::infinity();
    assert(checkGuard(true,result,invalid_report)==1);
    result.tdcp_frequency_corrections[0].current_epoch_index=2;
    assert(rejects());
    result.tdcp_frequency_corrections[0]=correction;
    result.tdcp_frequency_corrections[0].satellite.prn=2;
    assert(rejects());
    result.tdcp_frequency_corrections[0]=correction;
    result.tdcp_frequency_corrections[0].signal=libgnss::SignalType::GPS_L5;
    assert(rejects());
    result.tdcp_frequency_corrections[0]=correction;
    result.tdcp_frequency_corrections[0].alpha_slant_change_m=std::numeric_limits<double>::quiet_NaN();
    assert(rejects());
    result.tdcp_frequency_corrections[0]=correction;
    config.use_native_tdcp_frequency_residual_states=false;
    assert(rejects()); // OFF mode cannot silently consume an export
}
'''
        with tempfile.TemporaryDirectory(prefix='tdcp-correction-test-') as directory:
            executable = Path(directory) / 'test'
            compiled = subprocess.run([os.environ.get('CXX', 'c++'), '-std=c++17', '-O0',
                '-I'+str(ROOT/'include'), '-I/usr/include/eigen3', '-x', 'c++', '-',
                '-o', str(executable)], input=code, text=True, capture_output=True)
            self.assertEqual(compiled.returncode, 0, compiled.stderr)
            ran = subprocess.run([str(executable)], capture_output=True, text=True)
            self.assertEqual(ran.returncode, 0, ran.stderr)


if __name__ == '__main__':
    unittest.main()
