#pragma once

#include <Eigen/Core>

#include <fstream>
#include <iosfwd>

namespace libgnss::ppp_clas_diagnostics {

std::ofstream* ambiguityDatumStream();
std::ofstream* geometryDumpStream();
std::ofstream* codeDumpStream();

bool phaseRowDumpEnabled();
void writeCodePhaseExtension(std::ostream& out);
void writePhasePhaseExtension(
    std::ostream& out,
    double cpc_m,
    double carrier_correction_m,
    double cpc_minus_trop_m,
    double phase_bias_m,
    double windup_m,
    double phase_compensation_m,
    double raw_l_m,
    double corrected_l_m);

bool selectedGeometryDumpTow(double tow);
Eigen::Vector3d geometryDumpReceiverPosition(const Eigen::Vector3d& fallback);

}  // namespace libgnss::ppp_clas_diagnostics
