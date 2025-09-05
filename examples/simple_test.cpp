#include <libgnss++/core/types.hpp>
#include <iostream>

int main() {
    libgnss::SatelliteId sat(libgnss::GNSSSystem::GPS, 1);
    std::cout << "Creating satellite: " << sat.toString() << std::endl;
    std::cout << "Test successful!" << std::endl;
    return 0;
}
