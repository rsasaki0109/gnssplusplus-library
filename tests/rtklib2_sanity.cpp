#include <cassert>
#include <iostream>

#include "rtklib2/api/rtklib2.hpp"
#include "rtklib2/common/version.hpp"

int main() {
    rtklib2::api::Library library;
    library.initialize();

    auto version = rtklib2::common::library_version();
    assert(!version.empty());

    std::cout << "RTKLIB2 version: " << version << '\n';
    std::cout << "Sanity test completed successfully." << std::endl;
    return 0;
}
