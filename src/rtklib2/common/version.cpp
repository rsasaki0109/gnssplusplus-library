#include "rtklib2/common/version.hpp"

#include <string_view>

namespace {
constexpr std::string_view kLibraryVersion{"0.1.0"};
}  // namespace

namespace rtklib2::common {

std::string library_version() {
    return std::string{kLibraryVersion};
}

}  // namespace rtklib2::common
