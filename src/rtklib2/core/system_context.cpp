#include "rtklib2/core/system_context.hpp"

#include <utility>

namespace rtklib2::core {

SystemContext::SystemContext() = default;
SystemContext::~SystemContext() = default;

SystemContext::SystemContext(SystemContext&&) noexcept = default;
SystemContext& SystemContext::operator=(SystemContext&&) noexcept = default;

const std::string& SystemContext::profile() const noexcept { return profile_; }

void SystemContext::set_profile(std::string profile) { profile_ = std::move(profile); }

}  // namespace rtklib2::core
