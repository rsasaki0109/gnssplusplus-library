#include "rtklib2/services/service_registry.hpp"

namespace rtklib2::services {

ServiceRegistry::ServiceRegistry() = default;
ServiceRegistry::~ServiceRegistry() = default;

ServiceRegistry::ServiceRegistry(ServiceRegistry&&) noexcept = default;
ServiceRegistry& ServiceRegistry::operator=(ServiceRegistry&&) noexcept = default;

void ServiceRegistry::clear() { services_.clear(); }

}  // namespace rtklib2::services
