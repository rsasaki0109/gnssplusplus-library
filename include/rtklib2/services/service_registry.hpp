#pragma once

#include <functional>
#include <memory>
#include <string>
#include <typeindex>
#include <unordered_map>

namespace rtklib2::services {

class ServiceRegistry {
public:
    ServiceRegistry();
    ~ServiceRegistry();

    ServiceRegistry(ServiceRegistry&&) noexcept;
    ServiceRegistry& operator=(ServiceRegistry&&) noexcept;

    ServiceRegistry(const ServiceRegistry&) = delete;
    ServiceRegistry& operator=(const ServiceRegistry&) = delete;

    template <typename Service>
    void register_service(std::shared_ptr<Service> service) {
        services_[std::type_index(typeid(Service))] = std::move(service);
    }

    template <typename Service>
    [[nodiscard]] std::shared_ptr<Service> get_service() const {
        auto it = services_.find(std::type_index(typeid(Service)));
        if (it == services_.end()) {
            return nullptr;
        }
        return std::static_pointer_cast<Service>(it->second);
    }

    void clear();

private:
    std::unordered_map<std::type_index, std::shared_ptr<void>> services_;
};

}  // namespace rtklib2::services
