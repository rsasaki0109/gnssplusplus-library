#pragma once

#include <memory>
#include <string>
#include <typeindex>
#include <unordered_map>

namespace rtklib2::core {

class SystemContext {
public:
    SystemContext();
    ~SystemContext();

    SystemContext(SystemContext&&) noexcept;
    SystemContext& operator=(SystemContext&&) noexcept;

    SystemContext(const SystemContext&) = delete;
    SystemContext& operator=(const SystemContext&) = delete;

    [[nodiscard]] const std::string& profile() const noexcept;
    void set_profile(std::string profile);

    template <typename T>
    void register_component(std::shared_ptr<T> component) {
        components_[std::type_index(typeid(T))] = std::move(component);
    }

    template <typename T>
    [[nodiscard]] std::shared_ptr<T> get_component() const {
        auto it = components_.find(std::type_index(typeid(T)));
        if (it == components_.end()) {
            return nullptr;
        }
        return std::static_pointer_cast<T>(it->second);
    }

private:
    std::string profile_;
    std::unordered_map<std::type_index, std::shared_ptr<void>> components_;
};

}  // namespace rtklib2::core
