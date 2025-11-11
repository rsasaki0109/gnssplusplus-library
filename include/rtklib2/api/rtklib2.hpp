#pragma once

#include <memory>
#include <string>

#include "rtklib2/common/version.hpp"
#include "rtklib2/core/system_context.hpp"
#include "rtklib2/pipeline/pipeline.hpp"
#include "rtklib2/services/service_registry.hpp"
#include "rtklib2/telemetry/logger.hpp"

namespace rtklib2::api {

/**
 * @brief Entry point for initializing and configuring RTKLIB v2 subsystems.
 */
class Library final {
public:
    Library();
    ~Library();

    Library(const Library&) = delete;
    Library& operator=(const Library&) = delete;
    Library(Library&&) noexcept = default;
    Library& operator=(Library&&) noexcept = default;

    /**
     * @brief Load the library with optional configuration profile identifier.
     */
    void initialize(const std::string& profile = "default");

    /**
     * @brief Retrieve the underlying system context.
     */
    [[nodiscard]] core::SystemContext& context() noexcept;

    /**
     * @brief Access the service registry.
     */
    [[nodiscard]] services::ServiceRegistry& services() noexcept;

    /**
     * @brief Access the global telemetry logger facade.
     */
    [[nodiscard]] telemetry::Logger& logger() noexcept;

    /**
     * @brief Access the observation pipeline orchestrator.
     */
    [[nodiscard]] pipeline::Pipeline& pipeline() noexcept;

private:
    std::shared_ptr<core::SystemContext> context_;
    std::shared_ptr<services::ServiceRegistry> services_;
    std::shared_ptr<telemetry::Logger> logger_;
    std::shared_ptr<pipeline::Pipeline> pipeline_;
};

}  // namespace rtklib2::api
