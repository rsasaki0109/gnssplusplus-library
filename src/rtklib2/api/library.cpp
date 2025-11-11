#include "rtklib2/api/rtklib2.hpp"

#include <utility>

#include "rtklib2/pipeline/pipeline.hpp"
#include "rtklib2/services/service_registry.hpp"
#include "rtklib2/telemetry/logger.hpp"

namespace rtklib2::api {

Library::Library() = default;
Library::~Library() = default;

void Library::initialize(const std::string& profile) {
    context_ = std::make_shared<core::SystemContext>();
    services_ = std::make_shared<services::ServiceRegistry>();
    logger_ = telemetry::make_console_logger();
    pipeline_ = std::make_shared<pipeline::Pipeline>();

    context_->set_profile(profile);
    context_->register_component(services_);
    context_->register_component(logger_);
    context_->register_component(pipeline_);
}

core::SystemContext& Library::context() noexcept { return *context_; }

services::ServiceRegistry& Library::services() noexcept { return *services_; }

telemetry::Logger& Library::logger() noexcept { return *logger_; }

pipeline::Pipeline& Library::pipeline() noexcept { return *pipeline_; }

}  // namespace rtklib2::api
