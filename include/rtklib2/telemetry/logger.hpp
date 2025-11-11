#pragma once

#include <memory>
#include <string>

namespace rtklib2::telemetry {

class Logger {
public:
    virtual ~Logger() = default;

    virtual void info(const std::string& message) = 0;
    virtual void warn(const std::string& message) = 0;
    virtual void error(const std::string& message) = 0;
};

using LoggerPtr = std::shared_ptr<Logger>;

[[nodiscard]] LoggerPtr make_console_logger();

}  // namespace rtklib2::telemetry
