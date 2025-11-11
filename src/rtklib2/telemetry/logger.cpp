#include "rtklib2/telemetry/logger.hpp"

#include <iostream>

namespace rtklib2::telemetry {

namespace {

class ConsoleLogger final : public Logger {
public:
    void info(const std::string& message) override { std::clog << "[INFO] " << message << '\n'; }
    void warn(const std::string& message) override { std::clog << "[WARN] " << message << '\n'; }
    void error(const std::string& message) override { std::clog << "[ERROR] " << message << '\n'; }
};

}  // namespace

LoggerPtr make_console_logger() { return std::make_shared<ConsoleLogger>(); }

}  // namespace rtklib2::telemetry
