#pragma once

#include <memory>
#include <string>

namespace rtklib2::io {

class DataStream {
public:
    virtual ~DataStream();

    virtual void open(const std::string& source_uri) = 0;
    virtual void close() = 0;
    [[nodiscard]] virtual bool is_open() const noexcept = 0;
};

using DataStreamPtr = std::shared_ptr<DataStream>;

}  // namespace rtklib2::io
