#pragma once

#include <cstddef>
#include <functional>
#include <memory>
#include <mutex>
#include <vector>

namespace rtklib2::pipeline {

/**
 * @brief Represents a processing stage in the observation pipeline.
 */
class Stage {
public:
    virtual ~Stage() = default;
    virtual void process() = 0;
};

using StagePtr = std::shared_ptr<Stage>;

/**
 * @brief Orchestrates execution of registered pipeline stages.
 */
class Pipeline {
public:
    Pipeline();
    ~Pipeline();

    void add_stage(StagePtr stage);
    void execute();

    [[nodiscard]] std::size_t stage_count() const noexcept;

private:
    std::vector<StagePtr> stages_;
    mutable std::mutex mutex_;
};

}  // namespace rtklib2::pipeline
