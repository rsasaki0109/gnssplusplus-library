#include "rtklib2/pipeline/pipeline.hpp"

#include <utility>

namespace rtklib2::pipeline {

Pipeline::Pipeline() = default;
Pipeline::~Pipeline() = default;

void Pipeline::add_stage(StagePtr stage) {
    std::lock_guard<std::mutex> lock(mutex_);
    stages_.push_back(std::move(stage));
}

void Pipeline::execute() {
    std::vector<StagePtr> snapshot;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        snapshot = stages_;
    }

    for (const auto& stage : snapshot) {
        if (stage) {
            stage->process();
        }
    }
}

std::size_t Pipeline::stage_count() const noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    return stages_.size();
}

}  // namespace rtklib2::pipeline
