#include "fgo_cuda_backend.hpp"

#include <cuda_runtime.h>
#include <cusolverDn.h>

#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <cstring>
#include <string>

namespace libgnss::detail {
namespace {

enum class SolverMode { Off, Auto, On };

SolverMode configuredMode() {
    const char* raw = std::getenv("GNSSPP_FGO_CUDA_SOLVER");
    if (raw == nullptr) {
        return SolverMode::Off;
    }
    std::string value(raw);
    std::transform(value.begin(), value.end(), value.begin(),
                   [](unsigned char c) {
                       return static_cast<char>(std::tolower(c));
                   });
    if (value == "1" || value == "on" || value == "true") {
        return SolverMode::On;
    }
    if (value == "auto") {
        return SolverMode::Auto;
    }
    return SolverMode::Off;
}

class DenseSolverWorkspace {
public:
    ~DenseSolverWorkspace() {
        if (matrix_ != nullptr) cudaFree(matrix_);
        if (rhs_ != nullptr) cudaFree(rhs_);
        if (workspace_ != nullptr) cudaFree(workspace_);
        if (info_ != nullptr) cudaFree(info_);
        if (handle_ != nullptr) cusolverDnDestroy(handle_);
    }

    bool solve(const double* normal_matrix,
               int state_size,
               const double* rhs,
               int rhs_columns,
               double* solution) {
        if (normal_matrix == nullptr || rhs == nullptr || solution == nullptr ||
            state_size <= 0 || rhs_columns <= 0 ||
            !ensureCapacity(state_size, rhs_columns)) {
            return false;
        }
        const std::size_t matrix_bytes =
            static_cast<std::size_t>(state_size) * state_size * sizeof(double);
        const std::size_t rhs_bytes =
            static_cast<std::size_t>(state_size) * rhs_columns * sizeof(double);
        if (cudaMemcpy(matrix_, normal_matrix, matrix_bytes,
                       cudaMemcpyHostToDevice) != cudaSuccess ||
            cudaMemcpy(rhs_, rhs, rhs_bytes,
                       cudaMemcpyHostToDevice) != cudaSuccess) {
            return false;
        }
        if (cusolverDnDpotrf(handle_, CUBLAS_FILL_MODE_LOWER,
                            state_size, matrix_, state_size,
                            workspace_, workspace_elements_, info_) !=
            CUSOLVER_STATUS_SUCCESS) {
            return false;
        }
        int info = -1;
        if (cudaMemcpy(&info, info_, sizeof(info),
                       cudaMemcpyDeviceToHost) != cudaSuccess ||
            info != 0) {
            return false;
        }
        if (cusolverDnDpotrs(handle_, CUBLAS_FILL_MODE_LOWER,
                            state_size, rhs_columns,
                            matrix_, state_size,
                            rhs_, state_size, info_) !=
            CUSOLVER_STATUS_SUCCESS) {
            return false;
        }
        if (cudaMemcpy(&info, info_, sizeof(info),
                       cudaMemcpyDeviceToHost) != cudaSuccess ||
            info != 0) {
            return false;
        }
        return cudaMemcpy(solution, rhs_, rhs_bytes,
                          cudaMemcpyDeviceToHost) == cudaSuccess;
    }

private:
    bool ensureCapacity(int state_size, int rhs_columns) {
        if (handle_ == nullptr &&
            cusolverDnCreate(&handle_) != CUSOLVER_STATUS_SUCCESS) {
            return false;
        }
        int required_workspace = 0;
        const bool resize_matrix = state_size > matrix_capacity_;
        const bool resize_rhs =
            state_size > rhs_state_capacity_ ||
            rhs_columns > rhs_column_capacity_;
        if (resize_matrix) {
            if (matrix_ != nullptr) cudaFree(matrix_);
            matrix_ = nullptr;
            if (cudaMalloc(&matrix_,
                           static_cast<std::size_t>(state_size) * state_size *
                               sizeof(double)) != cudaSuccess) {
                return false;
            }
            matrix_capacity_ = state_size;
        }
        if (resize_rhs) {
            if (rhs_ != nullptr) cudaFree(rhs_);
            rhs_ = nullptr;
            if (cudaMalloc(&rhs_,
                           static_cast<std::size_t>(state_size) * rhs_columns *
                               sizeof(double)) != cudaSuccess) {
                return false;
            }
            rhs_state_capacity_ = state_size;
            rhs_column_capacity_ = rhs_columns;
        }
        if (info_ == nullptr && cudaMalloc(&info_, sizeof(int)) != cudaSuccess) {
            return false;
        }
        if (cusolverDnDpotrf_bufferSize(
                handle_, CUBLAS_FILL_MODE_LOWER, state_size,
                matrix_, state_size, &required_workspace) !=
            CUSOLVER_STATUS_SUCCESS) {
            return false;
        }
        if (required_workspace > workspace_capacity_) {
            if (workspace_ != nullptr) cudaFree(workspace_);
            workspace_ = nullptr;
            if (cudaMalloc(&workspace_,
                           static_cast<std::size_t>(required_workspace) *
                               sizeof(double)) != cudaSuccess) {
                return false;
            }
            workspace_capacity_ = required_workspace;
        }
        workspace_elements_ = required_workspace;
        return matrix_ != nullptr && rhs_ != nullptr && info_ != nullptr &&
               workspace_ != nullptr;
    }

    cusolverDnHandle_t handle_ = nullptr;
    double* matrix_ = nullptr;
    double* rhs_ = nullptr;
    double* workspace_ = nullptr;
    int* info_ = nullptr;
    int matrix_capacity_ = 0;
    int rhs_state_capacity_ = 0;
    int rhs_column_capacity_ = 0;
    int workspace_capacity_ = 0;
    int workspace_elements_ = 0;
};

thread_local DenseSolverWorkspace workspace;

}  // namespace

bool fgoCudaDenseSolverEnabled(int state_size) {
    // Dense assembly plus PCIe copies lose badly to Eigen's sparse path on the
    // fixed-lag windows used by the real-time shadow. Keep automatic dispatch
    // for genuinely large batches; `on` remains available for parity tests and
    // explicit large-problem benchmarking.
    constexpr int kAutoMinimumState = 2048;
    const SolverMode mode = configuredMode();
    const bool requested = mode == SolverMode::On ||
                           (mode == SolverMode::Auto &&
                            state_size >= kAutoMinimumState);
    if (!requested) {
        return false;
    }
    static const bool device_available = [] {
        int device_count = 0;
        return cudaGetDeviceCount(&device_count) == cudaSuccess &&
               device_count > 0;
    }();
    return device_available;
}

bool fgoCudaDenseSolve(const double* normal_matrix,
                       int state_size,
                       const double* rhs,
                       int rhs_columns,
                       double* solution) {
    return workspace.solve(normal_matrix, state_size, rhs,
                           rhs_columns, solution);
}

}  // namespace libgnss::detail
