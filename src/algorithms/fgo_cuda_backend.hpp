#pragma once

namespace libgnss::detail {

// Optional cuSOLVER dense SPD solve used by the native FGO backend. Matrices
// and right-hand sides use Eigen's default column-major layout. The function
// returns false on every CUDA/configuration failure so the caller can execute
// the reference Eigen path without changing solver authority.
bool fgoCudaDenseSolverEnabled(int state_size);
bool fgoCudaDenseSolve(const double* normal_matrix,
                       int state_size,
                       const double* rhs,
                       int rhs_columns,
                       double* solution);

}  // namespace libgnss::detail
