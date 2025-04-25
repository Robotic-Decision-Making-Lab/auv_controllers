#include "pseudoinverse.hpp"

#include <cmath>

namespace ik_solvers::pinv
{

auto least_squares(const Eigen::MatrixXd & jac) -> Eigen::MatrixXd
{
  // we don't need to do much here - this is mostly a convenience function to make it easier to try out alternative
  // pseudoinverse methods in the solvers
  return jac.completeOrthogonalDecomposition().pseudoInverse();
}

auto damped_least_squares(const Eigen::MatrixXd & jac, const Eigen::VectorXd & damping) -> Eigen::MatrixXd
{
  if (damping.size() != jac.rows()) {
    throw std::invalid_argument("Damping vector must have the same size as the number of rows in the Jacobian.");
  }
  return jac.transpose() * ((jac * jac.transpose()) + damping.asDiagonal().toDenseMatrix()).inverse();
}

auto damped_least_squares(const Eigen::MatrixXd & jac, double damping) -> Eigen::MatrixXd
{
  return damped_least_squares(jac, damping * Eigen::MatrixXd::Identity(jac.rows(), jac.rows()));
}

}  // namespace ik_solvers::pinv
