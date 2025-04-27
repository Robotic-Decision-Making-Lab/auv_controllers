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

auto damped_least_squares(const Eigen::MatrixXd & jac, double damping) -> Eigen::MatrixXd
{
  const Eigen::MatrixXd eye = Eigen::MatrixXd::Identity(jac.rows(), jac.rows());
  return jac.transpose() * ((jac * jac.transpose()) + (damping * eye)).inverse();
}

}  // namespace ik_solvers::pinv
