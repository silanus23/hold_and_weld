#include "hold_and_weld_application/kinematics/approach_validator.hpp"

namespace hold_and_weld_application
{
namespace kinematics
{
  double ApproachValidator::computeManipulatibility(
    const std::vector<double>& joint_values)
  {
    Eigen::MatrixXd Jacobian = kinematics_solver_->computeJacobian(joint_values);

    Eigen::MatrixXd JJT = Jacobian * Jacobian.transpose();
    double manipulatibility = std::sqrt(JJT.determinant());

    return manipulatibility;
  }
}
}
