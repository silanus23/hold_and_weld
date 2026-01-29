// Copyright 2026 Berkan Tali
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "hold_and_weld_application/kinematics/ceres_ik_solver.hpp"

#include <rclcpp/rclcpp.hpp>

namespace hold_and_weld
{
namespace kinematics
{

IKCostFunctor::IKCostFunctor(
  const KinematicsSolver * solver,
  const Eigen::Isometry3d & target_pose,
  double rotation_weight)
: solver_(solver),
  target_pose_(target_pose),
  rotation_weight_(rotation_weight)
{}

template<typename T>
bool IKCostFunctor::operator()(const T * const q_array, T * residuals) const
{
  std::vector<double> q(6);
  for (int i = 0; i < 6; ++i) {
    q[i] = static_cast<double>(q_array[i]);
  }

  Eigen::Isometry3d current_pose = solver_->compute_fk(q);

  Eigen::Vector3d pos_error = current_pose.translation() - target_pose_.translation();
  residuals[0] = T(pos_error.x());
  residuals[1] = T(pos_error.y());
  residuals[2] = T(pos_error.z());

  // rotation_error = R_target * R_current^T (rotation from current to target)
  Eigen::Matrix3d rotation_error = target_pose_.rotation() * current_pose.rotation().transpose();
  Eigen::AngleAxisd aa(rotation_error);
  Eigen::Vector3d rot_error = aa.angle() * aa.axis();

  residuals[3] = T(rotation_weight_ * rot_error.x());
  residuals[4] = T(rotation_weight_ * rot_error.y());
  residuals[5] = T(rotation_weight_ * rot_error.z());

  return true;
}

CeresIKSolver::CeresIKSolver(
  std::shared_ptr<KinematicsSolver> fk_solver,
  double rotation_weight)
: fk_solver_(fk_solver),
  rotation_weight_(rotation_weight),
  max_iterations_(100)
{
}

bool CeresIKSolver::solve(
  const Eigen::Isometry3d & target_pose,
  const std::vector<double> & q_seed,
  std::vector<double> & q_solution,
  double position_tolerance,
  double orientation_tolerance)
{
  auto logger = rclcpp::get_logger("ceres_ik_solver");

  if (q_seed.size() != fk_solver_->dof()) {
    RCLCPP_ERROR(
      logger, "Seed size mismatch: expected %zu, got %zu",
      fk_solver_->dof(), q_seed.size());
    return false;
  }

  q_solution = q_seed;
  double q_params[6];
  for (size_t i = 0; i < 6; ++i) {
    q_params[i] = q_seed[i];
  }

  ceres::Problem problem;
  // Using numeric differentiation (CENTRAL) instead of analytic Jacobian
  ceres::CostFunction * cost_function =
    new ceres::NumericDiffCostFunction<IKCostFunctor, ceres::CENTRAL, 6, 6>(
    new IKCostFunctor(fk_solver_.get(), target_pose, rotation_weight_));

  problem.AddResidualBlock(cost_function, nullptr, q_params);

  const auto & limits = fk_solver_->joint_limits();
  for (size_t i = 0; i < 6; ++i) {
    problem.SetParameterLowerBound(q_params, i, limits[i].first);
    problem.SetParameterUpperBound(q_params, i, limits[i].second);
  }

  ceres::Solver::Options options;
  options.max_num_iterations = max_iterations_;
  options.linear_solver_type = ceres::DENSE_QR;
  options.minimizer_progress_to_stdout = false;
  options.function_tolerance = 1e-6;
  options.gradient_tolerance = 1e-10;
  options.parameter_tolerance = 1e-8;

  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  for (size_t i = 0; i < 6; ++i) {
    q_solution[i] = q_params[i];
  }

  if (!summary.IsSolutionUsable()) {
    RCLCPP_WARN(
      logger, "IK failed to converge: %s",
      summary.message.c_str());
    return false;
  }

  // Verify solution accuracy
  Eigen::Isometry3d achieved_pose = fk_solver_->compute_fk(q_solution);
  Eigen::Vector3d pos_error = achieved_pose.translation() - target_pose.translation();
  double position_error = pos_error.norm();

  Eigen::Matrix3d rotation_error = target_pose.rotation() * achieved_pose.rotation().transpose();
  Eigen::AngleAxisd aa(rotation_error);
  double orientation_error = std::abs(aa.angle());

  if (position_error > position_tolerance) {
    RCLCPP_WARN(
      logger, "IK position error too large: %.6f m (tolerance: %.6f m)",
      position_error, position_tolerance);
    return false;
  }

  if (orientation_error > orientation_tolerance) {
    RCLCPP_WARN(
      logger, "IK orientation error too large: %.6f rad (tolerance: %.6f rad)",
      orientation_error, orientation_tolerance);
    return false;
  }

  RCLCPP_DEBUG(
    logger, "IK converged in %zu iterations (pos_err: %.4f mm, rot_err: %.4f deg)",
    summary.iterations.size(), position_error * 1000.0, orientation_error * 180.0 / M_PI);

  return true;
}

}  // namespace kinematics
}  // namespace hold_and_weld
