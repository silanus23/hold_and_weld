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

#include <Eigen/Dense>
#include <cmath>

#include <rclcpp/rclcpp.hpp>

namespace hold_and_weld
{
namespace kinematics
{

IKCostFunctor::IKCostFunctor(
  const KinematicsSolver * solver,
  const Eigen::Isometry3d & target_pose,
  double rotation_weight,
  double seed_weight)
: solver_(solver),
  target_pose_(target_pose),
  rotation_weight_(rotation_weight),
  seed_weight_(seed_weight)
{}

void IKCostFunctor::set_target_seed(const std::vector<double> & seed)
{
  q_target_seed_ = seed;
}

// NOTE: This templated operator is intentionally only ever instantiated with T = double.
// It is used via ceres::NumericDiffCostFunction which calls it with plain doubles only.
// If you ever switch to ceres::AutoDiffCostFunction, this function must be moved to
// the header so Ceres can instantiate it with T = ceres::Jet for automatic differentiation.
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

  Eigen::Quaterniond q_target(target_pose_.rotation());
  Eigen::Quaterniond q_current(current_pose.rotation());

  Eigen::Quaterniond q_err = q_target * q_current.conjugate();

  // Quaternions double-cover 3D rotations (q and -q are the same rotation).
  // This ensures we always take the shortest angular path.
  if (q_err.w() < 0.0) {
    q_err.coeffs() *= -1.0;
  }

  // The vector part (x, y, z) is perfectly smooth and differentiable at zero.
  // For small angles, it approximates (axis * theta / 2).
  residuals[3] = T(rotation_weight_ * q_err.x());
  residuals[4] = T(rotation_weight_ * q_err.y());
  residuals[5] = T(rotation_weight_ * q_err.z());

  if (q_target_seed_.size() == 6) {
    for (int i = 0; i < 6; ++i) {
      residuals[6 + i] = T(seed_weight_ * (static_cast<double>(q_array[i]) - q_target_seed_[i]));
    }
  } else {
    for (int i = 0; i < 6; ++i) {
      residuals[6 + i] = T(0.0);
    }
  }

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
  const CeresIKSolver::Vector6d & q_seed,
  CeresIKSolver::Vector6d & q_solution,
  double position_tolerance,
  double orientation_tolerance)
{
  auto logger = rclcpp::get_logger("ceres_ik_solver");

  double q_params[6];
  for (size_t i = 0; i < 6; ++i) {
    q_params[i] = q_seed[i];
  }

  ceres::Problem problem;

  double seed_weight = 0.01;
  std::vector<double> seed_vec(6);
  for (size_t i = 0; i < 6; ++i) {
    seed_vec[i] = q_seed[i];
  }

  auto * functor = new IKCostFunctor(
    fk_solver_.get(), target_pose, rotation_weight_, seed_weight);
  functor->set_target_seed(seed_vec);

  ceres::CostFunction * cost_function =
    new ceres::NumericDiffCostFunction<IKCostFunctor, ceres::FORWARD, 12, 6>(functor);

  problem.AddResidualBlock(cost_function, nullptr, q_params);

  const auto & limits = fk_solver_->joint_limits();
  for (int i = 0; i < 6; ++i) {
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
    RCLCPP_WARN(logger, "IK failed to converge: %s", summary.message.c_str());
    return false;
  }

  std::vector<double> solution_vec(6);
  for (size_t i = 0; i < 6; ++i) {
    solution_vec[i] = q_solution[i];
  }

  Eigen::Isometry3d achieved_pose;
  try {
    achieved_pose = fk_solver_->compute_fk(solution_vec);
  } catch (const std::exception & e) {
    RCLCPP_WARN(logger, "IK post-solve FK check failed: %s", e.what());
    return false;
  }
  double position_error = (achieved_pose.translation() - target_pose.translation()).norm();

  Eigen::AngleAxisd rot_err_aa(target_pose.rotation() * achieved_pose.rotation().transpose());
  double orientation_error = std::abs(rot_err_aa.angle());

  if (position_error > position_tolerance || orientation_error > orientation_tolerance) {
    RCLCPP_WARN(
      logger, "IK Accuracy Failed: Pos Err: %.6f m, Rot Err: %.6f rad",
      position_error, orientation_error);
    return false;
  }

  RCLCPP_DEBUG(logger, "IK Solution Valid within tolerances.");
  return true;
}

}  // namespace kinematics
}  // namespace hold_and_weld
