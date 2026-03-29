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

#include "hold_and_weld_application/kinematics/approach_validator.hpp"

#include <Eigen/Dense>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include "hold_and_weld_application/action_servers/welder_action_server.hpp"
#include "hold_and_weld_application/kinematics/ceres_ik_solver.hpp"
#include "hold_and_weld_application/kinematics/kinematics_solver.hpp"

namespace hold_and_weld_application
{
namespace kinematics
{

ApproachValidator::ApproachValidator(
  std::shared_ptr<KinematicsSolver> kin_solver,
  std::shared_ptr<CeresIKSolver> ik_solver,
  double manipulatibility_threshold)
: kinematics_solver_(std::move(kin_solver)),
  ceres_ik_solver_(std::move(ik_solver)),
  manipulatibility_threshold_(manipulatibility_threshold),
  seam_(nullptr)
{
}

bool ApproachValidator::is_approach_valid(const Vector6d & q_approach)
{
  auto logger = rclcpp::get_logger("approach_validator");

  if (!seam_ || seam_->poses.empty()) {
    RCLCPP_ERROR(logger, "Validation failed: No seam data or empty poses");
    return false;
  }

  RCLCPP_INFO(logger, "Validating approach for seam with %zu waypoints", seam_->poses.size());
  RCLCPP_INFO(logger, "Manipulability threshold: %.6f", manipulatibility_threshold_);

  RCLCPP_INFO(logger, "Phase 1: Validating approach → first seam point");

  Eigen::Isometry3d first_pose = Eigen::Isometry3d::Identity();
  first_pose.translation() << seam_->poses[0].position.x,
    seam_->poses[0].position.y,
    seam_->poses[0].position.z;

  Eigen::Quaterniond q_first(
    seam_->poses[0].orientation.w,
    seam_->poses[0].orientation.x,
    seam_->poses[0].orientation.y,
    seam_->poses[0].orientation.z);
  first_pose.linear() = q_first.toRotationMatrix();

  Vector6d q_first_point;

  // Relaxed tolerances for large jump
  bool success = ceres_ik_solver_->solve(
    first_pose,
    q_approach,
    q_first_point,
    0.20,
    0.10);

  if (!success) {
    RCLCPP_WARN(logger, "Phase 1 FAILED: Cannot reach first seam point from approach");
    return false;
  }
  double m_index_first = compute_manipulatibility(q_first_point);
  RCLCPP_DEBUG(logger, "First point manipulability: %.6f", m_index_first);

  if (m_index_first < manipulatibility_threshold_) {
    RCLCPP_WARN(logger, "Phase 1 FAILED: Low manipulability at first point: %.6f < %.6f",
                m_index_first, manipulatibility_threshold_);
    return false;
  }

  RCLCPP_INFO(logger, "Phase 1 PASSED: Approach → first point validated");

  RCLCPP_INFO(logger, "Phase 2: Validating seam continuity (%zu waypoints)",
              seam_->poses.size());

  Vector6d current_q = q_first_point;
  size_t waypoint_idx = 1;

  for (; waypoint_idx < seam_->poses.size(); ++waypoint_idx) {
    const auto & target_pose_msg = seam_->poses[waypoint_idx];
    Eigen::Isometry3d target_pose = Eigen::Isometry3d::Identity();
    target_pose.translation() << target_pose_msg.position.x,
      target_pose_msg.position.y,
      target_pose_msg.position.z;

    Eigen::Quaterniond q(
      target_pose_msg.orientation.w,
      target_pose_msg.orientation.x,
      target_pose_msg.orientation.y,
      target_pose_msg.orientation.z);
    target_pose.linear() = q.toRotationMatrix();

    Vector6d next_q;

    success = ceres_ik_solver_->solve(
      target_pose,
      current_q,
      next_q,
      0.03,
      0.02);

    if (!success) {
      RCLCPP_WARN(logger, "Phase 2 FAILED: IK failed at waypoint %zu/%zu",
                  waypoint_idx + 1, seam_->poses.size());
      return false;
    }

    double m_index = compute_manipulatibility(next_q);

    RCLCPP_DEBUG(logger, "Waypoint %zu/%zu: Manipulability = %.6f",
                 waypoint_idx + 1, seam_->poses.size(), m_index);

    if (m_index < manipulatibility_threshold_) {
      RCLCPP_WARN(logger, "Phase 2 FAILED: Low manipulability at waypoint %zu/%zu: %.6f < %.6f",
                  waypoint_idx + 1, seam_->poses.size(), m_index, manipulatibility_threshold_);
      return false;
    }

    current_q = next_q;
  }

  RCLCPP_INFO(logger, "Phase 2 PASSED: All %zu waypoints validated", seam_->poses.size());
  RCLCPP_INFO(logger, "Approach validation SUCCESSFUL (2-phase validation complete)");
  return true;
}

double ApproachValidator::compute_manipulatibility(const Vector6d & q)
{
  auto logger = rclcpp::get_logger("approach_validator");

  // Convert Vector6d to std::vector<double>
  std::vector<double> q_vec(6);
  for (size_t i = 0; i < 6; ++i) {
    q_vec[i] = q(i);
  }

  Eigen::MatrixXd J = kinematics_solver_->compute_jacobian(q_vec);

  // Compute Yoshikawa manipulability measure: sqrt(det(J * J^T))
  // This is more numerically stable and has better physical interpretation
  Eigen::MatrixXd JJT = J * J.transpose();
  double det_JJT = JJT.determinant();

  if (det_JJT < 0.0) {
    RCLCPP_WARN(logger, "Negative determinant in JJ^T: %.9f - using abs value", det_JJT);
    det_JJT = std::abs(det_JJT);
  }

  double yoshikawa_index = std::sqrt(det_JJT);

  // Also compute condition number as secondary check
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(J);
  double cond_number = svd.singularValues()(0) /
    svd.singularValues()(svd.singularValues().size() - 1);

  RCLCPP_DEBUG(logger, "Manipulability metrics - Yoshikawa: %.6f, Condition: %.2f",
               yoshikawa_index, cond_number);
  if (cond_number > 100.0) {
    RCLCPP_DEBUG(logger, "High condition number (%.2f) - configuration near singularity",
          cond_number);
  }

  return yoshikawa_index;
}

}  // namespace kinematics
}  // namespace hold_and_weld_application
