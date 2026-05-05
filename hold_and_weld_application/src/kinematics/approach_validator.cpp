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
#include "hold_and_weld_application/kinematics/ceres_ik_solver.hpp"
#include "hold_and_weld_application/kinematics/kinematics_solver.hpp"

namespace hold_and_weld
{
namespace kinematics
{

ApproachValidator::ApproachValidator(
  std::shared_ptr<KinematicsSolver> kin_solver,
  std::shared_ptr<CeresIKSolver> ik_solver,
  double manipulability_threshold)
: kinematics_solver_(std::move(kin_solver)),
  ceres_ik_solver_(std::move(ik_solver)),
  manipulability_threshold_(manipulability_threshold),
  seam_(std::nullopt)
{
}

bool ApproachValidator::is_approach_valid(const Vector6d & q_approach)
{
  auto logger = rclcpp::get_logger("kinematics");

  if (!seam_.has_value() || seam_->poses.empty()) {
    RCLCPP_ERROR(logger, "Validation failed: No seam data or empty poses");
    return false;
  }

  RCLCPP_INFO(logger, "Validating approach: %zu waypoints", seam_->poses.size());

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

  // Relaxed tolerances (20cm / 0.10 rad) for Phase 1: the approach configuration
  // is OMPL-generated and may be far from the first seam point in joint space.
  // Tight tolerances here cause spurious failures on valid approach poses.
  // Phase 2 uses tight tolerances (3cm / 0.02 rad) since it warm-starts from the previous solution.
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
  double m_index_first = compute_manipulability(q_first_point);
  RCLCPP_DEBUG(logger, "First point manipulability: %.6f", m_index_first);

  if (m_index_first < manipulability_threshold_) {
    RCLCPP_WARN(logger, "Phase 1 FAILED: Low manipulability at first point: %.6f < %.6f",
                m_index_first, manipulability_threshold_);
    return false;
  }


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

    double m_index = compute_manipulability(next_q);

    RCLCPP_DEBUG(logger, "Waypoint %zu/%zu: Manipulability = %.6f",
                 waypoint_idx + 1, seam_->poses.size(), m_index);

    if (m_index < manipulability_threshold_) {
      RCLCPP_WARN(logger, "Phase 2 FAILED: Low manipulability at waypoint %zu/%zu: %.6f < %.6f",
                  waypoint_idx + 1, seam_->poses.size(), m_index, manipulability_threshold_);
      return false;
    }

    current_q = next_q;
  }

  RCLCPP_INFO(logger, "Approach validation passed (%zu waypoints)", seam_->poses.size());
  return true;
}

double ApproachValidator::compute_manipulability(const Vector6d & q) const
{
  auto logger = rclcpp::get_logger("kinematics");

  std::vector<double> q_vec(6);
  for (size_t i = 0; i < 6; ++i) {
    q_vec[i] = q(i);
  }

  Eigen::MatrixXd J = kinematics_solver_->compute_jacobian(q_vec);

  Eigen::MatrixXd JJT = J * J.transpose();
  double det_JJT = JJT.determinant();

  if (det_JJT < 0.0) {
    RCLCPP_WARN(logger,
          "Negative determinant in JJ^T (%.9f) — numerical noise near singularity, returning 0.0",
          det_JJT);
    return 0.0;
  }

  double yoshikawa_index = std::sqrt(det_JJT);

  Eigen::JacobiSVD<Eigen::MatrixXd> svd(J);
  double cond_number = svd.singularValues()(0) /
    svd.singularValues()(svd.singularValues().size() - 1);

  RCLCPP_DEBUG(logger, "Manipulability: %.6f, condition: %.2f%s",
    yoshikawa_index, cond_number, cond_number > 100.0 ? " (near singularity)" : "");

  return yoshikawa_index;
}

}  // namespace kinematics
}  // namespace hold_and_weld
