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
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include "hold_and_weld_application/kinematics/ceres_ik_solver.hpp"
#include "hold_and_weld_application/kinematics/kinematics_solver.hpp"

namespace hold_and_weld
{
namespace kinematics
{

namespace
{
std::vector<double> to_std_vector(const ApproachValidator::Vector6d & q)
{
  return std::vector<double>(q.data(), q.data() + q.size());
}

// Pose message -> isometry with a normalised quaternion. Returns nullopt for a
// non-finite pose or a quaternion too close to zero to define a rotation.
std::optional<Eigen::Isometry3d> to_isometry(const geometry_msgs::msg::Pose & pose)
{
  Eigen::Quaterniond q(
    pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
  const Eigen::Vector3d p(pose.position.x, pose.position.y, pose.position.z);
  if (!q.coeffs().allFinite() || !p.allFinite() || q.norm() < 1e-9) {
    return std::nullopt;
  }
  Eigen::Isometry3d out = Eigen::Isometry3d::Identity();
  out.translation() = p;
  out.linear() = q.normalized().toRotationMatrix();
  return out;
}
}  // namespace

ApproachValidator::ApproachValidator(
  std::shared_ptr<KinematicsSolver> kin_solver,
  std::shared_ptr<CeresIKSolver> ik_solver,
  const ApproachValidatorParams & params)
: kinematics_solver_(std::move(kin_solver)),
  ceres_ik_solver_(std::move(ik_solver)),
  params_(params),
  seam_(std::nullopt)
{
  if (!kinematics_solver_ || !ceres_ik_solver_) {
    throw std::invalid_argument("ApproachValidator: kin_solver and ik_solver must not be null");
  }
  auto require_positive = [](double value, const char * name) {
      if (!std::isfinite(value) || value <= 0.0) {
        throw std::invalid_argument(
          std::string("ApproachValidatorParams.") + name + " must be positive, got " +
          std::to_string(value));
      }
    };
  require_positive(params_.first_point_tol_pos, "first_point_tol_pos");
  require_positive(params_.first_point_tol_rot, "first_point_tol_rot");
  require_positive(params_.seam_tol_pos, "seam_tol_pos");
  require_positive(params_.seam_tol_rot, "seam_tol_rot");
  if (!std::isfinite(params_.manipulability_threshold) || params_.manipulability_threshold < 0.0) {
    throw std::invalid_argument(
      "ApproachValidatorParams.manipulability_threshold must be non-negative, got " +
      std::to_string(params_.manipulability_threshold));
  }
}

bool ApproachValidator::is_approach_valid(const Vector6d & q_approach)
{
  auto logger = rclcpp::get_logger("kinematics");

  if (!seam_.has_value() || seam_->poses.empty()) {
    RCLCPP_ERROR(logger, "Validation failed: No seam data or empty poses");
    return false;
  }

  RCLCPP_INFO(logger, "Validating approach: %zu waypoints", seam_->poses.size());

  const auto first_pose = to_isometry(seam_->poses[0]);
  if (!first_pose) {
    RCLCPP_ERROR(logger, "Validation failed: seam pose 1 is non-finite or has a zero quaternion");
    return false;
  }

  Vector6d q_first_point;

  // Phase 1: first seam point, seeded from the approach configuration. The
  // tolerances are how far the solved pose may be from the seam point, not how far
  // the seed may be in joint space (Ceres copes with a distant seed on its own).
  bool success = ceres_ik_solver_->solve(
    *first_pose,
    q_approach,
    q_first_point,
    params_.first_point_tol_pos,
    params_.first_point_tol_rot);

  if (!success) {
    RCLCPP_WARN(logger, "Phase 1 FAILED: Cannot reach first seam point from approach");
    return false;
  }
  double m_index_first = kinematics_solver_->compute_yoshikawa_index(to_std_vector(q_first_point));
  RCLCPP_DEBUG(logger, "First point manipulability: %.6f", m_index_first);

  if (m_index_first < params_.manipulability_threshold) {
    RCLCPP_WARN(logger, "Phase 1 FAILED: Low manipulability at first point: %.6f < %.6f",
                m_index_first, params_.manipulability_threshold);
    return false;
  }

  // Phase 2: walk the rest of the seam, each solution seeding the next.
  Vector6d current_q = q_first_point;

  for (size_t waypoint_idx = 1; waypoint_idx < seam_->poses.size(); ++waypoint_idx) {
    const auto target_pose = to_isometry(seam_->poses[waypoint_idx]);
    if (!target_pose) {
      RCLCPP_ERROR(
        logger, "Phase 2 FAILED: seam pose %zu/%zu is non-finite or has a zero quaternion",
        waypoint_idx + 1, seam_->poses.size());
      return false;
    }

    Vector6d next_q;

    success = ceres_ik_solver_->solve(
      *target_pose,
      current_q,
      next_q,
      params_.seam_tol_pos,
      params_.seam_tol_rot);

    if (!success) {
      RCLCPP_WARN(logger, "Phase 2 FAILED: IK failed at waypoint %zu/%zu",
                  waypoint_idx + 1, seam_->poses.size());
      return false;
    }

    double m_index = kinematics_solver_->compute_yoshikawa_index(to_std_vector(next_q));

    RCLCPP_DEBUG(logger, "Waypoint %zu/%zu: Manipulability = %.6f",
                 waypoint_idx + 1, seam_->poses.size(), m_index);

    if (m_index < params_.manipulability_threshold) {
      RCLCPP_WARN(logger, "Phase 2 FAILED: Low manipulability at waypoint %zu/%zu: %.6f < %.6f",
                  waypoint_idx + 1, seam_->poses.size(), m_index,
                  params_.manipulability_threshold);
      return false;
    }

    current_q = next_q;
  }

  RCLCPP_INFO(logger, "Approach validation passed (%zu waypoints)", seam_->poses.size());
  return true;
}

}  // namespace kinematics
}  // namespace hold_and_weld
