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

#include "hold_and_weld_application/kinematics/kinematics_solver.hpp"

#include <stdexcept>
#include <rclcpp/rclcpp.hpp>

namespace hold_and_weld
{
namespace kinematics
{

KinematicsSolver::KinematicsSolver(const ParsedChain & chain)
: dof_(chain.dof()),
  tool_transform_(chain.tool_transform),
  base_link_(chain.base_link),
  tip_link_(chain.tip_link)
{
  auto logger = rclcpp::get_logger("kinematics");

  if (dof_ != 6) {
    RCLCPP_ERROR(
      logger, "KinematicsSolver expects 6-DOF robot, got %zu DOF",
      dof_);
    throw std::runtime_error(
            "KinematicsSolver expects 6-DOF robot, got " +
            std::to_string(dof_) + " DOF");
  }

  joint_local_transforms_.reserve(dof_);
  joint_axes_.reserve(dof_);
  is_revolute_.reserve(dof_);
  joint_limits_.reserve(dof_);

  for (const auto & joint : chain.actuated_joints) {
    joint_local_transforms_.push_back(joint.origin_transform);
    joint_axes_.push_back(joint.axis);
    is_revolute_.push_back(joint.is_revolute);
    joint_limits_.emplace_back(joint.q_min, joint.q_max);
  }

  RCLCPP_INFO(
    logger, "KinematicsSolver initialized for %s -> %s (%zu DOF)",
    base_link_.c_str(), tip_link_.c_str(), dof_);
}

void KinematicsSolver::validate_joint_vector(const std::vector<double> & q) const
{
  if (q.size() != dof_) {
    throw std::invalid_argument(
            "Joint vector size mismatch: expected " + std::to_string(dof_) +
            ", got " + std::to_string(q.size()));
  }
}

Eigen::Isometry3d KinematicsSolver::compute_fk(const std::vector<double> & q) const
{
  validate_joint_vector(q);

  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();

  for (size_t i = 0; i < dof_; ++i) {
    transform = transform * joint_local_transforms_[i];

    if (is_revolute_[i]) {
      Eigen::AngleAxisd rotation(q[i], joint_axes_[i]);
      transform.rotate(rotation);
    } else {
      transform.translate(q[i] * joint_axes_[i]);
    }
  }

  transform = transform * tool_transform_;

  return transform;
}

Eigen::MatrixXd KinematicsSolver::compute_jacobian(const std::vector<double> & q) const
{
  validate_joint_vector(q);

  Eigen::MatrixXd J(6, dof_);

  std::vector<Eigen::Isometry3d> joint_transforms(dof_ + 1);
  joint_transforms[0] = Eigen::Isometry3d::Identity();

  for (size_t i = 0; i < dof_; ++i) {
    joint_transforms[i + 1] = joint_transforms[i] * joint_local_transforms_[i];

    if (is_revolute_[i]) {
      Eigen::AngleAxisd rotation(q[i], joint_axes_[i]);
      joint_transforms[i + 1].rotate(rotation);
    } else {
      joint_transforms[i + 1].translate(q[i] * joint_axes_[i]);
    }
  }

  Eigen::Vector3d tcp_position = (joint_transforms[dof_] * tool_transform_).translation();

  for (size_t i = 0; i < dof_; ++i) {
    Eigen::Matrix3d rotation_to_joint =
      joint_transforms[i].rotation() * joint_local_transforms_[i].rotation();
    Eigen::Vector3d joint_axis = rotation_to_joint * joint_axes_[i];

    Eigen::Vector3d joint_origin = (joint_transforms[i] * joint_local_transforms_[i]).translation();

    if (is_revolute_[i]) {
      J.block<3, 1>(0, i) = joint_axis.cross(tcp_position - joint_origin);
      J.block<3, 1>(3, i) = joint_axis;
    } else {
      J.block<3, 1>(0, i) = joint_axis;
      J.block<3, 1>(3, i) = Eigen::Vector3d::Zero();
    }
  }

  return J;
}

double KinematicsSolver::compute_yoshikawa_index(const std::vector<double> & q) const
{
  Eigen::MatrixXd J = compute_jacobian(q);

  Eigen::MatrixXd JJT = J * J.transpose();
  double det = JJT.determinant();

  if (det < 0.0) {
    return 0.0;
  }

  return std::sqrt(det);
}

double KinematicsSolver::compute_condition_number(const std::vector<double> & q) const
{
  Eigen::MatrixXd J = compute_jacobian(q);

  Eigen::JacobiSVD<Eigen::MatrixXd> svd(J);
  Eigen::VectorXd singular_values = svd.singularValues();

  double sigma_max = singular_values(0);
  double sigma_min = singular_values(singular_values.size() - 1);

  if (sigma_min < 1e-10) {
    return std::numeric_limits<double>::infinity();
  }

  return sigma_max / sigma_min;
}

bool KinematicsSolver::is_near_singularity(
  const std::vector<double> & q,
  double threshold) const
{
  double manipulability = compute_yoshikawa_index(q);
  return manipulability < threshold;
}

bool KinematicsSolver::check_joint_limits(
  const std::vector<double> & q,
  double margin) const
{
  validate_joint_vector(q);

  if (margin < 0.0) {
    throw std::invalid_argument(
      "check_joint_limits: margin must be non-negative, got " + std::to_string(margin));
  }

  for (size_t i = 0; i < dof_; ++i) {
    double q_min_safe = joint_limits_[i].first + margin;
    double q_max_safe = joint_limits_[i].second - margin;

    if (q[i] < q_min_safe || q[i] > q_max_safe) {
      return false;
    }
  }

  return true;
}

}  // namespace kinematics
}  // namespace hold_and_weld
