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

bool ApproachValidator::isApproachValid(const Vector6d & q_approach)
{
  if (!seam_ || seam_->poses.empty()) {
    return false;
  }

  Vector6d current_q = q_approach;

  for (const auto & target_pose_msg : seam_->poses) {
    // Convert geometry_msgs::Pose to Eigen::Isometry3d
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

    bool success = ceres_ik_solver_->solve(
      target_pose,
      current_q,
      next_q,
      1e-4,
      1e-3);

    if (!success) {
      return false;
    }

    double m_index = computeManipulatibility(next_q);
    if (m_index < manipulatibility_threshold_) {
      return false;
    }

    current_q = next_q;
  }

  return true;
}

double ApproachValidator::computeManipulatibility(const Vector6d & q)
{
  // Convert Vector6d to std::vector<double>
  std::vector<double> q_vec(6);
  for (size_t i = 0; i < 6; ++i) {
    q_vec[i] = q(i);
  }

  Eigen::MatrixXd J = kinematics_solver_->compute_jacobian(q_vec);
  return std::abs(J.determinant());
}

}  // namespace kinematics
}  // namespace hold_and_weld_application
