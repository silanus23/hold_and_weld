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

#include <memory>
#include <string>

#include <gp_Vec.hxx>

#include "hold_and_weld_gripper_sampler/collision/jaw_clearance_check.hpp"

namespace hold_and_weld_gripper_sampler
{
namespace geometry
{

JawClearanceCheck::JawClearanceCheck(
  const JawClearanceConfig & config,
  const ParsedGripper & gripper,
  double finger_length)
: config_(config),
  tcp_offset_(gripper.tcp_offset),
  finger_length_(finger_length)
{
  if (config_.enabled && finger_length_ <= 0.0) {
    RCLCPP_WARN(logger_,
      "JawClearanceCheck: finger_length is %.4f — jaw-clearance check disabled",
      finger_length_);
    config_.enabled = false;
  }
}

void JawClearanceCheck::set_fcl_checker(
  std::shared_ptr<const FCLCollisionChecker> fcl_checker)
{
  fcl_checker_ = fcl_checker;
}

bool JawClearanceCheck::intrudes(
  const gp_Trsf & gripper_transform,
  double grip_distance) const
{
  if (!config_.enabled) {
    return false;
  }

  if (!fcl_checker_ || !fcl_checker_->is_valid()) {
    RCLCPP_WARN(logger_, "FCL checker not available - rejecting grasp conservatively");
    return true;
  }

  const double radius = grip_distance / 2.0 + config_.clearance_margin;

  // The cylinder ends at the TCP and runs back along the jaw axis (local -Z) by
  // one finger length, so its centre sits half a finger length behind the TCP.
  gp_Trsf cylinder_offset;
  cylinder_offset.SetTranslation(
    gp_Vec(
      tcp_offset_.x(),
      tcp_offset_.y(),
      tcp_offset_.z() - finger_length_ / 2.0));

  const gp_Trsf cylinder_pose = gripper_transform * cylinder_offset;

  return fcl_checker_->cylinder_collides_with_obstacles(cylinder_pose, radius, finger_length_);
}

std::string JawClearanceCheck::get_name() const
{
  return "JawClearanceCheck";
}

}  // namespace geometry
}  // namespace hold_and_weld_gripper_sampler
