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

#ifndef HOLD_AND_WELD_GRIPPER_SAMPLER__COLLISION__JAW_CLEARANCE_CHECK_HPP_
#define HOLD_AND_WELD_GRIPPER_SAMPLER__COLLISION__JAW_CLEARANCE_CHECK_HPP_

#include <Eigen/Dense>
#include <memory>
#include <string>

#include <gp_Trsf.hxx>
#include <rclcpp/rclcpp.hpp>

#include "hold_and_weld_gripper_sampler/collision/fcl_collision_checker.hpp"
#include "hold_and_weld_gripper_sampler/core/gripper.hpp"

namespace hold_and_weld_gripper_sampler
{
namespace geometry
{

/**
 * @brief Configuration for the jaw-clearance check
 */
struct JawClearanceConfig
{
  bool enabled = false;
  double clearance_margin = 0.005;
};

/**
 * @brief Rejects grasps with constraint geometry in or beside the jaw mouth
 *
 * The mouth is the one region the gripper solid does not occupy,
 * so the exact pose check can never see into it at any tolerance.
 * This check covers it with a cylinder: radius
 * grip_distance / 2 + clearance_margin, length = finger length, centred on the
 * jaw axis and ending at the TCP.
 * Only the constraint layer's geometry is tested — secondaries and exclusion
 * volumes.
 */
class JawClearanceCheck
{
public:
  /**
   * @brief Constructor
   *
   * @param config Enable flag and clearance margin
   * @param gripper Parsed gripper — supplies the TCP offset the cylinder ends at
   * @param finger_length Cylinder length along the jaw axis (meters)
   */
  JawClearanceCheck(
    const JawClearanceConfig & config,
    const ParsedGripper & gripper,
    double finger_length);

  /**
   * @brief Set FCL collision checker for fast collision queries
   *
   * @param fcl_checker Shared pointer to FCL collision checker
   */
  void set_fcl_checker(std::shared_ptr<const FCLCollisionChecker> fcl_checker);

  /**
   * @brief Check whether obstacle geometry sits in or beside the jaw mouth
   *
   * @param gripper_transform 6-DOF pose of the gripper at the grasp (local Z is the jaw axis)
   * @param grip_distance Distance between fingers for this candidate
   * @return true if the candidate must be eliminated
   */
  bool intrudes(const gp_Trsf & gripper_transform, double grip_distance) const;

  /**
   * @brief Cylinder length along the jaw axis
   */
  double get_length() const {return finger_length_;}

  /**
   * @brief Name for logging
   */
  std::string get_name() const;

private:
  JawClearanceConfig config_;
  Eigen::Vector3d tcp_offset_;
  double finger_length_;

  std::shared_ptr<const FCLCollisionChecker> fcl_checker_;
  rclcpp::Logger logger_ = rclcpp::get_logger("gripper_sampler");
};

}  // namespace geometry
}  // namespace hold_and_weld_gripper_sampler

#endif  // HOLD_AND_WELD_GRIPPER_SAMPLER__COLLISION__JAW_CLEARANCE_CHECK_HPP_
