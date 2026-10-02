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

#ifndef HOLD_AND_WELD_GRIPPER_SAMPLER__CORE__GRIPPER_HPP_
#define HOLD_AND_WELD_GRIPPER_SAMPLER__CORE__GRIPPER_HPP_

#include <Eigen/Dense>

#include <string>

#include <gp_Trsf.hxx>
#include <TopoDS_Shape.hxx>

namespace hold_and_weld_gripper_sampler
{

/**
 * @brief Parsed gripper kinematic information
 *
 * Contains all geometric and kinematic data needed for grasp planning
 * and collision checking.
 */
struct ParsedGripper
{
  /** Finger 1 collision geometry in the base link frame, at the closed pose. */
  TopoDS_Shape finger_1;
  /** Finger 2 collision geometry in the base link frame, at the closed pose. */
  TopoDS_Shape finger_2;
  /** Base collision geometry in the base link frame. */
  TopoDS_Shape base;

  /** Unit opening direction of finger 1 in the base link frame. */
  Eigen::Vector3d finger_1_axis;
  /** Unit opening direction of finger 2 in the base link frame; opposite finger_1_axis. */
  Eigen::Vector3d finger_2_axis;

  /** Full opening, twice the shared finger joint travel [m]. */
  double max_opening = 0.0;

  /** From <gripper_type>; "parallel" when absent. */
  std::string gripper_type;
  /** TCP position in the base link frame [m]. */
  Eigen::Vector3d tcp_offset;
  /** TCP orientation as URDF roll, pitch, yaw [rad]. TODO(silanus23): stored but not used yet. */
  Eigen::Vector3d tcp_rpy;

  std::string base_link_name;
  std::string finger_1_link_name;
  std::string finger_2_link_name;
  std::string finger_1_joint_name;
  std::string finger_2_joint_name;

  /**
   * @brief Configure the gripper to a specified grip distance
   *
   * Translates each finger along its opening axis by the amount needed to
   * achieve the requested grip distance. Clamps to [0, max_opening].
   *
   * @param grip_distance Target distance between finger contact points [m]
   * @return Compound shape: finger_1 + finger_2 + base at configured state
   */
  TopoDS_Shape configure(double grip_distance) const;
};

namespace core
{

/**
 * @deprecated Use ParsedGripper::configure() instead.
 */
[[deprecated("Use ParsedGripper::configure() instead")]]
inline TopoDS_Shape configure_gripper(
  const ParsedGripper & gripper, double grip_distance)
{
  return gripper.configure(grip_distance);
}

}  // namespace core
}  // namespace hold_and_weld_gripper_sampler

#endif  // HOLD_AND_WELD_GRIPPER_SAMPLER__CORE__GRIPPER_HPP_
