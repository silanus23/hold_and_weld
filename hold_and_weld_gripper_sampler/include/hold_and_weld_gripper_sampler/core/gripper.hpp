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
 * and collision checking. Lives in the root namespace because it is a
 * runtime kinematics struct used across the entire pipeline — it is not
 * a transient IO artifact.
 */
struct ParsedGripper
{
  TopoDS_Shape finger_1;
  TopoDS_Shape finger_2;
  TopoDS_Shape base;

  Eigen::Vector3d finger_1_axis;
  Eigen::Vector3d finger_2_axis;

  double max_opening;

  std::string gripper_type;
  Eigen::Vector3d tcp_offset;
  Eigen::Vector3d tcp_rpy;

  std::string base_link_name;
  std::string finger_1_link_name;
  std::string finger_2_link_name;
  std::string finger_1_joint_name;
  std::string finger_2_joint_name;
};

/**
 * @brief Configure gripper to a specified grip distance
 *
 * Translates each finger along its opening axis by the amount needed to
 * achieve the requested grip distance. Clamps to [0, max_opening].
 * Both fingers move symmetrically from their closed (reference) positions,
 * which are defined by the joint origin transforms baked into finger_1/finger_2.
 *
 * @param gripper Parsed gripper data
 * @param grip_distance Target distance between finger contact points (meters)
 * @return Compound shape: finger_1 + finger_2 + base at configured state
 */
TopoDS_Shape configure_gripper(const ParsedGripper & gripper, double grip_distance);

}  // namespace hold_and_weld_gripper_sampler

#endif  // HOLD_AND_WELD_GRIPPER_SAMPLER__CORE__GRIPPER_HPP_
