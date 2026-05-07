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

#ifndef TEST_HELPERS_HPP_
#define TEST_HELPERS_HPP_

#include <Eigen/Dense>

#include <memory>

#include <BRepBuilderAPI_Transform.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <gp_Trsf.hxx>
#include <gp_Vec.hxx>
#include <TopoDS_Shape.hxx>

#include "hold_and_weld_gripper_sampler/collision/fcl_collision_checker.hpp"
#include "hold_and_weld_gripper_sampler/core/gripper.hpp"

using hold_and_weld_gripper_sampler::ParsedGripper;
using hold_and_weld_gripper_sampler::geometry::FCLCollisionChecker;

namespace
{

inline TopoDS_Shape create_box_helper(double width, double depth, double height)
{
  return BRepPrimAPI_MakeBox(width, depth, height).Shape();
}

inline TopoDS_Shape create_box_at_helper(
  double width, double depth, double height,
  double x, double y, double z)
{
  gp_Trsf transform;
  transform.SetTranslation(gp_Vec(x, y, z));
  TopoDS_Shape box = BRepPrimAPI_MakeBox(width, depth, height).Shape();
  return BRepBuilderAPI_Transform(box, transform, Standard_True).Shape();
}

}  // namespace

/**
 * @brief Build an FCLCollisionChecker from a gripper and primary shape.
 *
 * This is the common sub-operation shared by all wire_fcl() helpers in the
 * test fixtures: construct the checker so that FCL BVH meshes are built for
 * the primary shape and the gripper geometry.
 *
 * @param gripper      Gripper description (fingers, base).
 * @param primary_shape Primary workpiece shape to triangulate into BVH.
 * @return Shared pointer to a ready-to-use FCLCollisionChecker.
 */
inline std::shared_ptr<FCLCollisionChecker> make_fcl_checker(
  const ParsedGripper & gripper,
  const TopoDS_Shape & primary_shape)
{
  return std::make_shared<FCLCollisionChecker>(gripper, primary_shape);
}

inline ParsedGripper create_mock_gripper()
{
  ParsedGripper gripper;

  gripper.base = create_box_helper(0.08, 0.12, 0.04);

  // Position them at their closed state positions
  gripper.finger_1 = create_box_at_helper(0.03, 0.04, 0.20, -0.015, 0.01, -0.20);
  gripper.finger_2 = create_box_at_helper(0.03, 0.04, 0.20, -0.015, 0.07, -0.20);

  gripper.finger_1_axis = Eigen::Vector3d(0, -1, 0);  // Moves in -Y
  gripper.finger_2_axis = Eigen::Vector3d(0, 1, 0);   // Moves in +Y

  gripper.max_opening = 0.32;

  gripper.gripper_type = "parallel";
  gripper.tcp_offset = Eigen::Vector3d(0, 0.04, -0.10);
  gripper.tcp_rpy = Eigen::Vector3d(0, 0, 0);

  gripper.base_link_name = "gripper_base";
  gripper.finger_1_link_name = "finger_1";
  gripper.finger_2_link_name = "finger_2";
  gripper.finger_1_joint_name = "finger_1_joint";
  gripper.finger_2_joint_name = "finger_2_joint";

  return gripper;
}

inline ParsedGripper create_small_gripper()
{
  // Half-scale version of test_gripper.urdf convention:
  // - Fingers extend forward along +Z, centered at z=+0.05 from joint origin
  // - Left finger joint at y=+0.015, z=+0.01 -> moves in +Y to open
  // - Right finger joint at y=-0.015, z=+0.01 -> moves in -Y to open
  // - TCP at z=+0.11 (forward tip of fingers)
  // This matches the real gripper convention so collision tests are meaningful.
  ParsedGripper gripper;

  // Base: 40x60x20 mm centered at origin
  gripper.base = create_box_at_helper(0.04, 0.06, 0.02, -0.02, -0.03, 0.0);

  // Left finger: 15x20x100 mm, joint origin at (0, +0.015, +0.01),
  // collision geometry centered at z=+0.05 above joint -> world z = +0.06
  gripper.finger_1 = create_box_at_helper(0.015, 0.02, 0.10, -0.0075, 0.005, 0.01);

  // Right finger: same geometry, joint origin at (0, -0.015, +0.01)
  gripper.finger_2 = create_box_at_helper(0.015, 0.02, 0.10, -0.0075, -0.035, 0.01);

  gripper.finger_1_axis = Eigen::Vector3d(0, 1, 0);   // Opens in +Y
  gripper.finger_2_axis = Eigen::Vector3d(0, -1, 0);  // Opens in -Y

  gripper.max_opening = 0.18;

  gripper.gripper_type = "parallel";
  gripper.tcp_offset = Eigen::Vector3d(0, 0, 0.11);
  gripper.tcp_rpy = Eigen::Vector3d(0, 0, 0);

  gripper.base_link_name = "gripper_base";
  gripper.finger_1_link_name = "left_finger";
  gripper.finger_2_link_name = "right_finger";
  gripper.finger_1_joint_name = "left_finger_joint";
  gripper.finger_2_joint_name = "right_finger_joint";

  return gripper;
}

#endif  // TEST_HELPERS_HPP_
