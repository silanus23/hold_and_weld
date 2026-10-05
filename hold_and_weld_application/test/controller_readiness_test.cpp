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

#include <gtest/gtest.h>

#include <string>
#include <vector>

#include <controller_manager_msgs/msg/controller_state.hpp>

#include "hold_and_weld_application/utils.hpp"

using controller_manager_msgs::msg::ControllerState;
using hold_and_weld::controller_name_from_action_topic;
using hold_and_weld::is_controller_active;
using hold_and_weld::joints_without_active_controller;

namespace
{
ControllerState controller(
  const std::string & name, const std::string & state,
  const std::vector<std::string> & claimed_interfaces = {})
{
  ControllerState c;
  c.name = name;
  c.state = state;
  c.claimed_interfaces = claimed_interfaces;
  return c;
}
}  // namespace

TEST(ControllerReadiness, NameIsTheSegmentBeforeTheActionName)
{
  EXPECT_EQ(controller_name_from_action_topic(
      "/robot1_gripper_controller/follow_joint_trajectory"), "robot1_gripper_controller");
  EXPECT_EQ(controller_name_from_action_topic(
      "robot1_gripper_controller/follow_joint_trajectory"), "robot1_gripper_controller");
  EXPECT_EQ(controller_name_from_action_topic(
      "/cell/robot1_gripper_controller/follow_joint_trajectory"), "robot1_gripper_controller");
  EXPECT_EQ(controller_name_from_action_topic("follow_joint_trajectory"), "");
}

// The spawner loads and configures before activating; a configured (inactive)
// controller already serves its action, but accepts goals it never executes.
TEST(ControllerReadiness, OnlyAnActiveControllerIsReady)
{
  const std::string name = "robot1_gripper_controller";
  EXPECT_FALSE(is_controller_active({}, name));
  EXPECT_FALSE(is_controller_active({controller(name, "inactive")}, name));
  EXPECT_FALSE(is_controller_active({controller("robot1_arm_controller", "active")}, name));
  EXPECT_TRUE(is_controller_active(
      {controller("robot1_arm_controller", "active"), controller(name, "active")}, name));
}

// The welder-only bringup activates robot2_arm_controller seconds after the server is
// up; until then the group's joints have no driver and every trajectory is refused.
TEST(ControllerReadiness, GroupJointsNeedAnActiveControllerClaimingThem)
{
  const std::vector<std::string> joints = {"robot2_joint_1", "robot2_joint_2"};
  const std::vector<std::string> claimed = {"robot2_joint_1/position", "robot2_joint_2/position"};

  EXPECT_EQ(joints_without_active_controller({}, joints), joints);
  EXPECT_EQ(joints_without_active_controller(
      {controller("robot2_arm_controller", "inactive", claimed)}, joints), joints);
  EXPECT_EQ(joints_without_active_controller(
      {controller("robot2_arm_controller", "active", {"robot2_joint_1/position"})}, joints),
    std::vector<std::string>{"robot2_joint_2"});
  EXPECT_TRUE(joints_without_active_controller(
      {controller("robot2_arm_controller", "active", claimed)}, joints).empty());
}
