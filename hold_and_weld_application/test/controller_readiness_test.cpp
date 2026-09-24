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

#include "hold_and_weld_application/action_servers/controller_readiness.hpp"

using controller_manager_msgs::msg::ControllerState;
using hold_and_weld::controller_name_from_action_topic;
using hold_and_weld::is_controller_active;

namespace
{
ControllerState controller(const std::string & name, const std::string & state)
{
  ControllerState c;
  c.name = name;
  c.state = state;
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
