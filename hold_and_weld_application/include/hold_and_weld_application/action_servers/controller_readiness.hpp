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

#ifndef HOLD_AND_WELD_APPLICATION__ACTION_SERVERS__CONTROLLER_READINESS_HPP_
#define HOLD_AND_WELD_APPLICATION__ACTION_SERVERS__CONTROLLER_READINESS_HPP_

#include <string>
#include <vector>

#include <controller_manager_msgs/msg/controller_state.hpp>

namespace hold_and_weld
{

/**
 * @brief Controller name owning an action topic: the segment before the action name,
 *        so namespaces are skipped, e.g.
 *        "/cell/robot1_gripper_controller/follow_joint_trajectory" -> "robot1_gripper_controller".
 *        Empty if the topic has no controller segment.
 */
std::string controller_name_from_action_topic(const std::string & action_topic);

/**
 * @brief Whether controller @p name is listed and in the "active" state.
 *
 * A configured but inactive JointTrajectoryController already serves its action
 * and accepts goals, then reports them succeeded without moving, so goals must
 * wait for "active".
 */
bool is_controller_active(
  const std::vector<controller_manager_msgs::msg::ControllerState> & controllers,
  const std::string & name);

}  // namespace hold_and_weld

#endif  // HOLD_AND_WELD_APPLICATION__ACTION_SERVERS__CONTROLLER_READINESS_HPP_
