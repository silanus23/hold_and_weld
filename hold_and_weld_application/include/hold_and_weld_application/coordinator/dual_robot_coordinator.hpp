// Copyright 2025 Berkan Tali
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

#ifndef HOLD_AND_WELD_APPLICATION__COORDINATOR__DUAL_ROBOT_COORDINATOR_HPP_
#define HOLD_AND_WELD_APPLICATION__COORDINATOR__DUAL_ROBOT_COORDINATOR_HPP_

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "hold_and_weld_application/action/trigger_gripper.hpp"
#include "hold_and_weld_application/action/trigger_welder.hpp"
#include <moveit/move_group_interface/move_group_interface.hpp>

namespace hold_and_weld
{

/**
 * @class DualRobotCoordinator
 * @brief Coordinates synchronized operations between gripper and welder robots.
 *
 * This class manages the orchestration of gripper picking operations and welder
 * execution, ensuring proper sequencing and safety of dual-robot manipulation
 * and welding tasks.
 */
class DualRobotCoordinator : public rclcpp::Node {
public:
  using TriggerGripper = hold_and_weld_application::action::TriggerGripper;
  using TriggerWelder = hold_and_weld_application::action::TriggerWelder;
  using GoalHandleTriggerGripper = rclcpp_action::ClientGoalHandle<TriggerGripper>;
  using GoalHandleTriggerWelder = rclcpp_action::ClientGoalHandle<TriggerWelder>;

  /**
   * @brief Construct a new DualRobotCoordinator object.
   * @param options ROS2 node options for configuration.
   */
  explicit DualRobotCoordinator(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /**
   * @brief Main execution loop for coordinating robot operations.
   */
  void run();

private:
  /**
   * @brief Execute a gripper job by sending goal to gripper action server.
   * @return true if gripper job completed successfully, false otherwise.
   */
  bool execute_gripper_job();

  /**
   * @brief Execute a welder job by sending goal to welder action server.
   * @return true if welder job completed successfully, false otherwise.
   */
  bool execute_welder_job();

  /**
   * @brief Move the welder arm to a safe position.
   * @return true if welder reached safety position, false otherwise.
   */
  bool move_welder_to_safety();

  /**
   * @brief Spin the ROS executor for a specified duration.
   * @param seconds Duration to spin in seconds.
   */
  void spin_for_duration(double seconds);

  /**
   * @brief Initialize MoveIt interface for the welder arm.
   */
  void init_moveit();

  // Temporary
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> welder_move_group_;

  // Action clients
  rclcpp_action::Client<TriggerGripper>::SharedPtr gripper_client_;
  rclcpp_action::Client<TriggerWelder>::SharedPtr welder_client_;

  rclcpp::TimerBase::SharedPtr init_timer_;
  std::atomic<bool> moveit_ready_{false};
};

}  // namespace hold_and_weld

#endif  // HOLD_AND_WELD_APPLICATION__COORDINATOR__DUAL_ROBOT_COORDINATOR_HPP_
