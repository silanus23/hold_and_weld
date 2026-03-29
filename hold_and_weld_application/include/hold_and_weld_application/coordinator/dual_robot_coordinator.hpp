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
#include <atomic>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <lifecycle_msgs/msg/transition.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include "hold_and_weld_application/action/trigger_gripper.hpp"
#include "hold_and_weld_application/action/trigger_welder.hpp"
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit_msgs/srv/get_cartesian_path.hpp>

namespace hold_and_weld
{

/**
 * @class DualRobotCoordinator
 * @brief Event-driven lifecycle coordinator for synchronized dual-robot operations.
 *
 * This coordinator manages the orchestration of gripper picking and welder operations
 * in a fully event-driven manner, eliminating arbitrary timeouts and blocking calls.
 *
 * Features:
 * - Event-driven readiness monitoring (checks controller availability)
 * - Automatic execution when ready (configurable via auto_start parameter)
 * - Manual trigger service for on-demand execution
 * - Async action execution with proper sequencing
 * - No blocking threads, timers, or arbitrary delays
 */
class DualRobotCoordinator : public rclcpp_lifecycle::LifecycleNode {
public:
  using TriggerGripper = hold_and_weld_application::action::TriggerGripper;
  using TriggerWelder = hold_and_weld_application::action::TriggerWelder;
  using GoalHandleTriggerGripper = rclcpp_action::ClientGoalHandle<TriggerGripper>;
  using GoalHandleTriggerWelder = rclcpp_action::ClientGoalHandle<TriggerWelder>;
  using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;

  /**
   * @brief Construct a new DualRobotCoordinator object.
   * @param options ROS2 node options for configuration.
   */
  explicit DualRobotCoordinator(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  // Lifecycle callbacks
  /**
   * @brief Configure lifecycle transition callback.
   * Initializes action clients, services, and MoveIt interface.
   * @param state Current lifecycle state.
   * @return Transition callback result.
   */
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State & state);

  /**
   * @brief Activate lifecycle transition callback.
   * Starts readiness monitoring timer.
   * @param state Current lifecycle state.
   * @return Transition callback result.
   */
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State & state);

  /**
   * @brief Deactivate lifecycle transition callback.
   * Stops execution and readiness monitoring.
   * @param state Current lifecycle state.
   * @return Transition callback result.
   */
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State & state);

  /**
   * @brief Cleanup lifecycle transition callback.
   * Releases all resources.
   * @param state Current lifecycle state.
   * @return Transition callback result.
   */
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State & state);

  /**
   * @brief Shutdown lifecycle transition callback.
   * @param state Current lifecycle state.
   * @return Transition callback result.
   */
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_shutdown(const rclcpp_lifecycle::State & state);

private:
  /**
   * @brief Periodically checks if all dependencies are ready.
   * When ready and auto_start is enabled, automatically triggers execution.
   */
  void check_readiness();

  /**
   * @brief Check if all required controller action servers are available.
   * @return true if all controllers are ready, false otherwise.
   */
  bool check_controllers_ready();

  /**
   * @brief Service callback to manually trigger the coordinated sequence.
   * @param request Service request (empty).
   * @param response Service response with success status and message.
   */
  void handle_trigger_service(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  /**
   * @brief Execute the complete dual-robot coordinated sequence.
   * Steps: 1) Move welder to safety, 2) Execute gripper job, 3) Execute welder job.
   */
  void execute_sequence();

  /**
   * @brief Move the welder arm to a safe position (async).
   * Calls step_gripper_job() upon completion.
   */
  void step_welder_safety();

  /**
   * @brief Execute gripper job (async).
   * Calls step_welder_job() upon completion.
   */
  void step_gripper_job();

  /**
   * @brief Execute welder job (async).
   * Final step in the sequence.
   */
  void step_welder_job();

  /**
   * @brief Callback for gripper action feedback.
   * @param feedback Feedback message from gripper action.
   */
  void gripper_feedback_callback(
    GoalHandleTriggerGripper::SharedPtr,
    const std::shared_ptr<const TriggerGripper::Feedback> feedback);

  /**
   * @brief Callback for gripper action result.
   * @param result Wrapped result from gripper action.
   */
  void gripper_result_callback(const GoalHandleTriggerGripper::WrappedResult & result);

  /**
   * @brief Callback for welder action feedback.
   * @param feedback Feedback message from welder action.
   */
  void welder_feedback_callback(
    GoalHandleTriggerWelder::SharedPtr,
    const std::shared_ptr<const TriggerWelder::Feedback> feedback);

  /**
   * @brief Callback for welder action result.
   * @param result Wrapped result from welder action.
   */
  void welder_result_callback(const GoalHandleTriggerWelder::WrappedResult & result);

  // Parameters
  bool auto_start_{true};  ///< Auto-start sequence when ready

  // State tracking
  std::atomic<bool> is_active_{false};
  std::atomic<bool> system_ready_{false};
  std::atomic<bool> sequence_started_{false};
  std::atomic<bool> sequence_running_{false};

  // Readiness monitoring
  rclcpp::TimerBase::SharedPtr readiness_timer_;

  // Controller action clients (for readiness checking)
  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr robot1_controller_client_;
  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr robot2_controller_client_;
  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr gripper_controller_client_;

  // Application action clients
  rclcpp_action::Client<TriggerGripper>::SharedPtr gripper_client_;
  rclcpp_action::Client<TriggerWelder>::SharedPtr welder_client_;

  // Manual trigger service
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr trigger_service_;

  // Mutexes
  std::mutex state_mutex_;
  std::mutex move_group_mutex_;

  // MoveIt interface for safety positioning
  std::shared_ptr<rclcpp::Node> moveit_node_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> welder_move_group_;

  // Logger
  rclcpp::Logger logger_;
};

}  // namespace hold_and_weld

#endif  // HOLD_AND_WELD_APPLICATION__COORDINATOR__DUAL_ROBOT_COORDINATOR_HPP_
