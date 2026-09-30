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

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <controller_manager_msgs/srv/list_controllers.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <lifecycle_msgs/msg/transition.hpp>
#include <std_srvs/srv/trigger.hpp>
#include "hold_and_weld_application/action/trigger_gripper.hpp"
#include "hold_and_weld_application/action/trigger_welder.hpp"


namespace hold_and_weld
{
namespace application
{

/**
 * @class DualRobotCoordinator
 * @brief Lifecycle coordinator that runs the gripper job, then the welder job.
 *
 * - Readiness: a 500 ms timer asks controller_manager (asynchronously) whether both arm
 *   controllers and the gripper controller are active.
 * - Starts the sequence automatically once ready (auto_start), or on ~/trigger_sequence.
 * - Steps are chained through action result callbacks; nothing blocks while it runs.
 *   on_configure does block, up to 60 s per server, waiting for trigger_gripper and
 *   trigger_welder to appear.
 * - Every sequence gets a generation number, and callbacks from an older generation
 *   (e.g. a result that arrives after deactivate -> activate) are ignored.
 */
class DualRobotCoordinator : public rclcpp_lifecycle::LifecycleNode {
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

  // Lifecycle callbacks
  /**
   * @brief Configure lifecycle transition callback.
   * Initializes action clients and services.
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
   * @brief Handle the list_controllers answer sent by check_readiness().
   * @param future Response from controller_manager.
   */
  void handle_controller_list(
    rclcpp::Client<controller_manager_msgs::srv::ListControllers>::SharedFuture future);

  /**
   * @brief Cancel in-flight goals and invalidate their callbacks (deactivate, shutdown).
   */
  void stop_sequence();

  /**
   * @brief End the current sequence after a failed or rejected step.
   * @param generation Sequence the failing callback belongs to.
   */
  void abort_sequence(uint64_t generation);

  /**
   * @brief Whether a callback belongs to the current, still-active sequence.
   * @param generation Sequence the callback was created for.
   * @param what Callback description, logged when it is ignored.
   * @return false (with a log) for callbacks after deactivation or from an older sequence.
   */
  bool is_current(uint64_t generation, const char * what) const;

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
   * Steps: 1) Execute gripper job, 2) Execute welder job.
   */
  void execute_sequence();

  /**
   * @brief Execute gripper job (async).
   * Calls step_welder_job() upon completion.
   * @param generation Sequence this step belongs to.
   */
  void step_gripper_job(uint64_t generation);

  /**
   * @brief Execute welder job (async).
   * Final step in the sequence.
   * @param generation Sequence this step belongs to.
   */
  void step_welder_job(uint64_t generation);

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
   * @param generation Sequence the goal was sent for.
   */
  void gripper_result_callback(
    const GoalHandleTriggerGripper::WrappedResult & result, uint64_t generation);

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
   * @param generation Sequence the goal was sent for.
   */
  void welder_result_callback(
    const GoalHandleTriggerWelder::WrappedResult & result, uint64_t generation);

  bool auto_start_{true};

  std::atomic<bool> is_active_{false};
  std::atomic<bool> system_ready_{false};
  std::atomic<bool> sequence_started_{false};
  std::atomic<bool> sequence_running_{false};
  std::atomic<bool> readiness_request_in_flight_{false};
  std::atomic<uint64_t> sequence_generation_{0};

  rclcpp::TimerBase::SharedPtr readiness_timer_;
  rclcpp::Client<controller_manager_msgs::srv::ListControllers>::SharedPtr
    list_controllers_client_;
  std::vector<std::string> required_controllers_;

  rclcpp_action::Client<TriggerGripper>::SharedPtr gripper_client_;
  rclcpp_action::Client<TriggerWelder>::SharedPtr welder_client_;

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr trigger_service_;
  rclcpp::Logger logger_;

  std::mutex state_mutex_;
};

}  // namespace application
}  // namespace hold_and_weld

#endif  // HOLD_AND_WELD_APPLICATION__COORDINATOR__DUAL_ROBOT_COORDINATOR_HPP_
