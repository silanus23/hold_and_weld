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

#ifndef HOLD_AND_WELD_APPLICATION__UTILS_HPP_
#define HOLD_AND_WELD_APPLICATION__UTILS_HPP_

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <controller_manager_msgs/msg/controller_state.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

namespace hold_and_weld
{

/**
 * @struct WeldSeam
 * @brief One seam of a weld path JSON, as parsed by parse_weld_seams().
 *
 * Every position is in the world (planning) frame, in metres.
 */
struct WeldSeam
{
  std::string seam_id;
  double length_m = 0.0;
  std::array<double, 3> start = {0.0, 0.0, 0.0};
  std::array<double, 3> end = {0.0, 0.0, 0.0};
  std::vector<geometry_msgs::msg::Pose> poses;
  size_t num_poses = 0;
  std::string segment_type;
  std::array<double, 3> center = {0.0, 0.0, 0.0};
  double radius = 0.0;
  bool has_arc_geometry = false;
};

/**
 * @brief End a goal the worker will not (or can no longer) run to completion.
 *
 * Reports CANCELED if the client asked to cancel, ABORTED otherwise, with
 * @p message as the result message. A goal that already reached a terminal state is
 * left alone. Never throws: it runs on error and shutdown paths.
 *
 * @tparam ActionT Action type; its Result must have `success` and `message`
 * @param goal_handle Goal to end
 * @param message Reason reported to the client
 * @param logger ROS logger to use
 */
template<typename ActionT>
void end_goal_early(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>> & goal_handle,
  const std::string & message,
  const rclcpp::Logger & logger)
{
  try {
    if (!goal_handle || !goal_handle->is_active()) {
      return;
    }
    auto result = std::make_shared<typename ActionT::Result>();
    result->success = false;
    result->message = message;
    if (goal_handle->is_canceling()) {
      goal_handle->canceled(result);
    } else {
      goal_handle->abort(result);
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger, "Failed to end goal (%s): %s", message.c_str(), e.what());
  }
}

/**
 * @brief Convert a duration parameter to a type wait_for() and sleep_for() accept.
 * @param seconds Duration [s]
 * @return The same duration in nanoseconds
 */
inline std::chrono::nanoseconds to_nanoseconds(double seconds)
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(seconds));
}

/**
 * @brief Check an auto_trigger_delay_sec parameter value.
 * @param delay_sec Delay before the auto-trigger fires [s]
 * @return true if it is finite, non-negative and at most one hour
 */
inline bool is_valid_auto_trigger_delay(double delay_sec)
{
  return std::isfinite(delay_sec) && delay_sec >= 0.0 && delay_sec <= 3600.0;
}

/**
 * @brief Wait for a ROS2 service to become available with a timeout and periodic logging.
 *
 * Polls the service once per second. Logs a warning every 10 seconds if still waiting.
 * Returns false (with an ERROR log) if the timeout is exceeded or rclcpp is shut down.
 *
 * @tparam ClientT rclcpp::Client<ServiceT> type
 * @param client        The service client to wait on
 * @param service_name  Human-readable name for log messages
 * @param logger        ROS logger to use
 * @param timeout_sec   Maximum seconds to wait (default 60)
 * @return true if the service became available, false on timeout or shutdown
 */
template<typename ClientT>
bool wait_for_service(
  const std::shared_ptr<ClientT> & client,
  const std::string & service_name,
  const rclcpp::Logger & logger,
  int timeout_sec = 60)
{
  int waited = 0;
  if (timeout_sec <= 0) {
    RCLCPP_ERROR(logger, "wait_for_service called with invalid timeout (%d) for %s",
      timeout_sec, service_name.c_str());
    return false;
  }
  while (!client->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(logger, "Interrupted while waiting for %s", service_name.c_str());
      return false;
    }
    ++waited;
    if (waited >= timeout_sec) {
      RCLCPP_ERROR(logger, "%s not available after %d seconds", service_name.c_str(), timeout_sec);
      return false;
    }
    if (waited % 10 == 0) {
      RCLCPP_WARN(logger, "Still waiting for %s (%d/%d s)", service_name.c_str(), waited,
          timeout_sec);
    }
  }
  return true;
}

/**
 * @brief Wait for a ROS2 action server to become available with a timeout and periodic logging.
 *
 * Polls the action server once per second. Logs a warning every 10 seconds if still waiting.
 * Returns false (with an ERROR log) if the timeout is exceeded or rclcpp is shut down.
 *
 * @tparam ActionClientT rclcpp_action::Client<ActionT> type
 * @param client        The action client to wait on
 * @param server_name   Human-readable name for log messages
 * @param logger        ROS logger to use
 * @param timeout_sec   Maximum seconds to wait (default 60)
 * @return true if the action server became available, false on timeout or shutdown
 */
template<typename ActionClientT>
bool wait_for_action_server(
  const std::shared_ptr<ActionClientT> & client,
  const std::string & server_name,
  const rclcpp::Logger & logger,
  int timeout_sec = 60)
{
  int waited = 0;
  if (timeout_sec <= 0) {
    RCLCPP_ERROR(logger, "wait_for_action_server called with invalid timeout (%d) for %s",
      timeout_sec, server_name.c_str());
    return false;
  }
  while (!client->wait_for_action_server(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(logger, "Interrupted while waiting for %s", server_name.c_str());
      return false;
    }
    ++waited;
    if (waited >= timeout_sec) {
      RCLCPP_ERROR(logger, "%s not available after %d seconds", server_name.c_str(), timeout_sec);
      return false;
    }
    if (waited % 10 == 0) {
      RCLCPP_WARN(logger, "Still waiting for %s (%d/%d s)", server_name.c_str(), waited,
          timeout_sec);
    }
  }
  return true;
}

/**
 * @brief Controller name owning an action topic: the segment before the action name,
 *        so namespaces are skipped, e.g.
 *        "/cell/robot1_gripper_controller/follow_joint_trajectory" -> "robot1_gripper_controller".
 * @param action_topic Action topic of a controller, "/<ns...>/<controller>/<action>".
 * @return The controller name, or empty if the topic has no controller segment.
 */
inline std::string controller_name_from_action_topic(const std::string & action_topic)
{
  const auto action_slash = action_topic.find_last_of('/');
  if (action_slash == std::string::npos || action_slash == 0) {
    return "";
  }
  const auto name_slash = action_topic.find_last_of('/', action_slash - 1);
  const auto start = name_slash == std::string::npos ? 0 : name_slash + 1;
  return action_topic.substr(start, action_slash - start);
}

/**
 * @brief Whether controller @p name is listed and in the "active" state.
 *
 * A configured but inactive JointTrajectoryController already serves its action
 * and accepts goals, then reports them succeeded without moving, so goals must
 * wait for "active".
 *
 * @param controllers Controllers as listed by controller_manager/list_controllers.
 * @param name Controller to look for.
 * @return true if @p name is listed with state "active".
 */
inline bool is_controller_active(
  const std::vector<controller_manager_msgs::msg::ControllerState> & controllers,
  const std::string & name)
{
  return std::any_of(controllers.begin(), controllers.end(), [&name](const auto & controller) {
             return controller.name == name && controller.state == "active";
           });
}

}  // namespace hold_and_weld

#endif  // HOLD_AND_WELD_APPLICATION__UTILS_HPP_
