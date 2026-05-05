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

#include <array>
#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <rclcpp/rclcpp.hpp>

namespace hold_and_weld
{

/**
 * @struct WeldSeam
 * @brief Represents a welding seam with geometric and pose information.
 */
struct WeldSeam
{
  std::string seam_id;
  double length_m = 0.0;
  std::array<double, 3> start = {0.0, 0.0, 0.0};
  std::array<double, 3> end = {0.0, 0.0, 0.0};
  std::vector<geometry_msgs::msg::Pose> poses;
  size_t num_poses = 0;
};

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

}  // namespace hold_and_weld

#endif  // HOLD_AND_WELD_APPLICATION__UTILS_HPP_
