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

#ifndef HOLD_AND_WELD_APPLICATION__ACTION_SERVERS__WELDER_ACTION_SERVER_HPP_
#define HOLD_AND_WELD_APPLICATION__ACTION_SERVERS__WELDER_ACTION_SERVER_HPP_

#include <string>
#include <vector>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <array>
#include <memory>

#include <nlohmann/json.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include "hold_and_weld_application/action/trigger_welder.hpp"

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
 * @struct WelderConfig
 * @brief Configuration parameters for the welder action server.
 */
struct WelderConfig
{
  std::string welder_group_name = "robot2_gp25_welder_arm";
  double approach_offset_z = 0.1;
  double retract_offset_z = 0.15;
  double cartesian_path_threshold = 0.95;
  double cartesian_step_size = 0.01;
  double velocity_scaling = 0.3;
  int max_approach_retries = 3;
  int max_cartesian_retries = 2;
  std::string json_file;
};

/**
 * @class WelderActionServer
 * @brief ROS2 action server for controlling welding operations with MoveIt integration.
 *
 * This class implements an action server that handles welding seam execution,
 * including approach/retract motions, cartesian path planning, and feedback updates
 * during the welding process.
 */
class WelderActionServer : public rclcpp::Node {
public:
  using TriggerWelder = hold_and_weld_application::action::TriggerWelder;
  using GoalHandleTriggerWelder = rclcpp_action::ServerGoalHandle<TriggerWelder>;

  /**
   * @brief Construct a new WelderActionServer object.
   * @param options ROS2 node options for configuration.
   */
  explicit WelderActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /**
   * @brief Destroy the WelderActionServer object, ensuring proper cleanup of worker thread.
   */
  ~WelderActionServer() override;

private:
    // Action server callbacks
  /**
   * @brief Handle incoming goal requests from action clients.
   * @param uuid Unique identifier for the goal.
   * @param goal Goal message containing the trigger welder request.
   * @return GoalResponse indicating whether the goal is accepted.
   */
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const TriggerWelder::Goal> goal);

  /**
   * @brief Handle cancellation requests for active goals.
   * @param goal_handle Handle to the goal being cancelled.
   * @return CancelResponse indicating whether cancellation is accepted.
   */
  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleTriggerWelder> goal_handle);

  /**
   * @brief Handle accepted goals by queuing them for the worker thread.
   * @param goal_handle Handle to the accepted goal.
   */
  void handle_accepted(const std::shared_ptr<GoalHandleTriggerWelder> goal_handle);


    // Initialization
  /**
   * @brief Initialize MoveIt interface and planning scene.
   */
  void initialize_moveit();

  /**
   * @brief Load welder configuration from YAML file.
   */
  void load_config_from_yaml();

  /**
   * @brief Find the latest JSON file containing weld seam data.
   * @return Path to the latest JSON file, or empty string if not found.
   */
  std::string find_latest_json();

    // JSON loading
  /**
   * @brief Load weld seams from a JSON file.
   * @param filepath Path to the JSON file containing seam definitions.
   * @return Vector of WeldSeam structures loaded from the file.
   */
  std::vector<WeldSeam> load_seams_from_json(const std::string & filepath);

    // Worker thread
  /**
   * @brief Worker thread function for asynchronous goal execution.
   */
  void worker_thread_func();

  /**
   * @brief Shutdown the worker thread gracefully.
   */
  void shutdown_worker();

    // Execution
  /**
   * @brief Execute welding operation for a given goal.
   * @param goal_handle Handle to the goal being executed.
   */
  void execute_weld(const std::shared_ptr<GoalHandleTriggerWelder> goal_handle);

  /**
   * @brief Move the welder arm to approach position above the first seam pose.
   * @param first_pose First pose along the weld seam.
   * @return true if approach motion was successful, false otherwise.
   */
  bool approach_seam(const geometry_msgs::msg::Pose & first_pose);

  /**
   * @brief Retract the welder arm from the last seam pose.
   * @param last_pose Last pose along the weld seam.
   * @return true if retract motion was successful, false otherwise.
   */
  bool retract_from_seam(const geometry_msgs::msg::Pose & last_pose);

  /**
   * @brief Execute a cartesian path along the weld seam.
   * @param waypoints Sequence of poses to follow along the seam.
   * @param goal_handle Handle to the goal for sending feedback and results.
   * @param feedback Feedback message to update with progress.
   * @param points_before_seam Number of waypoints in approach phase.
   * @param total_waypoints Total number of waypoints in the complete path.
   * @return true if cartesian path execution was successful, false otherwise.
   */
  bool execute_cartesian_path(
    const std::vector<geometry_msgs::msg::Pose> & waypoints,
    const std::shared_ptr<GoalHandleTriggerWelder> & goal_handle,
    std::shared_ptr<TriggerWelder::Feedback> & feedback,
    int32_t points_before_seam,
    int32_t total_waypoints);

  /**
   * @brief Convert a JSON pose object to a geometry_msgs::msg::Pose message.
   * @param pose_data JSON object containing position and quaternion arrays.
   * @return Converted Pose message.
   */
  geometry_msgs::msg::Pose json_to_pose(const nlohmann::json & pose_data)
  {
    geometry_msgs::msg::Pose pose;

    pose.position.x = pose_data["position"][0];
    pose.position.y = pose_data["position"][1];
    pose.position.z = pose_data["position"][2];

    pose.orientation.x = pose_data["quaternion"][0];
    pose.orientation.y = pose_data["quaternion"][1];
    pose.orientation.z = pose_data["quaternion"][2];
    pose.orientation.w = pose_data["quaternion"][3];
    return pose;
  }

    // MoveIt
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;

    // Action server
  rclcpp_action::Server<TriggerWelder>::SharedPtr action_server_;

    // Timer for delayed initialization
  rclcpp::TimerBase::SharedPtr init_timer_;

    // Worker thread
  std::thread worker_thread_;
  std::mutex work_mutex_;
  std::condition_variable work_cv_;
  std::shared_ptr<GoalHandleTriggerWelder> pending_goal_;
  bool shutdown_requested_ = false;

    // Configuration
  WelderConfig config_;
  bool initialized_ = false;
};

}  // namespace hold_and_weld

#endif  // HOLD_AND_WELD_APPLICATION__ACTION_SERVERS__WELDER_ACTION_SERVER_HPP_
