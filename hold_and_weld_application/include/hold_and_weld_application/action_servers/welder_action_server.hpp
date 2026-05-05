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

#include <array>
#include <atomic>
#include <condition_variable>
#include <future>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <nlohmann/json.hpp>

#include <controller_manager_msgs/srv/list_controllers.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <lifecycle_msgs/msg/transition.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit_msgs/srv/get_cartesian_path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include "hold_and_weld_application/action/trigger_welder.hpp"
#include "hold_and_weld_application/kinematics/approach_validator.hpp"
#include "hold_and_weld_application/kinematics/ceres_ik_solver.hpp"
#include "hold_and_weld_application/kinematics/kinematics_solver.hpp"
#include "hold_and_weld_application/kinematics/urdf_parser.hpp"

#include "hold_and_weld_application/utils.hpp"

namespace hold_and_weld
{
namespace application
{

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
  int max_ompl_planning_attempts = 3;
  int max_approach_validation_retries = 3;
  int max_cartesian_retries = 2;
  bool use_approach_validator = true;
  std::string json_file;
  double manipulability_threshold = 1e-6;
};

/**
 * @class WelderActionServer
 * @brief ROS2 lifecycle action server for controlling welding operations with MoveIt integration.
 *
 * This class implements a lifecycle action server that handles welding seam execution,
 * including approach/retract motions, cartesian path planning, and feedback updates
 * during the welding process.
 */
class WelderActionServer : public rclcpp_lifecycle::LifecycleNode {
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

  // Lifecycle callbacks
  /**
   * @brief Configure lifecycle transition callback.
   * @param state Current lifecycle state.
   * @return Transition callback result.
   */
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State & state);

  /**
   * @brief Activate lifecycle transition callback.
   * @param state Current lifecycle state.
   * @return Transition callback result.
   */
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State & state);

  /**
   * @brief Deactivate lifecycle transition callback.
   * @param state Current lifecycle state.
   * @return Transition callback result.
   */
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State & state);

  /**
   * @brief Cleanup lifecycle transition callback.
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

  /**
   * @brief Clean shutdown called from main() after spin() returns, before rclcpp::shutdown().
   *
   * Sets shutdown_requested_, calls stop() while the ROS context is still valid,
   * then waits up to 2 s for the current execute() to return before giving up.
   */
  void manual_shutdown();

private:
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
  std::string find_latest_json() const;

  /**
   * @brief Load weld seams from a JSON file.
   * @param filepath Path to the JSON file containing seam definitions.
   * @return Vector of WeldSeam structures loaded from the file.
   */
  std::vector<WeldSeam> load_seams_from_json(const std::string & filepath) const;

  /**
   * @brief Worker thread function for asynchronous goal execution.
   */
  void worker_thread_func();

  /**
   * @brief Shutdown the worker thread gracefully.
   */
  void shutdown_worker();

  /**
   * @brief Execute welding operation for a given goal.
   * @param goal_handle Handle to the goal being executed.
   */
  void execute_weld(const std::shared_ptr<GoalHandleTriggerWelder> goal_handle);

  /**
   * @brief Move the welder arm to approach position 5cm back along the seam direction.
   * @param seam The weld seam containing start/end points and poses.
   * @return true if approach motion was successful, false otherwise.
   */
  bool approach_seam(const WeldSeam & seam);

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
   * TODO(@silanus23): Put controls over this
   * @param pose_data JSON object containing position and quaternion arrays.
   * @return Converted Pose message.
   */
  geometry_msgs::msg::Pose json_to_pose(const nlohmann::json & pose_data) const
  {
    // Validate required fields
    if (!pose_data.contains("position") || !pose_data.contains("quaternion")) {
      throw std::runtime_error("Pose missing 'position' or 'quaternion' key");
    }

    if (pose_data["position"].size() != 3) {
      throw std::runtime_error("Position array must have exactly 3 elements");
    }

    if (pose_data["quaternion"].size() != 4) {
      throw std::runtime_error("Quaternion array must have exactly 4 elements");
    }

    geometry_msgs::msg::Pose pose;
    pose.position.x = pose_data["position"][0];
    pose.position.y = pose_data["position"][1];
    pose.position.z = pose_data["position"][2];

    // Load into Eigen for normalization and manipulation
    // Note: Eigen::Quaterniond constructor takes (w, x, y, z)
    Eigen::Quaterniond q(
      pose_data["quaternion"][3],
      pose_data["quaternion"][0],
      pose_data["quaternion"][1],
      pose_data["quaternion"][2]
    );
    q.normalize();

    // WARNING: Hardcoded 180° flip around X-axis for specific welding torch orientation
    // DO NOT MODIFY unless torch or torch mounting changes
    Eigen::Quaterniond flip_rotation(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()));
    q = q * flip_rotation;

    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();
    pose.orientation.w = q.w();

    return pose;
  }
  /**
   * @return The pose in the robot's base frame.
   */
  geometry_msgs::msg::Pose transform_pose_to_base_frame(
    const geometry_msgs::msg::Pose & world_pose,
    const Eigen::Isometry3d & base_to_world_transform) const;

  rclcpp_action::Server<TriggerWelder>::SharedPtr action_server_;

  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr moveit_executor_;
  std::thread moveit_thread_;

  std::thread worker_thread_;
  std::mutex execution_mutex_;
  std::condition_variable execution_cv_;
  std::shared_ptr<GoalHandleTriggerWelder> pending_goal_;
  std::atomic<bool> shutdown_requested_{false};
  std::shared_future<void> execution_future_;
  std::mutex execution_future_mutex_;
  std::mutex move_group_mutex_;

  std::shared_ptr<hold_and_weld::kinematics::CeresIKSolver> ceres_solver_;
  std::shared_ptr<hold_and_weld::kinematics::KinematicsSolver> kinematics_solver_;
  std::unique_ptr<hold_and_weld::kinematics::ApproachValidator> approach_validator_;

  WelderConfig config_;
  rclcpp::Logger logger_;
};

}  // namespace application
}  // namespace hold_and_weld

#endif  // HOLD_AND_WELD_APPLICATION__ACTION_SERVERS__WELDER_ACTION_SERVER_HPP_
