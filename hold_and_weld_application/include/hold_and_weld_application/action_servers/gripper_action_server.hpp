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

#ifndef HOLD_AND_WELD_APPLICATION__ACTION_SERVERS__GRIPPER_ACTION_SERVER_HPP_
#define HOLD_AND_WELD_APPLICATION__ACTION_SERVERS__GRIPPER_ACTION_SERVER_HPP_

#include <memory>
#include <map>
#include <mutex>
#include <string>
#include <vector>
#include <cmath>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/srv/apply_planning_scene.hpp>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include "hold_and_weld_application/action/trigger_gripper.hpp"


namespace hold_and_weld
{

/**
 * @brief Timing constants for gripper operations and action handling.
 */
namespace timing
{
constexpr double GRIPPER_MOTION_DURATION_SEC = 2.0;
constexpr int ACTION_SERVER_TIMEOUT_SEC = 5;
constexpr int GRIPPER_RESULT_TIMEOUT_SEC = 10;
constexpr int ATTACH_SETTLE_TIME_MS = 500;
constexpr int DETACH_SETTLE_TIME_MS = 250;
constexpr int MOTION_SETTLE_TIME_MS = 250;
}

/**
 * @struct GripperJob
 * @brief Represents a complete gripper operation job with target and motion poses.
 */
struct GripperJob
{
  std::string target_id;
  geometry_msgs::msg::Pose approach_pose;
  geometry_msgs::msg::Pose pick_pose;
  geometry_msgs::msg::Pose retract_pose;
  geometry_msgs::msg::Pose place_pose;
};

/**
 * @class GripperActionServer
 * @brief ROS2 action server for controlling gripper operations with MoveIt integration.
 *
 * This class implements an action server that handles gripper motion planning and execution,
 * including object attachment/detachment, collision object management, and synchronized
 * gripper control through follow joint trajectory actions.
 */
class GripperActionServer : public rclcpp::Node {
public:
  using TriggerGripper = hold_and_weld_application::action::TriggerGripper;
  using GoalHandleTriggerGripper = rclcpp_action::ServerGoalHandle<TriggerGripper>;
  using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;

  /**
   * @brief Construct a new GripperActionServer object.
   * @param options ROS2 node options for configuration.
   */
  explicit GripperActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // Initialization
  /**
   * @brief Initialize MoveIt interface and planning scene.
   */
  void initialize_moveit();

  // Action server callbacks
  /**
   * @brief Handle incoming goal requests from action clients.
   * @param uuid Unique identifier for the goal.
   * @param goal Goal message containing the trigger gripper request.
   * @return GoalResponse indicating whether the goal is accepted.
   */
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const TriggerGripper::Goal> goal);

  /**
   * @brief Handle cancellation requests for active goals.
   * @param goal_handle Handle to the goal being cancelled.
   * @return CancelResponse indicating whether cancellation is accepted.
   */
  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleTriggerGripper> goal_handle);

  /**
   * @brief Handle accepted goals by spawning execution thread.
   * @param goal_handle Handle to the accepted goal.
   */
  void handle_accepted(const std::shared_ptr<GoalHandleTriggerGripper> goal_handle);

  /**
   * @brief Execute the gripper job for a given goal.
   * @param goal_handle Handle to the goal being executed.
   */
  void execute_job(const std::shared_ptr<GoalHandleTriggerGripper> goal_handle);

  // Configuration loading
  /**
   * @brief Load job configuration from YAML file.
   * @param yaml_path Path to the YAML configuration file.
   */
  void load_job_from_yaml(const std::string & yaml_path);

  // Gripper control
  /**
   * @brief Set the gripper to a specific position.
   * @param position Target position for the gripper (0.0 = closed, 0.15 = open).
   * @return true if position was successfully set, false otherwise.
   */
  bool set_gripper_position(double position);

  // Motion
  /**
   * @brief Move the arm to a specified pose.
   * @param pose Target pose for the arm.
   * @param step_name Name of the motion step for logging/feedback.
   * @return true if motion was successful, false otherwise.
   */
  bool move_to_pose(const geometry_msgs::msg::Pose & pose, const std::string & step_name);

  // Collision objects
  /**
   * @brief Attach an object to the gripper in the planning scene.
   * @param object_id Identifier of the object to attach.
   * @return true if attachment was successful, false otherwise.
   */
  bool attach_object(const std::string & object_id);

  /**
   * @brief Detach an object from the gripper in the planning scene.
   * @param object_id Identifier of the object to detach.
   * @return true if detachment was successful, false otherwise.
   */
  bool detach_object(const std::string & object_id);

  // Utility
  /**
   * @brief Normalize a quaternion to unit length.
   * @param q Quaternion to normalize (modified in-place).
   */
  void normalize_quaternion(geometry_msgs::msg::Quaternion & q);

  /**
   * @brief Wait for planning scene updates to be processed.
   * @param millis Timeout duration in milliseconds.
   * @return true if update was received within timeout, false otherwise.
   */
  bool wait_for_planning_scene_update(int millis);

  /**
   * @brief Allow collision between cube and workpiece for placement.
   * @return true if collision matrix was updated successfully, false otherwise.
   */
  bool allow_collision_for_placement();

  // Members
  rclcpp_action::Server<TriggerGripper>::SharedPtr action_server_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  rclcpp::Client<moveit_msgs::srv::ApplyPlanningScene>::SharedPtr planning_scene_client_;

  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr gripper_action_client_;
  rclcpp::Publisher<moveit_msgs::msg::AttachedCollisionObject>::SharedPtr attached_collision_pub_;
  rclcpp::Client<moveit_msgs::srv::GetPlanningScene>::SharedPtr get_planning_scene_client_;

  // Configuration (protected by mutex)
  std::mutex config_mutex_;
  GripperJob job_;
  bool job_loaded_ = false;

  double open_position_ = 0.15;
  double close_position_ = 0.0;

  std::vector<std::string> gripper_joint_names_ = {
    "robot1_left_finger_joint", "robot1_right_finger_joint"};
  std::vector<std::string> touch_links_ = {
    "robot1_tool0", "robot1_link_6_t", "robot1_flange",
    "robot1_gripper_base", "robot1_left_finger", "robot1_right_finger"};
  std::string attach_link_ = "robot1_link_6_t";

  // Execution thread management
  std::shared_ptr<std::thread> execution_thread_;
  std::mutex execution_mutex_;

  // Delayed init
  rclcpp::TimerBase::SharedPtr init_timer_;
  bool initialized_ = false;
  std::string arm_group_name_;
};

}  // namespace hold_and_weld

#endif  // HOLD_AND_WELD_APPLICATION__ACTION_SERVERS__GRIPPER_ACTION_SERVER_HPP_
