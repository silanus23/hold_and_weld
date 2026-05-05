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

#ifndef HOLD_AND_WELD_APPLICATION__ACTION_SERVERS__MOVE_TO_POSE_ACTION_SERVER_HPP_
#define HOLD_AND_WELD_APPLICATION__ACTION_SERVERS__MOVE_TO_POSE_ACTION_SERVER_HPP_

#include <atomic>
#include <future>
#include <map>
#include <memory>
#include <mutex>
#include <condition_variable>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include "hold_and_weld_application/action/move_to_pose.hpp"

namespace hold_and_weld
{
namespace application
{

/**
 * @struct MoveToPoseConfig
 * @brief Configuration parameters for the move to pose action server.
 */
struct MoveToPoseConfig
{
  double planning_time = 5.0;
  int max_planning_attempts = 10;
  double velocity_scaling = 0.3;
  double acceleration_scaling = 0.3;
  double goal_tolerance = 0.01;
};

/**
 * @class MoveToPoseActionServer
 * @brief ROS2 action server for moving robot to target positions in joint or Cartesian space.
 *
 * This class implements an action server that handles robot motion planning and execution
 * in both joint space (specified joint angles) and Cartesian space (end-effector pose).
 * It uses MoveIt for motion planning and provides feedback during execution.
 */
class MoveToPoseActionServer : public rclcpp::Node
{
public:
  using MoveToPose = hold_and_weld_application::action::MoveToPose;
  using GoalHandleMoveToPose = rclcpp_action::ServerGoalHandle<MoveToPose>;

  /**
   * @brief Construct a new MoveToPoseActionServer object.
   * @param options ROS2 node options for configuration.
   */
  explicit MoveToPoseActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /**
   * @brief Destroy the MoveToPoseActionServer object, ensuring proper cleanup of worker thread.
   */
  ~MoveToPoseActionServer() override;

  /**
   * @brief Clean shutdown called from main() after spin() returns, before rclcpp::shutdown().
   *
   * Sets shutdown_requested_, calls stop() on all cached move groups while the ROS context
   * is still valid, then waits for execute() to return before joining the worker thread.
   */
  void manual_shutdown();

private:
  /**
   * @brief Handle incoming goal requests from action clients.
   * @param uuid Unique identifier for the goal.
   * @param goal Goal message containing the move to pose request.
   * @return GoalResponse indicating whether the goal is accepted.
   */
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const MoveToPose::Goal> goal);

  /**
   * @brief Handle cancellation requests for active goals.
   * @param goal_handle Handle to the goal being cancelled.
   * @return CancelResponse indicating whether cancellation is accepted.
   */
  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleMoveToPose> goal_handle);

  /**
   * @brief Handle accepted goals by queuing them for the worker thread.
   * @param goal_handle Handle to the accepted goal.
   */
  void handle_accepted(const std::shared_ptr<GoalHandleMoveToPose> goal_handle);

  /**
   * @brief Worker thread function for asynchronous goal execution.
   */
  void worker_thread_func();

  /**
   * @brief Execute a single goal in the worker thread.
   * @param goal_handle Handle to the goal being executed.
   */
  void execute_goal(const std::shared_ptr<GoalHandleMoveToPose> goal_handle);

  /**
   * @brief Execute joint space motion.
   * @param goal_handle Handle to the goal being executed.
   * @param goal The goal message.
   * @param move_group MoveIt move group interface.
   * @return True if successful, false otherwise.
   */
  bool execute_joint_space_motion(
    const std::shared_ptr<GoalHandleMoveToPose> goal_handle,
    const std::shared_ptr<const MoveToPose::Goal> goal,
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group);

  /**
   * @brief Execute Cartesian space motion.
   * @param goal_handle Handle to the goal being executed.
   * @param goal The goal message.
   * @param move_group MoveIt move group interface.
   * @return True if successful, false otherwise.
   */
  bool execute_cartesian_space_motion(
    const std::shared_ptr<GoalHandleMoveToPose> goal_handle,
    const std::shared_ptr<const MoveToPose::Goal> goal,
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group);

  /**
   * @brief Publish feedback for the current goal.
   * @param goal_handle Handle to the goal.
   * @param step Current step description.
   * @param percentage Completion percentage.
   */
  void publish_feedback(
    const std::shared_ptr<GoalHandleMoveToPose> goal_handle,
    const std::string & step,
    float percentage);

  rclcpp_action::Server<MoveToPose>::SharedPtr action_server_;
  MoveToPoseConfig config_;
  rclcpp::Logger logger_;

  std::thread worker_thread_;
  std::mutex execution_mutex_;
  std::condition_variable execution_cv_;
  std::shared_ptr<GoalHandleMoveToPose> pending_goal_;
  std::atomic<bool> shutdown_requested_{false};
  std::shared_future<void> execution_future_;
  std::mutex execution_future_mutex_;

  std::mutex move_group_cache_mutex_;
  std::map<std::string, std::shared_ptr<moveit::planning_interface::MoveGroupInterface>>
  move_group_cache_;
};

}  // namespace application
}  // namespace hold_and_weld

#endif  // HOLD_AND_WELD_APPLICATION__ACTION_SERVERS__MOVE_TO_POSE_ACTION_SERVER_HPP_
