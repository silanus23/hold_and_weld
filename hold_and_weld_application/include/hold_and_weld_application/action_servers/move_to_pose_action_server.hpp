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
#include <chrono>
#include <condition_variable>
#include <future>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "hold_and_weld_application/action/move_to_pose.hpp"

namespace hold_and_weld
{
namespace application
{

/**
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
 * @brief ROS2 action server for moving robot to target positions in joint or Cartesian space.
 *
 * Plans and executes one goal at a time with MoveIt, in joint space (joint angles for a
 * subset or all of the group; joints not named keep their current position) or Cartesian
 * space (end-effector pose in the planning frame). A goal arriving while another is queued
 * or running is rejected.
 *
 * Callbacks run on the executor thread; one worker thread runs execute_goal() and caches
 * one MoveGroupInterface per group. execution_mutex_ guards the goal hand-off and
 * active_move_group_ (which handle_cancel stops); move_group_cache_mutex_ guards the cache.
 * The cached interfaces hold this node, so manual_shutdown() clears the cache.
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
   * @brief Stop the arm and the worker before the ROS context goes away.
   *
   * Meant to run from a context pre-shutdown callback (see move_to_pose_server_main.cpp),
   * when the context is still valid, so stop() reaches the controller. Waits up to
   * shutdown_wait_sec for the running goal to end, joins the worker, then clears the move
   * group cache. If the goal does not end in time the worker is detached, as the process
   * is exiting anyway. Idempotent; the destructor calls it too.
   */
  void manual_shutdown();

private:
  /**
   * @brief Reject a malformed goal, or one arriving while another is queued or running.
   */
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const MoveToPose::Goal> goal);

  /**
   * @brief Accept a cancel and stop the running motion.
   */
  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleMoveToPose> goal_handle);

  /**
   * @brief Queue an accepted goal for the worker thread.
   */
  void handle_accepted(const std::shared_ptr<GoalHandleMoveToPose> goal_handle);

  /**
   * @brief Worker loop: waits for queued goals and executes them.
   */
  void worker_thread_func();

  /**
   * @brief Execute one goal and send its result.
   */
  void execute_goal(const std::shared_ptr<GoalHandleMoveToPose> goal_handle);

  /**
   * @brief Set the joint target (unnamed joints keep their position), then plan and execute.
   */
  bool execute_joint_space_motion(
    const std::shared_ptr<GoalHandleMoveToPose> goal_handle,
    const std::shared_ptr<const MoveToPose::Goal> goal,
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group);

  /**
   * @brief Set the end-effector pose target, then plan and execute.
   */
  bool execute_cartesian_space_motion(
    const std::shared_ptr<GoalHandleMoveToPose> goal_handle,
    const std::shared_ptr<const MoveToPose::Goal> goal,
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group);

  /**
   * @brief Plan the target already set on move_group and execute it, unless cancelled.
   */
  bool plan_and_execute(
    const std::shared_ptr<GoalHandleMoveToPose> & goal_handle,
    const std::shared_ptr<moveit::planning_interface::MoveGroupInterface> & move_group,
    const std::string & label);

  /**
   * @brief Publish feedback for the current goal.
   */
  void publish_feedback(
    const std::shared_ptr<GoalHandleMoveToPose> goal_handle,
    const std::string & step,
    float percentage);

  rclcpp_action::Server<MoveToPose>::SharedPtr action_server_;
  MoveToPoseConfig config_;
  std::chrono::nanoseconds shutdown_wait_time_{0};
  rclcpp::Logger logger_;

  std::thread worker_thread_;
  std::mutex execution_mutex_;
  std::condition_variable worker_cv_;
  std::shared_ptr<GoalHandleMoveToPose> pending_goal_;
  bool shutdown_requested_ = false;
  std::shared_future<void> execution_future_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> active_move_group_;
  std::atomic<bool> manual_shutdown_done_{false};

  std::mutex move_group_cache_mutex_;
  std::map<std::string, std::shared_ptr<moveit::planning_interface::MoveGroupInterface>>
  move_group_cache_;
};

}  // namespace application
}  // namespace hold_and_weld

#endif  // HOLD_AND_WELD_APPLICATION__ACTION_SERVERS__MOVE_TO_POSE_ACTION_SERVER_HPP_
