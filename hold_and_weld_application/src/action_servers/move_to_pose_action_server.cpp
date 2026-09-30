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

#include "hold_and_weld_application/action_servers/move_to_pose_action_server.hpp"

#include <chrono>
#include <cmath>
#include <functional>
#include <set>
#include <stdexcept>
#include <utility>
#include <vector>

#include "hold_and_weld_application/utils.hpp"

namespace hold_and_weld
{
namespace application
{

namespace
{
// Below this norm a goal quaternion has no usable direction (an unset Pose has w = 0).
constexpr double kMinQuaternionNorm = 1e-6;
}  // namespace

MoveToPoseActionServer::MoveToPoseActionServer(const rclcpp::NodeOptions & options)
: Node("move_to_pose_action_server", options),
  logger_(rclcpp::get_logger("application"))
{
  declare_parameter("planning_time", 5.0);
  declare_parameter("max_planning_attempts", 10);
  declare_parameter("velocity_scaling", 0.3);
  declare_parameter("acceleration_scaling", 0.3);
  declare_parameter("goal_tolerance", 0.01);
  declare_parameter("shutdown_wait_sec", 5.0);

  config_.planning_time = get_parameter("planning_time").as_double();
  const int64_t attempts = get_parameter("max_planning_attempts").as_int();
  config_.velocity_scaling = get_parameter("velocity_scaling").as_double();
  config_.acceleration_scaling = get_parameter("acceleration_scaling").as_double();
  config_.goal_tolerance = get_parameter("goal_tolerance").as_double();
  const double shutdown_wait_sec = get_parameter("shutdown_wait_sec").as_double();

  // Each check is written so that NaN fails it.
  if (!(std::isfinite(config_.planning_time) && config_.planning_time > 0.0)) {
    throw std::invalid_argument(
            "planning_time must be positive, got " + std::to_string(config_.planning_time));
  }
  if (attempts < 1 || attempts > 1000) {
    throw std::invalid_argument(
            "max_planning_attempts must be in [1, 1000], got " + std::to_string(attempts));
  }
  config_.max_planning_attempts = static_cast<int>(attempts);
  if (!(config_.velocity_scaling > 0.0 && config_.velocity_scaling <= 1.0) ||
    !(config_.acceleration_scaling > 0.0 && config_.acceleration_scaling <= 1.0))
  {
    throw std::invalid_argument("velocity_scaling and acceleration_scaling must be in (0, 1]");
  }
  if (!(std::isfinite(config_.goal_tolerance) && config_.goal_tolerance > 0.0)) {
    throw std::invalid_argument(
            "goal_tolerance must be positive, got " + std::to_string(config_.goal_tolerance));
  }
  if (!(shutdown_wait_sec > 0.0 && shutdown_wait_sec <= 60.0)) {
    throw std::invalid_argument(
            "shutdown_wait_sec must be in (0, 60], got " + std::to_string(shutdown_wait_sec));
  }
  shutdown_wait_time_ = hold_and_weld::to_nanoseconds(shutdown_wait_sec);

  RCLCPP_INFO(logger_, "Planning time %.1f s, velocity scaling %.2f, acceleration scaling %.2f",
    config_.planning_time, config_.velocity_scaling, config_.acceleration_scaling);

  using std::placeholders::_1;
  using std::placeholders::_2;

  action_server_ = rclcpp_action::create_server<MoveToPose>(
    this,
    "move_to_pose",
    std::bind(&MoveToPoseActionServer::handle_goal, this, _1, _2),
    std::bind(&MoveToPoseActionServer::handle_cancel, this, _1),
    std::bind(&MoveToPoseActionServer::handle_accepted, this, _1)
  );

  worker_thread_ = std::thread(&MoveToPoseActionServer::worker_thread_func, this);
}

MoveToPoseActionServer::~MoveToPoseActionServer()
{
  // Normally already done by the pre-shutdown callback registered in main().
  manual_shutdown();
}

void MoveToPoseActionServer::manual_shutdown()
{
  if (manual_shutdown_done_.exchange(true)) {
    return;
  }
  RCLCPP_INFO(logger_, "Manual shutdown: stopping the arm and the worker");

  std::shared_future<void> job;
  std::shared_ptr<GoalHandleMoveToPose> queued;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> active;
  {
    std::lock_guard<std::mutex> lock(execution_mutex_);
    shutdown_requested_ = true;
    queued = std::exchange(pending_goal_, nullptr);
    job = execution_future_;
    active = active_move_group_;
  }
  worker_cv_.notify_all();
  hold_and_weld::end_goal_early<MoveToPose>(
    queued, "move_to_pose server shut down before the goal started", logger_);

  try {
    if (active) {
      active->stop();
    }
  } catch (const std::exception & e) {
    RCLCPP_WARN(logger_, "Exception while stopping the move group during shutdown: %s",
      e.what());
  }

  if (job.valid() &&
    job.wait_for(shutdown_wait_time_) != std::future_status::ready)
  {
    // Joining would hang the exit. The process is going away, so detach; the thread may
    // still touch this node until it does. The cache stays, so the node stays alive too.
    RCLCPP_WARN(logger_, "Goal still running %.1f s after stop(); detaching the worker thread",
      std::chrono::duration<double>(shutdown_wait_time_).count());
    if (worker_thread_.joinable()) {
      worker_thread_.detach();
    }
    return;
  }

  if (worker_thread_.joinable()) {
    worker_thread_.join();
  }

  // Each cached MoveGroupInterface holds this node; dropping them lets it be destroyed.
  std::lock_guard<std::mutex> lock(move_group_cache_mutex_);
  move_group_cache_.clear();
}

rclcpp_action::GoalResponse MoveToPoseActionServer::handle_goal(
  [[maybe_unused]] const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const MoveToPose::Goal> goal)
{
  RCLCPP_INFO(logger_, "Received %s goal for group '%s'",
    goal->mode == MoveToPose::Goal::JOINT_SPACE ? "JOINT_SPACE" : "CARTESIAN_SPACE",
    goal->move_group_name.c_str());

  if (goal->move_group_name.empty()) {
    RCLCPP_WARN(logger_, "Move group name cannot be empty");
    return rclcpp_action::GoalResponse::REJECT;
  }

  if (goal->mode == MoveToPose::Goal::JOINT_SPACE) {
    if (goal->joint_names.empty() || goal->joint_positions.empty()) {
      RCLCPP_WARN(logger_, "Joint names and positions cannot be empty for JOINT_SPACE mode");
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (goal->joint_names.size() != goal->joint_positions.size()) {
      RCLCPP_WARN(logger_, "Joint names and positions must have the same size");
      return rclcpp_action::GoalResponse::REJECT;
    }
    // Duplicates would silently collapse to the last value in the target map.
    const std::set<std::string> unique_names(goal->joint_names.begin(), goal->joint_names.end());
    if (unique_names.size() != goal->joint_names.size()) {
      RCLCPP_WARN(logger_, "Joint names must be unique");
      return rclcpp_action::GoalResponse::REJECT;
    }
    for (size_t i = 0; i < goal->joint_positions.size(); ++i) {
      if (!std::isfinite(goal->joint_positions[i])) {
        RCLCPP_WARN(logger_, "Joint '%s' target is not finite", goal->joint_names[i].c_str());
        return rclcpp_action::GoalResponse::REJECT;
      }
    }
  } else if (goal->mode == MoveToPose::Goal::CARTESIAN_SPACE) {
    const auto & p = goal->cartesian_target.position;
    const auto & q = goal->cartesian_target.orientation;
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
      RCLCPP_WARN(logger_, "Cartesian target position is not finite");
      return rclcpp_action::GoalResponse::REJECT;
    }
    const double q_norm = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
    if (!(std::isfinite(q_norm) && q_norm >= kMinQuaternionNorm)) {
      RCLCPP_WARN(logger_, "Cartesian target orientation is zero or not finite "
        "(an unset orientation has w = 0)");
      return rclcpp_action::GoalResponse::REJECT;
    }
    RCLCPP_INFO(logger_, "Cartesian target: [%.3f, %.3f, %.3f]", p.x, p.y, p.z);
  } else {
    RCLCPP_WARN(logger_, "Invalid mode: %d", goal->mode);
    return rclcpp_action::GoalResponse::REJECT;
  }

  {
    std::lock_guard<std::mutex> lock(execution_mutex_);
    const bool running = execution_future_.valid() &&
      execution_future_.wait_for(std::chrono::seconds(0)) != std::future_status::ready;
    if (shutdown_requested_ || pending_goal_ || running) {
      RCLCPP_WARN(logger_, "Cannot accept goal: another goal is queued or running");
      return rclcpp_action::GoalResponse::REJECT;
    }
  }

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse MoveToPoseActionServer::handle_cancel(
  [[maybe_unused]] const std::shared_ptr<GoalHandleMoveToPose> goal_handle)
{
  RCLCPP_INFO(logger_, "Received cancel request");

  // Only one goal exists at a time, so stop only the group that goal is moving. stop() is
  // called outside the lock; the worker sees is_canceling() before its next motion.
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> active;
  {
    std::lock_guard<std::mutex> lock(execution_mutex_);
    active = active_move_group_;
  }
  try {
    if (active) {
      active->stop();
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger_, "Failed to stop move group: %s", e.what());
  }

  return rclcpp_action::CancelResponse::ACCEPT;
}

void MoveToPoseActionServer::handle_accepted(
  const std::shared_ptr<GoalHandleMoveToPose> goal_handle)
{
  bool queued = false;
  {
    std::lock_guard<std::mutex> lock(execution_mutex_);
    if (!pending_goal_) {
      pending_goal_ = goal_handle;
      queued = true;
    }
  }
  if (!queued) {
    hold_and_weld::end_goal_early<MoveToPose>(goal_handle, "A goal is already queued", logger_);
    return;
  }
  worker_cv_.notify_one();
}

void MoveToPoseActionServer::worker_thread_func()
{
  while (true) {
    std::shared_ptr<GoalHandleMoveToPose> goal_handle;
    std::promise<void> done;

    {
      std::unique_lock<std::mutex> lock(execution_mutex_);
      worker_cv_.wait(lock, [this] {
          return pending_goal_ != nullptr || shutdown_requested_;
        });

      if (shutdown_requested_) {
        break;
      }

      goal_handle = std::exchange(pending_goal_, nullptr);
      execution_future_ = done.get_future().share();
    }

    try {
      execute_goal(goal_handle);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(logger_, "Goal failed with an exception: %s", e.what());
      hold_and_weld::end_goal_early<MoveToPose>(
        goal_handle, std::string("Motion failed: ") + e.what(), logger_);
    } catch (...) {
      RCLCPP_ERROR(logger_, "Goal failed with an unknown exception");
      hold_and_weld::end_goal_early<MoveToPose>(
        goal_handle, "Motion failed with an unknown exception", logger_);
    }
    // No-op if the goal already ended; catches a path that returned without a result.
    hold_and_weld::end_goal_early<MoveToPose>(goal_handle, "Motion ended without a result",
      logger_);
    {
      std::lock_guard<std::mutex> lock(execution_mutex_);
      active_move_group_.reset();
    }
    done.set_value();
  }
}

void MoveToPoseActionServer::execute_goal(
  const std::shared_ptr<GoalHandleMoveToPose> goal_handle)
{
  const auto goal = goal_handle->get_goal();
  auto result = std::make_shared<MoveToPose::Result>();
  const auto start_time = std::chrono::steady_clock::now();

  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group;
  {
    std::lock_guard<std::mutex> lock(move_group_cache_mutex_);
    auto it = move_group_cache_.find(goal->move_group_name);
    if (it != move_group_cache_.end()) {
      move_group = it->second;
    }
  }

  if (!move_group) {
    // Construction waits for the robot model and MoveIt's action servers, which can take
    // seconds, so the cache lock is not held across it. Only this worker creates groups,
    // so no second instance can race in.
    RCLCPP_INFO(logger_, "Creating new move group: %s", goal->move_group_name.c_str());
    try {
      move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
        shared_from_this(), goal->move_group_name);

      move_group->setPlanningTime(config_.planning_time);
      move_group->setNumPlanningAttempts(config_.max_planning_attempts);
      move_group->setMaxVelocityScalingFactor(config_.velocity_scaling);
      move_group->setMaxAccelerationScalingFactor(config_.acceleration_scaling);
      move_group->setGoalTolerance(config_.goal_tolerance);

      std::lock_guard<std::mutex> lock(move_group_cache_mutex_);
      move_group_cache_[goal->move_group_name] = move_group;
    } catch (const std::exception & e) {
      RCLCPP_ERROR(logger_, "Failed to create move group: %s", e.what());
      result->success = false;
      result->message = "Failed to create move group: " + std::string(e.what());
      goal_handle->abort(result);
      return;
    }
  }

  {
    std::lock_guard<std::mutex> lock(execution_mutex_);
    active_move_group_ = move_group;
  }

  bool success = false;
  if (!goal_handle->is_canceling()) {
    if (goal->mode == MoveToPose::Goal::JOINT_SPACE) {
      success = execute_joint_space_motion(goal_handle, goal, move_group);
    } else if (goal->mode == MoveToPose::Goal::CARTESIAN_SPACE) {
      success = execute_cartesian_space_motion(goal_handle, goal, move_group);
    }
  }

  result->success = success;
  result->execution_time_sec =
    std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time).count();

  if (success) {
    result->message = "Motion completed successfully";
    RCLCPP_INFO(logger_, "Goal succeeded in %.2f seconds", result->execution_time_sec);
    goal_handle->succeed(result);
  } else if (goal_handle->is_canceling()) {
    result->message = "Canceled by client";
    RCLCPP_WARN(logger_, "Goal canceled after %.2f seconds", result->execution_time_sec);
    goal_handle->canceled(result);
  } else {
    result->message = "Motion failed";
    RCLCPP_ERROR(logger_, "Goal failed after %.2f seconds", result->execution_time_sec);
    goal_handle->abort(result);
  }
}

bool MoveToPoseActionServer::execute_joint_space_motion(
  const std::shared_ptr<GoalHandleMoveToPose> goal_handle,
  const std::shared_ptr<const MoveToPose::Goal> goal,
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group)
{
  // A cached group keeps the previous goal's target, so seed every joint from the
  // current state first: joints the goal does not name then stay where they are.
  const auto current_state = move_group->getCurrentState(2.0);
  if (!current_state) {
    RCLCPP_ERROR(logger_, "Current robot state unavailable; cannot build a joint target");
    return false;
  }
  move_group->setStartStateToCurrentState();
  move_group->setJointValueTarget(*current_state);

  std::map<std::string, double> joint_targets;
  for (size_t i = 0; i < goal->joint_names.size(); ++i) {
    joint_targets[goal->joint_names[i]] = goal->joint_positions[i];
  }
  if (!move_group->setJointValueTarget(joint_targets)) {
    RCLCPP_ERROR(logger_, "Joint target rejected: unknown joint for group '%s' or outside "
      "joint limits", goal->move_group_name.c_str());
    return false;
  }

  return plan_and_execute(goal_handle, move_group, "joint space");
}

bool MoveToPoseActionServer::execute_cartesian_space_motion(
  const std::shared_ptr<GoalHandleMoveToPose> goal_handle,
  const std::shared_ptr<const MoveToPose::Goal> goal,
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group)
{
  // handle_goal() rejected zero and non-finite quaternions; normalise the rest.
  geometry_msgs::msg::Pose target = goal->cartesian_target;
  auto & q = target.orientation;
  const double q_norm = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
  q.x /= q_norm;
  q.y /= q_norm;
  q.z /= q_norm;
  q.w /= q_norm;

  move_group->setStartStateToCurrentState();
  if (!move_group->setPoseTarget(target)) {
    RCLCPP_ERROR(logger_, "Pose target rejected by MoveIt");
    return false;
  }

  return plan_and_execute(goal_handle, move_group, "Cartesian space");
}

bool MoveToPoseActionServer::plan_and_execute(
  const std::shared_ptr<GoalHandleMoveToPose> & goal_handle,
  const std::shared_ptr<moveit::planning_interface::MoveGroupInterface> & move_group,
  const std::string & label)
{
  publish_feedback(goal_handle, "Planning " + label + " motion", 10.0);
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  if (move_group->plan(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(logger_, "Planning failed");
    return false;
  }

  if (goal_handle->is_canceling()) {
    return false;
  }

  publish_feedback(goal_handle, "Executing " + label + " motion", 50.0);
  if (move_group->execute(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(logger_, "Execution failed");
    return false;
  }
  publish_feedback(goal_handle, label + " motion complete", 100.0);
  return true;
}

void MoveToPoseActionServer::publish_feedback(
  const std::shared_ptr<GoalHandleMoveToPose> goal_handle,
  const std::string & step,
  float percentage)
{
  auto feedback = std::make_shared<MoveToPose::Feedback>();
  feedback->current_step = step;
  feedback->completion_percentage = percentage;
  goal_handle->publish_feedback(feedback);
}

}  // namespace application
}  // namespace hold_and_weld
