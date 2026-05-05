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
#include <vector>

namespace hold_and_weld
{
namespace application
{

MoveToPoseActionServer::MoveToPoseActionServer(const rclcpp::NodeOptions & options)
: Node("move_to_pose_action_server", options),
  logger_(rclcpp::get_logger("application"))
{
  declare_parameter("planning_time", 5.0);
  declare_parameter("max_planning_attempts", 10);
  declare_parameter("velocity_scaling", 0.3);
  declare_parameter("acceleration_scaling", 0.3);
  declare_parameter("goal_tolerance", 0.01);

  config_.planning_time = get_parameter("planning_time").as_double();
  config_.max_planning_attempts = get_parameter("max_planning_attempts").as_int();
  config_.velocity_scaling = get_parameter("velocity_scaling").as_double();
  config_.acceleration_scaling = get_parameter("acceleration_scaling").as_double();
  config_.goal_tolerance = get_parameter("goal_tolerance").as_double();

  RCLCPP_INFO(logger_, "  Planning time: %.1f s", config_.planning_time);
  RCLCPP_INFO(logger_, "  Velocity scaling: %.2f", config_.velocity_scaling);
  RCLCPP_INFO(logger_, "  Acceleration scaling: %.2f", config_.acceleration_scaling);

  using namespace std::placeholders;

  action_server_ = rclcpp_action::create_server<MoveToPose>(
    this,
    "move_to_pose",
    std::bind(&MoveToPoseActionServer::handle_goal, this, _1, _2),
    std::bind(&MoveToPoseActionServer::handle_cancel, this, _1),
    std::bind(&MoveToPoseActionServer::handle_accepted, this, _1)
  );

  // Start the persistent worker thread after the action server is live
  // so it cannot receive goals before the server is ready to handle them.
  worker_thread_ = std::thread(&MoveToPoseActionServer::worker_thread_func, this);
}

MoveToPoseActionServer::~MoveToPoseActionServer()
{
  // manual_shutdown() should have already been called from main().
  // This is a safety net for any path that bypasses main().
  manual_shutdown();
}

void MoveToPoseActionServer::manual_shutdown()
{
  // Idempotent — safe to call multiple times (destructor calls it as a safety net).
  if (shutdown_requested_.exchange(true)) {
    return;
  }

  RCLCPP_DEBUG(logger_, "Manual shutdown: signalling stop");

  // Snapshot the cache and call stop() on every move group while the ROS context
  // is still valid, so the cancel request can reach the controller.
  // move_group_cache_mutex_ is released before calling stop() to avoid holding
  // the lock across any MoveIt internal callback processing.
  std::vector<std::shared_ptr<moveit::planning_interface::MoveGroupInterface>> mg_snapshot;
  {
    std::lock_guard<std::mutex> lock(move_group_cache_mutex_);
    for (auto & [name, mg] : move_group_cache_) {
      mg_snapshot.push_back(mg);
    }
  }
  for (auto & mg : mg_snapshot) {
    try {
      mg->stop();
    } catch (...) {
      RCLCPP_WARN(logger_, "Exception caught while stopping a move group during shutdown");
    }
  }

  // Poll until execute() returns or the ROS context dies — whichever comes first.
  std::shared_future<void> future_copy;
  {
    std::lock_guard<std::mutex> lock(execution_future_mutex_);
    future_copy = execution_future_;
  }

  if (future_copy.valid()) {
    while (rclcpp::ok() &&
      future_copy.wait_for(std::chrono::milliseconds(10)) == std::future_status::timeout)
    {}

    if (future_copy.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready) {
      RCLCPP_INFO(logger_, "Execution finished cleanly.");
    } else {
      // Context died before execute() returned — detach worker to avoid std::terminate().
      RCLCPP_WARN(logger_, "ROS context shut down before execute() returned — detaching worker."
                           " Move group cache kept alive by cached shared_ptrs.");
      execution_cv_.notify_all();
      if (worker_thread_.joinable()) {
        worker_thread_.detach();
      }
      return;
    }
  }

  // Worker is idle — wake it so it can see shutdown_requested_ and exit.
  execution_cv_.notify_all();
  if (worker_thread_.joinable()) {
    worker_thread_.join();
  }
}

rclcpp_action::GoalResponse MoveToPoseActionServer::handle_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const MoveToPose::Goal> goal)
{
  (void)uuid;

  RCLCPP_INFO(logger_, "Received goal request:");
  RCLCPP_INFO(logger_, "  Mode: %s",
              goal->mode == MoveToPose::Goal::JOINT_SPACE ? "JOINT_SPACE" : "CARTESIAN_SPACE");
  RCLCPP_INFO(logger_, "  Move group: %s", goal->move_group_name.c_str());

  if (goal->move_group_name.empty()) {
    RCLCPP_ERROR(logger_, "Move group name cannot be empty");
    return rclcpp_action::GoalResponse::REJECT;
  }

  if (goal->mode == MoveToPose::Goal::JOINT_SPACE) {
    if (goal->joint_names.empty() || goal->joint_positions.empty()) {
      RCLCPP_ERROR(logger_, "Joint names and positions cannot be empty for JOINT_SPACE mode");
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (goal->joint_names.size() != goal->joint_positions.size()) {
      RCLCPP_ERROR(logger_, "Joint names and positions must have the same size");
      return rclcpp_action::GoalResponse::REJECT;
    }
    RCLCPP_INFO(logger_, "  Joint targets: %zu joints", goal->joint_names.size());
  } else if (goal->mode == MoveToPose::Goal::CARTESIAN_SPACE) {
    RCLCPP_INFO(logger_, "  Cartesian target: [%.3f, %.3f, %.3f]",
                goal->cartesian_target.position.x,
                goal->cartesian_target.position.y,
                goal->cartesian_target.position.z);
  } else {
    RCLCPP_ERROR(logger_, "Invalid mode: %d", goal->mode);
    return rclcpp_action::GoalResponse::REJECT;
  }

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse MoveToPoseActionServer::handle_cancel(
  const std::shared_ptr<GoalHandleMoveToPose> goal_handle)
{
  RCLCPP_INFO(logger_, "Received request to cancel goal");
  (void)goal_handle;

  // Snapshot the cache under the lock, then call stop() without holding it.
  // move_group_cache_mutex_ must not be held across stop() calls: stop() is a
  // non-blocking publish but this keeps the pattern consistent with the rule
  // that we never hold the cache lock across MoveIt calls.
  std::vector<std::shared_ptr<moveit::planning_interface::MoveGroupInterface>> snapshot;
  {
    std::lock_guard<std::mutex> lock(move_group_cache_mutex_);
    for (auto & [name, mg] : move_group_cache_) {
      snapshot.push_back(mg);
    }
  }
  for (auto & mg : snapshot) {
    try {
      mg->stop();
    } catch (const std::exception & e) {
      RCLCPP_ERROR(logger_, "Failed to stop move group: %s", e.what());
    }
  }

  return rclcpp_action::CancelResponse::ACCEPT;
}

void MoveToPoseActionServer::handle_accepted(
  const std::shared_ptr<GoalHandleMoveToPose> goal_handle)
{
  {
    std::lock_guard<std::mutex> lock(execution_mutex_);
    pending_goal_ = goal_handle;
  }
  execution_cv_.notify_one();
}

void MoveToPoseActionServer::worker_thread_func()
{
  // TODO(berkan): Add a watchdog timeout on execute_goal() to prevent the worker from
  // blocking indefinitely if the controller stops responding.
  while (true) {
    std::shared_ptr<GoalHandleMoveToPose> goal_handle;

    {
      std::unique_lock<std::mutex> lock(execution_mutex_);
      execution_cv_.wait(lock, [this] {
          return pending_goal_ != nullptr || shutdown_requested_.load();
        });

      if (shutdown_requested_.load() && pending_goal_ == nullptr) {
        break;
      }

      goal_handle = pending_goal_;
      pending_goal_ = nullptr;
    }

    if (goal_handle) {
      auto promise = std::make_shared<std::promise<void>>();
      {
        std::lock_guard<std::mutex> lock(execution_future_mutex_);
        execution_future_ = promise->get_future().share();
      }
      execute_goal(goal_handle);
      promise->set_value();
    }
  }
}

void MoveToPoseActionServer::execute_goal(
  const std::shared_ptr<GoalHandleMoveToPose> goal_handle)
{
  const auto goal = goal_handle->get_goal();
  auto result = std::make_shared<MoveToPose::Result>();

  RCLCPP_INFO(logger_, "Executing goal");

  auto start_time = std::chrono::steady_clock::now();

  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group;
  {
    std::lock_guard<std::mutex> lock(move_group_cache_mutex_);
    auto it = move_group_cache_.find(goal->move_group_name);
    if (it != move_group_cache_.end()) {
      move_group = it->second;
      RCLCPP_INFO(logger_, "Using cached move group: %s", goal->move_group_name.c_str());
    }
  }

  if (!move_group) {
    // MoveGroupInterface construction makes synchronous service calls that must
    // be processed by the node's executor (running on the main thread via
    // rclcpp::spin). The mutex MUST NOT be held during construction: handle_cancel
    // also acquires move_group_cache_mutex_, and since handle_cancel runs on the
    // same executor thread that needs to process those service responses, holding
    // the lock here would deadlock.
    RCLCPP_INFO(logger_, "Creating new move group: %s", goal->move_group_name.c_str());
    try {
      auto new_mg = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
        shared_from_this(), goal->move_group_name);

      new_mg->setPlanningTime(config_.planning_time);
      new_mg->setNumPlanningAttempts(config_.max_planning_attempts);
      new_mg->setMaxVelocityScalingFactor(config_.velocity_scaling);
      new_mg->setMaxAccelerationScalingFactor(config_.acceleration_scaling);
      new_mg->setGoalTolerance(config_.goal_tolerance);

      // Re-acquire the lock only to insert the finished object into the cache.
      // Check again in case a concurrent goal already inserted the same group.
      std::lock_guard<std::mutex> lock(move_group_cache_mutex_);
      auto it = move_group_cache_.find(goal->move_group_name);
      if (it != move_group_cache_.end()) {
        // Another goal won the race — use the already-cached instance.
        move_group = it->second;
      } else {
        move_group_cache_[goal->move_group_name] = new_mg;
        move_group = new_mg;
      }
    } catch (const std::exception & e) {
      RCLCPP_ERROR(logger_, "Failed to create move group: %s", e.what());
      result->success = false;
      result->message = "Failed to create move group: " + std::string(e.what());
      goal_handle->abort(result);
      return;
    }
  }

  bool success = false;

  if (goal->mode == MoveToPose::Goal::JOINT_SPACE) {
    success = execute_joint_space_motion(goal_handle, goal, move_group);
  } else if (goal->mode == MoveToPose::Goal::CARTESIAN_SPACE) {
    success = execute_cartesian_space_motion(goal_handle, goal, move_group);
  }

  auto end_time = std::chrono::steady_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);

  result->success = success;
  result->execution_time_sec = duration.count() / 1000.0;

  if (success) {
    result->message = "Motion completed successfully";
    RCLCPP_INFO(logger_, "Goal succeeded in %.2f seconds", result->execution_time_sec);
    goal_handle->succeed(result);
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
  RCLCPP_INFO(logger_, "Executing joint space motion");

  std::map<std::string, double> joint_targets;
  for (size_t i = 0; i < goal->joint_names.size(); ++i) {
    joint_targets[goal->joint_names[i]] = goal->joint_positions[i];
  }

  move_group->setJointValueTarget(joint_targets);

  publish_feedback(goal_handle, "Planning joint space motion", 10.0);
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool plan_success = (move_group->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

  if (!plan_success) {
    RCLCPP_ERROR(logger_, "Planning failed");
    return false;
  }

  RCLCPP_INFO(logger_, "Planning successful");

  if (goal_handle->is_canceling()) {
    RCLCPP_WARN(logger_, "Goal cancelled during planning");
    return false;
  }

  publish_feedback(goal_handle, "Executing joint space motion", 50.0);
  auto execute_result = move_group->execute(plan);

  if (execute_result == moveit::core::MoveItErrorCode::SUCCESS) {
    publish_feedback(goal_handle, "Joint space motion complete", 100.0);
    return true;
  } else {
    RCLCPP_ERROR(logger_, "Execution failed");
    return false;
  }
}

bool MoveToPoseActionServer::execute_cartesian_space_motion(
  const std::shared_ptr<GoalHandleMoveToPose> goal_handle,
  const std::shared_ptr<const MoveToPose::Goal> goal,
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group)
{
  RCLCPP_INFO(logger_, "Executing Cartesian space motion");

  move_group->setPoseTarget(goal->cartesian_target);

  publish_feedback(goal_handle, "Planning Cartesian space motion", 10.0);
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool plan_success = (move_group->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

  if (!plan_success) {
    RCLCPP_ERROR(logger_, "Planning failed");
    return false;
  }

  RCLCPP_INFO(logger_, "Planning successful");

  if (goal_handle->is_canceling()) {
    RCLCPP_WARN(logger_, "Goal cancelled during planning");
    return false;
  }

  publish_feedback(goal_handle, "Executing Cartesian space motion", 50.0);
  auto execute_result = move_group->execute(plan);

  if (execute_result == moveit::core::MoveItErrorCode::SUCCESS) {
    publish_feedback(goal_handle, "Cartesian space motion complete", 100.0);
    return true;
  } else {
    RCLCPP_ERROR(logger_, "Execution failed");
    return false;
  }
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
