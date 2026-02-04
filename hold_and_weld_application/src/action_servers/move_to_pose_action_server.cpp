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

namespace hold_and_weld
{

MoveToPoseActionServer::MoveToPoseActionServer(const rclcpp::NodeOptions & options)
: Node("move_to_pose_action_server", options)
{
  // Declare parameters
  declare_parameter("planning_time", 5.0);
  declare_parameter("max_planning_attempts", 10);
  declare_parameter("velocity_scaling", 0.3);
  declare_parameter("acceleration_scaling", 0.3);
  declare_parameter("goal_tolerance", 0.01);

  // Load configuration
  config_.planning_time = get_parameter("planning_time").as_double();
  config_.max_planning_attempts = get_parameter("max_planning_attempts").as_int();
  config_.velocity_scaling = get_parameter("velocity_scaling").as_double();
  config_.acceleration_scaling = get_parameter("acceleration_scaling").as_double();
  config_.goal_tolerance = get_parameter("goal_tolerance").as_double();

  RCLCPP_INFO(get_logger(), "MoveToPose Action Server starting");
  RCLCPP_INFO(get_logger(), "  Planning time: %.1f s", config_.planning_time);
  RCLCPP_INFO(get_logger(), "  Velocity scaling: %.2f", config_.velocity_scaling);
  RCLCPP_INFO(get_logger(), "  Acceleration scaling: %.2f", config_.acceleration_scaling);

  using namespace std::placeholders;

  // Create action server
  action_server_ = rclcpp_action::create_server<MoveToPose>(
    this,
    "move_to_pose",
    std::bind(&MoveToPoseActionServer::handle_goal, this, _1, _2),
    std::bind(&MoveToPoseActionServer::handle_cancel, this, _1),
    std::bind(&MoveToPoseActionServer::handle_accepted, this, _1)
  );

  // Start worker thread
  worker_thread_ = std::thread(&MoveToPoseActionServer::worker_thread_func, this);

  RCLCPP_INFO(get_logger(), "MoveToPose Action Server ready");
}

MoveToPoseActionServer::~MoveToPoseActionServer()
{
  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    shutdown_ = true;
  }
  queue_cv_.notify_all();

  if (worker_thread_.joinable()) {
    worker_thread_.join();
  }
}

rclcpp_action::GoalResponse MoveToPoseActionServer::handle_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const MoveToPose::Goal> goal)
{
  (void)uuid;

  RCLCPP_INFO(get_logger(), "Received goal request:");
  RCLCPP_INFO(get_logger(), "  Mode: %s",
              goal->mode == MoveToPose::Goal::JOINT_SPACE ? "JOINT_SPACE" : "CARTESIAN_SPACE");
  RCLCPP_INFO(get_logger(), "  Move group: %s", goal->move_group_name.c_str());

  if (goal->move_group_name.empty()) {
    RCLCPP_ERROR(get_logger(), "Move group name cannot be empty");
    return rclcpp_action::GoalResponse::REJECT;
  }

  if (goal->mode == MoveToPose::Goal::JOINT_SPACE) {
    if (goal->joint_names.empty() || goal->joint_positions.empty()) {
      RCLCPP_ERROR(get_logger(), "Joint names and positions cannot be empty for JOINT_SPACE mode");
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (goal->joint_names.size() != goal->joint_positions.size()) {
      RCLCPP_ERROR(get_logger(), "Joint names and positions must have the same size");
      return rclcpp_action::GoalResponse::REJECT;
    }
    RCLCPP_INFO(get_logger(), "  Joint targets: %zu joints", goal->joint_names.size());
  } else if (goal->mode == MoveToPose::Goal::CARTESIAN_SPACE) {
    RCLCPP_INFO(get_logger(), "  Cartesian target: [%.3f, %.3f, %.3f]",
                goal->cartesian_target.position.x,
                goal->cartesian_target.position.y,
                goal->cartesian_target.position.z);
  } else {
    RCLCPP_ERROR(get_logger(), "Invalid mode: %d", goal->mode);
    return rclcpp_action::GoalResponse::REJECT;
  }

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse MoveToPoseActionServer::handle_cancel(
  const std::shared_ptr<GoalHandleMoveToPose> goal_handle)
{
  RCLCPP_INFO(get_logger(), "Received request to cancel goal");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void MoveToPoseActionServer::handle_accepted(
  const std::shared_ptr<GoalHandleMoveToPose> goal_handle)
{
  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    pending_goal_ = goal_handle;
  }
  queue_cv_.notify_one();
}

void MoveToPoseActionServer::worker_thread_func()
{
  while (rclcpp::ok()) {
    std::shared_ptr<GoalHandleMoveToPose> goal_handle;

    {
      std::unique_lock<std::mutex> lock(queue_mutex_);
      queue_cv_.wait(lock, [this] { return pending_goal_ != nullptr || shutdown_; });

      if (shutdown_) {
        break;
      }

      goal_handle = pending_goal_;
      pending_goal_ = nullptr;
    }

    if (goal_handle) {
      execute_goal(goal_handle);
    }
  }
}

void MoveToPoseActionServer::execute_goal(
  const std::shared_ptr<GoalHandleMoveToPose> goal_handle)
{
  const auto goal = goal_handle->get_goal();
  auto result = std::make_shared<MoveToPose::Result>();

  RCLCPP_INFO(get_logger(), "Executing goal");

  auto start_time = std::chrono::steady_clock::now();

  // Get or create move group
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group;
  {
    std::lock_guard<std::mutex> lock(move_group_cache_mutex_);
    auto it = move_group_cache_.find(goal->move_group_name);
    if (it != move_group_cache_.end()) {
      move_group = it->second;
      RCLCPP_INFO(get_logger(), "Using cached move group: %s", goal->move_group_name.c_str());
    } else {
      RCLCPP_INFO(get_logger(), "Creating new move group: %s", goal->move_group_name.c_str());
      try {
        move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
          shared_from_this(), goal->move_group_name);

        // Configure move group
        move_group->setPlanningTime(config_.planning_time);
        move_group->setNumPlanningAttempts(config_.max_planning_attempts);
        move_group->setMaxVelocityScalingFactor(config_.velocity_scaling);
        move_group->setMaxAccelerationScalingFactor(config_.acceleration_scaling);
        move_group->setGoalTolerance(config_.goal_tolerance);

        move_group_cache_[goal->move_group_name] = move_group;
      } catch (const std::exception & e) {
        RCLCPP_ERROR(get_logger(), "Failed to create move group: %s", e.what());
        result->success = false;
        result->message = "Failed to create move group: " + std::string(e.what());
        goal_handle->abort(result);
        return;
      }
    }
  }

  bool success = false;

  // Execute based on mode
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
    RCLCPP_INFO(get_logger(), "Goal succeeded in %.2f seconds", result->execution_time_sec);
    goal_handle->succeed(result);
  } else {
    result->message = "Motion failed";
    RCLCPP_ERROR(get_logger(), "Goal failed after %.2f seconds", result->execution_time_sec);
    goal_handle->abort(result);
  }
}

bool MoveToPoseActionServer::execute_joint_space_motion(
  const std::shared_ptr<GoalHandleMoveToPose> goal_handle,
  const std::shared_ptr<const MoveToPose::Goal> goal,
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group)
{
  RCLCPP_INFO(get_logger(), "Executing joint space motion");

  // Convert parallel arrays to std::map
  std::map<std::string, double> joint_targets;
  for (size_t i = 0; i < goal->joint_names.size(); ++i) {
    joint_targets[goal->joint_names[i]] = goal->joint_positions[i];
  }

  // Set joint value target
  move_group->setJointValueTarget(joint_targets);

  // Plan
  publish_feedback(goal_handle, "Planning joint space motion", 10.0);
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool plan_success = (move_group->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

  if (!plan_success) {
    RCLCPP_ERROR(get_logger(), "Planning failed");
    return false;
  }

  RCLCPP_INFO(get_logger(), "Planning successful");

  // Check for cancellation
  if (goal_handle->is_canceling()) {
    RCLCPP_WARN(get_logger(), "Goal cancelled during planning");
    return false;
  }

  // Execute
  publish_feedback(goal_handle, "Executing joint space motion", 50.0);
  auto execute_result = move_group->execute(plan);

  if (execute_result == moveit::core::MoveItErrorCode::SUCCESS) {
    publish_feedback(goal_handle, "Joint space motion complete", 100.0);
    return true;
  } else {
    RCLCPP_ERROR(get_logger(), "Execution failed");
    return false;
  }
}

bool MoveToPoseActionServer::execute_cartesian_space_motion(
  const std::shared_ptr<GoalHandleMoveToPose> goal_handle,
  const std::shared_ptr<const MoveToPose::Goal> goal,
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group)
{
  RCLCPP_INFO(get_logger(), "Executing Cartesian space motion");

  // Set pose target
  move_group->setPoseTarget(goal->cartesian_target);

  // Plan
  publish_feedback(goal_handle, "Planning Cartesian space motion", 10.0);
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool plan_success = (move_group->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

  if (!plan_success) {
    RCLCPP_ERROR(get_logger(), "Planning failed");
    return false;
  }

  RCLCPP_INFO(get_logger(), "Planning successful");

  // Check for cancellation
  if (goal_handle->is_canceling()) {
    RCLCPP_WARN(get_logger(), "Goal cancelled during planning");
    return false;
  }

  // Execute
  publish_feedback(goal_handle, "Executing Cartesian space motion", 50.0);
  auto execute_result = move_group->execute(plan);

  if (execute_result == moveit::core::MoveItErrorCode::SUCCESS) {
    publish_feedback(goal_handle, "Cartesian space motion complete", 100.0);
    return true;
  } else {
    RCLCPP_ERROR(get_logger(), "Execution failed");
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

}  // namespace hold_and_weld
