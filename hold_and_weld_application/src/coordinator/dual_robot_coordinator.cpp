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

#include "hold_and_weld_application/coordinator/dual_robot_coordinator.hpp"

namespace hold_and_weld
{

DualRobotCoordinator::DualRobotCoordinator(const rclcpp::NodeOptions & options)
: Node("dual_robot_coordinator", options)
{
    // Create gripper action client
  gripper_client_ = rclcpp_action::create_client<TriggerGripper>(
        this, "trigger_gripper");

  RCLCPP_INFO(get_logger(), "Waiting for gripper action server");
  if (!gripper_client_->wait_for_action_server(std::chrono::seconds(10))) {
    RCLCPP_ERROR(get_logger(), "Gripper action server not available!");
  } else {
    RCLCPP_INFO(get_logger(), "Connected to gripper action server");
  }

    // Create welder action client
  welder_client_ = rclcpp_action::create_client<TriggerWelder>(
        this, "trigger_welder");

  RCLCPP_INFO(get_logger(), "Waiting for welder action server");
  if (!welder_client_->wait_for_action_server(std::chrono::seconds(10))) {
    RCLCPP_ERROR(get_logger(), "Welder action server not available!");
  } else {
    RCLCPP_INFO(get_logger(), "Connected to welder action server");
  }
  init_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&DualRobotCoordinator::init_moveit, this)
  );

  RCLCPP_INFO(get_logger(), "Coordinator initialized, waiting for MoveIt setup...");
}

void DualRobotCoordinator::run()
{
  RCLCPP_INFO(get_logger(), "Waiting for MoveIt to initialize...");
  while (!moveit_ready_ && rclcpp::ok()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  RCLCPP_INFO(get_logger(), "MoveIt ready, starting sequence");

  RCLCPP_INFO(get_logger(), "Waiting for system to initialize");
  spin_for_duration(5.0);

  if (!move_welder_to_safety()) {
    RCLCPP_ERROR(get_logger(), "Welder failed to look up. Aborting.");
    return;
  }

  if (!execute_gripper_job()) {
    RCLCPP_ERROR(get_logger(), "Gripper job failed!");
    return;
  }

  if (!execute_welder_job()) {
    RCLCPP_ERROR(get_logger(), "Welder job failed!");
    return;
  }
}

void DualRobotCoordinator::init_moveit()
{
    // Cancel the timer so this only runs once
  init_timer_->cancel();

  welder_move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
        shared_from_this(), "robot2_gp25_welder_arm");

    // Configure Safety Settings
  welder_move_group_->setPlanningTime(5.0);
  welder_move_group_->setNumPlanningAttempts(10);
  welder_move_group_->setMaxVelocityScalingFactor(0.2);
  welder_move_group_->setMaxAccelerationScalingFactor(0.2);

  RCLCPP_INFO(get_logger(), "Coordinator MoveGroup Ready for Safety Moves");
  moveit_ready_ = true;
}

bool DualRobotCoordinator::execute_gripper_job()
{
  auto goal_msg = TriggerGripper::Goal();
  auto send_goal_options = rclcpp_action::Client<TriggerGripper>::SendGoalOptions();

  send_goal_options.feedback_callback =
    [this](GoalHandleTriggerGripper::SharedPtr,
    const std::shared_ptr<const TriggerGripper::Feedback> feedback) {
      RCLCPP_INFO(get_logger(), "Gripper: %s (%.1f%%)",
                        feedback->current_step.c_str(),
                        feedback->completion_percentage);
    };

  send_goal_options.result_callback =
    [this](const GoalHandleTriggerGripper::WrappedResult & result) {
      if (result.code == rclcpp_action::ResultCode::ABORTED) {
        RCLCPP_ERROR(get_logger(), "Gripper aborted: %s", result.result->message.c_str());
      } else if (result.code == rclcpp_action::ResultCode::CANCELED) {
        RCLCPP_WARN(get_logger(), "Gripper canceled");
      }
    };

  RCLCPP_INFO(get_logger(), "Triggering gripper job");
  auto goal_handle_future = gripper_client_->async_send_goal(goal_msg, send_goal_options);

  if (goal_handle_future.wait_for(std::chrono::seconds(10)) != std::future_status::ready) {
    RCLCPP_ERROR(get_logger(), "Failed to send gripper goal (timeout)");
    return false;
  }

  auto goal_handle = goal_handle_future.get();
  if (!goal_handle) {
    RCLCPP_ERROR(get_logger(), "Gripper goal was rejected");
    return false;
  }

  auto result_future = gripper_client_->async_get_result(goal_handle);

    // Wait for result
  if (result_future.wait_for(std::chrono::hours(1)) != std::future_status::ready) {
    RCLCPP_ERROR(get_logger(), "Gripper job timed out!");
    return false;
  }

  auto result = result_future.get();

  if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
    RCLCPP_ERROR(get_logger(), "Gripper job failed");
    return false;
  }

  RCLCPP_INFO(get_logger(), "Gripper job completed successfully\n");
  return true;
}

bool DualRobotCoordinator::move_welder_to_safety()
{
  std::map<std::string, double> safety_joints;
  safety_joints["robot2_joint_1_s"] = 0.02367382699844696;
  safety_joints["robot2_joint_2_l"] = -0.26463564871997514;
  safety_joints["robot2_joint_3_u"] = 0.6452811253697497;
  safety_joints["robot2_joint_4_r"] = 0.02990070287091831;
  safety_joints["robot2_joint_5_b"] = -0.9101291555788971;
  safety_joints["robot2_joint_6_t"] = 6.26474330075644;

  welder_move_group_->setJointValueTarget(safety_joints);
  welder_move_group_->setMaxVelocityScalingFactor(0.3);
  welder_move_group_->setPlanningTime(5.0);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (welder_move_group_->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

  if (success) {
    RCLCPP_INFO(get_logger(), "Planning successful, moving");
    welder_move_group_->execute(my_plan);
    return true;
  } else {
    RCLCPP_ERROR(get_logger(), "Planning failed!");
    return false;
  }
}

bool DualRobotCoordinator::execute_welder_job()
{
    // Create empty goal
  auto goal_msg = TriggerWelder::Goal();

    // Send goal with callbacks
  auto send_goal_options = rclcpp_action::Client<TriggerWelder>::SendGoalOptions();

  send_goal_options.feedback_callback =
    [this](GoalHandleTriggerWelder::SharedPtr,
    const std::shared_ptr<const TriggerWelder::Feedback> feedback) {
      RCLCPP_INFO(get_logger(), "Welder: %s (%.1f%%) [%d/%d points]",
                        feedback->current_step.c_str(),
                        feedback->completion_percentage,
                        feedback->current_point,
                        feedback->total_points);
    };

  send_goal_options.result_callback =
    [this](const GoalHandleTriggerWelder::WrappedResult & result) {
      if (result.code == rclcpp_action::ResultCode::ABORTED) {
        RCLCPP_ERROR(get_logger(), "Welder aborted: %s", result.result->message.c_str());
      } else if (result.code == rclcpp_action::ResultCode::CANCELED) {
        RCLCPP_WARN(get_logger(), "Welder canceled");
      }
    };

  RCLCPP_INFO(get_logger(), "Triggering welder job");
  auto goal_handle_future = welder_client_->async_send_goal(goal_msg, send_goal_options);

  if (goal_handle_future.wait_for(std::chrono::seconds(10)) != std::future_status::ready) {
    RCLCPP_ERROR(get_logger(), "Failed to send welder goal (Timeout)");
    return false;
  }

  auto goal_handle = goal_handle_future.get();
  if (!goal_handle) {
    RCLCPP_ERROR(get_logger(), "Welder goal was rejected");
    return false;
  }

  auto result_future = welder_client_->async_get_result(goal_handle);

  if (result_future.wait_for(std::chrono::hours(1)) != std::future_status::ready) {
    RCLCPP_ERROR(get_logger(), "Welder job timed out or got stuck!");
    return false;
  }

  auto result = result_future.get();

  if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
    RCLCPP_ERROR(get_logger(), "Welder job failed");
    return false;
  }

  RCLCPP_INFO(get_logger(), "Welder job completed successfully\n");
  return true;
}
void DualRobotCoordinator::spin_for_duration(double seconds)
{
  std::this_thread::sleep_for(std::chrono::duration<double>(seconds));
}

}  // namespace hold_and_weld
