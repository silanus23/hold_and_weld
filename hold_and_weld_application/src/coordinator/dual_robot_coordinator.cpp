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
#include <lifecycle_msgs/msg/state.hpp>

namespace hold_and_weld
{

DualRobotCoordinator::DualRobotCoordinator(const rclcpp::NodeOptions & options)
: LifecycleNode("dual_robot_coordinator", options),
  logger_(rclcpp::get_logger("Application"))
{
  RCLCPP_INFO(logger_, "Dual Robot Coordinator constructed");

  this->declare_parameter("auto_start", true);
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
DualRobotCoordinator::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(logger_, "Configuring Dual Robot Coordinator");

  auto_start_ = this->get_parameter("auto_start").as_bool();
  RCLCPP_INFO(logger_, "Auto-start: %s", auto_start_ ? "enabled" : "disabled");

  // Create controller action clients for readiness checking
  robot1_controller_client_ = rclcpp_action::create_client<FollowJointTrajectory>(
    this->get_node_base_interface(),
    this->get_node_graph_interface(),
    this->get_node_logging_interface(),
    this->get_node_waitables_interface(),
    "/robot1_arm_controller/follow_joint_trajectory");

  robot2_controller_client_ = rclcpp_action::create_client<FollowJointTrajectory>(
    this->get_node_base_interface(),
    this->get_node_graph_interface(),
    this->get_node_logging_interface(),
    this->get_node_waitables_interface(),
    "/robot2_arm_controller/follow_joint_trajectory");

  gripper_controller_client_ = rclcpp_action::create_client<FollowJointTrajectory>(
    this->get_node_base_interface(),
    this->get_node_graph_interface(),
    this->get_node_logging_interface(),
    this->get_node_waitables_interface(),
    "/gripper_controller/follow_joint_trajectory");

  // Create application action clients
  gripper_client_ = rclcpp_action::create_client<TriggerGripper>(
    this->get_node_base_interface(),
    this->get_node_graph_interface(),
    this->get_node_logging_interface(),
    this->get_node_waitables_interface(),
    "trigger_gripper");

  welder_client_ = rclcpp_action::create_client<TriggerWelder>(
    this->get_node_base_interface(),
    this->get_node_graph_interface(),
    this->get_node_logging_interface(),
    this->get_node_waitables_interface(),
    "trigger_welder");

  constexpr int MAX_WAIT_SECONDS = 30;
  int wait_count = 0;

  RCLCPP_INFO(logger_, "Waiting for gripper action server");
  wait_count = 0;

  while (!gripper_client_->wait_for_action_server(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(logger_, "Interrupted while waiting for gripper action server");
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
    }

    wait_count++;

    if (wait_count >= MAX_WAIT_SECONDS) {
      RCLCPP_ERROR(
        logger_,
        "Gripper action server not available after %d seconds. Is gripper_server running?",
        MAX_WAIT_SECONDS);
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
    } else if (wait_count % 10 == 0) {
      RCLCPP_WARN(
        logger_,
        "Still waiting for gripper action server (%d/%d seconds)",
        wait_count,
        MAX_WAIT_SECONDS);
    } else if (wait_count % 5 == 0) {
      RCLCPP_INFO(
        logger_,
        "Waiting for gripper action server (%d/%d seconds)",
        wait_count,
        MAX_WAIT_SECONDS);
    }
  }
  RCLCPP_INFO(logger_, "Connected to gripper action server");

  RCLCPP_INFO(logger_, "Waiting for welder action server");
  wait_count = 0;

  while (!welder_client_->wait_for_action_server(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(logger_, "Interrupted while waiting for welder action server");
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
    }

    wait_count++;

    if (wait_count >= MAX_WAIT_SECONDS) {
      RCLCPP_ERROR(
        logger_,
        "Welder action server not available after %d seconds. Is welder_server running?",
        MAX_WAIT_SECONDS);
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
    } else if (wait_count % 10 == 0) {
      RCLCPP_WARN(
        logger_,
        "Still waiting for welder action server (%d/%d seconds)",
        wait_count,
        MAX_WAIT_SECONDS);
    } else if (wait_count % 5 == 0) {
      RCLCPP_INFO(
        logger_,
        "Waiting for welder action server (%d/%d seconds)",
        wait_count,
        MAX_WAIT_SECONDS);
    }
  }
  RCLCPP_INFO(logger_, "Connected to welder action server");

  trigger_service_ = this->create_service<std_srvs::srv::Trigger>(
    "~/trigger_sequence",
    std::bind(&DualRobotCoordinator::handle_trigger_service, this,
              std::placeholders::_1, std::placeholders::_2));

  RCLCPP_INFO(logger_, "Dual Robot Coordinator configured");
  RCLCPP_INFO(logger_, "Manual trigger service available at: ~/trigger_sequence");

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
DualRobotCoordinator::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(logger_, "Activating Dual Robot Coordinator");

  is_active_ = true;
  system_ready_ = false;
  sequence_started_ = false;
  sequence_running_ = false;

  // Start readiness monitoring timer (checks every 500ms)
  readiness_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(500),
    std::bind(&DualRobotCoordinator::check_readiness, this));

  RCLCPP_INFO(logger_, "Dual Robot Coordinator activated");
  RCLCPP_INFO(logger_, "Starting readiness monitoring...");

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
DualRobotCoordinator::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(logger_, "Deactivating Dual Robot Coordinator");

  is_active_ = false;
  sequence_running_ = false;

  if (readiness_timer_) {
    readiness_timer_->cancel();
    readiness_timer_.reset();
  }

  RCLCPP_INFO(logger_, "Dual Robot Coordinator deactivated");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
DualRobotCoordinator::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(logger_, "Cleaning up Dual Robot Coordinator");

  readiness_timer_.reset();
  robot1_controller_client_.reset();
  robot2_controller_client_.reset();
  gripper_controller_client_.reset();
  gripper_client_.reset();
  welder_client_.reset();
  trigger_service_.reset();

  RCLCPP_INFO(logger_, "Dual Robot Coordinator cleaned up");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
DualRobotCoordinator::on_shutdown(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(logger_, "Shutting down Dual Robot Coordinator");

  if (readiness_timer_) {readiness_timer_.reset();}
  if (robot1_controller_client_) {robot1_controller_client_.reset();}
  if (robot2_controller_client_) {robot2_controller_client_.reset();}
  if (gripper_controller_client_) {gripper_controller_client_.reset();}
  if (gripper_client_) {gripper_client_.reset();}
  if (welder_client_) {welder_client_.reset();}
  if (trigger_service_) {trigger_service_.reset();}

  RCLCPP_INFO(logger_, "Dual Robot Coordinator shutdown complete");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

void DualRobotCoordinator::check_readiness()
{
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!is_active_ || sequence_started_) {
      return;
    }
  }

  bool controllers_ready = check_controllers_ready();

  bool should_execute = false;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (controllers_ready && !system_ready_) {
      system_ready_ = true;
      RCLCPP_INFO(logger_, "All controllers are ready!");

      if (auto_start_) {
        RCLCPP_INFO(logger_, "Auto-start enabled, beginning coordinated sequence");
        should_execute = true;
      } else {
        RCLCPP_INFO(logger_, "System ready! Waiting for manual trigger");
        RCLCPP_INFO(logger_,
            "Call service: ros2 service call ~/trigger_sequence std_srvs/srv/Trigger");
      }
    }
  }

  if (should_execute) {
    execute_sequence();
  }
}

bool DualRobotCoordinator::check_controllers_ready()
{
  // Use non-blocking checks with short timeout
  bool robot1_ready =
    robot1_controller_client_->wait_for_action_server(std::chrono::milliseconds(100));
  bool robot2_ready =
    robot2_controller_client_->wait_for_action_server(std::chrono::milliseconds(100));
  bool gripper_ready =
    gripper_controller_client_->wait_for_action_server(std::chrono::milliseconds(100));

  if (!robot1_ready) {
    RCLCPP_DEBUG(logger_, "Waiting for robot1_arm_controller");
  }
  if (!robot2_ready) {
    RCLCPP_DEBUG(logger_, "Waiting for robot2_arm_controller");
  }
  if (!gripper_ready) {
    RCLCPP_DEBUG(logger_, "Waiting for gripper_controller");
  }

  return robot1_ready && robot2_ready && gripper_ready;
}

void DualRobotCoordinator::handle_trigger_service(
  const std::shared_ptr<std_srvs::srv::Trigger::Request>/*request*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  {
    std::lock_guard<std::mutex> lock(state_mutex_);

    if (!is_active_) {
      response->success = false;
      response->message = "Coordinator is not active";
      RCLCPP_WARN(logger_, "Trigger rejected: coordinator not active");
      return;
    }

    if (!system_ready_) {
      response->success = false;
      response->message = "System is not ready yet (controllers not available)";
      RCLCPP_WARN(logger_, "Trigger rejected: system not ready");
      return;
    }

    if (sequence_running_) {
      response->success = false;
      response->message = "Sequence is already running";
      RCLCPP_WARN(logger_, "Trigger rejected: sequence already running");
      return;
    }

    response->success = true;
    response->message = "Sequence triggered successfully";
    RCLCPP_INFO(logger_, "Manual trigger received, starting sequence...");
  }  // Lock automatically released here

  execute_sequence();
}

void DualRobotCoordinator::execute_sequence()
{
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (sequence_started_) {
      RCLCPP_WARN(logger_, "Sequence already started, ignoring duplicate call");
      return;
    }

    sequence_started_ = true;
    sequence_running_ = true;
  }

  // Cancel readiness monitoring — sequence is now in progress
  if (readiness_timer_) {
    readiness_timer_->cancel();
  }

  RCLCPP_INFO(logger_, "Starting dual-robot coordinated sequence");

  step_gripper_job();
}

void DualRobotCoordinator::step_gripper_job()
{
  RCLCPP_INFO(logger_, "[Step 2/3] Executing gripper job");

  auto goal_msg = TriggerGripper::Goal();
  auto send_goal_options = rclcpp_action::Client<TriggerGripper>::SendGoalOptions();

  send_goal_options.feedback_callback = std::bind(
    &DualRobotCoordinator::gripper_feedback_callback, this,
    std::placeholders::_1, std::placeholders::_2);

  send_goal_options.result_callback = std::bind(
    &DualRobotCoordinator::gripper_result_callback, this,
    std::placeholders::_1);

  // Future is intentionally discarded — sequence continuation is handled via result_callback
  gripper_client_->async_send_goal(goal_msg, send_goal_options);
}

void DualRobotCoordinator::gripper_feedback_callback(
  GoalHandleTriggerGripper::SharedPtr,
  const std::shared_ptr<const TriggerGripper::Feedback> feedback)
{
  RCLCPP_INFO(logger_, "  [Gripper] %s (%.1f%%)",
              feedback->current_step.c_str(),
              feedback->completion_percentage);
}

void DualRobotCoordinator::gripper_result_callback(
  const GoalHandleTriggerGripper::WrappedResult & result)
{
  if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
    RCLCPP_INFO(logger_, "Gripper job completed successfully");
    step_welder_job();
    return;
  }

  if (result.code == rclcpp_action::ResultCode::ABORTED) {
    RCLCPP_ERROR(logger_, "Gripper job aborted: %s", result.result->message.c_str());
  } else if (result.code == rclcpp_action::ResultCode::CANCELED) {
    RCLCPP_WARN(logger_, "Gripper job canceled");
  } else {
    RCLCPP_ERROR(logger_, "Gripper job failed with unknown result code");
  }

  RCLCPP_ERROR(logger_, "Sequence aborted");
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    sequence_running_ = false;
  }
}

void DualRobotCoordinator::step_welder_job()
{
  RCLCPP_INFO(logger_, "[Step 3/3] Executing welder job");

  auto goal_msg = TriggerWelder::Goal();
  auto send_goal_options = rclcpp_action::Client<TriggerWelder>::SendGoalOptions();

  send_goal_options.feedback_callback = std::bind(
    &DualRobotCoordinator::welder_feedback_callback, this,
    std::placeholders::_1, std::placeholders::_2);

  send_goal_options.result_callback = std::bind(
    &DualRobotCoordinator::welder_result_callback, this,
    std::placeholders::_1);

  // Future is intentionally discarded — sequence completion is handled via result_callback
  welder_client_->async_send_goal(goal_msg, send_goal_options);
}

void DualRobotCoordinator::welder_feedback_callback(
  GoalHandleTriggerWelder::SharedPtr,
  const std::shared_ptr<const TriggerWelder::Feedback> feedback)
{
  RCLCPP_INFO(logger_, "  [Welder] %s (%.1f%%) [%d/%d points]",
              feedback->current_step.c_str(),
              feedback->completion_percentage,
              feedback->current_point,
              feedback->total_points);
}

void DualRobotCoordinator::welder_result_callback(
  const GoalHandleTriggerWelder::WrappedResult & result)
{
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    sequence_running_ = false;
  }

  if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
    RCLCPP_INFO(logger_, "Welder job completed successfully");
    RCLCPP_INFO(logger_, "Dual-robot sequence completed successfully!");
  } else if (result.code == rclcpp_action::ResultCode::ABORTED) {
    RCLCPP_ERROR(logger_, "Welder job aborted: %s", result.result->message.c_str());
    RCLCPP_ERROR(logger_, "Sequence failed");
  } else if (result.code == rclcpp_action::ResultCode::CANCELED) {
    RCLCPP_WARN(logger_, "Welder job canceled");
    RCLCPP_WARN(logger_, "Sequence aborted");
  } else {
    RCLCPP_ERROR(logger_, "Welder job failed with unknown result code");
    RCLCPP_ERROR(logger_, "Sequence failed");
  }
}

}  // namespace hold_and_weld
