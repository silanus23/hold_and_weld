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
: LifecycleNode("dual_robot_coordinator", options)
{
  RCLCPP_INFO(get_logger(), "Dual Robot Coordinator constructed");

  this->declare_parameter("auto_start", true);
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
DualRobotCoordinator::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Configuring Dual Robot Coordinator");

  auto_start_ = this->get_parameter("auto_start").as_bool();
  RCLCPP_INFO(get_logger(), "Auto-start: %s", auto_start_ ? "enabled" : "disabled");

  // Wait for MoveIt to be available
  RCLCPP_INFO(get_logger(), "Waiting for MoveIt services");
  auto temp_node = std::make_shared<rclcpp::Node>("coordinator_service_waiter");

  auto cartesian_path_client = temp_node->create_client<moveit_msgs::srv::GetCartesianPath>(
    "/compute_cartesian_path");

  while (!cartesian_path_client->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(get_logger(), "Interrupted while waiting for MoveIt service");
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
    }
    RCLCPP_INFO(get_logger(), "Still waiting for MoveIt...");
  }
  RCLCPP_INFO(get_logger(), "MoveIt is available");

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

  RCLCPP_INFO(get_logger(), "Waiting for application action servers...");

  while (!gripper_client_->wait_for_action_server(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(get_logger(), "Interrupted while waiting for gripper action server");
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
    }
    RCLCPP_INFO(get_logger(), "Still waiting for gripper action server...");
  }
  RCLCPP_INFO(get_logger(), "Connected to gripper action server");

  while (!welder_client_->wait_for_action_server(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(get_logger(), "Interrupted while waiting for welder action server");
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
    }
    RCLCPP_INFO(get_logger(), "Still waiting for welder action server...");
  }
  RCLCPP_INFO(get_logger(), "Connected to welder action server");

  // Initialize MoveIt for welder safety positioning
  try {
    RCLCPP_INFO(get_logger(), "Initializing MoveIt for welder safety positioning");

    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);

    moveit_node_ = std::make_shared<rclcpp::Node>(
      "coordinator_moveit_internal",
      node_options);

    welder_move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
      moveit_node_, "robot2_gp25_welder_arm");

    welder_move_group_->setPlanningTime(5.0);
    welder_move_group_->setNumPlanningAttempts(10);
    welder_move_group_->setMaxVelocityScalingFactor(0.2);
    welder_move_group_->setMaxAccelerationScalingFactor(0.2);

    RCLCPP_INFO(get_logger(), "MoveIt initialized successfully for safety moves");
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Failed to initialize MoveIt: %s", e.what());
    welder_move_group_.reset();
    moveit_node_.reset();
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
  }

  trigger_service_ = this->create_service<std_srvs::srv::Trigger>(
    "~/trigger_sequence",
    std::bind(&DualRobotCoordinator::handle_trigger_service, this,
              std::placeholders::_1, std::placeholders::_2));

  RCLCPP_INFO(get_logger(), "Dual Robot Coordinator configured");
  RCLCPP_INFO(get_logger(), "Manual trigger service available at: ~/trigger_sequence");

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
DualRobotCoordinator::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Activating Dual Robot Coordinator");

  is_active_ = true;
  system_ready_ = false;
  sequence_started_ = false;
  sequence_running_ = false;

  // Start readiness monitoring timer (checks every 500ms)
  readiness_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(500),
    std::bind(&DualRobotCoordinator::check_readiness, this));

  RCLCPP_INFO(get_logger(), "Dual Robot Coordinator activated");
  RCLCPP_INFO(get_logger(), "Starting readiness monitoring...");

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
DualRobotCoordinator::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating Dual Robot Coordinator");

  is_active_ = false;
  sequence_running_ = false;

  if (readiness_timer_) {
    readiness_timer_->cancel();
    readiness_timer_.reset();
  }

  if (welder_move_group_) {
    welder_move_group_->stop();
  }

  RCLCPP_INFO(get_logger(), "Dual Robot Coordinator deactivated");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
DualRobotCoordinator::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Cleaning up Dual Robot Coordinator");

  readiness_timer_.reset();
  robot1_controller_client_.reset();
  robot2_controller_client_.reset();
  gripper_controller_client_.reset();
  gripper_client_.reset();
  welder_client_.reset();
  trigger_service_.reset();
  welder_move_group_.reset();
  moveit_node_.reset();

  RCLCPP_INFO(get_logger(), "Dual Robot Coordinator cleaned up");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
DualRobotCoordinator::on_shutdown(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Shutting down Dual Robot Coordinator");
  on_cleanup(get_current_state());
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

  std::lock_guard<std::mutex> lock(state_mutex_);
  if (controllers_ready && !system_ready_) {
    system_ready_ = true;
    RCLCPP_INFO(get_logger(), "All controllers are ready!");

    if (auto_start_) {
      RCLCPP_INFO(get_logger(), "Auto-start enabled, beginning coordinated sequence...");
      lock.~lock_guard();  // Unlock before calling execute_sequence
      execute_sequence();
    } else {
      RCLCPP_INFO(get_logger(), "System ready! Waiting for manual trigger...");
      RCLCPP_INFO(get_logger(),
          "Call service: ros2 service call ~/trigger_sequence std_srvs/srv/Trigger");
    }
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
    RCLCPP_DEBUG(get_logger(), "Waiting for robot1_arm_controller...");
  }
  if (!robot2_ready) {
    RCLCPP_DEBUG(get_logger(), "Waiting for robot2_arm_controller...");
  }
  if (!gripper_ready) {
    RCLCPP_DEBUG(get_logger(), "Waiting for gripper_controller...");
  }

  return robot1_ready && robot2_ready && gripper_ready;
}

void DualRobotCoordinator::handle_trigger_service(
  const std::shared_ptr<std_srvs::srv::Trigger::Request>/*request*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  std::lock_guard<std::mutex> lock(state_mutex_);

  if (!is_active_) {
    response->success = false;
    response->message = "Coordinator is not active";
    RCLCPP_WARN(get_logger(), "Trigger rejected: coordinator not active");
    return;
  }

  if (!system_ready_) {
    response->success = false;
    response->message = "System is not ready yet (controllers not available)";
    RCLCPP_WARN(get_logger(), "Trigger rejected: system not ready");
    return;
  }

  if (sequence_running_) {
    response->success = false;
    response->message = "Sequence is already running";
    RCLCPP_WARN(get_logger(), "Trigger rejected: sequence already running");
    return;
  }

  response->success = true;
  response->message = "Sequence triggered successfully";
  RCLCPP_INFO(get_logger(), "Manual trigger received, starting sequence...");

  lock.~lock_guard();  // Unlock before calling execute_sequence
  execute_sequence();
}

void DualRobotCoordinator::execute_sequence()
{
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (sequence_started_) {
      RCLCPP_WARN(get_logger(), "Sequence already started, ignoring duplicate call");
      return;
    }

    sequence_started_ = true;
    sequence_running_ = true;
  }

  // Cancel readiness monitoring
  if (readiness_timer_) {
    readiness_timer_->cancel();
  }

  RCLCPP_INFO(get_logger(), "Starting dual-robot coordinated sequence");

  step_welder_safety();
}

void DualRobotCoordinator::step_welder_safety()
{
  RCLCPP_INFO(get_logger(), "[Step 1/3] Moving welder to safety position");

  std::map<std::string, double> safety_joints;
  safety_joints["robot2_joint_1_s"] = 0.02367382699844696;
  safety_joints["robot2_joint_2_l"] = -0.26463564871997514;
  safety_joints["robot2_joint_3_u"] = 0.6452811253697497;
  safety_joints["robot2_joint_4_r"] = 0.02990070287091831;
  safety_joints["robot2_joint_5_b"] = -0.9101291555788971;
  safety_joints["robot2_joint_6_t"] = 6.26474330075644;

  try {
    std::lock_guard<std::mutex> lock(move_group_mutex_);

    welder_move_group_->setJointValueTarget(safety_joints);
    welder_move_group_->setMaxVelocityScalingFactor(0.3);
    welder_move_group_->setPlanningTime(5.0);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (welder_move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (!success) {
      RCLCPP_ERROR(get_logger(), "Failed to plan welder safety move!");
      RCLCPP_ERROR(get_logger(), "Sequence aborted");
      {
        std::lock_guard<std::mutex> state_lock(state_mutex_);
        sequence_running_ = false;
      }
      return;
    }

    RCLCPP_INFO(get_logger(), "Safety move planned, executing");
    auto result = welder_move_group_->execute(plan);

    if (result != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(get_logger(), "Failed to execute welder safety move!");
      RCLCPP_ERROR(get_logger(), "Sequence aborted");
      {
        std::lock_guard<std::mutex> state_lock(state_mutex_);
        sequence_running_ = false;
      }
      return;
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Exception during welder safety move: %s", e.what());
    RCLCPP_ERROR(get_logger(), "Sequence aborted");
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      sequence_running_ = false;
    }
    return;
  }

  RCLCPP_INFO(get_logger(), "Welder moved to safety position");

  step_gripper_job();
}

void DualRobotCoordinator::step_gripper_job()
{
  RCLCPP_INFO(get_logger(), "[Step 2/3] Executing gripper job");

  auto goal_msg = TriggerGripper::Goal();
  auto send_goal_options = rclcpp_action::Client<TriggerGripper>::SendGoalOptions();

  send_goal_options.feedback_callback = std::bind(
    &DualRobotCoordinator::gripper_feedback_callback, this,
    std::placeholders::_1, std::placeholders::_2);

  send_goal_options.result_callback = std::bind(
    &DualRobotCoordinator::gripper_result_callback, this,
    std::placeholders::_1);

  auto goal_handle_future = gripper_client_->async_send_goal(goal_msg, send_goal_options);

  // The result callback will handle proceeding to the next step
}

void DualRobotCoordinator::gripper_feedback_callback(
  GoalHandleTriggerGripper::SharedPtr,
  const std::shared_ptr<const TriggerGripper::Feedback> feedback)
{
  RCLCPP_INFO(get_logger(), "  [Gripper] %s (%.1f%%)",
              feedback->current_step.c_str(),
              feedback->completion_percentage);
}

void DualRobotCoordinator::gripper_result_callback(
  const GoalHandleTriggerGripper::WrappedResult & result)
{
  if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
    RCLCPP_INFO(get_logger(), "Gripper job completed successfully");
    step_welder_job();
  } else if (result.code == rclcpp_action::ResultCode::ABORTED) {
    RCLCPP_ERROR(get_logger(), "Gripper job aborted: %s", result.result->message.c_str());
    RCLCPP_ERROR(get_logger(), "Sequence aborted");
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      sequence_running_ = false;
    }
  } else if (result.code == rclcpp_action::ResultCode::CANCELED) {
    RCLCPP_WARN(get_logger(), "Gripper job canceled");
    RCLCPP_WARN(get_logger(), "Sequence aborted");
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      sequence_running_ = false;
    }
  } else {
    RCLCPP_ERROR(get_logger(), "Gripper job failed with unknown result code");
    RCLCPP_ERROR(get_logger(), "Sequence aborted");
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      sequence_running_ = false;
    }
  }
}

void DualRobotCoordinator::step_welder_job()
{
  RCLCPP_INFO(get_logger(), "[Step 3/3] Executing welder job");

  auto goal_msg = TriggerWelder::Goal();
  auto send_goal_options = rclcpp_action::Client<TriggerWelder>::SendGoalOptions();

  send_goal_options.feedback_callback = std::bind(
    &DualRobotCoordinator::welder_feedback_callback, this,
    std::placeholders::_1, std::placeholders::_2);

  send_goal_options.result_callback = std::bind(
    &DualRobotCoordinator::welder_result_callback, this,
    std::placeholders::_1);

  auto goal_handle_future = welder_client_->async_send_goal(goal_msg, send_goal_options);

  // The result callback will handle sequence completion
}

void DualRobotCoordinator::welder_feedback_callback(
  GoalHandleTriggerWelder::SharedPtr,
  const std::shared_ptr<const TriggerWelder::Feedback> feedback)
{
  RCLCPP_INFO(get_logger(), "  [Welder] %s (%.1f%%) [%d/%d points]",
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
    RCLCPP_INFO(get_logger(), "Welder job completed successfully");
    RCLCPP_INFO(get_logger(), "Dual-robot sequence completed successfully!");
  } else if (result.code == rclcpp_action::ResultCode::ABORTED) {
    RCLCPP_ERROR(get_logger(), "Welder job aborted: %s", result.result->message.c_str());
    RCLCPP_ERROR(get_logger(), "Sequence failed");
  } else if (result.code == rclcpp_action::ResultCode::CANCELED) {
    RCLCPP_WARN(get_logger(), "Welder job canceled");
    RCLCPP_WARN(get_logger(), "Sequence aborted");
  } else {
    RCLCPP_ERROR(get_logger(), "Welder job failed with unknown result code");
    RCLCPP_ERROR(get_logger(), "Sequence failed");
  }
}

}  // namespace hold_and_weld
