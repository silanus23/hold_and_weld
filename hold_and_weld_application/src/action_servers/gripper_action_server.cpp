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

#include "hold_and_weld_application/action_servers/gripper_action_server.hpp"

#include <yaml-cpp/yaml.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>


namespace hold_and_weld
{

GripperActionServer::GripperActionServer(const rclcpp::NodeOptions & options)
: Node("gripper_action_server", options)
{
  // Build default YAML path
  std::string default_yaml =
    ament_index_cpp::get_package_share_directory("hold_and_weld_application") +
    "/config/tasks/pick_place_targets.yaml";

  // Declare parameters
  declare_parameter("arm_group_name", "robot1_gp25_arm");
  declare_parameter("positions_yaml", default_yaml);
  declare_parameter("gripper_joint_names", std::vector<std::string>{
      "robot1_left_finger_joint", "robot1_right_finger_joint"});

  arm_group_name_ = get_parameter("arm_group_name").as_string();
  gripper_joint_names_ = get_parameter("gripper_joint_names").as_string_array();

  std::string yaml_path = get_parameter("positions_yaml").as_string();

RCLCPP_INFO(get_logger(), "Gripper Action Server starting");
  RCLCPP_INFO(get_logger(), "  Arm group: %s", arm_group_name_.c_str());

  // Initialize gripper action client
  gripper_action_client_ = rclcpp_action::create_client<FollowJointTrajectory>(
        this, "/gripper_controller/follow_joint_trajectory");

  // Initialize attached collision object publisher
  attached_collision_pub_ = create_publisher<moveit_msgs::msg::AttachedCollisionObject>(
        "/attached_collision_object", 10);

  // Load job from YAML
  load_job_from_yaml(yaml_path);

  // Delay MoveIt initialization
  init_timer_ = create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&GripperActionServer::initialize_moveit, this));

  planning_scene_client_ = create_client<moveit_msgs::srv::ApplyPlanningScene>(
        "/apply_planning_scene");

  get_planning_scene_client_ = create_client<moveit_msgs::srv::GetPlanningScene>
        ("/get_planning_scene");
}

void GripperActionServer::initialize_moveit()
{
  init_timer_->cancel();

  using namespace std::placeholders;

  RCLCPP_INFO(get_logger(), "Initializing MoveIt");

  try {
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            shared_from_this(), arm_group_name_);

    move_group_->setPlanningTime(10.0);
    move_group_->setNumPlanningAttempts(10);
    move_group_->setMaxVelocityScalingFactor(0.3);
    move_group_->setMaxAccelerationScalingFactor(0.3);

    RCLCPP_INFO(get_logger(), "MoveIt initialized successfully");
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Failed to initialize MoveIt: %s", e.what());
    return;
  }

  // Initialize action server
  action_server_ = rclcpp_action::create_server<TriggerGripper>(
    this,
    "trigger_gripper",
    // Goal Callback
    [this](const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const TriggerGripper::Goal> goal) {
      return this->handle_goal(uuid, goal);
    },
    // Cancel Callback
    [this](const std::shared_ptr<GoalHandleTriggerGripper> goal_handle) {
      return this->handle_cancel(goal_handle);
    },
    // Accepted Callback
    [this](const std::shared_ptr<GoalHandleTriggerGripper> goal_handle) {
      this->handle_accepted(goal_handle);
    }
  );

  initialized_ = true;
  RCLCPP_INFO(get_logger(), "Gripper Action Server ready!");

  std::lock_guard<std::mutex> lock(config_mutex_);
  if (job_loaded_) {
    RCLCPP_INFO(get_logger(), "  Job loaded for target: %s", job_.target_id.c_str());
  } else {
    RCLCPP_WARN(get_logger(), "  No job loaded!");
  }
}

void GripperActionServer::normalize_quaternion(geometry_msgs::msg::Quaternion & q)
{
  double norm = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
  if (norm < 1e-10) {
    RCLCPP_WARN(get_logger(), "Invalid quaternion (near-zero norm), setting to identity");
    q.x = 0.0;
    q.y = 0.0;
    q.z = 0.0;
    q.w = 1.0;
    return;
  }
  q.x /= norm;
  q.y /= norm;
  q.z /= norm;
  q.w /= norm;
}

void GripperActionServer::load_job_from_yaml(const std::string & yaml_path)
{
  std::lock_guard<std::mutex> lock(config_mutex_);

  try {
    RCLCPP_INFO(get_logger(), "Loading job from: %s", yaml_path.c_str());

    YAML::Node config = YAML::LoadFile(yaml_path);

    // Helper lambda to load a pose from YAML node
    auto load_pose = [this](const YAML::Node & pose_node, geometry_msgs::msg::Pose & pose) -> bool {
        if (!pose_node || !pose_node["position"] || !pose_node["orientation"]) {
          return false;
        }

        pose.position.x = pose_node["position"]["x"].as<double>();
        pose.position.y = pose_node["position"]["y"].as<double>();
        pose.position.z = pose_node["position"]["z"].as<double>();
        pose.orientation.x = pose_node["orientation"]["x"].as<double>();
        pose.orientation.y = pose_node["orientation"]["y"].as<double>();
        pose.orientation.z = pose_node["orientation"]["z"].as<double>();
        pose.orientation.w = pose_node["orientation"]["w"].as<double>();
        normalize_quaternion(pose.orientation);
        return true;
      };

    // Load first target from targets array
    if (config["targets"] && config["targets"].size() > 0) {
      const auto & target = config["targets"][0];      // Loading the first one for now

      job_.target_id = target["target_id"].as<std::string>("unknown_part");

      bool success = true;
      success &= load_pose(target["approach_pose"], job_.approach_pose);
      success &= load_pose(target["pick_pose"], job_.pick_pose);
      success &= load_pose(target["retract_pose"], job_.retract_pose);
      success &= load_pose(target["place_pose"], job_.place_pose);

      if (success) {
        job_loaded_ = true;
        RCLCPP_INFO(get_logger(), "Successfully loaded job for: %s", job_.target_id.c_str());
      } else {
        RCLCPP_ERROR(get_logger(), "Failed to parse one or more poses for target: %s",
            job_.target_id.c_str());
      }
    } else {
      RCLCPP_ERROR(get_logger(), "No targets found in YAML array!");
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Error loading YAML: %s", e.what());
  }
}

rclcpp_action::GoalResponse GripperActionServer::handle_goal(
  [[maybe_unused]] const rclcpp_action::GoalUUID & uuid,
  [[maybe_unused]] std::shared_ptr<const TriggerGripper::Goal> goal)
{
  if (!initialized_) {
    RCLCPP_ERROR(get_logger(), "Server not initialized yet");
    return rclcpp_action::GoalResponse::REJECT;
  }

  std::lock_guard<std::mutex> lock(config_mutex_);
  if (!job_loaded_) {
    RCLCPP_ERROR(get_logger(), "No job loaded");
    return rclcpp_action::GoalResponse::REJECT;
  }

  RCLCPP_INFO(get_logger(), "Received gripper trigger for target: %s", job_.target_id.c_str());
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse GripperActionServer::handle_cancel(
  [[maybe_unused]] const std::shared_ptr<GoalHandleTriggerGripper> goal_handle)
{
  RCLCPP_INFO(get_logger(), "Received cancel request");
  move_group_->stop();
  return rclcpp_action::CancelResponse::ACCEPT;
}

void GripperActionServer::handle_accepted(
  const std::shared_ptr<GoalHandleTriggerGripper> goal_handle)
{
  std::lock_guard<std::mutex> lock(execution_mutex_);

  // Wait for previous execution to complete if any
  if (execution_thread_ && execution_thread_->joinable()) {
    execution_thread_->join();
  }

  execution_thread_ = std::make_shared<std::thread>(
        &GripperActionServer::execute_job, this, goal_handle);
}

bool GripperActionServer::wait_for_planning_scene_update(int millis)
{
  auto start = this->now();
  auto duration = rclcpp::Duration::from_seconds(millis / 1000.0);

  while ((this->now() - start) < duration) {
    if (!rclcpp::ok()) {
      return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  return true;
}

void GripperActionServer::execute_job(const std::shared_ptr<GoalHandleTriggerGripper> goal_handle)
{
  auto feedback = std::make_shared<TriggerGripper::Feedback>();
  auto result = std::make_shared<TriggerGripper::Result>();

  // Copy job data with lock
  GripperJob job;
  {
    std::lock_guard<std::mutex> lock(config_mutex_);
    job = job_;
  }

  int current_step_idx = 0;
  const int total_steps = 6;   // Explicitly defined total for percentage calc

  auto update_feedback = [&](const std::string & step_name) {
      feedback->current_step = step_name;
      feedback->completion_percentage = (static_cast<float>(current_step_idx) / total_steps) *
        100.0f;
      goal_handle->publish_feedback(feedback);
      RCLCPP_INFO(get_logger(), "[Step %d/%d] %s", current_step_idx + 1, total_steps,
        step_name.c_str());
      current_step_idx++;
    };

  // 1. Open
  update_feedback("opening_gripper");
  if (!set_gripper_position(open_position_)) {
    result->success = false;
    result->message = "Failed to open gripper";
    goal_handle->abort(result);
    return;
  }

  // Check Cancel
  if (goal_handle->is_canceling()) {
    result->success = false;
    result->message = "Canceled";
    goal_handle->canceled(result);
    return;
  }

  // 2. Approach
  update_feedback("moving_to_approach");
  if (!move_to_pose(job.approach_pose, "approach")) {
    result->success = false;
    result->message = "Failed to move to approach";
    goal_handle->abort(result);
    return;
  }

  // 3. Pick Pose
  update_feedback("moving_to_pick");
  if (!move_to_pose(job.pick_pose, "pick")) {
    result->success = false;
    result->message = "Failed to move to pick";
    goal_handle->abort(result);
    return;
  }

  // 4. Close & Attach
  update_feedback("closing_gripper");
  if (!set_gripper_position(close_position_)) {
    result->success = false;
    result->message = "Failed to close gripper";
    goal_handle->abort(result);
    return;
  }
  attach_object(job.target_id);

  // 5. Retract
  update_feedback("moving_to_retract");
  if (!move_to_pose(job.retract_pose, "retract")) {
    result->success = false;
    result->message = "Failed to move to retract";
    goal_handle->abort(result);
    return;
  }

  // Updating collision matrix
  RCLCPP_INFO(get_logger(), "Allowing collision between cube and workpiece");
  if (!allow_collision_for_placement()) {
    RCLCPP_WARN(get_logger(), "Failed to update collision matrix, continuing anyway");
  }

  // 6. Place
  update_feedback("moving_to_place");
  if (!move_to_pose(job.place_pose, "place")) {
    result->success = false;
    result->message = "Failed to move to place";
    goal_handle->abort(result);
    return;
  }

  // Success
  feedback->current_step = "completed";
  feedback->completion_percentage = 100.0f;
  goal_handle->publish_feedback(feedback);

  result->success = true;
  result->message = "Gripper job completed";
  result->positions_executed = current_step_idx;
  goal_handle->succeed(result);
}

bool GripperActionServer::set_gripper_position(double position)
{
  RCLCPP_INFO(get_logger(), "[Gripper] Moving to %.3f", position);

  if (!gripper_action_client_->wait_for_action_server(
            std::chrono::seconds(timing::ACTION_SERVER_TIMEOUT_SEC)))
  {
    RCLCPP_ERROR(get_logger(), "[Gripper] Action server not available");
    return false;
  }

  auto goal_msg = FollowJointTrajectory::Goal();
  goal_msg.trajectory.joint_names = gripper_joint_names_;

  trajectory_msgs::msg::JointTrajectoryPoint point;
  point.positions = {position, position};
  point.time_from_start = rclcpp::Duration::from_seconds(timing::GRIPPER_MOTION_DURATION_SEC);
  goal_msg.trajectory.points.push_back(point);

  auto future = gripper_action_client_->async_send_goal(goal_msg);

  auto status = future.wait_for(std::chrono::seconds(timing::ACTION_SERVER_TIMEOUT_SEC));
  if (status != std::future_status::ready) {
    RCLCPP_ERROR(get_logger(), "[Gripper] Timeout sending goal");
    return false;
  }

  auto goal_handle = future.get();
  if (!goal_handle) {
    RCLCPP_ERROR(get_logger(), "[Gripper] Goal was rejected");
    return false;
  }

  auto result_future = gripper_action_client_->async_get_result(goal_handle);

  auto result_status =
    result_future.wait_for(std::chrono::seconds(timing::GRIPPER_RESULT_TIMEOUT_SEC));
  if (result_status != std::future_status::ready) {
    RCLCPP_ERROR(get_logger(), "[Gripper] Timeout waiting for result");
    return false;
  }

  auto result = result_future.get();
  if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
    RCLCPP_INFO(get_logger(), "[Gripper] Done.");
    return true;
  }

  RCLCPP_ERROR(get_logger(), "[Gripper] Failed to reach position");
  return false;
}

bool GripperActionServer::move_to_pose(
  const geometry_msgs::msg::Pose & pose,
  const std::string & step_name)
{
  RCLCPP_INFO(get_logger(), "[%s] Planning to (%.3f, %.3f, %.3f)",
               step_name.c_str(),
               pose.position.x,
               pose.position.y,
               pose.position.z);

  move_group_->setPoseTarget(pose);

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success = (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

  if (success) {
    RCLCPP_INFO(get_logger(), "[%s] Executing", step_name.c_str());
    auto exec_result = move_group_->execute(plan);

    if (exec_result != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(get_logger(), "[%s] Execution failed!", step_name.c_str());
      return false;
    }

    wait_for_planning_scene_update(timing::MOTION_SETTLE_TIME_MS);
    RCLCPP_INFO(get_logger(), "[%s] Done.", step_name.c_str());
    return true;
  } else {
    RCLCPP_ERROR(get_logger(), "[%s] Planning failed!", step_name.c_str());
    return false;
  }
}

bool GripperActionServer::attach_object(const std::string & object_id)
{
  RCLCPP_INFO(get_logger(), "[Attach] Attaching '%s' to '%s'", object_id.c_str(),
      attach_link_.c_str());

  moveit_msgs::msg::AttachedCollisionObject attached_object;
  attached_object.link_name = attach_link_;
  attached_object.object.id = object_id;
  attached_object.object.operation = moveit_msgs::msg::CollisionObject::ADD;
  attached_object.touch_links = touch_links_;

  attached_collision_pub_->publish(attached_object);
  wait_for_planning_scene_update(timing::ATTACH_SETTLE_TIME_MS);

  RCLCPP_INFO(get_logger(), "[Attach] Object attached.");
  return true;
}

bool GripperActionServer::detach_object(const std::string & object_id)
{
  RCLCPP_INFO(get_logger(), "[Detach] Detaching '%s'", object_id.c_str());

  moveit_msgs::msg::AttachedCollisionObject detach_object;
  detach_object.object.id = object_id;
  detach_object.object.operation = moveit_msgs::msg::CollisionObject::REMOVE;

  attached_collision_pub_->publish(detach_object);
  wait_for_planning_scene_update(timing::DETACH_SETTLE_TIME_MS);

  RCLCPP_INFO(get_logger(), "[Detach] Object detached.");
  return true;
}

bool GripperActionServer::allow_collision_for_placement()
{
  auto get_request = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
  get_request->components.components = moveit_msgs::msg::PlanningSceneComponents::ALLOWED_COLLISION_MATRIX;

  if (!get_planning_scene_client_->wait_for_service(std::chrono::seconds(2))) {
    RCLCPP_ERROR(get_logger(), "Get Planning Scene service not available");
    return false;
  }

  auto get_future = get_planning_scene_client_->async_send_request(get_request);
  if (get_future.wait_for(std::chrono::seconds(2)) != std::future_status::ready) {
    RCLCPP_ERROR(get_logger(), "Timeout getting planning scene");
    return false;
  }

  auto current_scene = get_future.get();
  auto & acm = current_scene->scene.allowed_collision_matrix;

  auto toggle_acm_bit = [&](const std::string & name1, const std::string & name2) {
    auto find_or_add = [&](const std::string & name) -> size_t {
      auto it = std::find(acm.entry_names.begin(), acm.entry_names.end(), name);
      if (it != acm.entry_names.end()) {
        return std::distance(acm.entry_names.begin(), it);
      }
      // If not found, add it and resize the values
      acm.entry_names.push_back(name);
      size_t new_idx = acm.entry_names.size() - 1;
      acm.entry_values.resize(acm.entry_names.size());
      for (auto & row : acm.entry_values) {
        row.enabled.resize(acm.entry_names.size(), false);
      }
      return new_idx;
    };

    size_t idx1 = find_or_add(name1);
    size_t idx2 = find_or_add(name2);

    // Set bit symmetrically
    acm.entry_values[idx1].enabled[idx2] = true;
    acm.entry_values[idx2].enabled[idx1] = true;
  };

  // Apply your specific "Hold and Weld" rule
  // Using job.target_id instead of hardcoded strings
  toggle_acm_bit(job_.target_id, "workpiece");

  auto apply_request = std::make_shared<moveit_msgs::srv::ApplyPlanningScene::Request>();
  apply_request->scene.allowed_collision_matrix = acm;
  apply_request->scene.is_diff = true;

  auto apply_future = planning_scene_client_->async_send_request(apply_request);
  if (apply_future.wait_for(std::chrono::seconds(2)) == std::future_status::ready) {
    return apply_future.get()->success;
  }

  return false;
}

}  // namespace hold_and_weld
