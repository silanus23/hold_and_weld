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

#include "hold_and_weld_application/action_servers/welder_action_server.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <filesystem>
#include <fstream>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <nlohmann/json.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <moveit_msgs/srv/get_cartesian_path.hpp>
#include <controller_manager_msgs/srv/list_controllers.hpp>


namespace hold_and_weld
{

WelderActionServer::WelderActionServer(const rclcpp::NodeOptions & options)
: LifecycleNode("welder_action_server", options)
{
  RCLCPP_INFO(get_logger(), "Welder Action Server constructed");
}

WelderActionServer::~WelderActionServer()
{
  shutdown_worker();
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
WelderActionServer::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Configuring Welder Action Server");

  // Wait for MoveIt and controllers to be available
  RCLCPP_INFO(get_logger(), "Waiting for required services to become available...");
  auto temp_node = std::make_shared<rclcpp::Node>("welder_service_waiter");

  // Wait for compute_cartesian_path service (indicates MoveIt is ready)
  auto cartesian_path_client = temp_node->create_client<moveit_msgs::srv::GetCartesianPath>(
    "/compute_cartesian_path");
  RCLCPP_INFO(get_logger(), "Waiting for MoveIt compute_cartesian_path service...");
  while (!cartesian_path_client->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(get_logger(), "Interrupted while waiting for MoveIt service");
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
    }
    RCLCPP_INFO(get_logger(), "Still waiting for MoveIt...");
  }
  RCLCPP_INFO(get_logger(), "MoveIt is available");

  // Wait for controller_manager services (indicates controllers are loaded)
  auto list_controllers_client = temp_node->create_client<
    controller_manager_msgs::srv::ListControllers>("/controller_manager/list_controllers");
  RCLCPP_INFO(get_logger(), "Waiting for controller_manager service...");
  while (!list_controllers_client->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(get_logger(), "Interrupted while waiting for controller_manager service");
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
    }
    RCLCPP_INFO(get_logger(), "Still waiting for controller_manager...");
  }
  RCLCPP_INFO(get_logger(), "Controllers are ready");

  load_config_from_yaml();

  worker_thread_ = std::thread(&WelderActionServer::worker_thread_func, this);

  // MoveGroupInterface needs rclcpp::Node, not LifecycleNode
  try {
    RCLCPP_INFO(get_logger(), "Initializing MoveIt");

    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);

    auto internal_node = std::make_shared<rclcpp::Node>(
      "welder_moveit_internal",
      node_options);

    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
      internal_node, config_.welder_group_name);

    move_group_->setPlanningTime(10.0);
    move_group_->setNumPlanningAttempts(10);
    move_group_->setMaxVelocityScalingFactor(config_.velocity_scaling);

    RCLCPP_INFO(get_logger(), "MoveIt initialized successfully");
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Failed to initialize MoveIt: %s", e.what());
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
  }

  using namespace std::placeholders;
  action_server_ = rclcpp_action::create_server<TriggerWelder>(
    this->get_node_base_interface(),
    this->get_node_clock_interface(),
    this->get_node_logging_interface(),
    this->get_node_waitables_interface(),
    "trigger_welder",
    std::bind(&WelderActionServer::handle_goal, this, _1, _2),
    std::bind(&WelderActionServer::handle_cancel, this, _1),
    std::bind(&WelderActionServer::handle_accepted, this, _1));

  RCLCPP_INFO(get_logger(), "Welder Action Server configured");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
WelderActionServer::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Activating Welder Action Server");
  RCLCPP_INFO(get_logger(), "Welder Action Server activated and ready!");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
WelderActionServer::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating Welder Action Server");

  if (move_group_) {
    move_group_->stop();
  }

  RCLCPP_INFO(get_logger(), "Welder Action Server deactivated");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
WelderActionServer::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Cleaning up Welder Action Server");

  shutdown_worker();

  action_server_.reset();

  move_group_.reset();

  RCLCPP_INFO(get_logger(), "Welder Action Server cleaned up");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
WelderActionServer::on_shutdown(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Shutting down Welder Action Server");
  on_cleanup(get_current_state());
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

void WelderActionServer::load_config_from_yaml()
{
  std::string yaml_path;
  try {
    yaml_path = ament_index_cpp::get_package_share_directory("hold_and_weld_application") +
      "/config/tasks/welding.yaml";
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Failed to get package share directory: %s", e.what());
    RCLCPP_WARN(get_logger(), "Using default configuration");
    return;
  }

  RCLCPP_INFO(get_logger(), "Loading config from: %s", yaml_path.c_str());

  if (!std::filesystem::exists(yaml_path)) {
    RCLCPP_WARN(get_logger(), "YAML file not found, using default configuration");
    return;
  }

  try {
    YAML::Node yaml = YAML::LoadFile(yaml_path);

    if (yaml["welder_group_name"]) {
      config_.welder_group_name = yaml["welder_group_name"].as<std::string>();
    }
    if (yaml["approach_offset_z"]) {
      config_.approach_offset_z = yaml["approach_offset_z"].as<double>();
    }
    if (yaml["retract_offset_z"]) {
      config_.retract_offset_z = yaml["retract_offset_z"].as<double>();
    }
    if (yaml["cartesian_path_threshold"]) {
      config_.cartesian_path_threshold = yaml["cartesian_path_threshold"].as<double>();
    }
    if (yaml["cartesian_step_size"]) {
      config_.cartesian_step_size = yaml["cartesian_step_size"].as<double>();
    }
    if (yaml["velocity_scaling"]) {
      config_.velocity_scaling = yaml["velocity_scaling"].as<double>();
    }
    if (yaml["max_approach_retries"]) {
      config_.max_approach_retries = yaml["max_approach_retries"].as<int>();
    }
    if (yaml["max_cartesian_retries"]) {
      config_.max_cartesian_retries = yaml["max_cartesian_retries"].as<int>();
    }
    if (yaml["json_file"]) {
      config_.json_file = yaml["json_file"].as<std::string>();
    }
    RCLCPP_INFO(get_logger(), "Configuration loaded successfully");
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Error parsing YAML: %s", e.what());
    RCLCPP_WARN(get_logger(), "Using default configuration");
  }
}

void WelderActionServer::worker_thread_func()
{
  // Wait for goals using condition variable to avoid busy-waiting
  while (!shutdown_requested_) {
    std::shared_ptr<GoalHandleTriggerWelder> goal_handle;

    {
      std::unique_lock<std::mutex> lock(work_mutex_);
      work_cv_.wait(lock, [this] {
          return pending_goal_ != nullptr || shutdown_requested_;
            });

      if (shutdown_requested_) {
        break;
      }

      goal_handle = pending_goal_;
      pending_goal_ = nullptr;
    }

    if (goal_handle) {
      execute_weld(goal_handle);
    }
  }
}

std::string WelderActionServer::find_latest_json()
{
  std::string generated_dir;

  try {
    std::string pkg_share =
      ament_index_cpp::get_package_share_directory("hold_and_weld_application");

    std::filesystem::path share_path(pkg_share);
    std::filesystem::path ws_root =
      share_path.parent_path().parent_path().parent_path().parent_path();

    std::filesystem::path src_path = ws_root / "src" / "hold_and_weld" /
      "hold_and_weld_application" / "trajectories";

    if (std::filesystem::exists(src_path)) {
      generated_dir = src_path.string();
      RCLCPP_INFO(get_logger(), "Using source trajectory dir: %s", generated_dir.c_str());
    } else {
      generated_dir = pkg_share + "/trajectories";
      RCLCPP_WARN(get_logger(), "Source dir not found, using install: %s", generated_dir.c_str());
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Failed to locate package: %s", e.what());
    return "";
  }

  RCLCPP_INFO(get_logger(), "Searching for JSON in: %s", generated_dir.c_str());

  if (!std::filesystem::exists(generated_dir)) {
    RCLCPP_ERROR(get_logger(), "Trajectory directory does not exist: %s", generated_dir.c_str());
    return "";
  }

  try {
    std::vector<std::filesystem::path> json_files;
    for (const auto & entry : std::filesystem::directory_iterator(generated_dir)) {
      if (entry.path().extension() == ".json") {
        json_files.push_back(entry.path());
      }
    }

    if (json_files.empty()) {
      RCLCPP_ERROR(get_logger(), "No JSON files found in: %s", generated_dir.c_str());
      return "";
    }

    // Sort by modification time (newest first)
    std::sort(json_files.begin(), json_files.end(),
      [](const auto & a, const auto & b) {
        return std::filesystem::last_write_time(a) >
               std::filesystem::last_write_time(b);
            });

    RCLCPP_INFO(get_logger(), "Found %zu JSON files, using latest: %s",
                   json_files.size(), json_files.front().filename().string().c_str());
    return json_files.front().string();
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Error finding JSON: %s", e.what());
    return "";
  }
}

std::vector<WeldSeam> WelderActionServer::load_seams_from_json(const std::string & filepath)
{
  std::vector<WeldSeam> seams;

  RCLCPP_INFO(get_logger(), "Loading seams from: %s", filepath.c_str());

  std::ifstream file(filepath);
  if (!file.is_open()) {
    RCLCPP_ERROR(get_logger(), "Failed to open file: %s", filepath.c_str());
    return seams;
  }

  try {
    nlohmann::json data = nlohmann::json::parse(file);
    if (data.contains("seams")) {
      seams.reserve(data["seams"].size());
    } else {
      RCLCPP_ERROR(get_logger(), "JSON missing 'seams' key");
      return seams;
    }

    for (const auto & [seam_id, seam_data] : data["seams"].items()) {
      if (!seam_data.contains("poses")) {
        RCLCPP_WARN(get_logger(), "Seam %s has no poses, skipping", seam_id.c_str());
        continue;
      }

      WeldSeam seam;
      seam.seam_id = seam_id;

      seam.length_m = seam_data.value("length_m", 0.0);

      if (seam_data.contains("start")) {
        auto s = seam_data["start"];
        seam.start = {s[0], s[1], s[2]};
      }
      if (seam_data.contains("end")) {
        auto e = seam_data["end"];
        seam.end = {e[0], e[1], e[2]};
      }

      seam.poses.reserve(seam_data["poses"].size());
      for (const auto & pose_data : seam_data["poses"]) {
        seam.poses.push_back(json_to_pose(pose_data));
      }

      seam.num_poses = seam.poses.size();
      seams.push_back(seam);
    }

    RCLCPP_INFO(get_logger(), "Loaded %zu seams from %s", seams.size(), filepath.c_str());
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Error parsing JSON: %s", e.what());
  }

  return seams;
}

rclcpp_action::GoalResponse WelderActionServer::handle_goal(
  [[maybe_unused]] const rclcpp_action::GoalUUID & uuid,
  [[maybe_unused]] std::shared_ptr<const TriggerWelder::Goal> goal)
{
  // Check if node is active
  if (get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    RCLCPP_ERROR(get_logger(), "Cannot accept goal: node is not active");
    return rclcpp_action::GoalResponse::REJECT;
  }

  RCLCPP_INFO(get_logger(), "Received welder trigger request");
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse WelderActionServer::handle_cancel(
  [[maybe_unused]] const std::shared_ptr<GoalHandleTriggerWelder> goal_handle)
{
  RCLCPP_INFO(get_logger(), "Received cancel request");
  if (move_group_) {
    move_group_->stop();
  }
  return rclcpp_action::CancelResponse::ACCEPT;
}

void WelderActionServer::handle_accepted(
  const std::shared_ptr<GoalHandleTriggerWelder> goal_handle)
{
  // Queue the goal for worker thread instead of blocking
  {
    std::lock_guard<std::mutex> lock(work_mutex_);
    pending_goal_ = goal_handle;
  }
  work_cv_.notify_one();
}

void WelderActionServer::execute_weld(const std::shared_ptr<GoalHandleTriggerWelder> goal_handle)
{
  auto feedback = std::make_shared<TriggerWelder::Feedback>();
  auto result = std::make_shared<TriggerWelder::Result>();

  std::string json_path;
  if (!config_.json_file.empty() && config_.json_file != "auto") {
    json_path = config_.json_file;
    RCLCPP_INFO(get_logger(), "Using configured JSON: %s", json_path.c_str());
  } else {
    json_path = find_latest_json();
    if (json_path.empty()) {
      result->success = false;
      result->message = "No JSON file found";
      goal_handle->abort(result);
      return;
    }
  }

  std::vector<WeldSeam> seams = load_seams_from_json(json_path);
  if (seams.empty()) {
    result->success = false;
    result->message = "No seams loaded from JSON";
    goal_handle->abort(result);
    return;
  }

  int32_t total_waypoints = 0;
  for (const auto & seam : seams) {
    total_waypoints += static_cast<int32_t>(seam.num_poses);
  }

  auto publish_progress = [&](const std::string & step, int32_t points_done) {
      feedback->current_step = step;
      feedback->completion_percentage = (static_cast<float>(points_done) / total_waypoints) *
        100.0f;
      goal_handle->publish_feedback(feedback);
      RCLCPP_INFO(get_logger(), "[%s] %.1f%% complete",
                   step.c_str(), feedback->completion_percentage);
    };

  std::vector<std::string> succeeded_seams;
  std::vector<std::string> failed_seams;
  int32_t total_points_executed = 0;
  int32_t points_processed = 0;

  // Process each seam
  RCLCPP_INFO(get_logger(), "Processing %zu seams", seams.size());

  // Execute each seam
  for (size_t seam_idx = 0; seam_idx < seams.size(); ++seam_idx) {
    const auto & seam = seams[seam_idx];
    const auto & waypoints = seam.poses;

    if (waypoints.empty()) {
      RCLCPP_WARN(get_logger(), "Seam %s has no waypoints, skipping", seam.seam_id.c_str());
      failed_seams.push_back(seam.seam_id);
      continue;
    }

    RCLCPP_INFO(get_logger(), "Executing seam %zu/%zu: %s",
                 seam_idx + 1, seams.size(), seam.seam_id.c_str());
    RCLCPP_INFO(get_logger(), "  Length: %.3f m", seam.length_m);
    RCLCPP_INFO(get_logger(), "  Points: %zu", seam.num_poses);

    if (goal_handle->is_canceling()) {
      result->success = false;
      result->message = "Canceled by client";
      goal_handle->canceled(result);
      return;
    }

    bool approach_success = false;
    for (int attempt = 0; attempt < config_.max_approach_retries; ++attempt) {
      publish_progress("approaching_seam_" + seam.seam_id, points_processed);
      if (approach_seam(waypoints.front())) {
        approach_success = true;
        break;
      }
      RCLCPP_WARN(get_logger(), "Approach attempt %d/%d failed",
                   attempt + 1, config_.max_approach_retries);
    }

    if (!approach_success) {
      RCLCPP_ERROR(get_logger(), "Failed to approach seam %s", seam.seam_id.c_str());
      failed_seams.push_back(seam.seam_id);
      continue;
    }

    if (goal_handle->is_canceling()) {
      result->success = false;
      result->message = "Canceled by client";
      goal_handle->canceled(result);
      return;
    }

    bool path_success = false;
    for (int attempt = 0; attempt < config_.max_cartesian_retries; ++attempt) {
      publish_progress("welding_seam_" + seam.seam_id, points_processed);

      if (execute_cartesian_path(waypoints, goal_handle, feedback,
            points_processed, total_waypoints))
      {
        path_success = true;
        break;
      }
      RCLCPP_WARN(get_logger(), "Cartesian path attempt %d/%d failed",
                   attempt + 1, config_.max_cartesian_retries);
    }

    if (!path_success) {
      RCLCPP_ERROR(get_logger(), "Failed to execute cartesian path for seam %s",
                     seam.seam_id.c_str());
      failed_seams.push_back(seam.seam_id);
      continue;
    }

    points_processed += static_cast<int32_t>(seam.num_poses);
    total_points_executed += static_cast<int32_t>(seam.num_poses);

    if (goal_handle->is_canceling()) {
      result->success = false;
      result->message = "Canceled by client";
      goal_handle->canceled(result);
      return;
    }

    publish_progress("retracting_from_seam_" + seam.seam_id, points_processed);
    if (!retract_from_seam(waypoints.back())) {
      RCLCPP_WARN(get_logger(), "Failed to retract from seam %s", seam.seam_id.c_str());
    }

    succeeded_seams.push_back(seam.seam_id);
    RCLCPP_INFO(get_logger(), "Seam %s completed successfully", seam.seam_id.c_str());
  }

  std::string msg = "Welding complete. ";
  if (!succeeded_seams.empty()) {
    msg += "Succeeded: ";
    for (size_t i = 0; i < succeeded_seams.size(); ++i) {
      msg += succeeded_seams[i];
      if (i < succeeded_seams.size() - 1) {msg += ", ";}
    }
    msg += ". ";
  }
  if (!failed_seams.empty()) {
    msg += "Failed: ";
    for (size_t i = 0; i < failed_seams.size(); ++i) {
      msg += failed_seams[i];
      if (i < failed_seams.size() - 1) {msg += ", ";}
    }
  }

  result->success = failed_seams.empty();
  result->message = msg;
  result->points_executed = total_points_executed;

  if (result->success) {
    goal_handle->succeed(result);
  } else {
    goal_handle->abort(result);
  }
}

bool WelderActionServer::approach_seam(const geometry_msgs::msg::Pose & first_pose)
{
  geometry_msgs::msg::Pose approach_pose = first_pose;
  approach_pose.position.z += config_.approach_offset_z;

  RCLCPP_INFO(get_logger(), "Approaching seam at (%.3f, %.3f, %.3f)",
               approach_pose.position.x,
               approach_pose.position.y,
               approach_pose.position.z);

  move_group_->setPoseTarget(approach_pose);
  auto result = move_group_->move();
  if (result == moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_INFO(get_logger(), "Approach complete");
    return true;
  }
  return false;
}

bool WelderActionServer::retract_from_seam(const geometry_msgs::msg::Pose & last_pose)
{
  geometry_msgs::msg::Pose retract_pose = last_pose;
  retract_pose.position.z += config_.retract_offset_z;

  RCLCPP_INFO(get_logger(), "Retracting from seam to (%.3f, %.3f, %.3f)",
               retract_pose.position.x,
               retract_pose.position.y,
               retract_pose.position.z);

  move_group_->setPoseTarget(retract_pose);
  auto result = move_group_->move();
  if (result == moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_INFO(get_logger(), "Retract complete");
    return true;
  }
  return false;
}

bool WelderActionServer::execute_cartesian_path(
  const std::vector<geometry_msgs::msg::Pose> & waypoints,
  const std::shared_ptr<GoalHandleTriggerWelder> & goal_handle,
  std::shared_ptr<TriggerWelder::Feedback> & feedback,
  int32_t points_before_seam,
  int32_t total_waypoints)
{
  RCLCPP_INFO(get_logger(), "Planning cartesian path with %zu waypoints", waypoints.size());

  moveit_msgs::msg::RobotTrajectory trajectory;
  double fraction = move_group_->computeCartesianPath(
        waypoints, config_.cartesian_step_size, trajectory);

  RCLCPP_INFO(get_logger(), "Cartesian path: %.2f%% achieved", fraction * 100.0);

  if (fraction < config_.cartesian_path_threshold) {
    RCLCPP_ERROR(get_logger(), "Cartesian path below threshold (%.2f%% < %.2f%%)",
                  fraction * 100.0, config_.cartesian_path_threshold * 100.0);
    return false;
  }

  auto execute_result = move_group_->execute(trajectory);
  if (execute_result != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(get_logger(), "Execution failed");
    return false;
  }

  int32_t points_after_seam = points_before_seam + static_cast<int32_t>(waypoints.size());
  feedback->completion_percentage = (static_cast<float>(points_after_seam) / total_waypoints) *
    100.0f;
  goal_handle->publish_feedback(feedback);

  RCLCPP_INFO(get_logger(), "Cartesian path executed successfully");
  return true;
}

void WelderActionServer::shutdown_worker()
{
  {
    std::lock_guard<std::mutex> lock(work_mutex_);
    shutdown_requested_ = true;
  }
  work_cv_.notify_one();

  if (worker_thread_.joinable()) {
    worker_thread_.join();
  }
}

}  // namespace hold_and_weld
