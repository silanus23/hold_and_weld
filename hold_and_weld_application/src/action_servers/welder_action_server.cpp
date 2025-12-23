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


namespace hold_and_weld
{

WelderActionServer::WelderActionServer(const rclcpp::NodeOptions & options)
: Node("welder_action_server", options)
{
  RCLCPP_INFO(get_logger(), "Welder Action Server starting");

  // Load configuration from YAML
  load_config_from_yaml();

  // Start worker thread
  worker_thread_ = std::thread(&WelderActionServer::worker_thread_func, this);

  // Delay MoveIt initialization to avoid bad_weak_ptr
  init_timer_ = create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&WelderActionServer::initialize_moveit, this));
}

WelderActionServer::~WelderActionServer()
{
  shutdown_worker();
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

void WelderActionServer::initialize_moveit()
{
  // Cancel timer - only run once
  init_timer_->cancel();

  using namespace std::placeholders;

  RCLCPP_INFO(get_logger(), "Initializing MoveIt");

  try {
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            shared_from_this(), config_.welder_group_name);

    move_group_->setPlanningTime(10.0);
    move_group_->setNumPlanningAttempts(10);
    move_group_->setMaxVelocityScalingFactor(config_.velocity_scaling);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Failed to initialize MoveIt: %s", e.what());
    return;
  }
    // Initialize action server
  action_server_ = rclcpp_action::create_server<TriggerWelder>(
        this,
        "trigger_welder",
        std::bind(&WelderActionServer::handle_goal, this, _1, _2),
        std::bind(&WelderActionServer::handle_cancel, this, _1),
        std::bind(&WelderActionServer::handle_accepted, this, _1));
  initialized_ = true;
  RCLCPP_INFO(get_logger(), "Welder Action Server ready!");
}

std::string WelderActionServer::find_latest_json()
{
  std::string generated_dir;
  try {
    generated_dir = ament_index_cpp::get_package_share_directory("hold_and_weld_planning") +
      "/generated";
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Failed to get hold_and_weld_planning package: %s", e.what());
    return "";
  }

  RCLCPP_INFO(get_logger(), "Searching for JSON in: %s", generated_dir.c_str());

  if (!std::filesystem::exists(generated_dir)) {
    RCLCPP_ERROR(get_logger(), "Generated directory does not exist: %s", generated_dir.c_str());
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
    if (data.contains("seams")) {seams.reserve(data["seams"].size());} else {
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
  if (!initialized_) {
    RCLCPP_ERROR(get_logger(), "Server not initialized yet");
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

void WelderActionServer::handle_accepted(const std::shared_ptr<GoalHandleTriggerWelder> goal_handle)
{
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

  // Determine JSON file path
  std::string json_path = config_.json_file;
  if (json_path.empty()) {
    json_path = find_latest_json();
  }

  if (json_path.empty()) {
    result->success = false;
    result->message = "No JSON file found";
    goal_handle->abort(result);
    return;
  }

  // Load seams from JSON
  std::vector<WeldSeam> seams = load_seams_from_json(json_path);

  if (seams.empty()) {
    result->success = false;
    result->message = "No seams loaded from JSON";
    goal_handle->abort(result);
    return;
  }

  // Calculate total waypoints for global completion percentage
  int32_t total_waypoints = 0;
  for (const auto & seam : seams) {
    total_waypoints += static_cast<int32_t>(seam.poses.size());
  }

  auto publish_progress = [&](const std::string & step, int current_pts) {
      feedback->current_step = step;
      feedback->current_point = current_pts;
      feedback->completion_percentage = (total_waypoints > 0) ?
        (static_cast<double>(current_pts) / total_waypoints) * 100.0 :
        0.0;
      feedback->total_points = total_waypoints;
      goal_handle->publish_feedback(feedback);
    };

  std::vector<std::string> succeeded_seams;
  std::vector<std::string> failed_seams;
  int32_t total_points_executed = 0;
  int32_t points_processed = 0;

  RCLCPP_INFO(get_logger(), "\nExecuting: %zu weld jobs (%d total waypoints)",
               seams.size(), total_waypoints);

  feedback->total_points = total_waypoints;

  for (size_t seam_idx = 0; seam_idx < seams.size(); ++seam_idx) {
    const auto & seam = seams[seam_idx];
    const auto & waypoints = seam.poses;

    RCLCPP_INFO(get_logger(), "\nSeam %zu/%zu: %s",
                   seam_idx + 1, seams.size(), seam.seam_id.c_str());
    RCLCPP_INFO(get_logger(), "Waypoints: %zu", waypoints.size());

    feedback->current_point = points_processed;
    feedback->completion_percentage = (total_waypoints > 0) ?
      (static_cast<double>(points_processed) / total_waypoints) * 100.0 :
      0.0;
    feedback->current_step = "approaching " + seam.seam_id;
    goal_handle->publish_feedback(feedback);

    // Check for cancel
    if (goal_handle->is_canceling()) {
      result->success = !succeeded_seams.empty();
      result->message = "Canceled. Succeeded: " + std::to_string(succeeded_seams.size()) +
        ", Failed: " + std::to_string(failed_seams.size());
      goal_handle->canceled(result);
      return;
    }

    publish_progress("approaching " + seam.seam_id, points_processed);
    // Approach with retry
    bool approach_success = false;
    for (int attempt = 1; attempt <= config_.max_approach_retries; ++attempt) {
      if (goal_handle->is_canceling()) {
        result->success = !succeeded_seams.empty();
        result->message = "Canceled during approach";
        goal_handle->canceled(result);
        return;
      }

      if (approach_seam(waypoints.front())) {
        approach_success = true;
        break;
      }

      RCLCPP_WARN(get_logger(), "Approach attempt %d/%d failed for %s",
                       attempt, config_.max_approach_retries, seam.seam_id.c_str());

      if (attempt < config_.max_approach_retries) {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
      }
    }

    if (!approach_success) {
      RCLCPP_ERROR(get_logger(), "FAILED to approach seam: %s, skipping", seam.seam_id.c_str());
      failed_seams.push_back(seam.seam_id);
      points_processed += static_cast<int32_t>(waypoints.size());
      continue;
    }

    // Execute path with retry
    feedback->current_step = "welding " + seam.seam_id;
    goal_handle->publish_feedback(feedback);

    bool path_success = false;
    for (int attempt = 1; attempt <= config_.max_cartesian_retries; ++attempt) {
      if (goal_handle->is_canceling()) {
        retract_from_seam(waypoints.back());
        result->success = !succeeded_seams.empty();
        result->message = "Canceled during welding";
        goal_handle->canceled(result);
        return;
      }

      if (execute_cartesian_path(waypoints, goal_handle, feedback, points_processed,
          total_waypoints))
      {
        path_success = true;
        break;
      }

      RCLCPP_WARN(get_logger(), "Cartesian path attempt %d/%d failed for %s",
                       attempt, config_.max_cartesian_retries, seam.seam_id.c_str());

      if (attempt < config_.max_cartesian_retries) {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
      }
    }
    publish_progress("welding " + seam.seam_id, points_processed);
    // Retract (always attempt)
    feedback->current_step = "retracting " + seam.seam_id;
    goal_handle->publish_feedback(feedback);
    retract_from_seam(waypoints.back());

    points_processed += static_cast<int32_t>(waypoints.size());

    if (path_success) {
      RCLCPP_INFO(get_logger(), "SUCCESS: %s", seam.seam_id.c_str());
      succeeded_seams.push_back(seam.seam_id);
      total_points_executed += static_cast<int32_t>(waypoints.size());
    } else {
      RCLCPP_ERROR(get_logger(), "FAILED: %s", seam.seam_id.c_str());
      failed_seams.push_back(seam.seam_id);
    }
  }

  // Build result message
  std::string msg = "Completed. Succeeded: ";
  for (size_t i = 0; i < succeeded_seams.size(); ++i) {
    if (i > 0) {msg += ", ";}
    msg += succeeded_seams[i];
  }
  if (succeeded_seams.empty()) {msg += "none";}

  msg += ". Failed: ";
  for (size_t i = 0; i < failed_seams.size(); ++i) {
    if (i > 0) {msg += ", ";}
    msg += failed_seams[i];
  }
  if (failed_seams.empty()) {msg += "none";}

  feedback->current_step = "completed";
  feedback->current_point = total_waypoints;
  feedback->completion_percentage = 100.0;
  goal_handle->publish_feedback(feedback);

  result->success = !succeeded_seams.empty();
  result->message = msg;
  result->points_executed = total_points_executed;
  goal_handle->succeed(result);

  RCLCPP_INFO(get_logger(), "\nFinished: %zu/%zu succeeded\n",
               succeeded_seams.size(), seams.size());
}

bool WelderActionServer::approach_seam(const geometry_msgs::msg::Pose & first_pose)
{
  geometry_msgs::msg::Pose approach_pose = first_pose;
  approach_pose.position.z += config_.approach_offset_z;

  RCLCPP_INFO(get_logger(), "[approach] Planning to (%.3f, %.3f, %.3f)",
               approach_pose.position.x,
               approach_pose.position.y,
               approach_pose.position.z);

  move_group_->setPoseTarget(approach_pose);
  auto result = move_group_->move();

  if (result == moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_INFO(get_logger(), "[approach] Done.");
    return true;
  }

  RCLCPP_ERROR(get_logger(), "[approach] Failed!");
  return false;
}

bool WelderActionServer::retract_from_seam(const geometry_msgs::msg::Pose & last_pose)
{
  geometry_msgs::msg::Pose retract_pose = last_pose;
  retract_pose.position.z += config_.retract_offset_z;

  RCLCPP_INFO(get_logger(), "[retract] Planning to (%.3f, %.3f, %.3f)",
               retract_pose.position.x,
               retract_pose.position.y,
               retract_pose.position.z);

  move_group_->setPoseTarget(retract_pose);
  auto result = move_group_->move();

  if (result == moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_INFO(get_logger(), "[retract] Done.");
    return true;
  }

  RCLCPP_ERROR(get_logger(), "[retract] Failed!");
  return false;
}

bool WelderActionServer::execute_cartesian_path(
  const std::vector<geometry_msgs::msg::Pose> & waypoints,
  const std::shared_ptr<GoalHandleTriggerWelder> & goal_handle,
  std::shared_ptr<TriggerWelder::Feedback> & feedback,
  int32_t points_before_seam,
  int32_t total_waypoints)
{
  moveit_msgs::msg::RobotTrajectory trajectory;
  double fraction = move_group_->computeCartesianPath(
        waypoints, config_.cartesian_step_size, trajectory);

  if (fraction < config_.cartesian_path_threshold) {
    RCLCPP_ERROR(get_logger(), "Cartesian path only %.2f%% complete (threshold: %.2f%%)",
                    fraction * 100.0, config_.cartesian_path_threshold * 100.0);
    return false;
  }

  RCLCPP_INFO(get_logger(), "Cartesian path %.2f%% complete, executing", fraction * 100.0);

  // Execute trajectory
  auto execute_result = move_group_->execute(trajectory);

  // Update feedback with global progress
  int32_t points_after_seam = points_before_seam + static_cast<int32_t>(waypoints.size());
  feedback->current_point = points_after_seam;
  feedback->completion_percentage = (total_waypoints > 0) ?
    (static_cast<double>(points_after_seam) / total_waypoints) * 100.0 :
    0.0;
  goal_handle->publish_feedback(feedback);

  if (execute_result == moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_INFO(get_logger(), "[cartesian] Done.");
    return true;
  }

  RCLCPP_ERROR(get_logger(), "[cartesian] Execution failed!");
  return false;
}

void WelderActionServer::shutdown_worker()
{
  {
    std::lock_guard<std::mutex> lock(work_mutex_);
    shutdown_requested_ = true;
  }
  work_cv_.notify_all();

  if (worker_thread_.joinable()) {
    worker_thread_.join();
  }

  if (move_group_) {
    move_group_->stop();
  }
}

}  // namespace hold_and_weld
