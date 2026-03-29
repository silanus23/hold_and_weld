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

#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <fstream>
#include <filesystem>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <controller_manager_msgs/srv/list_controllers.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <moveit_msgs/srv/get_cartesian_path.hpp>
#include <nlohmann/json.hpp>
#include <rclcpp/parameter_client.hpp>

namespace hold_and_weld
{

WelderActionServer::WelderActionServer(const rclcpp::NodeOptions & options)
: LifecycleNode("welder_action_server", options),
  logger_(rclcpp::get_logger("Application"))
{
  RCLCPP_INFO(logger_, "Welder Action Server constructed");
}

WelderActionServer::~WelderActionServer()
{
  shutdown_worker();

  if (moveit_executor_) {
    moveit_executor_->cancel();
  }
  if (moveit_thread_.joinable()) {
    moveit_thread_.join();
  }
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
WelderActionServer::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(logger_, "Configuring Welder Action Server");

  // Wait for MoveIt and controllers to be available before creating
  // MoveGroupInterface to prevent race conditions on startup
  RCLCPP_INFO(logger_, "Waiting for required services to become available");
  auto temp_node = std::make_shared<rclcpp::Node>("welder_service_waiter");

  // MoveIt's compute_cartesian_path service indicates that move_group is running
  auto cartesian_path_client = temp_node->create_client<moveit_msgs::srv::GetCartesianPath>(
    "/compute_cartesian_path");

  RCLCPP_INFO(logger_, "Waiting for MoveIt compute_cartesian_path service");
  constexpr int MAX_WAIT_SECONDS = 60;
  int wait_count = 0;

  while (!cartesian_path_client->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(logger_, "Interrupted while waiting for MoveIt service");
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
    }

    wait_count++;

    if (wait_count >= MAX_WAIT_SECONDS) {
      RCLCPP_ERROR(
        logger_,
        "MoveIt service not available after %d seconds. Is move_group running?",
        MAX_WAIT_SECONDS);
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
    } else if (wait_count % 10 == 0) {
      RCLCPP_WARN(
        logger_,
        "Still waiting for MoveIt service (%d/%d seconds)",
        wait_count,
        MAX_WAIT_SECONDS);
    } else if (wait_count % 5 == 0) {
      RCLCPP_INFO(
        logger_,
        "Waiting for MoveIt service (%d/%d seconds)",
        wait_count,
        MAX_WAIT_SECONDS);
    }
  }
  RCLCPP_INFO(logger_, "MoveIt is available");

  // Wait for controller_manager services (indicates controllers are loaded)
  auto list_controllers_client = temp_node->create_client<
    controller_manager_msgs::srv::ListControllers>("/controller_manager/list_controllers");

  RCLCPP_INFO(logger_, "Waiting for controller_manager service");
  wait_count = 0;

  while (!list_controllers_client->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(logger_, "Interrupted while waiting for controller_manager service");
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
    }

    wait_count++;

    if (wait_count >= MAX_WAIT_SECONDS) {
      RCLCPP_ERROR(
        logger_,
        "Controller manager service not available after %d seconds. Is controller_manager running?",
        MAX_WAIT_SECONDS);
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
    } else if (wait_count % 10 == 0) {
      RCLCPP_WARN(
        logger_,
        "Still waiting for controller_manager service (%d/%d seconds)",
        wait_count,
        MAX_WAIT_SECONDS);
    } else if (wait_count % 5 == 0) {
      RCLCPP_INFO(
        logger_,
        "Waiting for controller_manager service (%d/%d seconds)",
        wait_count,
        MAX_WAIT_SECONDS);
    }
  }
  RCLCPP_INFO(logger_, "Controllers are ready");

  load_config_from_yaml();

  std::string urdf_string;
  // URDF string is only needed for kinematics — fetch it here where we manage nodes
  if (config_.use_approach_validator) {
    RCLCPP_INFO(logger_, "Asking robot_state_publisher for the master URDF");

    auto param_client = std::make_shared<rclcpp::SyncParametersClient>(temp_node,
        "robot_state_publisher");

    if (param_client->wait_for_service(std::chrono::seconds(10))) {
      auto parameters = param_client->get_parameters({"robot_description"});
      if (!parameters.empty()) {
        urdf_string = parameters[0].as_string();
      }
    } else {
      RCLCPP_ERROR(logger_, "Failed to contact robot_state_publisher! Is it running?");
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
    }

    if (urdf_string.empty()) {
      RCLCPP_ERROR(logger_, "Retrieved robot_description is empty!");
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
    }
  }

  rclcpp::Node::SharedPtr internal_node;
  try {
    RCLCPP_INFO(logger_, "Initializing MoveIt");

    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);

    // MoveGroupInterface requires a separate executor thread to prevent deadlocks.
    // MoveIt uses internal action clients that need to be spun while the main
    // lifecycle node is executing callbacks.
    internal_node = std::make_shared<rclcpp::Node>(
      "welder_moveit_internal",
      node_options);

    bool use_sim_time = false;
    if (this->get_parameter("use_sim_time", use_sim_time)) {
      internal_node->set_parameter(rclcpp::Parameter("use_sim_time", use_sim_time));
      RCLCPP_INFO(logger_, "Copied use_sim_time=%s to internal node",
          use_sim_time ? "true" : "false");
    }

    std::string robot_description_semantic;
    if (this->has_parameter("robot_description_semantic")) {
      robot_description_semantic = this->get_parameter("robot_description_semantic").as_string();
    }

    if (!robot_description_semantic.empty()) {
      internal_node->declare_parameter("robot_description_semantic", robot_description_semantic);
    }

    internal_node->declare_parameter("robot_description", urdf_string);

    moveit_executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    moveit_executor_->add_node(internal_node);
    moveit_thread_ = std::thread([this]() {moveit_executor_->spin();});

    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
      internal_node, config_.welder_group_name);

    move_group_->setPlanningTime(10.0);
    move_group_->setNumPlanningAttempts(10);
    move_group_->setMaxVelocityScalingFactor(config_.velocity_scaling);

    RCLCPP_INFO(logger_, "MoveIt initialized successfully");
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger_, "Failed to initialize MoveIt: %s", e.what());
    shutdown_worker();
    if (moveit_executor_) {moveit_executor_->cancel();}
    if (moveit_thread_.joinable()) {moveit_thread_.join();}
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
  }

  if (config_.use_approach_validator) {
    try {
      RCLCPP_INFO(logger_, "Initializing kinematics solvers");

      const auto * joint_model_group = move_group_->getRobotModel()->getJointModelGroup(
        config_.welder_group_name);

      if (!joint_model_group) {
        RCLCPP_ERROR(logger_, "Joint model group '%s' not found in robot model",
                     config_.welder_group_name.c_str());
        shutdown_worker();
        if (moveit_executor_) {moveit_executor_->cancel();}
        if (moveit_thread_.joinable()) {moveit_thread_.join();}
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
      }

      const std::vector<std::string> & link_names = joint_model_group->getLinkModelNames();
      if (link_names.size() < 2) {
        RCLCPP_ERROR(logger_, "Joint model group has insufficient links: %zu",
                     link_names.size());
        shutdown_worker();
        if (moveit_executor_) {moveit_executor_->cancel();}
        if (moveit_thread_.joinable()) {moveit_thread_.join();}
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
      }

      const std::vector<const moveit::core::JointModel *> & joint_models =
        joint_model_group->getActiveJointModels();

      std::string base_link;
      if (!joint_models.empty() && joint_models.front()->getParentLinkModel()) {
        base_link = joint_models.front()->getParentLinkModel()->getName();
      } else {
        base_link = link_names.front();
      }

      std::string tip_link = link_names.back();

      RCLCPP_INFO(logger_, "Detected kinematic chain: %s -> %s",
                  base_link.c_str(), tip_link.c_str());

      auto urdf_parser = std::make_unique<hold_and_weld_application::kinematics::URDFParser>();

      hold_and_weld_application::kinematics::ParsedChain parsed_chain;
      parsed_chain = urdf_parser->extract_joint_chain_from_string(urdf_string, base_link, tip_link);

      RCLCPP_INFO(logger_, "Parsed kinematic chain with %zu actuated joints",
                  parsed_chain.actuated_joints.size());

      kinematics_solver_ =
        std::make_shared<hold_and_weld_application::kinematics::KinematicsSolver>(parsed_chain);
      RCLCPP_INFO(logger_, "Kinematics solver initialized");

      ceres_solver_ = std::make_shared<hold_and_weld_application::kinematics::CeresIKSolver>(
        kinematics_solver_, 1.0);
      RCLCPP_INFO(logger_, "Ceres IK solver initialized");

      approach_validator_ =
        std::make_unique<hold_and_weld_application::kinematics::ApproachValidator>(
        kinematics_solver_,
        ceres_solver_,
        0.000001);
      RCLCPP_INFO(logger_, "Approach validator initialized");
      RCLCPP_INFO(logger_, "All kinematics solvers initialized successfully");
    } catch (const std::exception & e) {
      RCLCPP_ERROR(logger_, "Failed to initialize kinematics: %s", e.what());
      shutdown_worker();
      if (moveit_executor_) {moveit_executor_->cancel();}
      if (moveit_thread_.joinable()) {moveit_thread_.join();}
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
    }
  }

  worker_thread_ = std::thread(&WelderActionServer::worker_thread_func, this);

  action_server_ = rclcpp_action::create_server<TriggerWelder>(
    this->get_node_base_interface(),
    this->get_node_clock_interface(),
    this->get_node_logging_interface(),
    this->get_node_waitables_interface(),
    "trigger_welder",
    std::bind(&WelderActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&WelderActionServer::handle_cancel, this, std::placeholders::_1),
    std::bind(&WelderActionServer::handle_accepted, this, std::placeholders::_1));

  RCLCPP_INFO(logger_, "Welder Action Server configured");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
WelderActionServer::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(logger_, "Activating Welder Action Server");
  RCLCPP_INFO(logger_, "Welder Action Server activated and ready!");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
WelderActionServer::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(logger_, "Deactivating Welder Action Server");

  if (move_group_) {
    move_group_->stop();
  }

  RCLCPP_INFO(logger_, "Welder Action Server deactivated");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
WelderActionServer::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(logger_, "Cleaning up Welder Action Server");

  shutdown_worker();
  action_server_.reset();
  move_group_.reset();

  if (moveit_executor_) {
    moveit_executor_->cancel();
  }
  if (moveit_thread_.joinable()) {
    moveit_thread_.join();
  }
  moveit_executor_.reset();

  approach_validator_.reset();
  kinematics_solver_.reset();
  ceres_solver_.reset();

  RCLCPP_INFO(logger_, "Welder Action Server cleaned up");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
WelderActionServer::on_shutdown(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(logger_, "Shutting down Welder Action Server");

  // Inline cleanup directly to ensure proper shutdown regardless of current lifecycle state
  shutdown_worker();

  if (action_server_) {
    action_server_.reset();
  }

  if (move_group_) {
    move_group_->stop();
    move_group_.reset();
  }

  if (moveit_executor_) {
    moveit_executor_->cancel();
  }
  if (moveit_thread_.joinable()) {
    moveit_thread_.join();
  }
  moveit_executor_.reset();

  approach_validator_.reset();
  kinematics_solver_.reset();
  ceres_solver_.reset();

  RCLCPP_INFO(logger_, "Welder Action Server shutdown complete");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

void WelderActionServer::load_config_from_yaml()
{
  std::string yaml_path;
  try {
    yaml_path = ament_index_cpp::get_package_share_directory("hold_and_weld_application") +
      "/config/tasks/welding.yaml";
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger_, "Failed to get package share directory: %s", e.what());
    RCLCPP_WARN(logger_, "Using default configuration");
    return;
  }

  RCLCPP_INFO(logger_, "Loading config from: %s", yaml_path.c_str());

  if (!std::filesystem::exists(yaml_path)) {
    RCLCPP_WARN(logger_, "YAML file not found, using default configuration");
    return;
  }

  // There are unneeded configurations here kept as backup
  try {
    YAML::Node yaml = YAML::LoadFile(yaml_path);

    if (yaml["welder_group_name"]) {
      config_.welder_group_name = yaml["welder_group_name"].as<std::string>();
    }
    if (yaml["approach_offset_z"]) {
      config_.approach_offset_z = yaml["approach_offset_z"].as<double>();
    }
    if (yaml["use_approach_validator"]) {
      config_.use_approach_validator = yaml["use_approach_validator"].as<bool>();
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
    if (yaml["max_ompl_planning_attempts"]) {
      config_.max_ompl_planning_attempts = yaml["max_ompl_planning_attempts"].as<int>();
    }
    if (yaml["max_approach_validation_retries"]) {
      config_.max_approach_validation_retries = yaml["max_approach_validation_retries"].as<int>();
    }
    if (yaml["max_cartesian_retries"]) {
      config_.max_cartesian_retries = yaml["max_cartesian_retries"].as<int>();
    }
    if (yaml["json_file"]) {
      config_.json_file = yaml["json_file"].as<std::string>();
    }
    RCLCPP_INFO(logger_, "Configuration loaded successfully");
  } catch (const YAML::Exception & e) {
    RCLCPP_ERROR(logger_, "YAML parsing error: %s", e.what());
    RCLCPP_WARN(logger_, "Using default configuration");
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger_, "Error parsing YAML: %s", e.what());
    RCLCPP_WARN(logger_, "Using default configuration");
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

std::string WelderActionServer::find_latest_json()
{
  // First, check if user provided a specific trajectory directory via parameter
  std::string trajectory_dir;
  if (this->has_parameter("trajectory_directory")) {
    trajectory_dir = this->get_parameter("trajectory_directory").as_string();
    RCLCPP_INFO(logger_, "Using trajectory directory from parameter: %s",
                trajectory_dir.c_str());
  } else {
    // Fallback: use installed trajectory directory
    try {
      std::string pkg_share =
        ament_index_cpp::get_package_share_directory("hold_and_weld_application");
      trajectory_dir = pkg_share + "/trajectories";
      RCLCPP_INFO(logger_, "Using installed trajectory directory: %s",
                  trajectory_dir.c_str());
    } catch (const std::exception & e) {
      RCLCPP_ERROR(logger_, "Failed to locate package: %s", e.what());
      return "";
    }
  }

  if (!std::filesystem::exists(trajectory_dir)) {
    RCLCPP_ERROR(logger_, "Trajectory directory does not exist: %s",
                 trajectory_dir.c_str());
    return "";
  }

  try {
    std::vector<std::filesystem::path> json_files;
    for (const auto & entry : std::filesystem::directory_iterator(trajectory_dir)) {
      if (entry.path().extension() == ".json") {
        json_files.push_back(entry.path());
      }
    }

    if (json_files.empty()) {
      RCLCPP_ERROR(logger_, "No JSON files found in: %s", trajectory_dir.c_str());
      return "";
    }

    std::sort(json_files.begin(), json_files.end(),
      [](const auto & a, const auto & b) {
        return std::filesystem::last_write_time(a) > std::filesystem::last_write_time(b);
      });

    RCLCPP_INFO(logger_, "Found %zu JSON files, using latest: %s",
                json_files.size(), json_files.front().filename().string().c_str());
    return json_files.front().string();
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger_, "Error finding JSON: %s", e.what());
    return "";
  }
}

std::vector<WeldSeam> WelderActionServer::load_seams_from_json(const std::string & filepath)
{
  std::vector<WeldSeam> seams;

  RCLCPP_INFO(logger_, "Loading seams from: %s", filepath.c_str());

  std::ifstream file(filepath);
  if (!file.is_open()) {
    RCLCPP_ERROR(logger_, "Failed to open file: %s", filepath.c_str());
    return seams;
  }

  try {
    nlohmann::json data = nlohmann::json::parse(file);
    if (data.contains("seams")) {
      seams.reserve(data["seams"].size());
    } else {
      RCLCPP_ERROR(logger_, "JSON missing 'seams' key");
      return seams;
    }

    for (const auto & [seam_id, seam_data] : data["seams"].items()) {
      WeldSeam seam;
      seam.seam_id = seam_id;

      if (!seam_data.contains("poses")) {
        RCLCPP_WARN(logger_, "Seam %s has no poses, skipping", seam_id.c_str());
        continue;
      }

      if (seam_data["poses"].size() == 0) {
        RCLCPP_WARN(logger_, "Seam %s has empty poses array, skipping", seam_id.c_str());
        continue;
      }

      if (seam_data.contains("start") && seam_data["start"].size() == 3) {
        auto s = seam_data["start"];
        seam.start = {s[0], s[1], s[2]};
      }
      // If missing or wrong size, stays at default {0,0,0}

      if (seam_data.contains("end") && seam_data["end"].size() == 3) {
        auto e = seam_data["end"];
        seam.end = {e[0], e[1], e[2]};
      }

      seam.length_m = seam_data.value("length_m", 0.0);

      seam.poses.reserve(seam_data["poses"].size());
      for (const auto & pose_data : seam_data["poses"]) {
        seam.poses.push_back(json_to_pose(pose_data));
      }

      seam.num_poses = seam.poses.size();
      seams.push_back(seam);
    }

    RCLCPP_INFO(logger_, "Loaded %zu seams from %s", seams.size(), filepath.c_str());
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger_, "Error parsing JSON: %s", e.what());
  }

  return seams;
}

rclcpp_action::GoalResponse WelderActionServer::handle_goal(
  [[maybe_unused]] const rclcpp_action::GoalUUID & uuid,
  [[maybe_unused]] std::shared_ptr<const TriggerWelder::Goal> goal)
{
  if (get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    RCLCPP_ERROR(logger_, "Cannot accept goal: node is not active");
    return rclcpp_action::GoalResponse::REJECT;
  }

  RCLCPP_INFO(logger_, "Received welder trigger request");
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse WelderActionServer::handle_cancel(
  [[maybe_unused]] const std::shared_ptr<GoalHandleTriggerWelder> goal_handle)
{
  RCLCPP_INFO(logger_, "Received cancel request");
  if (move_group_) {
    move_group_->stop();
  }
  return rclcpp_action::CancelResponse::ACCEPT;
}

void WelderActionServer::handle_accepted(
  const std::shared_ptr<GoalHandleTriggerWelder> goal_handle)
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

  std::string json_path;
  if (!config_.json_file.empty() && config_.json_file != "auto") {
    json_path = config_.json_file;
    RCLCPP_INFO(logger_, "Using configured JSON: %s", json_path.c_str());
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

  if (total_waypoints == 0) {
    result->success = false;
    result->message = "No waypoints to execute";
    goal_handle->abort(result);
    return;
  }

  auto publish_progress = [&](const std::string & step, int32_t points_done) {
      feedback->current_step = step;
      feedback->completion_percentage = (static_cast<float>(points_done) / total_waypoints) *
        100.0f;
      goal_handle->publish_feedback(feedback);
      RCLCPP_INFO(logger_, "[%s] %.1f%% complete",
                   step.c_str(), feedback->completion_percentage);
    };

  std::vector<std::string> succeeded_seams;
  std::vector<std::string> failed_seams;
  int32_t total_points_executed = 0;
  int32_t points_processed = 0;

  RCLCPP_INFO(logger_, "Processing %zu seams", seams.size());

  for (size_t seam_idx = 0; seam_idx < seams.size(); ++seam_idx) {
    const auto & seam = seams[seam_idx];
    const auto & waypoints = seam.poses;

    if (waypoints.empty()) {
      RCLCPP_WARN(logger_, "Seam %s has no waypoints, skipping", seam.seam_id.c_str());
      failed_seams.push_back(seam.seam_id);
      continue;
    }

    RCLCPP_INFO(logger_, "Executing seam %zu/%zu: %s",
                 seam_idx + 1, seams.size(), seam.seam_id.c_str());
    RCLCPP_INFO(logger_, "  Length: %.3f m", seam.length_m);
    RCLCPP_INFO(logger_, "  Points: %zu", seam.num_poses);

    if (goal_handle->is_canceling()) {
      result->success = false;
      result->message = "Canceled by client";
      goal_handle->canceled(result);
      return;
    }

    publish_progress("approaching_seam_" + seam.seam_id, points_processed);
    if (!approach_seam(seam)) {
      RCLCPP_ERROR(logger_, "Failed to approach seam %s", seam.seam_id.c_str());
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
      RCLCPP_WARN(logger_, "Cartesian path attempt %d/%d failed",
                   attempt + 1, config_.max_cartesian_retries);
    }

    if (!path_success) {
      RCLCPP_ERROR(logger_, "Failed to execute cartesian path for seam %s",
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
      RCLCPP_ERROR(logger_, "Failed to retract from seam %s", seam.seam_id.c_str());
      failed_seams.push_back(seam.seam_id);
      continue;
    }

    succeeded_seams.push_back(seam.seam_id);
    RCLCPP_INFO(logger_, "Seam %s completed successfully", seam.seam_id.c_str());
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

bool WelderActionServer::approach_seam(const WeldSeam & seam)
{
  Eigen::Quaterniond q(
    seam.poses[0].orientation.w,
    seam.poses[0].orientation.x,
    seam.poses[0].orientation.y,
    seam.poses[0].orientation.z
  );

  // Torch tip link is antiparallel to the torch direction, so approach
  // is computed by backing up along the negative Z axis of the first pose
  Eigen::Vector3d torch_direction = q * Eigen::Vector3d(0, 0, -1);

  // 10cm offset gives enough clearance for OMPL to plan a smooth approach
  Eigen::Vector3d pos0(seam.poses[0].position.x, seam.poses[0].position.y,
    seam.poses[0].position.z);
  Eigen::Vector3d approach_pos = pos0 - torch_direction * 0.10;

  geometry_msgs::msg::Pose approach_pose;
  approach_pose.position.x = approach_pos.x();
  approach_pose.position.y = approach_pos.y();
  approach_pose.position.z = approach_pos.z();
  approach_pose.orientation = seam.poses[0].orientation;

  RCLCPP_INFO(logger_, "Approach Position: (%.3f, %.3f, %.3f)",
               approach_pose.position.x, approach_pose.position.y, approach_pose.position.z);

  move_group_->setStartStateToCurrentState();
  auto current_state = move_group_->getCurrentState();
  if (!current_state) {
    RCLCPP_ERROR(logger_, "Failed to get current robot state!");
    return false;
  }

  auto current_pose = move_group_->getCurrentPose();
  double distance = std::sqrt(
    std::pow(approach_pose.position.x - current_pose.pose.position.x, 2) +
    std::pow(approach_pose.position.y - current_pose.pose.position.y, 2) +
    std::pow(approach_pose.position.z - current_pose.pose.position.z, 2)
  );
  RCLCPP_INFO(logger_, "Distance to approach target: %.3f m", distance);

  move_group_->setPoseTarget(approach_pose);
  move_group_->setGoalPositionTolerance(0.001);
  move_group_->setGoalOrientationTolerance(0.01);

  // Transform seam waypoints from world frame to robot base frame for the validator.
  // The approach validator solves IK in the robot's own base frame.
  WeldSeam local_seam = seam;
  const auto * joint_model_group = current_state->getJointModelGroup(config_.welder_group_name);
  std::string base_link_name = joint_model_group->getLinkModelNames().front();
  Eigen::Isometry3d world_to_base = current_state->getGlobalLinkTransform(base_link_name);
  Eigen::Isometry3d base_to_world = world_to_base.inverse();
  for (auto & pose : local_seam.poses) {
    pose = transform_pose_to_base_frame(pose, base_to_world);
  }

  if (config_.use_approach_validator) {
    approach_validator_->set_weld_seam(local_seam);
  }

  for (int ompl_attempt = 1; ompl_attempt <= config_.max_ompl_planning_attempts; ++ompl_attempt) {
    RCLCPP_INFO(logger_, "OMPL planning attempt %d/%d for approach pose",
                ompl_attempt, config_.max_ompl_planning_attempts);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    auto plan_result = move_group_->plan(plan);

    if (plan_result != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_WARN(logger_, "OMPL planning attempt %d failed", ompl_attempt);
      continue;
    }

    const auto & trajectory = plan.trajectory.joint_trajectory;
    if (trajectory.points.empty()) {
      RCLCPP_ERROR(logger_, "Planned trajectory has no points");
      continue;
    }

    const auto & final_point = trajectory.points.back();
    if (final_point.positions.size() < 6) {
      RCLCPP_ERROR(logger_, "Final point has insufficient joint values: %zu",
                   final_point.positions.size());
      continue;
    }

    Eigen::Matrix<double, 6, 1> q_approach;
    for (size_t i = 0; i < 6; ++i) {
      q_approach(i) = final_point.positions[i];
    }

    if (config_.use_approach_validator) {
      for (int val_attempt = 1; val_attempt <= config_.max_approach_validation_retries;
        ++val_attempt)
      {
        RCLCPP_INFO(logger_, "Validation attempt %d/%d for OMPL plan %d",
                    val_attempt, config_.max_approach_validation_retries, ompl_attempt);

        if (approach_validator_->is_approach_valid(q_approach)) {
          RCLCPP_INFO(logger_, "Approach configuration validated! Executing plan");

          auto execute_result = move_group_->execute(plan);
          if (execute_result == moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_INFO(logger_, "Approach complete (OMPL attempt %d, validation attempt %d)",
                        ompl_attempt, val_attempt);
            return true;
          } else {
            RCLCPP_ERROR(logger_, "Execution failed despite valid plan");
            return false;
          }
        } else {
          RCLCPP_WARN(logger_, "Validation attempt %d/%d failed",
                      val_attempt, config_.max_approach_validation_retries);
        }
      }
      RCLCPP_WARN(logger_, "All %d validation attempts failed for OMPL plan %d",
                  config_.max_approach_validation_retries, ompl_attempt);
    } else {
      auto execute_result = move_group_->execute(plan);
      if (execute_result == moveit::core::MoveItErrorCode::SUCCESS) {
        return true;
      }
    }
  }

  RCLCPP_ERROR(logger_, "Failed to find valid approach after %d OMPL attempts",
               config_.max_ompl_planning_attempts);
  return false;
}

bool WelderActionServer::retract_from_seam(const geometry_msgs::msg::Pose & last_pose)
{
  geometry_msgs::msg::Pose retract_pose = last_pose;
  retract_pose.position.z += config_.retract_offset_z;

  RCLCPP_INFO(logger_, "Retracting from seam to (%.3f, %.3f, %.3f)",
               retract_pose.position.x,
               retract_pose.position.y,
               retract_pose.position.z);

  move_group_->setPoseTarget(retract_pose);
  auto result = move_group_->move();
  if (result == moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_INFO(logger_, "Retract complete");
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
  RCLCPP_INFO(logger_, "Planning cartesian path with %zu waypoints", waypoints.size());

  moveit_msgs::msg::RobotTrajectory trajectory;
  double fraction = move_group_->computeCartesianPath(
        waypoints, config_.cartesian_step_size, trajectory);

  RCLCPP_INFO(logger_, "Cartesian path: %.2f%% achieved", fraction * 100.0);

  if (fraction < config_.cartesian_path_threshold) {
    RCLCPP_ERROR(logger_, "Cartesian path below threshold (%.2f%% < %.2f%%)",
                  fraction * 100.0, config_.cartesian_path_threshold * 100.0);
    return false;
  }

  auto execute_result = move_group_->execute(trajectory);
  if (execute_result != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(logger_, "Execution failed");
    return false;
  }

  int32_t points_after_seam = points_before_seam + static_cast<int32_t>(waypoints.size());
  feedback->completion_percentage = (static_cast<float>(points_after_seam) / total_waypoints) *
    100.0f;
  goal_handle->publish_feedback(feedback);

  RCLCPP_INFO(logger_, "Cartesian path executed successfully");
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

geometry_msgs::msg::Pose WelderActionServer::transform_pose_to_base_frame(
  const geometry_msgs::msg::Pose & world_pose,
  const Eigen::Isometry3d & base_to_world_transform)
{
  Eigen::Isometry3d eigen_world_pose = Eigen::Isometry3d::Identity();
  eigen_world_pose.translation() << world_pose.position.x,
    world_pose.position.y, world_pose.position.z;
  Eigen::Quaterniond q_world(
    world_pose.orientation.w, world_pose.orientation.x,
    world_pose.orientation.y, world_pose.orientation.z);
  eigen_world_pose.linear() = q_world.toRotationMatrix();

  Eigen::Isometry3d eigen_base_pose = base_to_world_transform * eigen_world_pose;

  geometry_msgs::msg::Pose base_pose;
  base_pose.position.x = eigen_base_pose.translation().x();
  base_pose.position.y = eigen_base_pose.translation().y();
  base_pose.position.z = eigen_base_pose.translation().z();

  Eigen::Quaterniond q_base(eigen_base_pose.rotation());
  base_pose.orientation.w = q_base.w();
  base_pose.orientation.x = q_base.x();
  base_pose.orientation.y = q_base.y();
  base_pose.orientation.z = q_base.z();

  return base_pose;
}

}  // namespace hold_and_weld
