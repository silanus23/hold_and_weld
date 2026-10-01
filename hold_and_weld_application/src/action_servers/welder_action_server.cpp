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
#include <cmath>
#include <cstdio>
#include <fstream>
#include <functional>
#include <filesystem>
#include <sstream>
#include <utility>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <controller_manager_msgs/srv/list_controllers.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/position_constraint.hpp>
#include <moveit_msgs/srv/get_cartesian_path.hpp>
#include <rclcpp/parameter_client.hpp>
#include <tf2_eigen/tf2_eigen.hpp>


namespace hold_and_weld
{
namespace application
{

namespace
{

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

WeldSeam seam_in_base_frame(const WeldSeam & seam, const moveit::core::RobotStatePtr & state)
{
  const Eigen::Isometry3d base_from_world =
    state->getGlobalLinkTransform("robot2_base_link").inverse();
  WeldSeam base_seam = seam;
  for (auto & pose : base_seam.poses) {
    Eigen::Isometry3d world_iso;
    tf2::fromMsg(pose, world_iso);
    pose = tf2::toMsg(Eigen::Isometry3d(base_from_world * world_iso));
  }
  return base_seam;
}

std::string default_welding_yaml()
{
  try {
    return ament_index_cpp::get_package_share_directory("hold_and_weld_bringup") +
           "/config/tasks/welding.yaml";
  } catch (const std::exception &) {
    return "";
  }
}

}  // namespace

std::string WelderConfig::validate() const
{
  auto bad = [](const std::string & field, double value, const std::string & rule) {
      return field + " must be " + rule + ", got " + std::to_string(value);
    };
  // Each check is written so that NaN fails it.
  if (welder_group_name.empty()) {
    return "welder_group_name must not be empty";
  }
  if (!(std::isfinite(approach_offset_z) && approach_offset_z > 0.0)) {
    return bad("approach_offset_z", approach_offset_z, "positive");
  }
  if (!(cartesian_path_threshold > 0.0 && cartesian_path_threshold <= 1.0)) {
    return bad("cartesian_path_threshold", cartesian_path_threshold, "in (0, 1]");
  }
  if (!(std::isfinite(cartesian_step_size) && cartesian_step_size > 0.0)) {
    return bad("cartesian_step_size", cartesian_step_size, "positive");
  }
  if (!(velocity_scaling > 0.0 && velocity_scaling <= 1.0)) {
    return bad("velocity_scaling", velocity_scaling, "in (0, 1]");
  }
  if (!(std::isfinite(goal_position_tolerance) && goal_position_tolerance > 0.0)) {
    return bad("goal_position_tolerance", goal_position_tolerance, "positive");
  }
  if (!(std::isfinite(goal_orientation_tolerance) && goal_orientation_tolerance > 0.0)) {
    return bad("goal_orientation_tolerance", goal_orientation_tolerance, "positive");
  }
  if (max_ompl_planning_attempts < 1) {
    return bad("max_ompl_planning_attempts", max_ompl_planning_attempts, ">= 1");
  }
  if (max_cartesian_retries < 1) {
    return bad("max_cartesian_retries", max_cartesian_retries, ">= 1");
  }
  if (!(std::isfinite(manipulability_threshold) && manipulability_threshold >= 0.0)) {
    return bad("manipulability_threshold", manipulability_threshold, "non-negative");
  }
  if (finder_max_ompl_candidates < 1) {
    return bad("finder.max_ompl_candidates", finder_max_ompl_candidates, ">= 1");
  }
  for (const auto & [joint, position] : home_configuration) {
    if (!std::isfinite(position)) {
      return "home_configuration." + joint + " must be finite";
    }
  }
  return "";
}

WelderActionServer::WelderActionServer(const rclcpp::NodeOptions & options)
: LifecycleNode("welder_action_server", options),
  logger_(rclcpp::get_logger("application"))
{
  // Declared here, not in on_configure, so configure -> cleanup -> configure
  // does not throw ParameterAlreadyDeclaredException.
  declare_parameter("auto_trigger", false);
  declare_parameter("auto_trigger_delay_sec", 3.0);
  declare_parameter("welding_config_yaml", default_welding_yaml());
  declare_parameter("welder_group_name", "");
  declare_parameter("auto_load_latest", true);
  declare_parameter("trajectory_directory", "");
  declare_parameter("shutdown_wait_sec", 5.0);
}

WelderActionServer::~WelderActionServer()
{
  // Normally already done by the pre-shutdown callback registered in main().
  manual_shutdown();

  if (moveit_executor_) {
    moveit_executor_->cancel();
  }
  if (moveit_thread_.joinable()) {
    moveit_thread_.join();
  }
}

void WelderActionServer::manual_shutdown()
{
  if (manual_shutdown_done_.exchange(true)) {
    return;
  }
  RCLCPP_INFO(logger_, "Manual shutdown: stopping the arm and the worker");

  request_stop();

  if (!wait_for_running_job(shutdown_wait_time_)) {
    // Joining would hang the exit. The process is going away, so detach; the thread may
    // still touch this node until it does.
    RCLCPP_WARN(
      logger_, "Weld job still running %.1f s after stop(); detaching the worker thread",
      std::chrono::duration<double>(shutdown_wait_time_).count());
    {
      std::lock_guard<std::mutex> lock(execution_mutex_);
      shutdown_requested_ = true;
    }
    worker_cv_.notify_all();
    if (worker_thread_.joinable()) {
      worker_thread_.detach();
    }
    return;
  }

  shutdown_worker();
}

void WelderActionServer::request_stop()
{
  stop_requested_ = true;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group;
  {
    std::lock_guard<std::mutex> lock(move_group_mutex_);
    move_group = move_group_;
  }
  try {
    if (move_group) {
      move_group->stop();
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger_, "Failed to stop move group: %s", e.what());
  }
}

bool WelderActionServer::wait_for_running_job(std::optional<std::chrono::nanoseconds> timeout)
{
  std::shared_future<void> job;
  {
    std::lock_guard<std::mutex> lock(execution_mutex_);
    job = execution_future_;
  }
  if (!job.valid()) {
    return true;
  }
  if (!timeout) {
    job.wait();
    return true;
  }
  return job.wait_for(*timeout) == std::future_status::ready;
}

CallbackReturn WelderActionServer::on_configure(const rclcpp_lifecycle::State & state)
{
  auto_trigger_ = get_parameter("auto_trigger").as_bool();
  auto_trigger_delay_sec_ = get_parameter("auto_trigger_delay_sec").as_double();
  if (!hold_and_weld::is_valid_auto_trigger_delay(auto_trigger_delay_sec_)) {
    RCLCPP_ERROR(logger_, "auto_trigger_delay_sec must be in [0, 3600], got %.3f",
      auto_trigger_delay_sec_);
    return CallbackReturn::FAILURE;
  }
  const double shutdown_wait_sec = get_parameter("shutdown_wait_sec").as_double();
  if (!(shutdown_wait_sec > 0.0 && shutdown_wait_sec <= 60.0)) {
    RCLCPP_ERROR(logger_, "shutdown_wait_sec must be in (0, 60], got %.3f", shutdown_wait_sec);
    return CallbackReturn::FAILURE;
  }
  shutdown_wait_time_ = hold_and_weld::to_nanoseconds(shutdown_wait_sec);
  auto_load_latest_ = get_parameter("auto_load_latest").as_bool();
  trajectory_directory_ = get_parameter("trajectory_directory").as_string();

  WelderConfig config;
  if (!load_config_from_yaml(get_parameter("welding_config_yaml").as_string(), config)) {
    return CallbackReturn::FAILURE;
  }
  const std::string group_override = get_parameter("welder_group_name").as_string();
  if (!group_override.empty() && group_override != config.welder_group_name) {
    RCLCPP_INFO(logger_, "welder_group_name parameter '%s' overrides welding.yaml's '%s'",
      group_override.c_str(), config.welder_group_name.c_str());
    config.welder_group_name = group_override;
  }
  if ((config.json_file.empty() || config.json_file == "auto") && !auto_load_latest_) {
    RCLCPP_ERROR(logger_, "welding.yaml sets no json_file and auto_load_latest is false: "
      "there is no weld path to load");
    return CallbackReturn::FAILURE;
  }
  config_ = config;

  // This lifecycle node is not spinning freely during on_configure, so service calls on
  // 'this' would deadlock.
  auto temp_node = std::make_shared<rclcpp::Node>("welder_service_waiter");
  auto cartesian_path_client = temp_node->create_client<moveit_msgs::srv::GetCartesianPath>(
    "/compute_cartesian_path");

  RCLCPP_INFO(logger_, "Waiting for MoveIt compute_cartesian_path service");
  if (!hold_and_weld::wait_for_service(cartesian_path_client, "MoveIt compute_cartesian_path",
      logger_))
  {
    return CallbackReturn::FAILURE;
  }
  RCLCPP_INFO(logger_, "MoveIt is available");

  auto list_controllers_client = temp_node->create_client<
    controller_manager_msgs::srv::ListControllers>("/controller_manager/list_controllers");

  RCLCPP_INFO(logger_, "Waiting for controller_manager service");
  if (!hold_and_weld::wait_for_service(list_controllers_client,
      "controller_manager/list_controllers", logger_))
  {
    return CallbackReturn::FAILURE;
  }
  RCLCPP_INFO(logger_, "Controllers are ready");

  if (config_.use_approach_validator) {
    RCLCPP_WARN(logger_, "use_approach_validation is a legacy option; prefer "
      "use_configuration_finder, which takes precedence on the approach when both are set");
  }

  // MoveGroupInterface builds its robot model from robot_description, so this is needed
  // whether or not the kinematics solvers are.
  RCLCPP_DEBUG(logger_, "Fetching robot_description from robot_state_publisher");
  auto param_client = std::make_shared<rclcpp::SyncParametersClient>(temp_node,
      "robot_state_publisher");
  if (!param_client->wait_for_service(std::chrono::seconds(10))) {
    RCLCPP_ERROR(logger_, "Failed to contact robot_state_publisher; is it running?");
    return CallbackReturn::FAILURE;
  }
  std::string urdf_string;
  auto parameters = param_client->get_parameters({"robot_description"});
  if (!parameters.empty()) {
    urdf_string = parameters[0].as_string();
  }
  if (urdf_string.empty()) {
    RCLCPP_ERROR(logger_, "Retrieved robot_description is empty");
    return CallbackReturn::FAILURE;
  }

  try {
    RCLCPP_INFO(logger_, "Initializing MoveIt");

    // The launch file's parameters are /** overrides, so the internal node picks up
    // robot_description_semantic (and the rest) by auto-declaring them. Do not declare
    // them on this lifecycle node as well: the internal node's own declare would throw.
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);

    // MoveGroupInterface takes an rclcpp::Node, not a LifecycleNode, hence this plain node.
    // It outlives this scope: MoveGroupInterface keeps its own shared_ptr to it (the
    // executor holds only a weak one), so it lives until move_group_ is reset.
    auto internal_node = std::make_shared<rclcpp::Node>(
      "welder_moveit_internal",
      node_options);

    bool use_sim_time = false;
    if (this->get_parameter("use_sim_time", use_sim_time)) {
      internal_node->set_parameter(rclcpp::Parameter("use_sim_time", use_sim_time));
      RCLCPP_DEBUG(logger_, "Copied use_sim_time=%s to internal node",
          use_sim_time ? "true" : "false");
    }

    internal_node->declare_parameter("robot_description", urdf_string);

    moveit_executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    moveit_executor_->add_node(internal_node);
    moveit_thread_ = std::thread([this]() {moveit_executor_->spin();});

    auto move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
      internal_node, config_.welder_group_name);
    move_group->setPlanningTime(10.0);
    move_group->setNumPlanningAttempts(10);
    move_group->setMaxVelocityScalingFactor(config_.velocity_scaling);
    {
      std::lock_guard<std::mutex> lock(move_group_mutex_);
      move_group_ = move_group;
    }

    RCLCPP_INFO(logger_, "MoveIt initialized successfully");
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger_, "Failed to initialize MoveIt: %s", e.what());
    // on_cleanup() is null-safe, so it also undoes a partial configure.
    on_cleanup(state);
    return CallbackReturn::FAILURE;
  }

  if (config_.use_approach_validator || config_.use_configuration_finder) {
    try {
      RCLCPP_INFO(logger_, "Initializing kinematics solvers");

      const auto * joint_model_group = move_group_->getRobotModel()->getJointModelGroup(
        config_.welder_group_name);
      if (!joint_model_group) {
        throw std::runtime_error(
                "joint model group '" + config_.welder_group_name + "' not found in robot model");
      }

      auto urdf_parser = std::make_unique<hold_and_weld::kinematics::URDFParser>();

      hold_and_weld::kinematics::ParsedChain parsed_chain;
      parsed_chain = urdf_parser->extract_joint_chain_from_string(
        urdf_string, "robot2_base_link", "robot2_wire_tip");

      RCLCPP_DEBUG(logger_,
        "Parsed kinematic chain robot2_base_link -> robot2_wire_tip with %zu actuated joints",
        parsed_chain.actuated_joints.size());

      kinematics_solver_ =
        std::make_shared<hold_and_weld::kinematics::KinematicsSolver>(parsed_chain);

      // Solver joint vectors are exchanged with MoveIt by index, so both must list the same
      // joints in the same order (KinematicsSolver already enforces 6 of them).
      const auto & joint_names = joint_model_group->getVariableNames();
      std::vector<std::string> chain_names;
      for (const auto & joint : parsed_chain.actuated_joints) {
        chain_names.push_back(joint.name);
      }
      if (joint_names != chain_names) {
        throw std::runtime_error(
                "MoveIt group " + config_.welder_group_name +
                " and the robot2_base_link -> robot2_wire_tip chain disagree on joints or order");
      }

      ceres_solver_ =
        std::make_shared<hold_and_weld::kinematics::CeresIKSolver>(kinematics_solver_);

      auto validator_params = config_.approach_validator;
      validator_params.manipulability_threshold = config_.manipulability_threshold;
      approach_validator_ =
        std::make_unique<hold_and_weld::kinematics::ApproachValidator>(
        kinematics_solver_,
        ceres_solver_,
        validator_params);

      if (config_.use_configuration_finder) {
        const std::string ee_link = move_group_->getEndEffectorLink();
        if (ee_link != "robot2_wire_tip") {
          throw std::runtime_error(
                  "configuration finder models robot2_wire_tip but Pilz targets '" + ee_link +
                  "'; fix the SRDF or set use_configuration_finder: false");
        }

        const auto & limits = kinematics_solver_->joint_limits();
        for (size_t i = 0; i < 6; ++i) {
          auto it = config_.home_configuration.find(joint_names[i]);
          if (it == config_.home_configuration.end()) {
            throw std::runtime_error(
                    "home_configuration (or safety_pose.joint_positions) is missing joint '" +
                    joint_names[i] + "'");
          }
          if (!(limits[i].first <= it->second && it->second <= limits[i].second)) {
            throw std::runtime_error(
                    "home_configuration joint '" + joint_names[i] + "' = " +
                    std::to_string(it->second) + " is outside its limits [" +
                    std::to_string(limits[i].first) + ", " + std::to_string(limits[i].second) +
                    "]");
          }
          q_home_(i) = it->second;
        }

        auto finder_params = config_.finder;
        finder_params.manipulability_threshold = config_.manipulability_threshold;
        configuration_finder_ = std::make_unique<hold_and_weld::kinematics::ConfigurationFinder>(
          kinematics_solver_, ceres_solver_, finder_params);
      }

      RCLCPP_INFO(logger_, "Kinematics solvers initialized");
    } catch (const std::exception & e) {
      RCLCPP_ERROR(logger_, "Failed to initialize kinematics: %s", e.what());
      on_cleanup(state);
      return CallbackReturn::FAILURE;
    }
  }

  using std::placeholders::_1;
  using std::placeholders::_2;
  action_server_ = rclcpp_action::create_server<TriggerWelder>(
    this,
    "trigger_welder",
    std::bind(&WelderActionServer::handle_goal, this, _1, _2),
    std::bind(&WelderActionServer::handle_cancel, this, _1),
    std::bind(&WelderActionServer::handle_accepted, this, _1)
  );

  self_trigger_client_ = rclcpp_action::create_client<TriggerWelder>(
    this->get_node_base_interface(),
    this->get_node_graph_interface(),
    this->get_node_logging_interface(),
    this->get_node_waitables_interface(),
    "trigger_welder");

  {
    std::lock_guard<std::mutex> lock(execution_mutex_);
    shutdown_requested_ = false;
    execution_future_ = std::shared_future<void>();
  }
  stop_requested_ = false;
  auto_trigger_fired_ = false;
  worker_thread_ = std::thread(&WelderActionServer::worker_thread_func, this);

  return CallbackReturn::SUCCESS;
}

CallbackReturn WelderActionServer::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(logger_, "Activating welder action server");
  if (auto_trigger_ && !auto_trigger_fired_) {
    RCLCPP_INFO(logger_, "Auto-trigger enabled, will start in %.1f seconds",
      auto_trigger_delay_sec_);

    auto_trigger_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(auto_trigger_delay_sec_)),
      [this]() {send_auto_trigger_goal();});
  }

  return CallbackReturn::SUCCESS;
}

void WelderActionServer::send_auto_trigger_goal()
{
  auto_trigger_timer_->cancel();
  auto_trigger_fired_ = true;
  RCLCPP_INFO(logger_, "Auto-triggering welder job via trigger_welder action");

  if (!hold_and_weld::wait_for_action_server(
      self_trigger_client_, "trigger_welder", logger_, 10))
  {
    return;
  }

  auto goal_msg = TriggerWelder::Goal();
  auto send_goal_options = rclcpp_action::Client<TriggerWelder>::SendGoalOptions();
  send_goal_options.result_callback =
    [this](const rclcpp_action::ClientGoalHandle<TriggerWelder>::WrappedResult & result)
    {
      const char * message = result.result ? result.result->message.c_str() : "";
      if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
        RCLCPP_INFO(logger_, "Auto-triggered welder job succeeded: %s", message);
      } else {
        RCLCPP_ERROR(logger_, "Auto-triggered welder job failed: %s", message);
      }
    };

  self_trigger_client_->async_send_goal(goal_msg, send_goal_options);
}

CallbackReturn WelderActionServer::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(logger_, "Deactivating welder action server");
  if (auto_trigger_timer_) {
    auto_trigger_timer_->cancel();
    auto_trigger_timer_.reset();
  }

  // The worker takes a goal and publishes execution_future_ under execution_mutex_, so
  // once the queued goal is gone here, the only job left to wait for is a running one.
  std::shared_ptr<GoalHandleTriggerWelder> queued;
  {
    std::lock_guard<std::mutex> lock(execution_mutex_);
    queued = std::exchange(pending_goal_, nullptr);
  }
  hold_and_weld::end_goal_early<TriggerWelder>(
    queued, "Welder server deactivated before the goal started", logger_);

  request_stop();
  wait_for_running_job();

  return CallbackReturn::SUCCESS;
}

CallbackReturn WelderActionServer::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(logger_, "Cleaning up welder action server");
  if (auto_trigger_timer_) {
    auto_trigger_timer_->cancel();
    auto_trigger_timer_.reset();
  }

  // Before moveit_executor_ is cancelled below: it delivers stop() to the controller.
  request_stop();
  shutdown_worker();

  action_server_.reset();
  self_trigger_client_.reset();

  {
    std::lock_guard<std::mutex> lock(move_group_mutex_);
    move_group_.reset();
  }

  try {
    if (moveit_executor_) {
      moveit_executor_->cancel();
    }
    if (moveit_thread_.joinable()) {
      moveit_thread_.join();
    }
    moveit_executor_.reset();
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger_, "Failed to cleanup MoveIt executor: %s", e.what());
  }

  configuration_finder_.reset();
  approach_validator_.reset();
  kinematics_solver_.reset();
  ceres_solver_.reset();
  return CallbackReturn::SUCCESS;
}

CallbackReturn WelderActionServer::on_shutdown(const rclcpp_lifecycle::State & state)
{
  // Reachable from any primary state; on_cleanup() is null-safe for whatever was never set up.
  RCLCPP_INFO(logger_, "Shutting down welder action server");
  return on_cleanup(state);
}

bool WelderActionServer::load_config_from_yaml(
  const std::string & yaml_path, WelderConfig & config) const
{
  RCLCPP_INFO(logger_, "Loading config from: %s", yaml_path.c_str());
  if (yaml_path.empty() || !std::filesystem::exists(yaml_path)) {
    RCLCPP_ERROR(logger_, "welding config not found: '%s'", yaml_path.c_str());
    return false;
  }

  WelderConfig parsed;
  try {
    YAML::Node yaml = YAML::LoadFile(yaml_path);

    if (yaml["welder_group_name"]) {
      parsed.welder_group_name = yaml["welder_group_name"].as<std::string>();
    }
    if (yaml["approach_offset_z"]) {
      parsed.approach_offset_z = yaml["approach_offset_z"].as<double>();
    }
    if (yaml["use_approach_validation"]) {
      parsed.use_approach_validator = yaml["use_approach_validation"].as<bool>();
    }
    if (yaml["cartesian_path_threshold"]) {
      parsed.cartesian_path_threshold = yaml["cartesian_path_threshold"].as<double>();
    }
    if (yaml["cartesian_step_size"]) {
      parsed.cartesian_step_size = yaml["cartesian_step_size"].as<double>();
    }
    if (yaml["velocity_scaling"]) {
      parsed.velocity_scaling = yaml["velocity_scaling"].as<double>();
    }
    if (yaml["max_ompl_planning_attempts"]) {
      parsed.max_ompl_planning_attempts = yaml["max_ompl_planning_attempts"].as<int>();
    }
    if (yaml["goal_position_tolerance"]) {
      parsed.goal_position_tolerance = yaml["goal_position_tolerance"].as<double>();
    }
    if (yaml["goal_orientation_tolerance"]) {
      parsed.goal_orientation_tolerance = yaml["goal_orientation_tolerance"].as<double>();
    }
    if (yaml["max_approach_validation_retries"]) {
      RCLCPP_WARN(logger_, "max_approach_validation_retries is no longer used: the approach "
        "validator is deterministic, so a rejected plan is replanned by OMPL instead");
    }
    if (yaml["max_cartesian_retries"]) {
      parsed.max_cartesian_retries = yaml["max_cartesian_retries"].as<int>();
    }
    if (yaml["use_pilz"]) {
      parsed.use_pilz = yaml["use_pilz"].as<bool>();
    }
    if (yaml["json_file"]) {
      parsed.json_file = yaml["json_file"].as<std::string>();
    }
    if (yaml["manipulability_threshold"]) {
      parsed.manipulability_threshold = yaml["manipulability_threshold"].as<double>();
    }
    if (yaml["use_configuration_finder"]) {
      parsed.use_configuration_finder = yaml["use_configuration_finder"].as<bool>();
    }
    if (yaml["home_configuration"]) {
      parsed.home_configuration =
        yaml["home_configuration"].as<std::map<std::string, double>>();
    } else if (yaml["safety_pose"] && yaml["safety_pose"]["joint_positions"]) {
      parsed.home_configuration =
        yaml["safety_pose"]["joint_positions"].as<std::map<std::string, double>>();
    }
    if (const YAML::Node finder = yaml["finder"]) {
      auto read = [&finder](const char * key, double & value) {
          if (finder[key]) {value = finder[key].as<double>();}
        };
      auto & f = parsed.finder;
      read("path_step", f.path_step);
      read("path_step_rot", f.path_step_rot);
      read("max_joint_step", f.max_joint_step);
      read("branch_seed_weight", f.branch_seed_weight);
      read("walk_seed_weight", f.walk_seed_weight);
      read("tol_pos", f.tol_pos);
      read("tol_rot", f.tol_rot);
      read("w_limit_margin", f.w_limit_margin);
      read("w_manipulability", f.w_manipulability);
      read("w_home", f.w_home);
      read("dedupe_epsilon", f.dedupe_epsilon);
      if (finder["max_ompl_candidates"]) {
        parsed.finder_max_ompl_candidates = finder["max_ompl_candidates"].as<int>();
      }
    }
    if (const YAML::Node validator = yaml["approach_validator"]) {
      auto read = [&validator](const char * key, double & value) {
          if (validator[key]) {value = validator[key].as<double>();}
        };
      auto & v = parsed.approach_validator;
      read("first_point_tol_pos", v.first_point_tol_pos);
      read("first_point_tol_rot", v.first_point_tol_rot);
      read("seam_tol_pos", v.seam_tol_pos);
      read("seam_tol_rot", v.seam_tol_rot);
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger_, "Failed to parse %s: %s", yaml_path.c_str(), e.what());
    return false;
  }

  const std::string error = parsed.validate();
  if (!error.empty()) {
    RCLCPP_ERROR(logger_, "Invalid %s: %s", yaml_path.c_str(), error.c_str());
    return false;
  }
  config = parsed;
  RCLCPP_INFO(logger_, "Configuration loaded successfully");
  return true;
}

void WelderActionServer::worker_thread_func()
{
  while (true) {
    std::shared_ptr<GoalHandleTriggerWelder> goal_handle;
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
      // Published under the lock on_deactivate takes, so a transition either sees this
      // job's future or has already removed the goal before the worker got it.
      execution_future_ = done.get_future().share();
      stop_requested_ = false;
    }

    try {
      execute_weld(goal_handle);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(logger_, "Weld job failed with an exception: %s", e.what());
      hold_and_weld::end_goal_early<TriggerWelder>(
        goal_handle, std::string("Weld job failed: ") + e.what(), logger_);
    } catch (...) {
      RCLCPP_ERROR(logger_, "Weld job failed with an unknown exception");
      hold_and_weld::end_goal_early<TriggerWelder>(
        goal_handle, "Weld job failed with an unknown exception", logger_);
    }
    // No-op if the goal already ended; catches a path that returned without a result.
    hold_and_weld::end_goal_early<TriggerWelder>(
      goal_handle, "Weld job ended without a result", logger_);
    done.set_value();
  }
}

void WelderActionServer::shutdown_worker()
{
  std::shared_ptr<GoalHandleTriggerWelder> queued;
  {
    std::lock_guard<std::mutex> lock(execution_mutex_);
    shutdown_requested_ = true;
    queued = std::exchange(pending_goal_, nullptr);
  }
  hold_and_weld::end_goal_early<TriggerWelder>(
    queued, "Welder server shut down before the goal started", logger_);
  worker_cv_.notify_all();

  if (worker_thread_.joinable()) {
    worker_thread_.join();
  }
}

std::string WelderActionServer::find_latest_json() const
{
  std::string trajectory_dir = trajectory_directory_;
  if (!trajectory_dir.empty()) {
    RCLCPP_INFO(logger_, "Using trajectory directory from parameter: %s",
                trajectory_dir.c_str());
  } else {
    try {
      std::string pkg_share =
        ament_index_cpp::get_package_share_directory("hold_and_weld_application");
      std::filesystem::path install_share(pkg_share);
      std::filesystem::path ws_root = install_share.parent_path()
        .parent_path()
        .parent_path()
        .parent_path();
      std::filesystem::path src_trajectories =
        ws_root / "src" / "hold_and_weld" / "hold_and_weld_application" / "trajectories";

      if (std::filesystem::exists(src_trajectories)) {
        trajectory_dir = src_trajectories.string();
        RCLCPP_INFO(logger_, "Using source-tree trajectory directory: %s",
                    trajectory_dir.c_str());
      } else {
        trajectory_dir = pkg_share + "/trajectories";
        RCLCPP_INFO(logger_, "Using installed trajectory directory: %s",
                    trajectory_dir.c_str());
      }
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
      if (entry.is_regular_file() && entry.path().extension() == ".json") {
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

WeldJob WelderActionServer::load_seams_from_json(const std::string & filepath) const
{
  std::ifstream file(filepath);
  if (!file.is_open()) {
    RCLCPP_ERROR(logger_, "Failed to open file: %s", filepath.c_str());
    return {};
  }
  std::stringstream contents;
  contents << file.rdbuf();

  try {
    auto parsed = hold_and_weld::parse_weld_seams(contents.str());
    for (const auto & problem : parsed.problems) {
      RCLCPP_WARN(logger_, "%s", problem.c_str());
    }
    RCLCPP_INFO(logger_, "Loaded %zu seams from %s (%zu skipped, %zu partial)",
                parsed.seams.size(), filepath.c_str(), parsed.skipped.size(),
                parsed.partial.size());
    return parsed;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger_, "Rejected %s: %s", filepath.c_str(), e.what());
    return {};
  }
}

rclcpp_action::GoalResponse WelderActionServer::handle_goal(
  [[maybe_unused]] const rclcpp_action::GoalUUID & uuid,
  [[maybe_unused]] std::shared_ptr<const TriggerWelder::Goal> goal)
{
  if (get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    RCLCPP_WARN(logger_, "Cannot accept goal: node is not active");
    return rclcpp_action::GoalResponse::REJECT;
  }

  {
    std::lock_guard<std::mutex> lock(execution_mutex_);
    const bool running = execution_future_.valid() &&
      execution_future_.wait_for(std::chrono::seconds(0)) != std::future_status::ready;
    if (pending_goal_ || running) {
      RCLCPP_WARN(logger_, "Cannot accept goal: a weld job is already queued or running");
      return rclcpp_action::GoalResponse::REJECT;
    }
  }

  RCLCPP_INFO(logger_, "Received welder trigger request");
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse WelderActionServer::handle_cancel(
  [[maybe_unused]] const std::shared_ptr<GoalHandleTriggerWelder> goal_handle)
{
  RCLCPP_INFO(logger_, "Received cancel request");
  // Only one goal exists at a time, so this is the running (or about to run) one. The job
  // sees is_canceling() at its next check and does not start another motion.
  request_stop();

  return rclcpp_action::CancelResponse::ACCEPT;
}

void WelderActionServer::handle_accepted(
  const std::shared_ptr<GoalHandleTriggerWelder> goal_handle)
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
    hold_and_weld::end_goal_early<TriggerWelder>(
      goal_handle, "A weld job is already queued", logger_);
    return;
  }
  worker_cv_.notify_one();
}

void WelderActionServer::execute_weld(const std::shared_ptr<GoalHandleTriggerWelder> goal_handle)
{
  auto feedback = std::make_shared<TriggerWelder::Feedback>();
  auto result = std::make_shared<TriggerWelder::Result>();

  const std::function<bool()> should_stop = [this, &goal_handle]() {
      return stop_requested_.load() || goal_handle->is_canceling();
    };
  auto end_stopped = [this, &goal_handle]() {
      hold_and_weld::end_goal_early<TriggerWelder>(
        goal_handle, goal_handle->is_canceling() ? "Canceled by client" :
        "Stopped: welder server is deactivating or shutting down", logger_);
    };

  if (should_stop()) {
    end_stopped();
    return;
  }

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

  const WeldJob parsed = load_seams_from_json(json_path);
  const std::vector<WeldSeam> & seams = parsed.seams;
  if (seams.empty()) {
    result->success = false;
    result->message = "No valid seams loaded from " + json_path + " (see the server log)";
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
      RCLCPP_INFO(logger_, "[%s] %.1f%% complete",
                   step.c_str(), feedback->completion_percentage);
    };

  std::vector<std::string> succeeded_seams;
  std::vector<std::string> failed_seams;
  int32_t total_points_executed = 0;
  int32_t points_processed = 0;

  RCLCPP_INFO(logger_, "Processing %zu seams", seams.size());

  // After a cancel or a transition the job stops where it is: no further motion,
  // not even a retreat.
  for (size_t seam_idx = 0; seam_idx < seams.size(); ++seam_idx) {
    const auto & seam = seams[seam_idx];
    const auto & waypoints = seam.poses;

    RCLCPP_INFO(logger_, "Seam %zu/%zu: %s (%.3f m, %zu points)",
                 seam_idx + 1, seams.size(), seam.seam_id.c_str(),
                 seam.length_m, seam.num_poses);

    if (should_stop()) {
      end_stopped();
      return;
    }

    publish_progress("approaching_seam_" + seam.seam_id, points_processed);
    if (!move_to_seam_boundary(seam, waypoints.front(), true, should_stop)) {
      if (should_stop()) {
        end_stopped();
        return;
      }
      RCLCPP_ERROR(logger_, "Failed to approach seam %s", seam.seam_id.c_str());
      failed_seams.push_back(seam.seam_id);
      continue;
    }

    if (should_stop()) {
      end_stopped();
      return;
    }

    publish_progress("welding_seam_" + seam.seam_id, points_processed);
    bool torch_left_standoff = false;
    if (!execute_cartesian_path(seam, goal_handle, feedback, points_processed, total_waypoints,
        should_stop, torch_left_standoff))
    {
      if (should_stop()) {
        end_stopped();
        return;
      }
      RCLCPP_ERROR(logger_, "Failed to weld seam %s", seam.seam_id.c_str());
      failed_seams.push_back(seam.seam_id);
      if (torch_left_standoff) {
        publish_progress("retreating_from_seam_" + seam.seam_id, points_processed);
        retreat_from_part(seam.seam_id, should_stop);
      }
      continue;
    }

    points_processed += static_cast<int32_t>(seam.num_poses);
    total_points_executed += static_cast<int32_t>(seam.num_poses);

    if (should_stop()) {
      end_stopped();
      return;
    }

    publish_progress("retracting_from_seam_" + seam.seam_id, points_processed);
    if (!move_to_seam_boundary(seam, waypoints.back(), false, should_stop)) {
      if (should_stop()) {
        end_stopped();
        return;
      }
      RCLCPP_ERROR(logger_, "Failed to retract from seam %s", seam.seam_id.c_str());
      failed_seams.push_back(seam.seam_id);
      retreat_from_part(seam.seam_id, should_stop);
      continue;
    }

    succeeded_seams.push_back(seam.seam_id);
    RCLCPP_INFO(logger_, "Seam %s completed successfully", seam.seam_id.c_str());
  }

  auto join = [](const std::vector<std::string> & ids) {
      std::string joined;
      for (size_t i = 0; i < ids.size(); ++i) {
        joined += (i ? ", " : "") + ids[i];
      }
      return joined;
    };
  std::string msg = "Welding complete. ";
  if (!succeeded_seams.empty()) {
    msg += "Succeeded: " + join(succeeded_seams) + ". ";
  }
  if (!failed_seams.empty()) {
    msg += "Failed: " + join(failed_seams) + ". ";
  }
  // Loading is best effort: what could not be loaded is reported here, but does not by
  // itself fail the job (see parse_weld_seams()).
  if (!parsed.skipped.empty()) {
    msg += "Skipped (unusable in JSON): " + join(parsed.skipped) + ". ";
  }
  if (!parsed.partial.empty()) {
    msg += "Partial (poses dropped from JSON): " + join(parsed.partial) + ". ";
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

bool WelderActionServer::move_to_seam_boundary(
  const WeldSeam & seam, const geometry_msgs::msg::Pose & ref_pose, bool is_approach,
  const std::function<bool()> & should_stop)
{
  // Self-resetting: execute_cartesian_path() may have left the Pilz pipeline/planner
  move_group_->setPlanningPipelineId("ompl");
  move_group_->setPlannerId("");
  move_group_->clearPathConstraints();

  move_group_->setStartStateToCurrentState();
  auto current_state = move_group_->getCurrentState();
  if (!current_state) {
    RCLCPP_ERROR(logger_, "Failed to get current robot state");
    return false;
  }

  if (is_approach && configuration_finder_) {
    return approach_via_configuration_finder(
      seam_in_base_frame(seam, current_state), should_stop);
  }

  // Same standoff maths as the configuration finder, so its approach pose is this one.
  Eigen::Isometry3d ref_iso;
  tf2::fromMsg(ref_pose, ref_iso);
  const Eigen::Vector3d target_pos = hold_and_weld::kinematics::standoff_pose(
    ref_iso, config_.approach_offset_z).translation();

  geometry_msgs::msg::Pose target_pose;
  target_pose.position.x = target_pos.x();
  target_pose.position.y = target_pos.y();
  target_pose.position.z = target_pos.z();
  target_pose.orientation = ref_pose.orientation;

  RCLCPP_DEBUG(logger_, "Boundary Position: (%.3f, %.3f, %.3f)",
               target_pose.position.x, target_pose.position.y, target_pose.position.z);

  const bool validate = is_approach && config_.use_approach_validator;

  move_group_->setPoseTarget(target_pose);
  move_group_->setGoalPositionTolerance(config_.goal_position_tolerance);
  move_group_->setGoalOrientationTolerance(config_.goal_orientation_tolerance);

  std::vector<std::string> ik_joint_names;
  if (validate) {
    approach_validator_->set_weld_seam(seam_in_base_frame(seam, current_state));

    ik_joint_names = move_group_->getRobotModel()
      ->getJointModelGroup(config_.welder_group_name)->getVariableNames();
  }

  for (int ompl_attempt = 1; ompl_attempt <= config_.max_ompl_planning_attempts; ++ompl_attempt) {
    if (should_stop()) {
      return false;
    }
    RCLCPP_INFO(logger_, "OMPL planning attempt %d/%d for %s pose",
                ompl_attempt, config_.max_ompl_planning_attempts,
                is_approach ? "approach" : "retract");

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (move_group_->plan(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_WARN(logger_, "OMPL planning attempt %d failed", ompl_attempt);
      continue;
    }

    if (validate) {
      const auto & trajectory = plan.trajectory.joint_trajectory;
      if (trajectory.points.empty()) {
        RCLCPP_ERROR(logger_, "Planned trajectory has no points");
        continue;
      }
      const auto & final_point = trajectory.points.back();

      Eigen::Matrix<double, 6, 1> q_approach;
      bool joint_mapping_ok = true;
      for (size_t i = 0; i < 6; ++i) {
        auto it = std::find(
          trajectory.joint_names.begin(), trajectory.joint_names.end(), ik_joint_names[i]);
        const size_t traj_idx = static_cast<size_t>(
          std::distance(trajectory.joint_names.begin(), it));
        if (it == trajectory.joint_names.end() || traj_idx >= final_point.positions.size()) {
          RCLCPP_ERROR(logger_,
                       "Required joint '%s' missing from the planned trajectory — "
                       "skipping OMPL attempt %d",
                       ik_joint_names[i].c_str(), ompl_attempt);
          joint_mapping_ok = false;
          break;
        }
        q_approach(i) = final_point.positions[traj_idx];
      }
      if (!joint_mapping_ok) {
        continue;
      }

      // The validator is deterministic, so a rejected plan is replanned, not re-checked.
      if (!approach_validator_->is_approach_valid(q_approach)) {
        RCLCPP_WARN(logger_, "Approach validation rejected OMPL plan %d", ompl_attempt);
        continue;
      }
      RCLCPP_INFO(logger_, "Approach configuration validated; executing plan");
    }

    if (should_stop()) {
      return false;
    }
    // The arm may have moved, so a failed execution ends the move instead of replanning.
    if (move_group_->execute(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(logger_, "Execution failed on OMPL attempt %d", ompl_attempt);
      return false;
    }
    RCLCPP_INFO(logger_, "%s complete (OMPL attempt %d)",
      is_approach ? "Approach" : "Retract", ompl_attempt);
    return true;
  }

  RCLCPP_ERROR(logger_, "Failed to find valid boundary pose after %d OMPL attempts",
               config_.max_ompl_planning_attempts);
  return false;
}

bool WelderActionServer::approach_via_configuration_finder(
  const WeldSeam & seam, const std::function<bool()> & should_stop)
{
  std::vector<Eigen::Isometry3d> path;
  path.reserve(seam.poses.size());
  for (const auto & pose : seam.poses) {
    Eigen::Isometry3d pose_iso;
    tf2::fromMsg(pose, pose_iso);
    path.push_back(pose_iso);
  }

  const auto ranked = configuration_finder_->find(
    path, seam.segment_type, config_.approach_offset_z, q_home_);

  auto describe = [](const hold_and_weld::kinematics::Candidate & c) {
      char buf[256];
      std::snprintf(
        buf, sizeof(buf), "#%zu [%.3f %.3f %.3f %.3f %.3f %.3f]", c.index,
        c.q_start(0), c.q_start(1), c.q_start(2), c.q_start(3), c.q_start(4), c.q_start(5));
      std::string out = buf;
      if (c.walk.feasible) {
        std::snprintf(
          buf, sizeof(buf), " score %.3f (margin %.3f rad, manip %.4f, home %.3f rad)",
          c.score, c.walk.min_limit_margin, c.walk.min_manipulability, c.home_distance);
      } else {
        std::snprintf(
          buf, sizeof(buf), " infeasible: %s at path sample %zu",
          hold_and_weld::kinematics::to_string(c.walk.failure).c_str(), c.walk.fail_index);
      }
      return out + buf;
    };

  size_t feasible = 0;
  for (const auto & c : ranked) {
    feasible += c.walk.feasible ? 1 : 0;
    RCLCPP_DEBUG(logger_, "Seam %s candidate %s", seam.seam_id.c_str(), describe(c).c_str());
  }

  if (feasible == 0) {
    std::string reasons;
    for (const auto & c : ranked) {
      reasons += "\n  " + describe(c);
    }
    RCLCPP_ERROR(
      logger_, "No configuration can weld seam %s (%zu candidates):%s",
      seam.seam_id.c_str(), ranked.size(), ranked.empty() ? " IK found no solution" :
      reasons.c_str());
    return false;
  }

  const size_t to_try = std::min(
    feasible, static_cast<size_t>(config_.finder_max_ompl_candidates));
  for (size_t rank = 0; rank < to_try; ++rank) {
    const auto & candidate = ranked[rank];
    RCLCPP_INFO(
      logger_, "Seam %s: start config rank %zu/%zu %s", seam.seam_id.c_str(), rank + 1,
      feasible, describe(candidate).c_str());

    const std::vector<double> joint_goal(
      candidate.q_start.data(), candidate.q_start.data() + candidate.q_start.size());
    if (!move_group_->setJointValueTarget(joint_goal)) {
      RCLCPP_WARN(logger_, "Seam %s: joint goal rejected by MoveIt", seam.seam_id.c_str());
      continue;
    }

    for (int attempt = 1; attempt <= config_.max_ompl_planning_attempts; ++attempt) {
      if (should_stop()) {
        return false;
      }
      moveit::planning_interface::MoveGroupInterface::Plan plan;
      if (move_group_->plan(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_WARN(
          logger_, "Seam %s: OMPL attempt %d/%d to rank %zu failed", seam.seam_id.c_str(),
          attempt, config_.max_ompl_planning_attempts, rank + 1);
        continue;
      }
      if (should_stop()) {
        return false;
      }
      // The arm may have moved, so a failed execution ends the approach.
      if (move_group_->execute(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_ERROR(logger_, "Seam %s: approach execution failed", seam.seam_id.c_str());
        return false;
      }
      RCLCPP_INFO(logger_, "Seam %s: approach complete at rank %zu", seam.seam_id.c_str(),
        rank + 1);
      return true;
    }
  }

  RCLCPP_ERROR(
    logger_, "Seam %s: %zu feasible configs, OMPL could not reach any (tried top %zu)",
    seam.seam_id.c_str(), feasible, to_try);
  return false;
}

WelderActionServer::MotionOutcome WelderActionServer::plan_and_execute_pilz(
  const std::string & planner_id,
  const geometry_msgs::msg::Pose & target,
  const moveit_msgs::msg::Constraints * path_constraints,
  const std::string & seam_id,
  const std::function<bool()> & should_stop)
{
  move_group_->setPlanningPipelineId("pilz_industrial_motion_planner");
  move_group_->setPlannerId(planner_id);
  move_group_->setPoseTarget(target);
  if (path_constraints) {
    move_group_->setPathConstraints(*path_constraints);
  } else {
    move_group_->clearPathConstraints();
  }

  // Only planning is retried: nothing has moved yet, so another attempt is harmless.
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool planned = false;
  for (int attempt = 1; attempt <= config_.max_cartesian_retries && !should_stop(); ++attempt) {
    move_group_->setStartStateToCurrentState();
    if (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
      planned = true;
      break;
    }
    RCLCPP_WARN(logger_, "Seam %s: %s planning attempt %d/%d failed", seam_id.c_str(),
      planner_id.c_str(), attempt, config_.max_cartesian_retries);
  }

  MotionOutcome outcome = MotionOutcome::kPlanningFailed;
  if (!planned) {
    RCLCPP_ERROR(logger_, "Seam %s: %s planning failed", seam_id.c_str(), planner_id.c_str());
  } else if (!should_stop()) {
    if (move_group_->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
      outcome = MotionOutcome::kSucceeded;
    } else {
      RCLCPP_ERROR(logger_, "Seam %s: %s execution failed", seam_id.c_str(), planner_id.c_str());
      outcome = MotionOutcome::kExecutionFailed;
    }
  }

  move_group_->clearPathConstraints();
  return outcome;
}

WelderActionServer::MotionOutcome WelderActionServer::plan_and_execute_cartesian(
  const std::vector<geometry_msgs::msg::Pose> & waypoints,
  const std::string & seam_id,
  const std::function<bool()> & should_stop)
{
  moveit_msgs::msg::RobotTrajectory trajectory;
  bool planned = false;
  for (int attempt = 1; attempt <= config_.max_cartesian_retries && !should_stop(); ++attempt) {
    move_group_->setStartStateToCurrentState();
    const double fraction = move_group_->computeCartesianPath(
      waypoints, config_.cartesian_step_size, trajectory);
    RCLCPP_INFO(logger_, "Seam %s: Cartesian path %.2f%% achieved", seam_id.c_str(),
      fraction * 100.0);
    if (fraction >= config_.cartesian_path_threshold) {
      planned = true;
      break;
    }
    RCLCPP_WARN(logger_, "Cartesian path below threshold (%.2f%% < %.2f%%), attempt %d/%d",
      fraction * 100.0, config_.cartesian_path_threshold * 100.0,
      attempt, config_.max_cartesian_retries);
  }
  if (!planned || should_stop()) {
    return MotionOutcome::kPlanningFailed;
  }
  if (move_group_->execute(trajectory) != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(logger_, "Seam %s: Cartesian path execution failed", seam_id.c_str());
    return MotionOutcome::kExecutionFailed;
  }
  return MotionOutcome::kSucceeded;
}

bool WelderActionServer::execute_cartesian_path(
  const WeldSeam & seam,
  const std::shared_ptr<GoalHandleTriggerWelder> & goal_handle,
  std::shared_ptr<TriggerWelder::Feedback> & feedback,
  int32_t points_before_seam,
  int32_t total_waypoints,
  const std::function<bool()> & should_stop,
  bool & torch_left_standoff)
{
  const auto & waypoints = seam.poses;
  torch_left_standoff = false;

  if (config_.use_pilz && (seam.segment_type == "line" || seam.segment_type == "arc")) {
    // parse_weld_seams() guarantees >= 3 poses for an arc, >= 2 otherwise.
    const bool is_arc = (seam.segment_type == "arc");

    // move_to_seam_boundary() leaves the torch backed off by approach_offset_z,
    // so plunge onto the seam start first; LIN/CIRC must start exactly on the seam.
    RCLCPP_INFO(logger_, "Seam %s: LIN plunge to seam start via Pilz", seam.seam_id.c_str());
    const MotionOutcome plunge =
      plan_and_execute_pilz("LIN", waypoints.front(), nullptr, seam.seam_id, should_stop);
    torch_left_standoff = plunge != MotionOutcome::kPlanningFailed;
    if (plunge != MotionOutcome::kSucceeded) {
      return false;
    }

    MotionOutcome weld = MotionOutcome::kPlanningFailed;
    if (!is_arc) {
      RCLCPP_INFO(logger_, "Seam %s: executing LIN weld motion via Pilz", seam.seam_id.c_str());
      weld = plan_and_execute_pilz("LIN", waypoints.back(), nullptr, seam.seam_id, should_stop);
    } else {
      // "interim" rather than "center": a center point is ambiguous for arcs >= 180 deg
      // (Pilz takes the short way round, and at exactly 180 deg the points are colinear).
      const auto & interim_pose = waypoints[waypoints.size() / 2];

      moveit_msgs::msg::PositionConstraint interim_constraint;
      interim_constraint.header.frame_id = move_group_->getPlanningFrame();
      interim_constraint.link_name = move_group_->getEndEffectorLink();
      interim_constraint.constraint_region.primitive_poses.push_back(interim_pose);

      moveit_msgs::msg::Constraints path_constraints;
      path_constraints.name = "interim";
      path_constraints.position_constraints.push_back(interim_constraint);

      RCLCPP_INFO(logger_, "Seam %s: executing CIRC weld motion via Pilz", seam.seam_id.c_str());
      weld = plan_and_execute_pilz("CIRC", waypoints.back(), &path_constraints,
          seam.seam_id, should_stop);
    }
    if (weld != MotionOutcome::kSucceeded) {
      return false;
    }
  } else {
    // TODO(silanus23): split ptp seams into LIN/CIRC pieces sent as one blended Pilz
    // sequence; separate Pilz motions would stop the torch between every piece.
    RCLCPP_INFO(logger_, "Seam %s: segment_type '%s' — using computeCartesianPath",
      seam.seam_id.c_str(), seam.segment_type.c_str());
    const MotionOutcome weld = plan_and_execute_cartesian(waypoints, seam.seam_id, should_stop);
    torch_left_standoff = weld != MotionOutcome::kPlanningFailed;
    if (weld != MotionOutcome::kSucceeded) {
      return false;
    }
  }

  int32_t points_after_seam = points_before_seam + static_cast<int32_t>(waypoints.size());
  feedback->completion_percentage = (static_cast<float>(points_after_seam) / total_waypoints) *
    100.0f;
  goal_handle->publish_feedback(feedback);

  RCLCPP_INFO(logger_, "Cartesian path executed successfully");
  return true;
}

bool WelderActionServer::retreat_from_part(
  const std::string & seam_id, const std::function<bool()> & should_stop)
{
  const auto current = move_group_->getCurrentPose();
  if (current.header.frame_id.empty()) {
    RCLCPP_ERROR(logger_, "Seam %s: cannot retreat, current end-effector pose unknown",
      seam_id.c_str());
    return false;
  }
  // Seam poses are torch poses, so the standoff lies along the tool's own +Z.
  Eigen::Isometry3d current_iso;
  tf2::fromMsg(current.pose, current_iso);
  const geometry_msgs::msg::Pose target = tf2::toMsg(
    hold_and_weld::kinematics::standoff_pose(current_iso, config_.approach_offset_z));

  RCLCPP_WARN(logger_, "Seam %s: retreating %.3f m along the tool Z axis before the next seam",
    seam_id.c_str(), config_.approach_offset_z);
  const MotionOutcome retreat = config_.use_pilz ?
    plan_and_execute_pilz("LIN", target, nullptr, seam_id, should_stop) :
    plan_and_execute_cartesian({target}, seam_id, should_stop);
  if (retreat != MotionOutcome::kSucceeded) {
    RCLCPP_ERROR(logger_, "Seam %s: retreat failed; the torch may still be on the part",
      seam_id.c_str());
    return false;
  }
  return true;
}

}  // namespace application
}  // namespace hold_and_weld
