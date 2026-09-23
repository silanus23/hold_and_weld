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
#include <cstdio>
#include <fstream>
#include <filesystem>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <controller_manager_msgs/srv/list_controllers.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/position_constraint.hpp>
#include <moveit_msgs/srv/get_cartesian_path.hpp>
#include <nlohmann/json.hpp>
#include <rclcpp/parameter_client.hpp>

namespace hold_and_weld
{
namespace application
{

WelderActionServer::WelderActionServer(const rclcpp::NodeOptions & options)
: LifecycleNode("welder_action_server", options),
  logger_(rclcpp::get_logger("application"))
{
  // Declared here, not in on_configure, so configure -> cleanup -> configure
  // does not throw ParameterAlreadyDeclaredException.
  declare_parameter("auto_trigger", false);
  declare_parameter("auto_trigger_delay_sec", 3.0);
}

WelderActionServer::~WelderActionServer()
{
  // manual_shutdown() should have already been called from main().
  // This is a safety net for any path that bypasses main() (e.g. tests).
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
  // Idempotent safe to call multiple times (destructor calls it as safety net).
  // shutdown_worker() sets shutdown_requested_ under the execution_mutex_; check it first.
  if (shutdown_requested_.exchange(true)) {
    return;
  }

  RCLCPP_DEBUG(logger_, "Manual shutdown: signalling stop");

  // stop() is a topic publish — works as long as the ROS context is still valid.
  // Call before rclcpp::shutdown() so move_group can process the cancel.
  try {
    if (move_group_) {
      move_group_->stop();
    }
  } catch (...) {
    RCLCPP_WARN(logger_, "Exception caught while stopping move_group during shutdown");
  }

  // Poll until execute_weld() returns or the ROS context dies — whichever comes first.
  // move_group_->execute(plan) inside execute_weld() blocks until the controller finishes
  // the trajectory, so we must break the wait externally when the context is invalidated
  // to avoid spinning forever.
  {
    // Take a local copy of the future to poll without holding the mutex.
    std::shared_future<void> future_copy;
    {
      std::lock_guard<std::mutex> lock(execution_future_mutex_);
      future_copy = execution_future_;
    }

    if (future_copy.valid()) {
      while (rclcpp::ok() &&
        future_copy.wait_for(std::chrono::milliseconds(10)) ==
        std::future_status::timeout)
      {}

      if (future_copy.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready) {
        RCLCPP_INFO(logger_, "Execution finished cleanly.");
      } else {
        RCLCPP_WARN(
          logger_,
          "ROS context shut down before execute() returned — proceeding with worker "
          "shutdown. move_group_ is kept alive by the worker thread's captured "
          "shared_ptr.");
      }
    }
  }

  // Shut down the worker regardless of whether execute() returned.
  shutdown_worker();
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
WelderActionServer::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  auto_trigger_ = get_parameter("auto_trigger").as_bool();
  auto_trigger_delay_sec_ = get_parameter("auto_trigger_delay_sec").as_double();
  if (auto_trigger_delay_sec_ < 0.0) {
    RCLCPP_ERROR(logger_, "auto_trigger_delay_sec must be >= 0, got %.3f",
      auto_trigger_delay_sec_);
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
  }

  // Temporary node used only for service availability checks during configuration.
  // A separate node is required because this lifecycle node's executor is not
  // spinning freely during on_configure, so service calls on 'this' would deadlock.
  auto temp_node = std::make_shared<rclcpp::Node>("welder_service_waiter");
  auto cartesian_path_client = temp_node->create_client<moveit_msgs::srv::GetCartesianPath>(
    "/compute_cartesian_path");

  RCLCPP_INFO(logger_, "Waiting for MoveIt compute_cartesian_path service");
  if (!hold_and_weld::wait_for_service(cartesian_path_client, "MoveIt compute_cartesian_path",
      logger_))
  {
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
  }
  RCLCPP_INFO(logger_, "MoveIt is available");

  auto list_controllers_client = temp_node->create_client<
    controller_manager_msgs::srv::ListControllers>("/controller_manager/list_controllers");

  RCLCPP_INFO(logger_, "Waiting for controller_manager service");
  if (!hold_and_weld::wait_for_service(list_controllers_client,
      "controller_manager/list_controllers", logger_))
  {
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
  }
  RCLCPP_INFO(logger_, "Controllers are ready");

  load_config_from_yaml();

  const bool needs_kinematics =
    config_.use_approach_validator || config_.use_configuration_finder;

  std::string urdf_string;
  if (needs_kinematics) {
    RCLCPP_DEBUG(logger_, "Fetching robot_description from robot_state_publisher");

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

    internal_node = std::make_shared<rclcpp::Node>(
      "welder_moveit_internal",
      node_options);

    bool use_sim_time = false;
    if (this->get_parameter("use_sim_time", use_sim_time)) {
      internal_node->set_parameter(rclcpp::Parameter("use_sim_time", use_sim_time));
      RCLCPP_DEBUG(logger_, "Copied use_sim_time=%s to internal node",
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

  if (needs_kinematics) {
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

      RCLCPP_INFO(logger_, "Kinematic chain: robot2_base_link -> robot2_wire_tip");

      auto urdf_parser = std::make_unique<hold_and_weld::kinematics::URDFParser>();

      hold_and_weld::kinematics::ParsedChain parsed_chain;
      parsed_chain = urdf_parser->extract_joint_chain_from_string(
        urdf_string, "robot2_base_link", "robot2_wire_tip");

      RCLCPP_DEBUG(logger_, "Parsed kinematic chain with %zu actuated joints",
                  parsed_chain.actuated_joints.size());

      kinematics_solver_ =
        std::make_shared<hold_and_weld::kinematics::KinematicsSolver>(parsed_chain);

      ceres_solver_ = std::make_shared<hold_and_weld::kinematics::CeresIKSolver>(
        kinematics_solver_, 1.0);

      auto validator_params = config_.approach_validator;
      validator_params.manipulability_threshold = config_.manipulability_threshold;
      approach_validator_ =
        std::make_unique<hold_and_weld::kinematics::ApproachValidator>(
        kinematics_solver_,
        ceres_solver_,
        validator_params);

      if (config_.use_configuration_finder) {
        // The finder simulates Pilz on the Ceres chain, so both must model the same point.
        const std::string ee_link = move_group_->getEndEffectorLink();
        if (ee_link != "robot2_wire_tip") {
          throw std::runtime_error(
                  "configuration finder models robot2_wire_tip but Pilz targets '" + ee_link +
                  "'; fix the SRDF or set use_configuration_finder: false");
        }

        const auto & joint_names = joint_model_group->getVariableNames();
        if (joint_names.size() != 6) {
          throw std::runtime_error(
                  "configuration finder expects 6 joints in " + config_.welder_group_name +
                  ", got " + std::to_string(joint_names.size()));
        }
        for (size_t i = 0; i < 6; ++i) {
          auto it = config_.home_configuration.find(joint_names[i]);
          if (it == config_.home_configuration.end()) {
            throw std::runtime_error(
                    "home_configuration (or safety_pose.joint_positions) is missing joint '" +
                    joint_names[i] + "'");
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
      shutdown_worker();
      if (moveit_executor_) {moveit_executor_->cancel();}
      if (moveit_thread_.joinable()) {moveit_thread_.join();}
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
    }
  }

  action_server_ = rclcpp_action::create_server<TriggerWelder>(
    this->get_node_base_interface(),
    this->get_node_clock_interface(),
    this->get_node_logging_interface(),
    this->get_node_waitables_interface(),
    "trigger_welder",
    [this](const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const TriggerWelder::Goal> goal)
    {
      return this->handle_goal(uuid, goal);
    },
    [this](const std::shared_ptr<GoalHandleTriggerWelder> handle) {
      return this->handle_cancel(handle);
    },
    [this](const std::shared_ptr<GoalHandleTriggerWelder> handle) {
      this->handle_accepted(handle);
    }
  );

  self_trigger_client_ = rclcpp_action::create_client<TriggerWelder>(
    this->get_node_base_interface(),
    this->get_node_graph_interface(),
    this->get_node_logging_interface(),
    this->get_node_waitables_interface(),
    "trigger_welder");

  // Start the persistent worker thread that will process queued goals.
  // Currently supports a single queued goal at a time, multi-goal queuing is deferred.
  // Worker is started last so it cannot receive goals before the action server is live.
  worker_thread_ = std::thread(&WelderActionServer::worker_thread_func, this);

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
WelderActionServer::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(logger_, "Activating welder action server");
  // No lifecycle publishers to activate. Worker thread is already running from
  // on_configure. If auto_trigger_ is set, start a timer that sends a goal to our
  // own trigger_welder action once it fires.
  if (auto_trigger_) {
    double delay = auto_trigger_delay_sec_;
    RCLCPP_INFO(logger_, "Auto-trigger enabled, will start in %.1f seconds", delay);

    auto_trigger_timer_ = create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(delay * 1000)),
      [this]() {
        auto_trigger_timer_->cancel();
        RCLCPP_INFO(logger_, "Auto-triggering welder job via trigger_welder action");

        if (!hold_and_weld::wait_for_action_server(
            self_trigger_client_, "trigger_welder", logger_, 10))
        {
          RCLCPP_ERROR(logger_, "Auto-trigger skipped: trigger_welder action server not found");
          return;
        }

        auto goal_msg = TriggerWelder::Goal();
        auto send_goal_options = rclcpp_action::Client<TriggerWelder>::SendGoalOptions();

        send_goal_options.goal_response_callback =
        [this](const rclcpp_action::ClientGoalHandle<TriggerWelder>::SharedPtr & goal_handle)
        {
          if (!goal_handle) {
            RCLCPP_ERROR(logger_, "Auto-triggered welder goal was rejected");
          }
        };

        send_goal_options.feedback_callback =
        [this](
          rclcpp_action::ClientGoalHandle<TriggerWelder>::SharedPtr,
          const std::shared_ptr<const TriggerWelder::Feedback> feedback)
        {
          RCLCPP_INFO(logger_, "  [Auto-trigger] %s (%.1f%%)",
                        feedback->current_step.c_str(), feedback->completion_percentage);
        };

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
    );
  }

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
WelderActionServer::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(logger_, "Deactivating welder action server");
  if (auto_trigger_timer_) {
    auto_trigger_timer_->cancel();
    auto_trigger_timer_.reset();
  }

  try {
    if (move_group_) {
      move_group_->stop();
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger_, "Failed to stop move group: %s", e.what());
  }

  // Wait for any in-flight execute_weld() to finish. We do not call shutdown_worker()
  // here because the worker must stay alive for re-activation. The executor must still
  // be running so execute()'s result callback can unblock.
  {
    std::shared_future<void> future_copy;
    {
      std::lock_guard<std::mutex> lock(execution_future_mutex_);
      future_copy = execution_future_;
    }
    if (future_copy.valid()) {
      future_copy.wait();
      RCLCPP_DEBUG(logger_, "In-flight weld execution finished before deactivation completed.");
    }
  }

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
WelderActionServer::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(logger_, "Cleaning up welder action server");
  // stop() while moveit_executor_ is still spinning so the cancel reaches the controller
  // and execute() returns cleanly before shutdown_worker() joins the worker thread.
  try {
    if (move_group_) {
      move_group_->stop();
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger_, "Failed to stop move group: %s", e.what());
  }

  // Wait for any in-flight execute_weld() to finish. The executor must still
  // be running so execute()'s result callback can unblock.
  {
    std::shared_future<void> future_copy;
    {
      std::lock_guard<std::mutex> lock(execution_future_mutex_);
      future_copy = execution_future_;
    }
    if (future_copy.valid()) {
      future_copy.wait();
    }
  }

  shutdown_worker();

  action_server_.reset();
  self_trigger_client_.reset();

  {
    std::lock_guard<std::mutex> lock(move_group_mutex_);
    move_group_.reset();
  }

  // Worker is done, safe to stop the executor now.
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

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
WelderActionServer::on_shutdown(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(logger_, "Shutting down welder action server");
  // stop() while moveit_executor_ is still spinning so the cancel reaches the controller
  // and execute() returns cleanly before shutdown_worker() joins the worker thread.
  try {
    if (move_group_) {
      move_group_->stop();
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger_, "Failed to stop move group: %s", e.what());
  }

  // Wait for any in-flight execute_weld() to finish. The executor must still
  // be running so execute()'s result callback can unblock.
  {
    std::shared_future<void> future_copy;
    {
      std::lock_guard<std::mutex> lock(execution_future_mutex_);
      future_copy = execution_future_;
    }
    if (future_copy.valid()) {
      future_copy.wait();
    }
  }

  shutdown_worker();

  action_server_.reset();
  self_trigger_client_.reset();

  {
    std::lock_guard<std::mutex> lock(move_group_mutex_);
    move_group_.reset();
  }

  // Worker is done, safe to stop the executor now.
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

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

void WelderActionServer::load_config_from_yaml()
{
  std::string yaml_path;
  try {
    yaml_path = ament_index_cpp::get_package_share_directory("hold_and_weld_bringup") +
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

  try {
    YAML::Node yaml = YAML::LoadFile(yaml_path);

    if (yaml["welder_group_name"]) {
      config_.welder_group_name = yaml["welder_group_name"].as<std::string>();
    }
    if (yaml["approach_offset_z"]) {
      config_.approach_offset_z = yaml["approach_offset_z"].as<double>();
    }
    if (yaml["use_approach_validation"]) {
      config_.use_approach_validator = yaml["use_approach_validation"].as<bool>();
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
    if (yaml["manipulability_threshold"]) {
      config_.manipulability_threshold = yaml["manipulability_threshold"].as<double>();
    }
    if (yaml["use_configuration_finder"]) {
      config_.use_configuration_finder = yaml["use_configuration_finder"].as<bool>();
    }
    if (yaml["home_configuration"]) {
      config_.home_configuration =
        yaml["home_configuration"].as<std::map<std::string, double>>();
    } else if (yaml["safety_pose"] && yaml["safety_pose"]["joint_positions"]) {
      config_.home_configuration =
        yaml["safety_pose"]["joint_positions"].as<std::map<std::string, double>>();
    }
    if (const YAML::Node finder = yaml["finder"]) {
      auto read = [&finder](const char * key, double & value) {
          if (finder[key]) {value = finder[key].as<double>();}
        };
      auto & f = config_.finder;
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
        config_.finder_max_ompl_candidates = finder["max_ompl_candidates"].as<int>();
      }
    }
    if (const YAML::Node validator = yaml["approach_validator"]) {
      auto read = [&validator](const char * key, double & value) {
          if (validator[key]) {value = validator[key].as<double>();}
        };
      auto & v = config_.approach_validator;
      read("first_point_tol_pos", v.first_point_tol_pos);
      read("first_point_tol_rot", v.first_point_tol_rot);
      read("seam_tol_pos", v.seam_tol_pos);
      read("seam_tol_rot", v.seam_tol_rot);
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
      std::unique_lock<std::mutex> lock(execution_mutex_);
      execution_cv_.wait(lock, [this] {
          return pending_goal_ != nullptr || shutdown_requested_;
        });

      if (shutdown_requested_) {
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
      execute_weld(goal_handle);
      promise->set_value();
    }
  }
}

std::string WelderActionServer::find_latest_json() const
{
  std::string trajectory_dir;
  if (this->has_parameter("trajectory_directory")) {
    trajectory_dir = this->get_parameter("trajectory_directory").as_string();
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

std::vector<WeldSeam> WelderActionServer::load_seams_from_json(const std::string & filepath) const
{
  std::vector<WeldSeam> seams;

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

      if (seam_data.contains("end") && seam_data["end"].size() == 3) {
        auto e = seam_data["end"];
        seam.end = {e[0], e[1], e[2]};
      }

      seam.length_m = seam_data.value("length_m", 0.0);
      seam.segment_type = seam_data.value("segment_type", "");

      if (seam.segment_type == "arc" &&
        seam_data.contains("center") && seam_data["center"].size() == 3 &&
        seam_data.contains("radius"))
      {
        auto c = seam_data["center"];
        seam.center = {c[0], c[1], c[2]};
        seam.radius = seam_data["radius"];
        seam.has_arc_geometry = true;
      }

      seam.poses.reserve(seam_data["poses"].size());
      for (const auto & pose_data : seam_data["poses"]) {
        try {
          seam.poses.push_back(json_to_pose(pose_data));
        } catch (const std::exception & e) {
          RCLCPP_ERROR(
            logger_, "Seam %s: failed to parse pose — skipping seam. Reason: %s",
            seam_id.c_str(), e.what());
          seam.poses.clear();
          break;
        }
      }

      if (seam.poses.empty()) {
        RCLCPP_WARN(logger_, "Seam %s has no valid poses after parsing, skipping", seam_id.c_str());
        continue;
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

  // stop() is safe to call without move_group_mutex_ (non-blocking signal).
  try {
    if (move_group_) {
      move_group_->stop();
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger_, "Failed to stop move_group: %s", e.what());
  }

  return rclcpp_action::CancelResponse::ACCEPT;
}

void WelderActionServer::handle_accepted(
  const std::shared_ptr<GoalHandleTriggerWelder> goal_handle)
{
  {
    std::lock_guard<std::mutex> lock(execution_mutex_);
    pending_goal_ = goal_handle;
  }
  execution_cv_.notify_one();
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

    RCLCPP_INFO(logger_, "Seam %zu/%zu: %s (%.3f m, %zu points)",
                 seam_idx + 1, seams.size(), seam.seam_id.c_str(),
                 seam.length_m, seam.num_poses);

    if (goal_handle->is_canceling()) {
      result->success = false;
      result->message = "Canceled by client";
      goal_handle->canceled(result);
      return;
    }

    publish_progress("approaching_seam_" + seam.seam_id, points_processed);
    if (!move_to_seam_boundary(seam, waypoints.front(), true)) {
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

      if (execute_cartesian_path(seam, goal_handle, feedback,
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
    if (!move_to_seam_boundary(seam, waypoints.back(), false)) {
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

bool WelderActionServer::move_to_seam_boundary(
  const WeldSeam & seam, const geometry_msgs::msg::Pose & ref_pose, bool is_approach)
{
  // Self-resetting: execute_cartesian_path() may have left the Pilz pipeline/planner
  move_group_->setPlanningPipelineId("ompl");
  move_group_->setPlannerId("");
  move_group_->clearPathConstraints();

  // Same standoff maths as the configuration finder, so its approach pose is this one.
  Eigen::Isometry3d ref = Eigen::Isometry3d::Identity();
  ref.translation() << ref_pose.position.x, ref_pose.position.y, ref_pose.position.z;
  ref.linear() = Eigen::Quaterniond(
    ref_pose.orientation.w, ref_pose.orientation.x,
    ref_pose.orientation.y, ref_pose.orientation.z).toRotationMatrix();
  const Eigen::Vector3d target_pos =
    hold_and_weld::kinematics::standoff_pose(ref, config_.approach_offset_z).translation();

  geometry_msgs::msg::Pose target_pose;
  target_pose.position.x = target_pos.x();
  target_pose.position.y = target_pos.y();
  target_pose.position.z = target_pos.z();
  target_pose.orientation = ref_pose.orientation;

  RCLCPP_DEBUG(logger_, "Boundary Position: (%.3f, %.3f, %.3f)",
               target_pose.position.x, target_pose.position.y, target_pose.position.z);

  move_group_->setStartStateToCurrentState();
  auto current_state = move_group_->getCurrentState();
  if (!current_state) {
    RCLCPP_ERROR(logger_, "Failed to get current robot state!");
    return false;
  }

  auto current_pose = move_group_->getCurrentPose();
  if (current_pose.header.frame_id.empty()) {
    RCLCPP_WARN(logger_, "getCurrentPose() returned invalid pose — skipping distance log");
  }
  double distance = std::sqrt(
    std::pow(target_pose.position.x - current_pose.pose.position.x, 2) +
    std::pow(target_pose.position.y - current_pose.pose.position.y, 2) +
    std::pow(target_pose.position.z - current_pose.pose.position.z, 2)
  );
  RCLCPP_DEBUG(logger_, "Distance to approach target: %.3f m", distance);

  if (is_approach && configuration_finder_) {
    return approach_via_configuration_finder(seam, current_state);
  }

  // The validator checks the weld from the approach; it means nothing for a retract.
  const bool validate = is_approach && config_.use_approach_validator;

  move_group_->setPoseTarget(target_pose);
  move_group_->setGoalPositionTolerance(0.001);
  move_group_->setGoalOrientationTolerance(0.01);

  // Transform seam waypoints from world frame to robot base frame for the validator.
  // The approach validator solves IK in the robot's own base frame.
  WeldSeam local_seam = seam;
  Eigen::Isometry3d world_to_base = current_state->getGlobalLinkTransform("robot2_base_link");
  Eigen::Isometry3d base_to_world = world_to_base.inverse();
  for (auto & pose : local_seam.poses) {
    pose = transform_pose_to_base_frame(pose, base_to_world);
  }

  if (validate) {
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

    const auto & ik_joint_names =
      move_group_->getRobotModel()
      ->getJointModelGroup(config_.welder_group_name)
      ->getVariableNames();

    if (ik_joint_names.size() != 6) {
      RCLCPP_ERROR(logger_, "Expected 6 IK joints, got %zu — skipping OMPL attempt %d",
                   ik_joint_names.size(), ompl_attempt);
      continue;
    }

    Eigen::Matrix<double, 6, 1> q_approach;
    bool joint_mapping_ok = true;
    for (size_t i = 0; i < 6; ++i) {
      const auto & expected_name = ik_joint_names[i];
      auto it = std::find(
        trajectory.joint_names.begin(),
        trajectory.joint_names.end(),
        expected_name);
      if (it == trajectory.joint_names.end()) {
        RCLCPP_ERROR(logger_,
                     "Required joint '%s' not found in trajectory joint_names — "
                     "skipping OMPL attempt %d",
                     expected_name.c_str(), ompl_attempt);
        joint_mapping_ok = false;
        break;
      }
      const size_t traj_idx = static_cast<size_t>(
        std::distance(trajectory.joint_names.begin(), it));
      q_approach(i) = final_point.positions[traj_idx];
    }
    if (!joint_mapping_ok) {
      continue;
    }

    if (validate) {
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
      RCLCPP_ERROR(logger_, "Execution failed on OMPL attempt %d", ompl_attempt);
    }
  }

  RCLCPP_ERROR(logger_, "Failed to find valid boundary pose after %d OMPL attempts",
               config_.max_ompl_planning_attempts);
  return false;
}

bool WelderActionServer::approach_via_configuration_finder(
  const WeldSeam & seam, const moveit::core::RobotStatePtr & current_state)
{
  const Eigen::Isometry3d base_to_world =
    current_state->getGlobalLinkTransform("robot2_base_link").inverse();

  std::vector<Eigen::Isometry3d> seam_base;
  seam_base.reserve(seam.poses.size());
  for (const auto & pose : seam.poses) {
    const auto p = transform_pose_to_base_frame(pose, base_to_world);
    Eigen::Isometry3d iso = Eigen::Isometry3d::Identity();
    iso.translation() << p.position.x, p.position.y, p.position.z;
    iso.linear() = Eigen::Quaterniond(
      p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z).toRotationMatrix();
    seam_base.push_back(iso);
  }

  const auto ranked = configuration_finder_->find(
    seam_base, seam.segment_type, config_.approach_offset_z, q_home_);

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
    feasible, static_cast<size_t>(std::max(config_.finder_max_ompl_candidates, 1)));
  for (size_t rank = 0; rank < to_try; ++rank) {
    const auto & candidate = ranked[rank];
    RCLCPP_INFO(
      logger_, "Seam %s: start config rank %zu/%zu %s", seam.seam_id.c_str(), rank + 1,
      feasible, describe(candidate).c_str());

    const std::vector<double> joint_goal(
      candidate.q_start.data(), candidate.q_start.data() + candidate.q_start.size());
    move_group_->setStartStateToCurrentState();
    if (!move_group_->setJointValueTarget(joint_goal)) {
      RCLCPP_WARN(logger_, "Seam %s: joint goal rejected by MoveIt", seam.seam_id.c_str());
      continue;
    }

    for (int attempt = 1; attempt <= config_.max_ompl_planning_attempts; ++attempt) {
      moveit::planning_interface::MoveGroupInterface::Plan plan;
      if (move_group_->plan(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_WARN(
          logger_, "Seam %s: OMPL attempt %d/%d to rank %zu failed", seam.seam_id.c_str(),
          attempt, config_.max_ompl_planning_attempts, rank + 1);
        continue;
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

bool WelderActionServer::plan_and_execute_pilz(
  const std::string & planner_id,
  const geometry_msgs::msg::Pose & target,
  const moveit_msgs::msg::Constraints * path_constraints,
  const std::string & seam_id)
{
  move_group_->setPlanningPipelineId("pilz_industrial_motion_planner");
  move_group_->setPlannerId(planner_id);
  move_group_->setStartStateToCurrentState();
  move_group_->setPoseTarget(target);
  if (path_constraints) {
    move_group_->setPathConstraints(*path_constraints);
  } else {
    move_group_->clearPathConstraints();
  }

  bool success = false;
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  auto plan_result = move_group_->plan(plan);
  if (plan_result == moveit::core::MoveItErrorCode::SUCCESS) {
    auto execute_result = move_group_->execute(plan);
    success = (execute_result == moveit::core::MoveItErrorCode::SUCCESS);
    if (!success) {
      RCLCPP_ERROR(logger_, "Seam %s: %s execution failed", seam_id.c_str(), planner_id.c_str());
    }
  } else {
    RCLCPP_ERROR(logger_, "Seam %s: %s planning failed", seam_id.c_str(), planner_id.c_str());
  }

  // Must clear on every exit path — a leftover CIRC constraint must not leak
  // into the retract move that immediately follows.
  move_group_->clearPathConstraints();
  return success;
}

bool WelderActionServer::execute_cartesian_path(
  const WeldSeam & seam,
  const std::shared_ptr<GoalHandleTriggerWelder> & goal_handle,
  std::shared_ptr<TriggerWelder::Feedback> & feedback,
  int32_t points_before_seam,
  int32_t total_waypoints)
{
  const auto & waypoints = seam.poses;
  bool success = false;

  if (seam.segment_type == "line" || seam.segment_type == "arc") {
    const bool is_arc = (seam.segment_type == "arc");
    if (is_arc && !seam.has_arc_geometry) {
      RCLCPP_ERROR(logger_,
        "Seam %s: segment_type is 'arc' but center/radius are missing — failing seam "
        "rather than falling back silently", seam.seam_id.c_str());
      return false;
    }
    if (is_arc && waypoints.size() < 3) {
      RCLCPP_ERROR(logger_, "Seam %s: arc needs >= 3 poses for a CIRC interim point, got %zu",
        seam.seam_id.c_str(), waypoints.size());
      return false;
    }

    // move_to_seam_boundary() leaves the torch backed off by approach_offset_z,
    // so plunge onto the seam start first; LIN/CIRC must start exactly on the seam.
    RCLCPP_INFO(logger_, "Seam %s: LIN plunge to seam start via Pilz", seam.seam_id.c_str());
    success = plan_and_execute_pilz("LIN", waypoints.front(), nullptr, seam.seam_id);

    if (success && !is_arc) {
      RCLCPP_INFO(logger_, "Seam %s: executing LIN weld motion via Pilz", seam.seam_id.c_str());
      success = plan_and_execute_pilz("LIN", waypoints.back(), nullptr, seam.seam_id);
    } else if (success) {
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
      success = plan_and_execute_pilz("CIRC", waypoints.back(), &path_constraints,
          seam.seam_id);
    }
  } else {
    // "ptp" / unknown / legacy: Pilz PTP is a single joint-space point-to-point,
    // not a match for the discrete multi-point path a PTP segment actually
    // represents, so this keeps the pre-Pilz computeCartesianPath() behavior.
    RCLCPP_INFO(logger_, "Seam %s: segment_type '%s' — using computeCartesianPath fallback",
      seam.seam_id.c_str(), seam.segment_type.c_str());

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
    success = (execute_result == moveit::core::MoveItErrorCode::SUCCESS);
    if (!success) {
      RCLCPP_ERROR(logger_, "Execution failed");
    }
  }

  if (!success) {
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
    std::lock_guard<std::mutex> lock(execution_mutex_);
    shutdown_requested_ = true;
  }
  execution_cv_.notify_one();

  if (worker_thread_.joinable()) {
    // If execute() is still running, worker_thread_.join() would block forever.
    // Check the execution_future_ to see if execute_weld() has already returned.
    bool execution_done = true;
    {
      std::lock_guard<std::mutex> lock(execution_future_mutex_);
      if (execution_future_.valid()) {
        execution_done =
          (execution_future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready);
      }
    }

    if (execution_done) {
      worker_thread_.join();
    } else {
      // execute() is still blocked. Detach so we don't hang.
      // worker_thread_ holds move_group_ via its captured shared_ptr,
      // so it won't access freed memory after the node is destroyed.
      RCLCPP_WARN(logger_, "Worker thread still in execute() — detaching."
                           " move_group_ kept alive by captured shared_ptr.");
      worker_thread_.detach();
    }
  }
}

geometry_msgs::msg::Pose WelderActionServer::transform_pose_to_base_frame(
  const geometry_msgs::msg::Pose & world_pose,
  const Eigen::Isometry3d & base_to_world_transform) const
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

}  // namespace application
}  // namespace hold_and_weld
