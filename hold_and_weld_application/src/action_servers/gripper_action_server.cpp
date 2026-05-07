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
#include <lifecycle_msgs/msg/state.hpp>
#include <moveit_msgs/srv/get_cartesian_path.hpp>

#include "hold_and_weld_application/utils.hpp"

namespace hold_and_weld
{
namespace application
{

GripperActionServer::GripperActionServer(const rclcpp::NodeOptions & options)
: LifecycleNode("gripper_action_server", options),
  logger_(rclcpp::get_logger("application"))
{
}

GripperActionServer::~GripperActionServer()
{
  // manual_shutdown() should have already been called from main().
  // This is a safety net for any path that bypasses main()
  manual_shutdown();

  if (moveit_executor_) {
    moveit_executor_->cancel();
  }
  if (moveit_thread_.joinable()) {
    moveit_thread_.join();
  }
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
GripperActionServer::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  // Temporary node used only for service availability checks during configuration.
  // A separate node is required because this lifecycle node's executor is not
  // spinning freely during on_configure, so service calls on 'this' would deadlock.
  auto temp_node = std::make_shared<rclcpp::Node>("gripper_service_waiter");
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

  std::string default_yaml =
    ament_index_cpp::get_package_share_directory("hold_and_weld_application") +
    "/config/tasks/pick_place_targets.yaml";

  declare_parameter("arm_group_name", "robot1_gp25_arm");
  declare_parameter("positions_yaml", default_yaml);
  declare_parameter("gripper_joint_names", std::vector<std::string>{
        "robot1_left_finger_joint", "robot1_right_finger_joint"});
  declare_parameter("auto_trigger", false);
  declare_parameter("auto_trigger_delay_sec", 3.0);

  arm_group_name_ = get_parameter("arm_group_name").as_string();
  gripper_joint_names_ = get_parameter("gripper_joint_names").as_string_array();
  yaml_path_ = get_parameter("positions_yaml").as_string();
  auto_trigger_ = get_parameter("auto_trigger").as_bool();
  auto_trigger_delay_sec_ = get_parameter("auto_trigger_delay_sec").as_double();

  RCLCPP_INFO(logger_, "Arm group: %s", arm_group_name_.c_str());

  gripper_action_client_ = rclcpp_action::create_client<FollowJointTrajectory>(
    this->get_node_base_interface(),
    this->get_node_graph_interface(),
    this->get_node_logging_interface(),
    this->get_node_waitables_interface(),
    "/gripper_controller/follow_joint_trajectory");

  attached_collision_pub_ = this->create_publisher<moveit_msgs::msg::AttachedCollisionObject>(
    "/attached_collision_object", 10);

  planning_scene_client_ = create_client<moveit_msgs::srv::ApplyPlanningScene>(
    "/apply_planning_scene");

  get_planning_scene_client_ =
    create_client<moveit_msgs::srv::GetPlanningScene>("/get_planning_scene");

  try {
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);

    std::string robot_description_semantic;
    if (this->has_parameter("robot_description_semantic")) {
      robot_description_semantic = this->get_parameter("robot_description_semantic").as_string();
    }

    auto internal_node = std::make_shared<rclcpp::Node>(
      "gripper_moveit_internal",
      node_options);

    if (!robot_description_semantic.empty()) {
      internal_node->declare_parameter("robot_description_semantic", robot_description_semantic);
    }

    std::string robot_description;
    if (this->has_parameter("robot_description")) {
      robot_description = this->get_parameter("robot_description").as_string();
      internal_node->declare_parameter("robot_description", robot_description);
    }

    moveit_executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    moveit_executor_->add_node(internal_node);
    moveit_thread_ = std::thread([this]() {moveit_executor_->spin();});

    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
      internal_node, arm_group_name_);

    move_group_->setPlanningTime(10.0);
    move_group_->setNumPlanningAttempts(10);
    move_group_->setMaxVelocityScalingFactor(0.3);
    move_group_->setMaxAccelerationScalingFactor(0.3);

    RCLCPP_INFO(logger_, "MoveIt initialized successfully");
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger_, "Failed to initialize MoveIt: %s", e.what());
    if (moveit_executor_) {
      moveit_executor_->cancel();
    }
    if (moveit_thread_.joinable()) {
      moveit_thread_.join();
    }
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
  }

  action_server_ = rclcpp_action::create_server<TriggerGripper>(
    this->get_node_base_interface(),
    this->get_node_clock_interface(),
    this->get_node_logging_interface(),
    this->get_node_waitables_interface(),
    "trigger_gripper",
    [this](const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const TriggerGripper::Goal> goal)
    {
      return this->handle_goal(uuid, goal);
    },
    [this](const std::shared_ptr<GoalHandleTriggerGripper> handle) {
      return this->handle_cancel(handle);
    },
    [this](const std::shared_ptr<GoalHandleTriggerGripper> handle) {
      this->handle_accepted(handle);
    }
  );

  load_object_config();
  load_job_from_yaml(yaml_path_);

  {
    std::lock_guard<std::mutex> lock(config_mutex_);
    if (job_loaded_) {
      RCLCPP_INFO(logger_, "Job loaded for target: %s", job_.target_id.c_str());
    } else {
      RCLCPP_WARN(logger_, "No job loaded - action server will reject goals!");
    }
  }

  // Start the persistent worker thread that will process queued goals.
  // Currently supports a single queued goal at a time — multi-goal queuing is deferred.
  worker_thread_ = std::thread(&GripperActionServer::worker_thread_func, this);

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
GripperActionServer::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(logger_, "Activating gripper action server");
  attached_collision_pub_->on_activate();

  if (auto_trigger_ && job_loaded_) {
    double delay = auto_trigger_delay_sec_;
    RCLCPP_INFO(logger_, "Auto-trigger enabled, will start in %.1f seconds", delay);

    auto_trigger_timer_ = create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(delay * 1000)),
      [this]() {
        auto_trigger_timer_->cancel();
        RCLCPP_INFO(logger_, "Auto-triggering gripper action — running directly on worker thread");
        auto promise = std::make_shared<std::promise<void>>();
        {
          std::lock_guard<std::mutex> lock(execution_future_mutex_);
          execution_future_ = promise->get_future().share();
        }
        std::thread([this, promise = std::move(promise)]() mutable {
          run_job([](const std::string &, float) {});
          promise->set_value();
        }).detach();
      }
    );
  }

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
GripperActionServer::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(logger_, "Deactivating gripper action server");
  if (auto_trigger_timer_) {
    auto_trigger_timer_->cancel();
    auto_trigger_timer_.reset();
  }

  // stop() signals the controller to halt while moveit_executor_ is still spinning,
  // so the cancel request can actually be delivered and execute() returns cleanly.
  try {
    if (move_group_) {
      move_group_->stop();
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger_, "Failed to stop move group: %s", e.what());
  }

  // Wait for any in-flight execute_job() to finish. We do not call shutdown_worker()
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
    }
  }
  attached_collision_pub_->on_deactivate();

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
GripperActionServer::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(logger_, "Cleaning up gripper action server");
  // stop() while moveit_executor_ is still spinning so the cancel reaches the controller
  // and execute() returns cleanly before shutdown_worker() joins the worker thread.
  try {
    if (move_group_) {
      move_group_->stop();
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger_, "Failed to stop move group: %s", e.what());
  }

  // Wait for any in-flight execute_job() to finish before joining the worker thread.
  // The executor must still be spinning here so execute()'s result callback can unblock.
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

  try {
    if (action_server_) {
      action_server_.reset();
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger_, "Failed to reset action_server: %s", e.what());
  }

  try {
    std::lock_guard<std::mutex> lock(move_group_mutex_);
    move_group_.reset();
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger_, "Failed to reset move_group: %s", e.what());
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

  try {
    if (gripper_action_client_) {gripper_action_client_.reset();}
    if (planning_scene_client_) {planning_scene_client_.reset();}
    if (get_planning_scene_client_) {get_planning_scene_client_.reset();}
    if (attached_collision_pub_) {attached_collision_pub_.reset();}
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger_, "Failed to reset clients/publishers: %s", e.what());
  }

  {
    std::lock_guard<std::mutex> lock(config_mutex_);
    job_loaded_ = false;
  }

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
GripperActionServer::on_shutdown(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(logger_, "Shutting down gripper action server");
  // stop() while moveit_executor_ is still spinning so the cancel reaches the controller
  // and execute() returns cleanly before shutdown_worker() joins the worker thread.
  try {
    if (move_group_) {
      move_group_->stop();
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger_, "Failed to stop move group: %s", e.what());
  }

  // Wait for any in-flight execute_job() to finish before joining the worker thread.
  // The executor must still be spinning here so execute()'s result callback can unblock.
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

  try {
    if (action_server_) {
      action_server_.reset();
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger_, "Failed to reset action_server: %s", e.what());
  }

  try {
    std::lock_guard<std::mutex> lock(move_group_mutex_);
    move_group_.reset();
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger_, "Failed to reset move_group: %s", e.what());
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

  try {
    if (gripper_action_client_) {gripper_action_client_.reset();}
    if (planning_scene_client_) {planning_scene_client_.reset();}
    if (get_planning_scene_client_) {get_planning_scene_client_.reset();}
    if (attached_collision_pub_) {attached_collision_pub_.reset();}
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger_, "Failed to reset clients/publishers: %s", e.what());
  }

  {
    std::lock_guard<std::mutex> lock(config_mutex_);
    job_loaded_ = false;
  }

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

void GripperActionServer::manual_shutdown()
{
  // Idempotent — safe to call multiple times (destructor calls it as a safety net).
  if (shutdown_requested_.exchange(true)) {
    return;
  }

  RCLCPP_DEBUG(logger_, "Manual shutdown: signalling stop");

  // stop() is a topic publish — works as long as the ROS context is still valid.
  // Call it before rclcpp::shutdown() so the move_group node can process it.
  try {
    if (move_group_) {
      move_group_->stop();
    }
  } catch (...) {
    RCLCPP_WARN(logger_, "Exception caught while stopping move_group during shutdown");
  }

  // Poll until execute() returns or the ROS context dies — whichever comes first.
  // execute() blocks until the controller responds, so we must break the wait
  // externally when the context is invalidated to avoid spinning forever.
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

    if (future_copy.wait_for(std::chrono::milliseconds(0)) ==
      std::future_status::ready)
    {
      RCLCPP_INFO(logger_, "Execution finished cleanly.");
    } else {
      // Context died before execute() returned — detach worker to avoid std::terminate().
      RCLCPP_WARN(logger_, "ROS context shut down before execute() returned — detaching worker."
                           " move_group_ is kept alive by the thread's captured shared_ptr.");
      execution_cv_.notify_all();
      if (worker_thread_.joinable()) {
        worker_thread_.detach();
      }
      return;
    }
  }

  // Worker is idle — wake it so it can see shutdown_requested_ and exit.
  execution_cv_.notify_all();
  if (worker_thread_.joinable()) {
    worker_thread_.join();
  }
}

void GripperActionServer::worker_thread_func()
{
  // TODO(berkan): Add a watchdog timeout on execute() to prevent the worker from blocking
  // indefinitely if the controller stops responding. On timeout, abort the in-flight
  // goal and allow the next queued goal to be processed.
  while (true) {
    std::shared_ptr<GoalHandleTriggerGripper> goal_handle;

    {
      std::unique_lock<std::mutex> lock(execution_mutex_);
      execution_cv_.wait(lock, [this] {
          return pending_goal_ != nullptr || shutdown_requested_.load();
        });

      if (shutdown_requested_.load() && pending_goal_ == nullptr) {
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
      execute_job(goal_handle);
      promise->set_value();
    }
  }
}

void GripperActionServer::shutdown_worker()
{
  {
    std::lock_guard<std::mutex> lock(execution_mutex_);
    shutdown_requested_ = true;
  }
  execution_cv_.notify_all();
  if (worker_thread_.joinable()) {
    worker_thread_.join();
  }
}

// TODO(@silanus23): Consider handling maybe_unused
rclcpp_action::GoalResponse GripperActionServer::handle_goal(
  [[maybe_unused]] const rclcpp_action::GoalUUID & uuid,
  [[maybe_unused]] std::shared_ptr<const TriggerGripper::Goal> goal)
{
  if (get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    RCLCPP_ERROR(logger_, "Cannot accept goal: node is not active");
    return rclcpp_action::GoalResponse::REJECT;
  }

  std::lock_guard<std::mutex> lock(config_mutex_);
  if (!job_loaded_) {
    RCLCPP_ERROR(logger_, "No job loaded");
    return rclcpp_action::GoalResponse::REJECT;
  }

  RCLCPP_INFO(logger_, "Received gripper trigger for target: %s", job_.target_id.c_str());
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse GripperActionServer::handle_cancel(
  [[maybe_unused]] const std::shared_ptr<GoalHandleTriggerGripper> goal_handle)
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

void GripperActionServer::handle_accepted(
  const std::shared_ptr<GoalHandleTriggerGripper> goal_handle)
{
  // Enqueue goal for worker thread without blocking the executor.
  {
    std::lock_guard<std::mutex> lock(execution_mutex_);
    pending_goal_ = goal_handle;
  }
  execution_cv_.notify_one();
}

void GripperActionServer::execute_job(const std::shared_ptr<GoalHandleTriggerGripper> goal_handle)
{
  auto feedback = std::make_shared<TriggerGripper::Feedback>();
  auto result = std::make_shared<TriggerGripper::Result>();

  auto feedback_callback = [&](const std::string & step_name, float pct) {
      if (goal_handle->is_canceling()) {
        return;
      }
      feedback->current_step = step_name;
      feedback->completion_percentage = pct;
      goal_handle->publish_feedback(feedback);
    };

  bool success = run_job(feedback_callback);

  if (goal_handle->is_canceling()) {
    result->success = false;
    result->message = "Canceled";
    goal_handle->canceled(result);
    return;
  }

  if (success) {
    feedback->current_step = "completed";
    feedback->completion_percentage = 100.0f;
    goal_handle->publish_feedback(feedback);

    result->success = true;
    result->message = "Gripper job completed";
    result->positions_executed = 6;
    goal_handle->succeed(result);
  } else {
    result->success = false;
    result->message = "Gripper job failed";
    goal_handle->abort(result);
  }
}

bool GripperActionServer::run_job(
  std::function<void(const std::string &, float)> feedback_callback)
{
  GripperJob job;
  {
    std::lock_guard<std::mutex> lock(config_mutex_);
    job = job_;
  }

  RCLCPP_INFO(logger_, "Starting gripper job for target: %s", job.target_id.c_str());

  int step = 0;
  const int total_steps = 6;

  auto step_feedback = [&](const std::string & name) {
      feedback_callback(name, (static_cast<float>(step) / total_steps) * 100.0f);
      RCLCPP_INFO(logger_, "[Step %d/%d] %s", step + 1, total_steps, name.c_str());
      step++;
    };

  step_feedback("opening_gripper");
  if (!set_finger_aperture(open_position_)) {
    RCLCPP_ERROR(logger_, "Failed to open gripper");
    return false;
  }

  step_feedback("moving_to_approach");
  if (!move_to_pose(job.approach_pose, "approach")) {
    RCLCPP_ERROR(logger_, "Failed to move to approach");
    return false;
  }

  step_feedback("moving_to_pick");
  if (!move_to_pose(job.pick_pose, "pick")) {
    RCLCPP_ERROR(logger_, "Failed to move to pick");
    return false;
  }

  step_feedback("closing_gripper");
  if (!set_finger_aperture(close_position_)) {
    RCLCPP_ERROR(logger_, "Failed to close gripper");
    return false;
  }
  if (!attach_object(job.target_id)) {
    RCLCPP_ERROR(logger_, "Failed to attach object '%s' — aborting job", job.target_id.c_str());
    return false;
  }

  step_feedback("moving_to_retract");
  if (!move_to_pose(job.retract_pose, "retract")) {
    RCLCPP_ERROR(logger_, "Failed to move to retract");
    return false;
  }

  if (!allow_collision_for_placement()) {
    RCLCPP_ERROR(logger_,
        "Failed to update ACM for placement — aborting to avoid collision planning failure");
    return false;
  }

  step_feedback("moving_to_place");
  if (!move_to_pose(job.place_pose, "place")) {
    RCLCPP_ERROR(logger_, "Failed to move to place");
    return false;
  }

  // TODO(berkan): call detach_object(job.target_id) here once the planning scene
  // teardown and re-grasp workflows are defined.
  RCLCPP_INFO(logger_, "Gripper job completed successfully");
  return true;
}

void GripperActionServer::load_object_config()
{
  try {
    std::string objects_yaml_path =
      ament_index_cpp::get_package_share_directory("hold_and_weld_application") +
      "/config/collision_objects/objects.yaml";

    YAML::Node config = YAML::LoadFile(objects_yaml_path);

    if (config["/**"] && config["/**"]["ros__parameters"] &&
      config["/**"]["ros__parameters"]["base_link"])
    {
      auto base_link = config["/**"]["ros__parameters"]["base_link"];
      if (base_link["id"]) {
        base_link_id_ = base_link["id"].as<std::string>();
        RCLCPP_INFO(logger_, "Loaded base_link_id: %s", base_link_id_.c_str());
      } else {
        RCLCPP_WARN(logger_, "base_link.id not found in objects.yaml, using default: %s",
                    base_link_id_.c_str());
      }
    } else {
      RCLCPP_WARN(logger_,
                  "base_link configuration not found in objects.yaml, using default: %s",
                  base_link_id_.c_str());
    }
  } catch (const YAML::Exception & e) {
    RCLCPP_WARN(logger_, "YAML error loading object config: %s. Using default base_link_id: %s",
                e.what(), base_link_id_.c_str());
  } catch (const std::exception & e) {
    RCLCPP_WARN(logger_, "Could not load object config: %s. Using default base_link_id: %s",
                e.what(), base_link_id_.c_str());
  }
}

void GripperActionServer::normalize_quaternion(geometry_msgs::msg::Quaternion & q)
{
  double norm = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
  if (norm < 1e-10) {
    RCLCPP_WARN(logger_, "Invalid quaternion (near-zero norm), setting to identity");
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

// TODO(@silanus23): Validate yaml values
void GripperActionServer::load_job_from_yaml(const std::string & yaml_path)
{
  std::lock_guard<std::mutex> lock(config_mutex_);
  RCLCPP_INFO(logger_, "Loading job from: %s", yaml_path.c_str());

  try {
    YAML::Node config = YAML::LoadFile(yaml_path);

    auto load_pose = [this](
      const YAML::Node & pose_node, geometry_msgs::msg::Pose & pose) -> bool {
        if (!pose_node || !pose_node["position"] || !pose_node["orientation"]) {
          RCLCPP_ERROR(logger_,
                       "Pose node missing required 'position' or 'orientation' fields");
          return false;
        }

        try {
          pose.position.x = pose_node["position"]["x"].as<double>();
          pose.position.y = pose_node["position"]["y"].as<double>();
          pose.position.z = pose_node["position"]["z"].as<double>();
          pose.orientation.x = pose_node["orientation"]["x"].as<double>();
          pose.orientation.y = pose_node["orientation"]["y"].as<double>();
          pose.orientation.z = pose_node["orientation"]["z"].as<double>();
          pose.orientation.w = pose_node["orientation"]["w"].as<double>();
          normalize_quaternion(pose.orientation);
          return true;
        } catch (const YAML::Exception & e) {
          RCLCPP_ERROR(logger_, "Failed to parse pose values: %s", e.what());
          return false;
        }
      };

    if (config["targets"] && config["targets"].size() > 0) {
      const auto & target = config["targets"][0];

      job_.target_id = target["target_id"].as<std::string>("unknown_part");

      bool success = true;
      success &= load_pose(target["approach_pose"], job_.approach_pose);
      success &= load_pose(target["pick_pose"], job_.pick_pose);
      success &= load_pose(target["retract_pose"], job_.retract_pose);
      success &= load_pose(target["place_pose"], job_.place_pose);

      if (success) {
        job_loaded_ = true;
        RCLCPP_INFO(logger_, "Successfully loaded job for: %s", job_.target_id.c_str());
      } else {
        RCLCPP_ERROR(logger_, "Failed to parse one or more poses for target: %s",
            job_.target_id.c_str());
      }
    } else {
      RCLCPP_ERROR(logger_, "No targets found in YAML array!");
    }
  } catch (const YAML::Exception & e) {
    RCLCPP_ERROR(logger_, "YAML parsing error: %s", e.what());
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger_, "Error loading YAML: %s", e.what());
  }
}

bool GripperActionServer::set_finger_aperture(double position)
{
  if (!gripper_action_client_->wait_for_action_server(
            std::chrono::seconds(timing::ACTION_SERVER_TIMEOUT_SEC)))
  {
    RCLCPP_ERROR(logger_, "Action server not available");
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
    RCLCPP_ERROR(logger_, "Timeout sending gripper goal (%.3f m)", position);
    return false;
  }

  auto goal_handle = future.get();
  if (!goal_handle) {
    RCLCPP_ERROR(logger_, "Gripper goal rejected (%.3f m)", position);
    return false;
  }

  auto result_future = gripper_action_client_->async_get_result(goal_handle);

  auto result_status =
    result_future.wait_for(std::chrono::seconds(timing::GRIPPER_RESULT_TIMEOUT_SEC));
  if (result_status != std::future_status::ready) {
    RCLCPP_ERROR(logger_, "Timeout waiting for gripper result (%.3f m)", position);
    return false;
  }

  auto result = result_future.get();
  if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
    return true;
  }

  RCLCPP_ERROR(logger_, "Failed to reach gripper position (%.3f m)", position);
  return false;
}

bool GripperActionServer::move_to_pose(
  const geometry_msgs::msg::Pose & pose,
  const std::string & step_name)
{
  RCLCPP_INFO(logger_, "[%s] Planning to (%.3f, %.3f, %.3f)",
               step_name.c_str(),
               pose.position.x,
               pose.position.y,
               pose.position.z);

  for (int attempt = 1; attempt <= max_planning_retries_; ++attempt) {
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool plan_success = false;
    moveit::core::MoveItErrorCode exec_result;

    {
      std::unique_lock<std::mutex> lock(move_group_mutex_);
      move_group_->setPoseTarget(pose);
      move_group_->setStartStateToCurrentState();

      plan_success = (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

      if (plan_success) {
        RCLCPP_INFO(logger_, "[%s] Executing", step_name.c_str());
        exec_result = move_group_->execute(plan);
      }
    }

    if (plan_success) {
      if (exec_result != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_ERROR(logger_, "[%s] Execution failed!", step_name.c_str());

        if (attempt < max_planning_retries_) {
          RCLCPP_WARN(logger_, "[%s] Retrying execution (attempt %d/%d)",
                      step_name.c_str(), attempt, max_planning_retries_);
          std::this_thread::sleep_for(std::chrono::milliseconds(500));
          continue;
        }
        return false;
      }

      wait_for_planning_scene_update(timing::MOTION_SETTLE_TIME_MS);
      return true;
    } else {
      RCLCPP_WARN(logger_, "[%s] Planning attempt %d/%d failed",
                  step_name.c_str(), attempt, max_planning_retries_);

      if (attempt < max_planning_retries_) {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
      }
    }
  }

  RCLCPP_ERROR(logger_, "[%s] Planning failed after %d attempts!",
               step_name.c_str(), max_planning_retries_);
  return false;
}

bool GripperActionServer::wait_for_planning_scene_update(int millis)
{
  // Intentional fixed-duration sleep: MoveIt's planning scene update is asynchronous
  // (publish -> apply pipeline). Without a dedicated scene monitor we cannot poll for
  // a version change, so we sleep to let the update propagate before the next plan call.
  auto start = std::chrono::steady_clock::now();
  auto duration = std::chrono::milliseconds(millis);
  std::this_thread::sleep_for(duration);

  if (std::chrono::steady_clock::now() - start >= duration) {
    return true;
  }
  return false;
}

// Collision objects
bool GripperActionServer::attach_object(const std::string & object_id)
{
  RCLCPP_INFO(logger_, "Attaching '%s' to '%s'", object_id.c_str(),
      attach_link_.c_str());

  if (!attached_collision_pub_->is_activated()) {
    RCLCPP_ERROR(logger_, "attached_collision_pub_ is not activated — cannot attach object");
    return false;
  }

  moveit_msgs::msg::AttachedCollisionObject attached_object;
  attached_object.link_name = attach_link_;
  attached_object.object.id = object_id;
  attached_object.object.operation = moveit_msgs::msg::CollisionObject::ADD;
  attached_object.touch_links = touch_links_;

  attached_collision_pub_->publish(attached_object);
  wait_for_planning_scene_update(timing::ATTACH_SETTLE_TIME_MS);

  return true;
}

bool GripperActionServer::detach_object(const std::string & object_id)
{
  // TODO(@silanus23): this function is intentionally not called yet.
  // It will be wired in run_job() once the planning scene teardown
  // and re-grasp workflows are defined.
  if (!attached_collision_pub_->is_activated()) {
    RCLCPP_ERROR(logger_, "attached_collision_pub_ is not activated — cannot detach object");
    return false;
  }

  moveit_msgs::msg::AttachedCollisionObject detach_object;
  detach_object.object.id = object_id;
  detach_object.object.operation = moveit_msgs::msg::CollisionObject::REMOVE;

  attached_collision_pub_->publish(detach_object);
  wait_for_planning_scene_update(timing::DETACH_SETTLE_TIME_MS);

  return true;
}

// TODO(@silanus23): Make this for whole object not a link
bool GripperActionServer::allow_collision_for_placement()
{
  auto get_request = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
  get_request->components.components =
    moveit_msgs::msg::PlanningSceneComponents::ALLOWED_COLLISION_MATRIX;

  if (!get_planning_scene_client_->wait_for_service(
      std::chrono::seconds(timing::PLANNING_SCENE_SERVICE_TIMEOUT_SEC)))
  {
    RCLCPP_ERROR(logger_, "Get Planning Scene service not available");
    return false;
  }

  auto get_future = get_planning_scene_client_->async_send_request(get_request);
  if (get_future.wait_for(
      std::chrono::seconds(timing::PLANNING_SCENE_RESPONSE_TIMEOUT_SEC)) !=
    std::future_status::ready)
  {
    RCLCPP_ERROR(logger_, "Timeout getting planning scene");
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
          // Object not in ACM yet — add it and expand the symmetric matrix to fit
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

      acm.entry_values[idx1].enabled[idx2] = true;
      acm.entry_values[idx2].enabled[idx1] = true;
    };

  toggle_acm_bit(job_.target_id, base_link_id_);

  auto apply_request = std::make_shared<moveit_msgs::srv::ApplyPlanningScene::Request>();
  apply_request->scene.allowed_collision_matrix = acm;
  apply_request->scene.is_diff = true;

  auto apply_future = planning_scene_client_->async_send_request(apply_request);
  if (apply_future.wait_for(
      std::chrono::seconds(timing::PLANNING_SCENE_RESPONSE_TIMEOUT_SEC)) ==
    std::future_status::ready)
  {
    return apply_future.get()->success;
  }

  return false;
}

}  // namespace application
}  // namespace hold_and_weld
