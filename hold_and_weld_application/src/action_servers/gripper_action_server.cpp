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

#include <algorithm>
#include <functional>
#include <utility>

#include <lifecycle_msgs/msg/state.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "hold_and_weld_application/action_servers/gripper_aperture.hpp"
#include "hold_and_weld_application/utils.hpp"

namespace hold_and_weld
{
namespace application
{

namespace
{

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

// Below this norm a quaternion has no usable direction (an unset Pose has w = 0).
constexpr double kMinQuaternionNorm = 1e-6;

/**
 * @brief Normalize q in place; false, leaving it untouched, if it has no orientation
 * (a non-finite component or a near-zero norm).
 */
bool normalize_quaternion(geometry_msgs::msg::Quaternion & q)
{
  const double norm = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
  // Written so that NaN fails too.
  if (!(std::isfinite(norm) && norm >= kMinQuaternionNorm)) {
    return false;
  }
  q.x /= norm;
  q.y /= norm;
  q.z /= norm;
  q.w /= norm;
  return true;
}

}  // namespace

GripperActionServer::GripperActionServer(const rclcpp::NodeOptions & options)
: LifecycleNode("gripper_action_server", options),
  logger_(rclcpp::get_logger("application"))
{
  // Declared here, not in on_configure, so configure -> cleanup -> configure
  // does not throw ParameterAlreadyDeclaredException.
  std::string default_yaml =
    ament_index_cpp::get_package_share_directory("hold_and_weld_application") +
    "/config/tasks/pick_place_targets.yaml";

  declare_parameter("arm_group_name", "robot1_gp25_arm");
  declare_parameter("positions_yaml", default_yaml);
  declare_parameter("gripper_joint_names", std::vector<std::string>{
        "robot1_left_finger_joint", "robot1_right_finger_joint"});
  declare_parameter("auto_trigger", false);
  declare_parameter("auto_trigger_delay_sec", 3.0);
  declare_parameter(
    "gripper_controller_topic", "/robot1_gripper_controller/follow_joint_trajectory");
  declare_parameter("planning_time", 10.0);
  declare_parameter("num_planning_attempts", 10);
  declare_parameter("velocity_scaling", 0.3);
  declare_parameter("acceleration_scaling", 0.3);
  declare_parameter("max_planning_retries", 3);
  declare_parameter("service_timeout_sec", 5.0);
  declare_parameter("controller_timeout_sec", 60.0);
  declare_parameter("shutdown_wait_sec", 5.0);
  declare_parameter("finger_motion_sec", 2.0);
  declare_parameter("finger_settle_sec", 0.5);
  declare_parameter("motion_settle_sec", 0.25);
}

GripperActionServer::~GripperActionServer()
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

CallbackReturn GripperActionServer::on_configure(const rclcpp_lifecycle::State & state)
{
  arm_group_name_ = get_parameter("arm_group_name").as_string();
  gripper_joint_names_ = get_parameter("gripper_joint_names").as_string_array();
  yaml_path_ = get_parameter("positions_yaml").as_string();
  auto_trigger_ = get_parameter("auto_trigger").as_bool();
  auto_trigger_delay_sec_ = get_parameter("auto_trigger_delay_sec").as_double();
  const std::string gripper_controller_topic =
    get_parameter("gripper_controller_topic").as_string();
  const double planning_time = get_parameter("planning_time").as_double();
  const int64_t num_planning_attempts = get_parameter("num_planning_attempts").as_int();
  const double velocity_scaling = get_parameter("velocity_scaling").as_double();
  const double acceleration_scaling = get_parameter("acceleration_scaling").as_double();
  const int64_t max_planning_retries = get_parameter("max_planning_retries").as_int();
  const double service_timeout_sec = get_parameter("service_timeout_sec").as_double();
  const double controller_timeout_sec = get_parameter("controller_timeout_sec").as_double();
  const double shutdown_wait_sec = get_parameter("shutdown_wait_sec").as_double();
  const double finger_motion_sec = get_parameter("finger_motion_sec").as_double();
  const double finger_settle_sec = get_parameter("finger_settle_sec").as_double();
  const double motion_settle_sec = get_parameter("motion_settle_sec").as_double();

  if (!hold_and_weld::is_valid_auto_trigger_delay(auto_trigger_delay_sec_)) {
    RCLCPP_ERROR(logger_, "auto_trigger_delay_sec must be in [0, 3600], got %.3f",
      auto_trigger_delay_sec_);
    return CallbackReturn::FAILURE;
  }
  if (!(std::isfinite(planning_time) && planning_time > 0.0)) {
    RCLCPP_ERROR(logger_, "planning_time must be positive, got %.3f", planning_time);
    return CallbackReturn::FAILURE;
  }
  if (num_planning_attempts < 1 || num_planning_attempts > 1000) {
    RCLCPP_ERROR(logger_, "num_planning_attempts must be in [1, 1000], got %s",
      std::to_string(num_planning_attempts).c_str());
    return CallbackReturn::FAILURE;
  }
  if (!(velocity_scaling > 0.0 && velocity_scaling <= 1.0) ||
    !(acceleration_scaling > 0.0 && acceleration_scaling <= 1.0))
  {
    RCLCPP_ERROR(logger_, "velocity_scaling and acceleration_scaling must be in (0, 1], "
      "got %.3f and %.3f", velocity_scaling, acceleration_scaling);
    return CallbackReturn::FAILURE;
  }
  if (max_planning_retries < 1 || max_planning_retries > 100) {
    RCLCPP_ERROR(logger_, "max_planning_retries must be in [1, 100], got %s",
      std::to_string(max_planning_retries).c_str());
    return CallbackReturn::FAILURE;
  }
  max_planning_retries_ = static_cast<int>(max_planning_retries);
  if (!(finger_settle_sec >= 0.0 && finger_settle_sec <= 10.0) ||
    !(motion_settle_sec >= 0.0 && motion_settle_sec <= 10.0))
  {
    RCLCPP_ERROR(logger_, "finger_settle_sec and motion_settle_sec must be in [0, 10], "
      "got %.3f and %.3f", finger_settle_sec, motion_settle_sec);
    return CallbackReturn::FAILURE;
  }
  if (!(service_timeout_sec > 0.0 && service_timeout_sec <= 60.0) ||
    !(shutdown_wait_sec > 0.0 && shutdown_wait_sec <= 60.0))
  {
    RCLCPP_ERROR(logger_, "service_timeout_sec and shutdown_wait_sec must be in (0, 60], "
      "got %.3f and %.3f", service_timeout_sec, shutdown_wait_sec);
    return CallbackReturn::FAILURE;
  }
  if (!(controller_timeout_sec > 0.0 && controller_timeout_sec <= 600.0)) {
    RCLCPP_ERROR(logger_, "controller_timeout_sec must be in (0, 600], got %.3f",
      controller_timeout_sec);
    return CallbackReturn::FAILURE;
  }
  service_timeout_ = hold_and_weld::to_nanoseconds(service_timeout_sec);
  controller_timeout_ = hold_and_weld::to_nanoseconds(controller_timeout_sec);
  shutdown_wait_time_ = hold_and_weld::to_nanoseconds(shutdown_wait_sec);
  if (!(finger_motion_sec > 0.0 && finger_motion_sec <= 10.0)) {
    RCLCPP_ERROR(logger_, "finger_motion_sec must be in (0, 10], got %.3f", finger_motion_sec);
    return CallbackReturn::FAILURE;
  }
  finger_motion_time_ = hold_and_weld::to_nanoseconds(finger_motion_sec);
  finger_settle_time_ = hold_and_weld::to_nanoseconds(finger_settle_sec);
  motion_settle_time_ = hold_and_weld::to_nanoseconds(motion_settle_sec);

  gripper_controller_name_ =
    hold_and_weld::controller_name_from_action_topic(gripper_controller_topic);
  if (gripper_controller_name_.empty()) {
    RCLCPP_ERROR(logger_, "gripper_controller_topic '%s' names no controller "
      "(expected /<controller>/<action>)", gripper_controller_topic.c_str());
    return CallbackReturn::FAILURE;
  }

  // This lifecycle node is not spinning freely during on_configure, so service calls on
  // 'this' would deadlock.
  auto temp_node = std::make_shared<rclcpp::Node>("gripper_service_waiter");
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

  RCLCPP_INFO(logger_, "Arm group: %s", arm_group_name_.c_str());

  try {
    // The launch file's parameters are /** overrides, so the internal node picks up
    // robot_description_semantic (and the rest) by auto-declaring them. Do not declare
    // them on this lifecycle node as well: the internal node's own declare would throw.
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);

    auto internal_node = std::make_shared<rclcpp::Node>(
      "gripper_moveit_internal",
      node_options);

    // Everything the worker waits on lives on the internal node, spun by moveit_executor_,
    // so a job never needs the main executor (which a transition may be blocking).
    gripper_action_client_ = rclcpp_action::create_client<FollowJointTrajectory>(
      internal_node, gripper_controller_topic);
    list_controllers_client_ = internal_node->create_client<
      controller_manager_msgs::srv::ListControllers>("/controller_manager/list_controllers");
    planning_scene_client_ = internal_node->create_client<moveit_msgs::srv::ApplyPlanningScene>(
      "/apply_planning_scene");
    get_planning_scene_client_ =
      internal_node->create_client<moveit_msgs::srv::GetPlanningScene>("/get_planning_scene");

    moveit_executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    moveit_executor_->add_node(internal_node);
    moveit_thread_ = std::thread([this]() {moveit_executor_->spin();});

    auto move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
      internal_node, arm_group_name_);
    move_group->setPlanningTime(planning_time);
    move_group->setNumPlanningAttempts(static_cast<unsigned int>(num_planning_attempts));
    move_group->setMaxVelocityScalingFactor(velocity_scaling);
    move_group->setMaxAccelerationScalingFactor(acceleration_scaling);
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

  load_object_config();
  if (!load_job_from_yaml(yaml_path_)) {
    RCLCPP_WARN(logger_, "No job loaded - action server will reject goals");
  }

  if (!resolve_finger_apertures()) {
    on_cleanup(state);
    return CallbackReturn::FAILURE;
  }

  using std::placeholders::_1;
  using std::placeholders::_2;
  action_server_ = rclcpp_action::create_server<TriggerGripper>(
    this,
    "trigger_gripper",
    std::bind(&GripperActionServer::handle_goal, this, _1, _2),
    std::bind(&GripperActionServer::handle_cancel, this, _1),
    std::bind(&GripperActionServer::handle_accepted, this, _1)
  );

  self_trigger_client_ = rclcpp_action::create_client<TriggerGripper>(
    this->get_node_base_interface(),
    this->get_node_graph_interface(),
    this->get_node_logging_interface(),
    this->get_node_waitables_interface(),
    "trigger_gripper");

  {
    std::lock_guard<std::mutex> lock(execution_mutex_);
    shutdown_requested_ = false;
    execution_future_ = std::shared_future<void>();
  }
  stop_requested_ = false;
  auto_trigger_fired_ = false;
  worker_thread_ = std::thread(&GripperActionServer::worker_thread_func, this);

  return CallbackReturn::SUCCESS;
}

CallbackReturn GripperActionServer::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(logger_, "Activating gripper action server");

  if (auto_trigger_ && job_loaded_ && !auto_trigger_fired_) {
    RCLCPP_INFO(logger_, "Auto-trigger enabled, will start in %.1f seconds",
      auto_trigger_delay_sec_);

    auto_trigger_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(auto_trigger_delay_sec_)),
      [this]() {send_auto_trigger_goal();});
  }

  return CallbackReturn::SUCCESS;
}

void GripperActionServer::send_auto_trigger_goal()
{
  auto_trigger_timer_->cancel();
  auto_trigger_fired_ = true;
  RCLCPP_INFO(logger_, "Auto-triggering gripper job via trigger_gripper action");

  if (!hold_and_weld::wait_for_action_server(
      self_trigger_client_, "trigger_gripper", logger_, 10))
  {
    return;
  }

  auto send_goal_options = rclcpp_action::Client<TriggerGripper>::SendGoalOptions();
  send_goal_options.result_callback =
    [this](const rclcpp_action::ClientGoalHandle<TriggerGripper>::WrappedResult & result)
    {
      const char * message = result.result ? result.result->message.c_str() : "";
      if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
        RCLCPP_INFO(logger_, "Auto-triggered gripper job succeeded: %s", message);
      } else {
        RCLCPP_ERROR(logger_, "Auto-triggered gripper job failed: %s", message);
      }
    };

  self_trigger_client_->async_send_goal(TriggerGripper::Goal(), send_goal_options);
}

CallbackReturn GripperActionServer::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(logger_, "Deactivating gripper action server");
  if (auto_trigger_timer_) {
    auto_trigger_timer_->cancel();
    auto_trigger_timer_.reset();
  }

  // The worker takes a goal and publishes execution_future_ under execution_mutex_, so
  // once the queued goal is gone here, the only job left to wait for is a running one.
  std::shared_ptr<GoalHandleTriggerGripper> queued;
  {
    std::lock_guard<std::mutex> lock(execution_mutex_);
    queued = std::exchange(pending_goal_, nullptr);
  }
  hold_and_weld::end_goal_early<TriggerGripper>(
    queued, "Gripper server deactivated before the goal started", logger_);

  request_stop();
  wait_for_running_job();

  return CallbackReturn::SUCCESS;
}

CallbackReturn GripperActionServer::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(logger_, "Cleaning up gripper action server");
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

  gripper_action_client_.reset();
  list_controllers_client_.reset();
  planning_scene_client_.reset();
  get_planning_scene_client_.reset();

  job_loaded_ = false;
  return CallbackReturn::SUCCESS;
}

CallbackReturn GripperActionServer::on_shutdown(const rclcpp_lifecycle::State & state)
{
  // Reachable from any primary state; on_cleanup() is null-safe for whatever was never set up.
  RCLCPP_INFO(logger_, "Shutting down gripper action server");
  return on_cleanup(state);
}

void GripperActionServer::manual_shutdown()
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
      logger_, "Gripper job still running %.1f s after stop(); detaching the worker thread",
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

void GripperActionServer::request_stop()
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

bool GripperActionServer::wait_for_running_job(std::optional<std::chrono::nanoseconds> timeout)
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

void GripperActionServer::worker_thread_func()
{
  while (true) {
    std::shared_ptr<GoalHandleTriggerGripper> goal_handle;
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
      execute_job(goal_handle);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(logger_, "Gripper job failed with an exception: %s", e.what());
      hold_and_weld::end_goal_early<TriggerGripper>(
        goal_handle, std::string("Gripper job failed: ") + e.what(), logger_);
    } catch (...) {
      RCLCPP_ERROR(logger_, "Gripper job failed with an unknown exception");
      hold_and_weld::end_goal_early<TriggerGripper>(
        goal_handle, "Gripper job failed with an unknown exception", logger_);
    }
    // No-op if the goal already ended; catches a path that returned without a result.
    hold_and_weld::end_goal_early<TriggerGripper>(
      goal_handle, "Gripper job ended without a result", logger_);
    done.set_value();
  }
}

void GripperActionServer::shutdown_worker()
{
  std::shared_ptr<GoalHandleTriggerGripper> queued;
  {
    std::lock_guard<std::mutex> lock(execution_mutex_);
    shutdown_requested_ = true;
    queued = std::exchange(pending_goal_, nullptr);
  }
  hold_and_weld::end_goal_early<TriggerGripper>(
    queued, "Gripper server shut down before the goal started", logger_);
  worker_cv_.notify_all();
  if (worker_thread_.joinable()) {
    worker_thread_.join();
  }
}

rclcpp_action::GoalResponse GripperActionServer::handle_goal(
  [[maybe_unused]] const rclcpp_action::GoalUUID & uuid,
  [[maybe_unused]] std::shared_ptr<const TriggerGripper::Goal> goal)
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
      RCLCPP_WARN(logger_, "Cannot accept goal: a gripper job is already queued or running");
      return rclcpp_action::GoalResponse::REJECT;
    }
  }

  if (!job_loaded_) {
    RCLCPP_WARN(logger_, "Cannot accept goal: no job loaded from %s", yaml_path_.c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }

  RCLCPP_INFO(logger_, "Received gripper trigger for target: %s", job_.target_id.c_str());
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse GripperActionServer::handle_cancel(
  [[maybe_unused]] const std::shared_ptr<GoalHandleTriggerGripper> goal_handle)
{
  RCLCPP_INFO(logger_, "Received cancel request");
  // Only one goal exists at a time, so this is the running (or about to run) one. The job
  // sees is_canceling() at its next check and starts no further step or retry.
  request_stop();

  return rclcpp_action::CancelResponse::ACCEPT;
}

void GripperActionServer::handle_accepted(
  const std::shared_ptr<GoalHandleTriggerGripper> goal_handle)
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
    hold_and_weld::end_goal_early<TriggerGripper>(
      goal_handle, "A gripper job is already queued", logger_);
    return;
  }
  worker_cv_.notify_one();
}

void GripperActionServer::execute_job(const std::shared_ptr<GoalHandleTriggerGripper> goal_handle)
{
  auto result = std::make_shared<TriggerGripper::Result>();

  const std::function<bool()> should_stop = [this, &goal_handle]() {
      return stop_requested_.load() || goal_handle->is_canceling();
    };

  using std::placeholders::_1;
  using std::placeholders::_2;
  const bool success = run_job(
    std::bind(&GripperActionServer::publish_feedback, this, goal_handle, _1, _2),
    should_stop);

  if (success) {
    publish_feedback(goal_handle, "completed", 100.0f);

    result->success = true;
    result->message = "Gripper job completed";
    result->positions_executed = 6;
    goal_handle->succeed(result);
  } else if (should_stop()) {
    hold_and_weld::end_goal_early<TriggerGripper>(
      goal_handle, goal_handle->is_canceling() ? "Canceled by client" :
      "Stopped: gripper server is deactivating or shutting down", logger_);
  } else {
    result->success = false;
    result->message = "Gripper job failed";
    goal_handle->abort(result);
  }
}

void GripperActionServer::publish_feedback(
  const std::shared_ptr<GoalHandleTriggerGripper> goal_handle,
  const std::string & step,
  float percentage)
{
  auto feedback = std::make_shared<TriggerGripper::Feedback>();
  feedback->current_step = step;
  feedback->completion_percentage = percentage;
  goal_handle->publish_feedback(feedback);
}

bool GripperActionServer::run_job(
  const std::function<void(const std::string &, float)> & feedback_callback,
  const std::function<bool()> & should_stop)
{
  const GripperJob & job = job_;
  const double open_position = open_position_;
  const double close_position = close_position_;

  RCLCPP_INFO(logger_, "Starting gripper job for target: %s", job.target_id.c_str());

  int step = 0;
  const int total_steps = 6;
  bool attached = false;
  bool collision_allowed = false;

  // False once the job must stop, so no step starts after a cancel or transition.
  auto begin_step = [&](const std::string & name) {
      if (should_stop()) {
        RCLCPP_WARN(logger_, "Gripper job stopped before '%s'", name.c_str());
        return false;
      }
      feedback_callback(name, (static_cast<float>(step) / total_steps) * 100.0f);
      RCLCPP_INFO(logger_, "[Step %d/%d] %s", step + 1, total_steps, name.c_str());
      step++;
      return true;
    };
  // The job does not undo its planning-scene changes (see the detach TODO below), so
  // say what it left behind; a re-run would otherwise plan with a phantom attached part.
  auto fail = [&](const std::string & reason) {
      if (!reason.empty()) {
        RCLCPP_ERROR(logger_, "%s", reason.c_str());
      }
      if (attached) {
        RCLCPP_WARN(logger_, "Planning scene left modified: '%s' is still attached to %s%s",
          job.target_id.c_str(), attach_link_.c_str(),
          collision_allowed ? ", and its collision with base_link is still allowed" : "");
      }
      return false;
    };

  if (!wait_for_gripper_controller(should_stop)) {
    return false;
  }

  if (!begin_step("opening_gripper")) {return fail("");}
  if (!set_finger_aperture(open_position, should_stop)) {
    return fail("Failed to open gripper");
  }

  if (!begin_step("moving_to_approach")) {return fail("");}
  if (!move_to_pose(job.approach_pose, "approach", should_stop)) {
    return fail("Failed to move to approach");
  }

  if (!begin_step("moving_to_pick")) {return fail("");}
  if (!move_to_pose(job.pick_pose, "pick", should_stop)) {
    return fail("Failed to move to pick");
  }

  if (!begin_step("closing_gripper")) {return fail("");}
  if (!set_finger_aperture(close_position, should_stop)) {
    return fail("Failed to close gripper");
  }
  if (!attach_object(job.target_id)) {
    return fail("Failed to attach object '" + job.target_id + "' — aborting job");
  }
  attached = true;

  if (!begin_step("moving_to_retract")) {return fail("");}
  if (!move_to_pose(job.retract_pose, "retract", should_stop)) {
    return fail("Failed to move to retract");
  }

  if (!allow_collision_for_placement(job.target_id)) {
    return fail("Failed to update ACM for placement — aborting to avoid collision planning "
             "failure");
  }
  collision_allowed = true;

  if (!begin_step("moving_to_place")) {return fail("");}
  if (!move_to_pose(job.place_pose, "place", should_stop)) {
    return fail("Failed to move to place");
  }

  // TODO(silanus23): call detach_object(job.target_id) here once the planning scene
  // teardown and re-grasp workflows are defined.
  RCLCPP_INFO(logger_, "Gripper job completed successfully");
  return true;
}

void GripperActionServer::load_object_config()
{
  try {
    const YAML::Node config = YAML::LoadFile(
      ament_index_cpp::get_package_share_directory("hold_and_weld_application") +
      "/config/collision_objects/objects.yaml");
    if (config["/**"] && config["/**"]["ros__parameters"] &&
      config["/**"]["ros__parameters"]["base_link"] &&
      config["/**"]["ros__parameters"]["base_link"]["id"])
    {
      base_link_id_ = config["/**"]["ros__parameters"]["base_link"]["id"].as<std::string>();
      RCLCPP_INFO(logger_, "Loaded base_link_id: %s", base_link_id_.c_str());
    } else {
      RCLCPP_WARN(logger_, "base_link.id not found in objects.yaml, using default: %s",
        base_link_id_.c_str());
    }
  } catch (const std::exception & e) {
    RCLCPP_WARN(logger_, "Could not load object config: %s. Using default base_link_id: %s",
      e.what(), base_link_id_.c_str());
  }
}

bool GripperActionServer::load_job_from_yaml(const std::string & yaml_path)
{
  RCLCPP_INFO(logger_, "Loading job from: %s", yaml_path.c_str());
  job_loaded_ = false;
  requested_open_position_.reset();
  requested_close_position_.reset();

  GripperJob job;
  std::optional<double> requested_open;
  std::optional<double> requested_close;

  auto load_pose = [this](
    const YAML::Node & pose_node, const char * name, geometry_msgs::msg::Pose & pose) -> bool {
      if (!pose_node || !pose_node["position"] || !pose_node["orientation"]) {
        RCLCPP_ERROR(logger_, "%s is missing 'position' or 'orientation'", name);
        return false;
      }
      pose.position.x = pose_node["position"]["x"].as<double>();
      pose.position.y = pose_node["position"]["y"].as<double>();
      pose.position.z = pose_node["position"]["z"].as<double>();
      pose.orientation.x = pose_node["orientation"]["x"].as<double>();
      pose.orientation.y = pose_node["orientation"]["y"].as<double>();
      pose.orientation.z = pose_node["orientation"]["z"].as<double>();
      pose.orientation.w = pose_node["orientation"]["w"].as<double>();
      if (!std::isfinite(pose.position.x) || !std::isfinite(pose.position.y) ||
        !std::isfinite(pose.position.z))
      {
        RCLCPP_ERROR(logger_, "%s position is not finite", name);
        return false;
      }
      if (!normalize_quaternion(pose.orientation)) {
        RCLCPP_ERROR(logger_, "%s orientation is zero or not finite", name);
        return false;
      }
      return true;
    };

  try {
    YAML::Node config = YAML::LoadFile(yaml_path);

    if (const YAML::Node gripper = config["gripper"]) {
      if (gripper["open_position"]) {
        requested_open = gripper["open_position"].as<double>();
      }
      if (gripper["close_position"]) {
        requested_close = gripper["close_position"].as<double>();
      }
      if (gripper["attach_link"] || gripper["touch_links"]) {
        RCLCPP_WARN(logger_, "gripper.attach_link / gripper.touch_links in %s are not read; "
          "the object is attached to %s with the server's built-in touch links",
          yaml_path.c_str(), attach_link_.c_str());
      }
    }

    const YAML::Node targets = config["targets"];
    if (!targets || !targets.IsSequence() || targets.size() == 0) {
      RCLCPP_ERROR(logger_, "No targets found in YAML array");
      return false;
    }
    if (targets.size() > 1) {
      RCLCPP_WARN(logger_, "%zu targets in %s; only the first is used",
        targets.size(), yaml_path.c_str());
    }
    const YAML::Node target = targets[0];

    if (!target["target_id"] || target["target_id"].as<std::string>().empty()) {
      RCLCPP_ERROR(logger_, "targets[0].target_id is required: it names the object to attach");
      return false;
    }
    job.target_id = target["target_id"].as<std::string>();

    if (!load_pose(target["approach_pose"], "approach_pose", job.approach_pose) ||
      !load_pose(target["pick_pose"], "pick_pose", job.pick_pose) ||
      !load_pose(target["retract_pose"], "retract_pose", job.retract_pose) ||
      !load_pose(target["place_pose"], "place_pose", job.place_pose))
    {
      return false;
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger_, "Error loading %s: %s", yaml_path.c_str(), e.what());
    return false;
  }

  job_ = job;
  requested_open_position_ = requested_open;
  requested_close_position_ = requested_close;
  job_loaded_ = true;
  RCLCPP_INFO(logger_, "Successfully loaded job for: %s", job_.target_id.c_str());
  return true;
}

bool GripperActionServer::resolve_finger_apertures()
{
  const auto robot_model = move_group_->getRobotModel();
  std::vector<hold_and_weld::FingerJointBounds> fingers;
  for (const auto & joint_name : gripper_joint_names_) {
    const auto * joint = robot_model->getJointModel(joint_name);
    if (!joint || joint->getVariableCount() != 1) {
      RCLCPP_ERROR(logger_, "Gripper joint '%s' is not a single-variable joint in the robot model",
        joint_name.c_str());
      return false;
    }
    const auto & bounds = joint->getVariableBounds().front();
    fingers.push_back({joint_name, bounds.min_position_, bounds.max_position_});
  }

  try {
    const auto apertures = hold_and_weld::resolve_gripper_apertures(
      fingers, requested_open_position_, requested_close_position_);
    open_position_ = apertures.open;
    close_position_ = apertures.close;
  } catch (const std::invalid_argument & e) {
    RCLCPP_ERROR(logger_, "Invalid gripper aperture: %s", e.what());
    return false;
  }
  RCLCPP_INFO(logger_, "Gripper apertures: open %.3f m (%s), close %.3f m (%s)",
    open_position_, requested_open_position_ ? "from positions YAML" : "from joint limit",
    close_position_, requested_close_position_ ? "from positions YAML" : "from joint limit");
  return true;
}

bool GripperActionServer::wait_for_gripper_controller(const std::function<bool()> & should_stop)
{
  const auto deadline = std::chrono::steady_clock::now() + controller_timeout_;
  bool logged_wait = false;
  while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
    if (should_stop()) {
      RCLCPP_WARN(logger_, "Stopped while waiting for %s", gripper_controller_name_.c_str());
      return false;
    }
    auto request = std::make_shared<controller_manager_msgs::srv::ListControllers::Request>();
    auto future = list_controllers_client_->async_send_request(request);
    const bool answered = future.wait_for(std::chrono::seconds(1)) == std::future_status::ready;
    if (answered &&
      hold_and_weld::is_controller_active(future.get()->controller, gripper_controller_name_))
    {
      return true;
    }
    if (!answered) {
      list_controllers_client_->remove_pending_request(future);
    }
    if (!logged_wait) {
      RCLCPP_INFO(logger_, "Waiting for %s to become active", gripper_controller_name_.c_str());
      logged_wait = true;
    }
    rclcpp::sleep_for(std::chrono::milliseconds(250));
  }
  RCLCPP_ERROR(logger_, "%s not active after %.1f s", gripper_controller_name_.c_str(),
    std::chrono::duration<double>(controller_timeout_).count());
  return false;
}

bool GripperActionServer::set_finger_aperture(
  double position, const std::function<bool()> & should_stop)
{
  if (!gripper_action_client_->wait_for_action_server(service_timeout_)) {
    RCLCPP_ERROR(logger_, "Gripper controller action server not available");
    return false;
  }

  auto goal_msg = FollowJointTrajectory::Goal();
  goal_msg.trajectory.joint_names = gripper_joint_names_;

  // Every finger joint gets the same command (see resolve_gripper_apertures).
  trajectory_msgs::msg::JointTrajectoryPoint point;
  point.positions = std::vector<double>(gripper_joint_names_.size(), position);
  point.time_from_start = rclcpp::Duration(finger_motion_time_);
  goal_msg.trajectory.points.push_back(point);

  auto promise = std::make_shared<std::promise<bool>>();
  auto future = promise->get_future();

  auto send_goal_options =
    rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();

  send_goal_options.goal_response_callback =
    [this, promise, position](
    const rclcpp_action::ClientGoalHandle<FollowJointTrajectory>::SharedPtr & handle)
    {
      if (!handle) {
        RCLCPP_ERROR(logger_, "Gripper goal rejected (%.3f m)", position);
        promise->set_value(false);
      }
    };

  send_goal_options.result_callback =
    [this, promise, position](
    const rclcpp_action::ClientGoalHandle<FollowJointTrajectory>::WrappedResult & result)
    {
      if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
        promise->set_value(true);
      } else {
        RCLCPP_ERROR(logger_, "Failed to reach gripper position (%.3f m)", position);
        promise->set_value(false);
      }
    };

  auto goal_handle_future =
    gripper_action_client_->async_send_goal(goal_msg, send_goal_options);

  // The callbacks run on moveit_executor_, so this wait never depends on the main
  // executor. Poll so a cancel or transition does not wait out the whole timeout.
  const auto timeout = finger_motion_time_ + std::chrono::seconds(kGripperResultMarginSec);
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (future.wait_for(std::chrono::milliseconds(50)) != std::future_status::ready) {
    const bool stopping = should_stop();
    if (!stopping && std::chrono::steady_clock::now() < deadline) {
      continue;
    }
    if (stopping) {
      RCLCPP_WARN(logger_, "Gripper motion to %.3f m interrupted: job stopping", position);
    } else {
      RCLCPP_ERROR(logger_, "Gripper did not reach %.3f m within %.1f s", position,
        std::chrono::duration<double>(timeout).count());
    }
    if (goal_handle_future.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
      if (auto handle = goal_handle_future.get()) {
        gripper_action_client_->async_cancel_goal(handle);
      }
    }
    return false;
  }

  const bool result = future.get();

  // The controller has no goal tolerances, so SUCCEEDED means the trajectory time ran out,
  // not that the fingers arrived (closing never does: the fingers stall on the part).
  if (result) {
    rclcpp::sleep_for(finger_settle_time_);
  }

  return result;
}

bool GripperActionServer::move_to_pose(
  const geometry_msgs::msg::Pose & pose,
  const std::string & step_name,
  const std::function<bool()> & should_stop)
{
  RCLCPP_INFO(logger_, "[%s] Planning to (%.3f, %.3f, %.3f)",
    step_name.c_str(), pose.position.x, pose.position.y, pose.position.z);

  for (int attempt = 1; attempt <= max_planning_retries_; ++attempt) {
    // A cancel makes execute() fail; checking here keeps that from being retried.
    if (should_stop()) {
      RCLCPP_WARN(logger_, "[%s] Stopped", step_name.c_str());
      return false;
    }

    move_group_->setPoseTarget(pose);
    move_group_->setStartStateToCurrentState();

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (move_group_->plan(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_WARN(logger_, "[%s] Planning attempt %d/%d failed",
        step_name.c_str(), attempt, max_planning_retries_);
      if (attempt < max_planning_retries_) {
        rclcpp::sleep_for(std::chrono::milliseconds(500));
      }
      continue;
    }

    if (should_stop()) {
      RCLCPP_WARN(logger_, "[%s] Stopped", step_name.c_str());
      return false;
    }
    RCLCPP_INFO(logger_, "[%s] Executing", step_name.c_str());
    if (move_group_->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
      rclcpp::sleep_for(motion_settle_time_);
      return true;
    }

    RCLCPP_ERROR(logger_, "[%s] Execution failed", step_name.c_str());
    if (attempt < max_planning_retries_ && !should_stop()) {
      RCLCPP_WARN(logger_, "[%s] Retrying from the current state (attempt %d/%d)",
        step_name.c_str(), attempt, max_planning_retries_);
      rclcpp::sleep_for(std::chrono::milliseconds(500));
    }
  }

  RCLCPP_ERROR(logger_, "[%s] Failed after %d attempts", step_name.c_str(),
    max_planning_retries_);
  return false;
}

// Collision objects
bool GripperActionServer::attach_object(const std::string & object_id)
{
  RCLCPP_INFO(logger_, "Attaching '%s' to '%s'", object_id.c_str(), attach_link_.c_str());

  moveit_msgs::msg::AttachedCollisionObject attached_object;
  attached_object.link_name = attach_link_;
  attached_object.object.id = object_id;
  attached_object.object.operation = moveit_msgs::msg::CollisionObject::ADD;
  attached_object.touch_links = touch_links_;

  moveit_msgs::msg::PlanningScene diff;
  diff.is_diff = true;
  diff.robot_state.is_diff = true;
  diff.robot_state.attached_collision_objects.push_back(attached_object);
  return apply_scene_diff(diff);
}

bool GripperActionServer::detach_object(const std::string & object_id)
{
  // TODO(silanus23): this function is intentionally not called yet.
  // It will be wired in run_job() once the planning scene teardown
  // and re-grasp workflows are defined.
  moveit_msgs::msg::AttachedCollisionObject detach_object;
  detach_object.object.id = object_id;
  detach_object.object.operation = moveit_msgs::msg::CollisionObject::REMOVE;

  moveit_msgs::msg::PlanningScene diff;
  diff.is_diff = true;
  diff.robot_state.is_diff = true;
  diff.robot_state.attached_collision_objects.push_back(detach_object);
  return apply_scene_diff(diff);
}

// TODO(silanus23): Make this for whole object not a link
bool GripperActionServer::allow_collision_for_placement(const std::string & target_id)
{
  const std::string & base_link_id = base_link_id_;

  auto get_request = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
  get_request->components.components =
    moveit_msgs::msg::PlanningSceneComponents::ALLOWED_COLLISION_MATRIX;

  if (!get_planning_scene_client_->wait_for_service(service_timeout_)) {
    RCLCPP_ERROR(logger_, "Get Planning Scene service not available");
    return false;
  }

  auto get_future = get_planning_scene_client_->async_send_request(get_request);
  if (get_future.wait_for(service_timeout_) != std::future_status::ready) {
    RCLCPP_ERROR(logger_, "Timeout getting planning scene");
    get_planning_scene_client_->remove_pending_request(get_future);
    return false;
  }

  auto current_scene = get_future.get();
  auto & acm = current_scene->scene.allowed_collision_matrix;

  auto find_or_add = [&acm](const std::string & name) -> size_t {
      auto it = std::find(acm.entry_names.begin(), acm.entry_names.end(), name);
      if (it != acm.entry_names.end()) {
        return std::distance(acm.entry_names.begin(), it);
      }
      acm.entry_names.push_back(name);
      acm.entry_values.resize(acm.entry_names.size());
      for (auto & row : acm.entry_values) {
        row.enabled.resize(acm.entry_names.size(), false);
      }
      return acm.entry_names.size() - 1;
    };
  const size_t target_idx = find_or_add(target_id);
  const size_t base_idx = find_or_add(base_link_id);
  acm.entry_values[target_idx].enabled[base_idx] = true;
  acm.entry_values[base_idx].enabled[target_idx] = true;

  moveit_msgs::msg::PlanningScene diff;
  diff.is_diff = true;
  diff.allowed_collision_matrix = acm;
  return apply_scene_diff(diff);
}

bool GripperActionServer::apply_scene_diff(const moveit_msgs::msg::PlanningScene & diff)
{
  if (!planning_scene_client_->wait_for_service(service_timeout_)) {
    RCLCPP_ERROR(logger_, "Apply Planning Scene service not available");
    return false;
  }

  auto request = std::make_shared<moveit_msgs::srv::ApplyPlanningScene::Request>();
  request->scene = diff;
  auto future = planning_scene_client_->async_send_request(request);
  if (future.wait_for(service_timeout_) != std::future_status::ready) {
    planning_scene_client_->remove_pending_request(future);
    RCLCPP_ERROR(logger_, "Timeout applying planning scene");
    return false;
  }
  return future.get()->success;
}

}  // namespace application
}  // namespace hold_and_weld
