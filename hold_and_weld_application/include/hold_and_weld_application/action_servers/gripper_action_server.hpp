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

#ifndef HOLD_AND_WELD_APPLICATION__ACTION_SERVERS__GRIPPER_ACTION_SERVER_HPP_
#define HOLD_AND_WELD_APPLICATION__ACTION_SERVERS__GRIPPER_ACTION_SERVER_HPP_

#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <controller_manager_msgs/srv/list_controllers.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <lifecycle_msgs/msg/transition.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/srv/apply_planning_scene.hpp>
#include <moveit_msgs/srv/get_cartesian_path.hpp>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include "hold_and_weld_application/action/trigger_gripper.hpp"

namespace hold_and_weld
{
namespace application
{

/**
 * @brief One pick-and-place job from the positions YAML.
 *
 * Poses are end-effector goals for the arm group, in MoveIt's planning frame.
 */
struct GripperJob
{
  std::string target_id;
  geometry_msgs::msg::Pose approach_pose;
  geometry_msgs::msg::Pose pick_pose;
  geometry_msgs::msg::Pose retract_pose;
  geometry_msgs::msg::Pose place_pose;
};

/**
 * @brief ROS2 lifecycle action server for controlling gripper operations with MoveIt integration.
 *
 * Runs the pick-and-place pipeline (open, approach, pick, close, attach, retract, place)
 * for one goal at a time; a goal arriving while another is queued or running is rejected.
 *
 * Threading model:
 * - Main executor thread: lifecycle transitions and the action server's goal, cancel and
 *   accepted callbacks. Only it creates or resets move_group_, the clients and the action
 *   server, and only while no job is running (transitions wait for the job first).
 * - Worker thread (worker_thread_func): runs execute_job() for one goal at a time; the
 *   only thread that plans or executes with move_group_ or calls the clients below.
 * - moveit_executor_ thread: spins the internal node, which owns MoveGroupInterface and
 *   every client the worker waits on (gripper controller action, list_controllers,
 *   planning scene services). The worker therefore never depends on the main executor,
 *   which may be blocked in a transition waiting for the worker.
 * - execution_mutex_ guards pending_goal_, execution_future_ and shutdown_requested_;
 *   move_group_mutex_ guards the move_group_ pointer against request_stop() on the
 *   pre-shutdown thread. The job, the apertures and base_link_id_ take no lock: they are
 *   written only in on_configure before the worker starts and in on_cleanup after it is
 *   joined.
 *   stop_requested_ is atomic and checked by the job before every step and retry.
 */
class GripperActionServer : public rclcpp_lifecycle::LifecycleNode {
public:
  using TriggerGripper = hold_and_weld_application::action::TriggerGripper;
  using GoalHandleTriggerGripper = rclcpp_action::ServerGoalHandle<TriggerGripper>;
  using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;

  /**
   * @brief Construct a new GripperActionServer object.
   * @param options ROS2 node options for configuration.
   */
  explicit GripperActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /**
   * @brief Destroy the GripperActionServer object, ensuring proper cleanup of worker thread.
   */
  ~GripperActionServer() override;

  // Lifecycle callbacks
  /**
   * @brief Validate parameters, wait for MoveIt and controller_manager, set up MoveIt and
   * load the job.
   *
   * A job that fails to load does not fail the transition; goals are rejected instead.
   *
   * @param state Current lifecycle state.
   * @return Transition callback result.
   */
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State & state);

  /**
   * @brief Start the auto-trigger timer if auto_trigger is set, a job is loaded and it has
   * not fired since configure.
   * @param state Current lifecycle state.
   * @return Transition callback result.
   */
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State & state);

  /**
   * @brief Abort a queued goal and stop a running one, waiting for it to end.
   * @param state Current lifecycle state.
   * @return Transition callback result.
   */
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State & state);

  /**
   * @brief Stop the worker and release MoveIt, the action server and the clients.
   * @param state Current lifecycle state.
   * @return Transition callback result.
   */
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State & state);

  /**
   * @brief Release everything via on_cleanup(); reachable from any primary state.
   * @param state Current lifecycle state.
   * @return Transition callback result.
   */
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_shutdown(const rclcpp_lifecycle::State & state);

  /**
   * @brief Stop the arm and the worker before the ROS context goes away.
   *
   * Meant to run from a context pre-shutdown callback (see gripper_server_main.cpp), when
   * the context is still valid, so stop() reaches the controller. Waits up to
   * shutdown_wait_sec for the running job to end, then joins the worker; if the
   * job does not end in time the worker is detached, as the process is exiting anyway.
   * Idempotent; the destructor calls it too.
   */
  void manual_shutdown();

private:
  static constexpr int kGripperResultMarginSec = 8;

  /**
   * @brief Reject a goal while the node is inactive, another is queued or running, or no
   * job is loaded.
   */
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const TriggerGripper::Goal> goal);

  /**
   * @brief Accept a cancel and stop the running job.
   */
  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleTriggerGripper> goal_handle);

  /**
   * @brief Queue an accepted goal for the worker thread.
   */
  void handle_accepted(const std::shared_ptr<GoalHandleTriggerGripper> goal_handle);

  /**
   * @brief Auto-trigger timer callback: fires once, sending an empty goal to our own
   * trigger_gripper action and logging its result.
   */
  void send_auto_trigger_goal();

  /**
   * @brief Persistent worker thread loop — waits for queued goals and executes them.
   */
  void worker_thread_func();

  /**
   * @brief Stop the worker thread and join it.
   *
   * Ends a still-queued goal, then joins. Callers must have made the running job end
   * first (request_stop()), or the join waits for it.
   */
  void shutdown_worker();

  /**
   * @brief Ask the running job to stop: sets stop_requested_ and calls move_group_->stop().
   */
  void request_stop();

  /**
   * @brief Block until the running job (if any) has returned, waiting at most @p timeout
   * (none: forever).
   * @return false if the job is still running when the timeout expires.
   */
  bool wait_for_running_job(std::optional<std::chrono::nanoseconds> timeout = std::nullopt);

  /**
   * @brief Execute the gripper job for a given goal (with action server feedback).
   */
  void execute_job(const std::shared_ptr<GoalHandleTriggerGripper> goal_handle);

  /**
   * @brief Publish feedback for the current goal.
   */
  void publish_feedback(
    const std::shared_ptr<GoalHandleTriggerGripper> goal_handle,
    const std::string & step,
    float percentage);

  /**
   * @brief Execute the pick-and-place sequence, reporting (step, percentage) through
   * @p feedback_callback and checking @p should_stop before every step and retry, so no
   * new motion starts after a cancel or transition.
   */
  bool run_job(
    const std::function<void(const std::string &, float)> & feedback_callback,
    const std::function<bool()> & should_stop);

  /**
   * @brief Load the job and gripper positions from the positions YAML.
   * @return false (with the reason logged) if the file or any pose in it is invalid;
   *         the previously loaded job is then cleared, never half-overwritten.
   */
  bool load_job_from_yaml(const std::string & yaml_path);

  /**
   * @brief Load base_link_id_ from the shared objects.yaml; keeps the default if absent.
   */
  void load_object_config();

  /**
   * @brief Resolve open_position_/close_position_ from the gripper joints' limits in
   * the robot model and the optional positions configured in the positions YAML.
   * @return false (with the reason logged) if a finger joint is missing from the model,
   *         a configured position is outside its limits, or open is not wider than close.
   */
  bool resolve_finger_apertures();

  /**
   * @brief Block until the gripper controller is active in the controller_manager.
   *
   * The gripper spawner can still be loading when a job starts, and a configured
   * but inactive controller reports goals succeeded without moving the fingers.
   *
   * @return false (with the reason logged) if it is not active within
   *         controller_timeout_sec, or the job was stopped.
   */
  bool wait_for_gripper_controller(const std::function<bool()> & should_stop);

  /**
   * @brief Command every finger joint to the same position and wait for the controller.
   * @param position Target finger position [m], within the finger joint limits.
   * @param should_stop Returns true once the job must stop; cancels the controller goal.
   * @return true if the controller reported success within finger_motion_sec plus
   *         kGripperResultMarginSec, false otherwise.
   */
  bool set_finger_aperture(double position, const std::function<bool()> & should_stop);

  /**
   * @brief Plan and execute an arm motion to a pose, retrying up to max_planning_retries.
   * @param pose Target end-effector pose, planning frame.
   * @param step_name Name of the motion step for logging/feedback.
   * @param should_stop Returns true once the job must stop; checked before each retry.
   */
  bool move_to_pose(
    const geometry_msgs::msg::Pose & pose, const std::string & step_name,
    const std::function<bool()> & should_stop);

  // Collision objects
  /**
   * @brief Attach an object to the gripper in the planning scene.
   */
  bool attach_object(const std::string & object_id);

  /**
   * @brief Detach an object from the gripper in the planning scene.
   */
  bool detach_object(const std::string & object_id);

  /**
   * @brief Send a planning-scene diff (is_diff must be set) through /apply_planning_scene.
   * @return true once MoveIt has applied it, false if the service is unavailable, times out
   *         or reports failure.
   */
  bool apply_scene_diff(const moveit_msgs::msg::PlanningScene & diff);

  /**
   * @brief Allow collision between the held object and base_link (workpiece) for placement.
   */
  bool allow_collision_for_placement(const std::string & target_id);

  rclcpp_action::Server<TriggerGripper>::SharedPtr action_server_;
  rclcpp_action::Client<TriggerGripper>::SharedPtr self_trigger_client_;

  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  std::mutex move_group_mutex_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr moveit_executor_;
  std::thread moveit_thread_;

  rclcpp::Client<moveit_msgs::srv::ApplyPlanningScene>::SharedPtr planning_scene_client_;
  rclcpp::Client<moveit_msgs::srv::GetPlanningScene>::SharedPtr get_planning_scene_client_;
  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr gripper_action_client_;
  rclcpp::Client<controller_manager_msgs::srv::ListControllers>::SharedPtr
    list_controllers_client_;
  std::string gripper_controller_name_;

  std::thread worker_thread_;
  std::mutex execution_mutex_;
  std::condition_variable worker_cv_;
  std::shared_ptr<GoalHandleTriggerGripper> pending_goal_;
  bool shutdown_requested_ = false;
  std::shared_future<void> execution_future_;
  std::atomic<bool> stop_requested_{false};
  std::atomic<bool> manual_shutdown_done_{false};

  GripperJob job_;
  bool job_loaded_ = false;
  std::string base_link_id_ = "base_link";
  std::optional<double> requested_open_position_;
  std::optional<double> requested_close_position_;
  double open_position_ = 0.0;
  double close_position_ = 0.0;
  std::vector<std::string> gripper_joint_names_ = {
    "robot1_left_finger_joint", "robot1_right_finger_joint"};
  std::vector<std::string> touch_links_ = {
    "robot1_tool0", "robot1_link_6", "robot1_flange",
    "robot1_gripper_base", "robot1_left_finger", "robot1_right_finger"};
  std::string attach_link_ = "robot1_link_6";
  int max_planning_retries_ = 3;
  std::chrono::nanoseconds service_timeout_{0};
  std::chrono::nanoseconds controller_timeout_{0};
  std::chrono::nanoseconds shutdown_wait_time_{0};
  std::chrono::nanoseconds finger_motion_time_{0};
  std::chrono::nanoseconds finger_settle_time_{0};
  std::chrono::nanoseconds motion_settle_time_{0};
  std::string arm_group_name_;
  std::string yaml_path_;
  bool auto_trigger_ = false;
  double auto_trigger_delay_sec_ = 3.0;
  bool auto_trigger_fired_ = false;

  rclcpp::TimerBase::SharedPtr auto_trigger_timer_;
  rclcpp::Logger logger_;
};

}  // namespace application
}  // namespace hold_and_weld

#endif  // HOLD_AND_WELD_APPLICATION__ACTION_SERVERS__GRIPPER_ACTION_SERVER_HPP_
