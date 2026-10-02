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

#ifndef HOLD_AND_WELD_APPLICATION__ACTION_SERVERS__WELDER_ACTION_SERVER_HPP_
#define HOLD_AND_WELD_APPLICATION__ACTION_SERVERS__WELDER_ACTION_SERVER_HPP_

#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <functional>
#include <future>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <controller_manager_msgs/srv/list_controllers.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <lifecycle_msgs/msg/transition.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/srv/get_cartesian_path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include "hold_and_weld_application/action/trigger_welder.hpp"
#include "hold_and_weld_application/action_servers/weld_seam_loader.hpp"
#include "hold_and_weld_application/kinematics/approach_validator.hpp"
#include "hold_and_weld_application/kinematics/ceres_ik_solver.hpp"
#include "hold_and_weld_application/kinematics/configuration_finder.hpp"
#include "hold_and_weld_application/kinematics/kinematics_solver.hpp"
#include "hold_and_weld_application/kinematics/urdf_parser.hpp"

#include "hold_and_weld_application/utils.hpp"

namespace hold_and_weld
{
namespace application
{

/**
 * @brief Welder settings from welding.yaml; see validate() for the accepted ranges.
 */
struct WelderConfig
{
  std::string welder_group_name = "robot2_welder_arm";
  double approach_offset_z = 0.1;
  double cartesian_path_threshold = 0.95;
  double cartesian_step_size = 0.01;
  double velocity_scaling = 0.3;
  int max_ompl_planning_attempts = 3;
  double goal_position_tolerance = 0.001;
  double goal_orientation_tolerance = 0.01;
  int max_cartesian_retries = 2;
  bool use_pilz = true;
  bool use_approach_validator = true;
  std::string json_file;
  double manipulability_threshold = 1e-6;
  bool use_configuration_finder = true;
  std::map<std::string, double> home_configuration;
  int finder_max_ompl_candidates = 5;
  hold_and_weld::kinematics::ConfigurationFinderParams finder;
  hold_and_weld::kinematics::ApproachValidatorParams approach_validator;

  /**
   * @brief Check every field this struct owns (the finder and validator sub-structs are
   * checked by their constructors).
   * @return Empty if valid, otherwise a description of the first bad field.
   */
  std::string validate() const;
};

/**
 * @brief ROS2 lifecycle action server for controlling welding operations with MoveIt integration.
 *
 * Handles welding seam execution: approach/retract motions, Pilz LIN/CIRC weld motions,
 * and feedback during the welding process. One goal at a time: a goal arriving while
 * another is queued or running is rejected.
 *
 * Threading model:
 * - Main executor thread: lifecycle transitions and the action server's goal, cancel and
 *   accepted callbacks. Only it creates or resets move_group_, the solvers and the action
 *   server, and only while no job is running (transitions wait for the job first).
 * - Worker thread (worker_thread_func): runs execute_weld() for one goal at a time and is
 *   the only thread that plans or executes with move_group_ and uses the solvers.
 * - moveit_executor_ thread: spins the internal node, mainly feeding /joint_states to
 *   MoveGroupInterface's current-state monitor (plan/execute replies use its own thread).
 * - execution_mutex_ guards pending_goal_, execution_future_ and shutdown_requested_;
 *   move_group_mutex_ guards the move_group_ pointer against request_stop() on the
 *   pre-shutdown thread (the worker reads it unlocked: it is only swapped while no job
 *   runs). stop_requested_ is atomic; handle_cancel and the transitions set it together with
 *   move_group_->stop(), and the worker checks it before every new motion.
 */
class WelderActionServer : public rclcpp_lifecycle::LifecycleNode {
public:
  using TriggerWelder = hold_and_weld_application::action::TriggerWelder;
  using GoalHandleTriggerWelder = rclcpp_action::ServerGoalHandle<TriggerWelder>;

  /**
   * @brief Construct a new WelderActionServer object.
   * @param options ROS2 node options for configuration.
   */
  explicit WelderActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /**
   * @brief Destroy the WelderActionServer object, ensuring proper cleanup of worker thread.
   */
  ~WelderActionServer() override;

  // Lifecycle callbacks
  /**
   * @brief Validate parameters and welding.yaml, wait for MoveIt and controller_manager, and
   * set up MoveIt plus, if the approach needs them, the kinematics solvers.
   *
   * The weld path itself is loaded per goal.
   *
   * @param state Current lifecycle state.
   * @return Transition callback result.
   */
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State & state);

  /**
   * @brief Start the auto-trigger timer if auto_trigger is set and it has not fired since
   * configure.
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
   * @brief Stop the worker and release MoveIt, the action server and the kinematics solvers.
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
   * Meant to run from a context pre-shutdown callback (see welder_server_main.cpp), when
   * the context is still valid, so stop() reaches the controller. Waits up to
   * shutdown_wait_sec for the running job to end, then joins the worker; if the job does
   * not end in time the worker is detached, as the process is exiting anyway.
   * Idempotent; the destructor calls it too.
   */
  void manual_shutdown();

private:
  /**
   * @brief Reject a goal while the node is inactive or another is queued or running.
   */
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const TriggerWelder::Goal> goal);

  /**
   * @brief Accept a cancel and stop the running job.
   */
  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleTriggerWelder> goal_handle);

  /**
   * @brief Queue an accepted goal for the worker thread.
   */
  void handle_accepted(const std::shared_ptr<GoalHandleTriggerWelder> goal_handle);

  /**
   * @brief Load and validate welding.yaml.
   * @param yaml_path Path to welding.yaml.
   * @param config Output; written only when the whole file is valid.
   * @return false (with the reason logged) if the file is missing, unparsable or invalid.
   */
  bool load_config_from_yaml(const std::string & yaml_path, WelderConfig & config) const;

  /**
   * @brief Find the latest JSON file containing weld seam data.
   * @return Path to the latest JSON file, or empty string if not found.
   */
  std::string find_latest_json() const;

  /**
   * @brief Load weld seams from a JSON file.
   * @return The usable seams plus the skipped/partial ones, best effort (see
   *         parse_weld_seams()); no seams, with the reason logged, if the file cannot be
   *         read or is not a weld JSON document.
   */
  WeldJob load_seams_from_json(const std::string & filepath) const;

  /**
   * @brief Worker thread function for asynchronous goal execution.
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
   * @brief Auto-trigger timer callback: fires once, sending an empty goal to our own
   * trigger_welder action and logging its result.
   */
  void send_auto_trigger_goal();

  /**
   * @brief Block until the running job (if any) has returned, waiting at most @p timeout
   * (none: forever).
   * @return false if the job is still running when the timeout expires.
   */
  bool wait_for_running_job(std::optional<std::chrono::nanoseconds> timeout = std::nullopt);

  /**
   * @brief Execute welding operation for a given goal.
   */
  void execute_weld(const std::shared_ptr<GoalHandleTriggerWelder> goal_handle);

  /**
   * @brief Move the welder arm to kinematics::standoff_pose(ref_pose, approach_offset_z).
   *
   * Approach with use_configuration_finder: OMPL gets the finder's chosen start
   * configuration as a joint goal. Approach otherwise: OMPL pose goal, optionally
   * checked by the ApproachValidator. Retract: always a plain OMPL pose goal.
   * Only planning is retried; an execution failure ends the move.
   *
   * @param seam The weld seam, world frame.
   * @param ref_pose Boundary pose to offset from (poses.front() or poses.back()), world frame.
   * @param is_approach true before the weld, false for the retract after it.
   * @param should_stop Returns true once the job must stop (cancel or transition).
   * @return true if the motion was planned, validated and executed successfully.
   */
  bool move_to_seam_boundary(
    const WeldSeam & seam, const geometry_msgs::msg::Pose & ref_pose, bool is_approach,
    const std::function<bool()> & should_stop);

  /**
   * @brief Approach a seam by choosing the Pilz start configuration first.
   *
   * Ranks start configurations with the ConfigurationFinder, then tries OMPL joint
   * goals in rank order and executes the first plan found.
   *
   * @param seam The weld seam, in the robot2_base_link frame.
   * @param should_stop Returns true once the job must stop (cancel or transition).
   * @return true if a ranked configuration was reached.
   */
  bool approach_via_configuration_finder(
    const WeldSeam & seam, const std::function<bool()> & should_stop);

  /**
   * @brief Execute the weld motion along a seam, dispatching on segment type.
   *
   * "line" and "arc" seams are driven through the Pilz industrial motion
   * planner (LIN / CIRC respectively) for deterministic constant-velocity /
   * true-circular motion, after a LIN plunge from the approach standoff onto
   * the seam start. CIRC uses the middle seam pose as its interim point.
   * "ptp"/unknown/legacy seams, and every seam when use_pilz is off, follow the
   * dense seam poses through `computeCartesianPath()` instead.
   *
   * @param seam The weld seam to execute (poses and segment_type).
   * @param goal_handle Handle to the goal for sending feedback and results.
   * @param feedback Feedback message to update with progress.
   * @param points_before_seam Waypoints of the seams already processed, for progress.
   * @param total_waypoints Total number of waypoints in the complete path.
   * @param should_stop Returns true once the job must stop (cancel or transition).
   * @param torch_left_standoff Output; true once any motion was executed (or started),
   *        so a failure leaves the torch on or near the part rather than at the standoff.
   */
  bool execute_cartesian_path(
    const WeldSeam & seam,
    const std::shared_ptr<GoalHandleTriggerWelder> & goal_handle,
    std::shared_ptr<TriggerWelder::Feedback> & feedback,
    int32_t points_before_seam,
    int32_t total_waypoints,
    const std::function<bool()> & should_stop,
    bool & torch_left_standoff);

  /**
   * @brief Outcome of one planned-and-executed motion.
   */
  enum class MotionOutcome
  {
    kSucceeded,
    kPlanningFailed,
    kExecutionFailed,
  };

  /**
   * @brief Plan (retrying up to max_cartesian_retries) and execute one Pilz motion from
   * the current state to a pose.
   * @param planner_id Pilz planner ("LIN" or "CIRC").
   * @param target Goal pose for the end effector, planning frame.
   * @param path_constraints CIRC auxiliary point constraint, or nullptr for none.
   * @param seam_id Seam id, for log messages.
   * @param should_stop Returns true once the job must stop; checked before each attempt.
   * @return Whether planning failed, execution failed, or both succeeded.
   */
  MotionOutcome plan_and_execute_pilz(
    const std::string & planner_id,
    const geometry_msgs::msg::Pose & target,
    const moveit_msgs::msg::Constraints * path_constraints,
    const std::string & seam_id,
    const std::function<bool()> & should_stop);

  /**
   * @brief Plan (retrying up to max_cartesian_retries) and execute a computeCartesianPath()
   * motion from the current state through the waypoints.
   * @param waypoints End-effector poses to pass through, planning frame.
   * @param seam_id Seam id, for log messages.
   * @param should_stop Returns true once the job must stop; checked before each attempt.
   * @return Whether planning failed, execution failed, or both succeeded.
   */
  MotionOutcome plan_and_execute_cartesian(
    const std::vector<geometry_msgs::msg::Pose> & waypoints,
    const std::string & seam_id,
    const std::function<bool()> & should_stop);

  /**
   * @brief Back the torch off the part after a failed weld, so the next seam's approach does
   * not start with the torch on the workpiece.
   *
   * A straight line (Pilz LIN, or computeCartesianPath() when use_pilz is off) from the
   * current end-effector pose to its standoff (approach_offset_z along the tool Z axis).
   */
  bool retreat_from_part(const std::string & seam_id, const std::function<bool()> & should_stop);

  rclcpp_action::Server<TriggerWelder>::SharedPtr action_server_;

  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  std::mutex move_group_mutex_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr moveit_executor_;
  std::thread moveit_thread_;

  std::thread worker_thread_;
  std::mutex execution_mutex_;
  std::condition_variable worker_cv_;
  std::shared_ptr<GoalHandleTriggerWelder> pending_goal_;
  bool shutdown_requested_ = false;
  std::shared_future<void> execution_future_;
  std::atomic<bool> stop_requested_{false};
  std::atomic<bool> manual_shutdown_done_{false};

  std::shared_ptr<hold_and_weld::kinematics::CeresIKSolver> ceres_solver_;
  std::shared_ptr<hold_and_weld::kinematics::KinematicsSolver> kinematics_solver_;
  std::unique_ptr<hold_and_weld::kinematics::ApproachValidator> approach_validator_;
  std::unique_ptr<hold_and_weld::kinematics::ConfigurationFinder> configuration_finder_;
  hold_and_weld::kinematics::ConfigurationFinder::Vector6d q_home_ =
    hold_and_weld::kinematics::ConfigurationFinder::Vector6d::Zero();

  WelderConfig config_;
  rclcpp::Logger logger_;

  std::chrono::nanoseconds shutdown_wait_time_{0};
  bool auto_trigger_ = false;
  double auto_trigger_delay_sec_ = 3.0;
  bool auto_trigger_fired_ = false;
  std::string trajectory_directory_;
  bool auto_load_latest_ = true;

  rclcpp::TimerBase::SharedPtr auto_trigger_timer_;
  rclcpp_action::Client<TriggerWelder>::SharedPtr self_trigger_client_;
};

}  // namespace application
}  // namespace hold_and_weld

#endif  // HOLD_AND_WELD_APPLICATION__ACTION_SERVERS__WELDER_ACTION_SERVER_HPP_
