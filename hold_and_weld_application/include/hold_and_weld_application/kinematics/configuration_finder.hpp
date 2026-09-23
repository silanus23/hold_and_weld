// Copyright 2026 Berkan Tali
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

#ifndef HOLD_AND_WELD_APPLICATION__KINEMATICS__CONFIGURATION_FINDER_HPP_
#define HOLD_AND_WELD_APPLICATION__KINEMATICS__CONFIGURATION_FINDER_HPP_

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <memory>
#include <string>
#include <vector>

#include "hold_and_weld_application/kinematics/ceres_ik_solver.hpp"
#include "hold_and_weld_application/kinematics/kinematics_solver.hpp"

namespace hold_and_weld
{
namespace kinematics
{

/**
 * @brief Tunables for ConfigurationFinder. Every field is exposed in welding.yaml
 * (manipulability_threshold via the top-level key). All must be finite; the
 * constructor rejects non-positive steps/tolerances and negative weights.
 */
struct ConfigurationFinderParams
{
  double path_step = 0.005;
  double path_step_rot = 0.02;
  double max_joint_step = 0.3;
  double manipulability_threshold = 1e-6;
  double branch_seed_weight = 0.0;
  double walk_seed_weight = 0.01;
  double tol_pos = 1e-4;
  double tol_rot = 1e-3;
  double w_limit_margin = 3.0;
  double w_manipulability = 2.0;
  double w_home = 1.0;
  double dedupe_epsilon = 1e-3;
};

/**
 * @brief Why a candidate start configuration cannot weld the seam.
 */
enum class WalkFailure
{
  kNone,
  kIkFailed,
  kJointLimit,
  kLowManipulability,
  kJointStep,
};

/**
 * @brief Human-readable name of a WalkFailure.
 * @param failure Failure to name
 * @return Short description for logs
 */
std::string to_string(WalkFailure failure);

/**
 * @brief Result of simulating the Pilz path from one start configuration.
 */
struct WalkResult
{
  bool feasible = false;
  WalkFailure failure = WalkFailure::kNone;
  size_t fail_index = 0;
  double min_limit_margin = 0.0;
  double min_manipulability = 0.0;
  double max_joint_step = 0.0;
};

/**
 * @brief One ranked start configuration for the Pilz weld.
 */
struct Candidate
{
  using Vector6d = Eigen::Matrix<double, 6, 1>;

  size_t index = 0;
  Vector6d q_start = Vector6d::Zero();
  WalkResult walk;
  double home_distance = 0.0;
  double score = 0.0;
};

/**
 * @brief Standoff pose used for approach and retract: ref moved approach_offset along
 * its local +Z, same orientation. WelderActionServer::move_to_seam_boundary uses it too.
 * @param ref Seam boundary pose
 * @param approach_offset Standoff distance [m]
 * @return The standoff pose, in ref's frame
 */
Eigen::Isometry3d standoff_pose(const Eigen::Isometry3d & ref, double approach_offset);

/**
 * @brief Chooses the joint configuration Pilz starts a weld from. Pilz LIN/CIRC is
 * deterministic given its start state, so it inherits the start configuration's branch
 * and J4/J6 2*pi copy. This class enumerates the IK solutions at the approach standoff,
 * simulates the path Pilz will execute from each, and ranks the ones that survive; the
 * same seam always yields the same ranking. All poses are in the robot base frame.
 */
class ConfigurationFinder
{
public:
  using Vector6d = Eigen::Matrix<double, 6, 1>;

  /**
   * @brief Construct a configuration finder
   * @param kin_solver FK / Jacobian / joint limits of the welder chain
   * @param ik_solver IK solver on the same chain
   * @param params Tunables (see ConfigurationFinderParams)
   */
  ConfigurationFinder(
    std::shared_ptr<KinematicsSolver> kin_solver,
    std::shared_ptr<CeresIKSolver> ik_solver,
    const ConfigurationFinderParams & params);

  /**
   * @brief Rank start configurations for a seam.
   * @param seam Seam poses in base frame (non-empty)
   * @param segment_type "line", "arc", or anything else for the dense-waypoint fallback
   * @param approach_offset Standoff distance used by the approach move [m]
   * @param q_home Home configuration; seeds branch generation and the proximity term
   * @return Feasible candidates by descending score, then infeasible ones by index
   */
  std::vector<Candidate> find(
    const std::vector<Eigen::Isometry3d> & seam,
    const std::string & segment_type,
    double approach_offset,
    const Vector6d & q_home) const;

  /**
   * @brief Every distinct in-limit IK solution at a pose that we can reach from the
   * home-derived seeds, expanded with wrist flips and J4/J6 2*pi copies.
   * @param pose Target pose in base frame
   * @param q_home Home configuration; seeds the shoulder/elbow multi-start
   * @return Deduplicated solutions in deterministic generation order
   */
  std::vector<Vector6d> generate_candidates(
    const Eigen::Isometry3d & pose,
    const Vector6d & q_home) const;

  /**
   * @brief Exact alternatives of q with the same FK: wrist flip (q4+pi, -q5, q6+pi)
   * and every in-limit q4 + 2*pi*k, q6 + 2*pi*k. Includes q itself if in limits.
   * @param q A solution of the spherical-wrist chain
   * @return All in-limit alternatives, q's family first, then the flipped family. Grows
   *         with the J4/J6 ranges: one copy per 2*pi of range.
   */
  std::vector<Vector6d> expand_solution(const Vector6d & q) const;

  /**
   * @brief Densely sampled Cartesian path Pilz will follow: LIN plunge from the approach
   * pose to seam[0], then LIN (line), CIRC (arc) or the seam poses (anything else).
   * An "arc" with fewer than 3 poses, or with colinear start/interim/end, has no
   * circle to follow and is sampled like the dense-waypoint fallback / LIN.
   * @param seam Seam poses in base frame
   * @param segment_type "line", "arc", or anything else for the dense-waypoint fallback
   * @param approach_pose Standoff pose the plunge starts from (path[0])
   * @return Samples no further apart than path_step / path_step_rot
   */
  std::vector<Eigen::Isometry3d> pilz_path(
    const std::vector<Eigen::Isometry3d> & seam,
    const std::string & segment_type,
    const Eigen::Isometry3d & approach_pose) const;

  /**
   * @brief Walk path[1..] from q_start (which must sit at path[0]) with warm-started IK.
   * @param q_start Start configuration; a NaN or out-of-limit q_start fails as kJointLimit
   * @param path Samples from pilz_path() (non-empty)
   * @return Feasibility, first failure, and the scoring statistics (the statistics stay
   *         zero when q_start itself is out of limits)
   */
  WalkResult walk(const Vector6d & q_start, const std::vector<Eigen::Isometry3d> & path) const;

  /**
   * @brief Tunables in use
   * @return The parameters this finder was built with
   */
  const ConfigurationFinderParams & params() const {return params_;}

private:
  /** @brief True if every joint of q is inside its URDF limits. */
  bool within_limits(const Vector6d & q) const;
  /** @brief Smallest distance of any joint of q to its nearest limit [rad]. */
  double limit_margin(const Vector6d & q) const;
  /** @brief Seed on the other elbow branch: elbow mirrored across shoulder -> wrist. */
  Vector6d elbow_mirror_seed(const Vector6d & q) const;

  std::shared_ptr<KinematicsSolver> kin_solver_;
  std::shared_ptr<CeresIKSolver> ik_solver_;
  ConfigurationFinderParams params_;
};

}  // namespace kinematics
}  // namespace hold_and_weld

#endif  // HOLD_AND_WELD_APPLICATION__KINEMATICS__CONFIGURATION_FINDER_HPP_
