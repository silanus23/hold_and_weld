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

#ifndef HOLD_AND_WELD_APPLICATION__KINEMATICS__APPROACH_VALIDATOR_HPP_
#define HOLD_AND_WELD_APPLICATION__KINEMATICS__APPROACH_VALIDATOR_HPP_

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <memory>
#include <optional>
#include <vector>

#include "hold_and_weld_application/kinematics/kinematics_solver.hpp"
#include "hold_and_weld_application/kinematics/ceres_ik_solver.hpp"
#include <geometry_msgs/msg/pose.hpp>

// Forward declaration to avoid circular dependency
namespace hold_and_weld
{
struct WeldSeam;
}

namespace hold_and_weld_application
{
namespace kinematics
{

/**
 * @brief Validates approach configurations for welding seams
 *
 * This class performs "static walk" validation through a seam trajectory.
 * Starting from an OMPL-generated approach configuration, it incrementally
 * solves IK for each seam waypoint while checking for:
 * - Reachability (IK convergence)
 * - Singularities (manipulability index)
 * - Joint limit violations (handled by IK solver)
 *
 * The validation uses warm-starting where each solved configuration
 * becomes the seed for the next waypoint, ensuring configuration continuity.
 */
class ApproachValidator
{
public:
  using Vector6d = Eigen::Matrix<double, 6, 1>;

  /**
   * @brief Construct approach validator
   * @param kin_solver Forward kinematics solver for Jacobian computation
   * @param ik_solver Inverse kinematics solver for trajectory following
   * @param manipulatibility_threshold Minimum manipulability index (singularity threshold)
   */
  explicit ApproachValidator(
    std::shared_ptr<KinematicsSolver> kin_solver,
    std::shared_ptr<CeresIKSolver> ik_solver,
    double manipulatibility_threshold);

  ~ApproachValidator() = default;

  /**
   * @brief Set the weld seam to validate against.
   * @param seam Weld seam containing the target poses for the static-walk validation.
   */
  void set_weld_seam(const hold_and_weld::WeldSeam & seam) {seam_ = &seam;}

  /**
   * @brief Validate approach configuration through entire seam
   *
   * Performs "static walk" validation:
   * 1. Start at the approach configuration (anchor point)
   * 2. For each seam waypoint:
   *    - Solve IK using previous solution as seed
   *    - Check manipulability index
   *    - Update anchor for next step
   * 3. Return true if all waypoints are reachable without singularities
   *
   * @param q_approach Joint configuration to validate (OMPL result)
   * @return true if approach is valid for entire seam
   */
  bool is_approach_valid(const Vector6d & q_approach);

private:
  /**
   * @brief Compute manipulability index (Yoshikawa)
   * @param q Joint configuration
   * @return Manipulability index (sqrt(det(J*J^T)))
   */
  double compute_manipulatibility(const Vector6d & q);

  std::shared_ptr<KinematicsSolver> kinematics_solver_;
  std::shared_ptr<CeresIKSolver> ceres_ik_solver_;

  double manipulatibility_threshold_;
  const hold_and_weld::WeldSeam * seam_;
};

}  // namespace kinematics
}  // namespace hold_and_weld_application

#endif  // HOLD_AND_WELD_APPLICATION__KINEMATICS__APPROACH_VALIDATOR_HPP_
