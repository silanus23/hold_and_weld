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

#include "hold_and_weld_application/utils.hpp"

namespace hold_and_weld
{
namespace kinematics
{

/**
 * @brief Tunables for ApproachValidator, read from the approach_validator section of
 * welding.yaml. Tolerances bound how far each IK solution's pose may be from its seam
 * point; defaults are the historical hard-coded values, loose for welding.
 */
struct ApproachValidatorParams
{
  double manipulability_threshold = 1e-6;
  double first_point_tol_pos = 0.20;
  double first_point_tol_rot = 0.10;
  double seam_tol_pos = 0.03;
  double seam_tol_rot = 0.02;
};

/**
 * @brief Validates approach configurations for welding seams by "static walking" a seam
 * trajectory from an OMPL-generated approach configuration, warm-starting IK from each
 * solved waypoint to the next and checking reachability and manipulability. Does not
 * check the joint step between waypoints (ConfigurationFinder does).
 */
class ApproachValidator
{
public:
  using Vector6d = Eigen::Matrix<double, 6, 1>;

  /**
   * @brief Construct approach validator
   * @param kin_solver Forward kinematics solver for Jacobian computation
   * @param ik_solver Inverse kinematics solver for trajectory following
   * @param params Tolerances and manipulability threshold (see ApproachValidatorParams)
   */
  ApproachValidator(
    std::shared_ptr<KinematicsSolver> kin_solver,
    std::shared_ptr<CeresIKSolver> ik_solver,
    const ApproachValidatorParams & params);

  ~ApproachValidator() = default;

  /**
   * @brief Set the weld seam to validate against.
   * @param seam Weld seam containing the target poses for the static-walk validation.
   */
  void set_weld_seam(const hold_and_weld::WeldSeam & seam) {seam_ = seam;}

  /**
   * @brief Validate approach configuration through entire seam by warm-started IK per
   * waypoint, checking manipulability at each step. Seam quaternions are normalised
   * before use.
   * @param q_approach Joint configuration to validate (OMPL result)
   * @return true if approach is valid for entire seam; false if no seam is set, the seam
   *         is empty, or a seam pose is non-finite or has a zero quaternion
   */
  bool is_approach_valid(const Vector6d & q_approach);

private:
  std::shared_ptr<KinematicsSolver> kinematics_solver_;
  std::shared_ptr<CeresIKSolver> ceres_ik_solver_;

  ApproachValidatorParams params_;
  std::optional<hold_and_weld::WeldSeam> seam_;
};

}  // namespace kinematics
}  // namespace hold_and_weld

#endif  // HOLD_AND_WELD_APPLICATION__KINEMATICS__APPROACH_VALIDATOR_HPP_
