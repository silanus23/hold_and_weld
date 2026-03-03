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

#ifndef HOLD_AND_WELD_APPLICATION__KINEMATICS__KINEMATICS_SOLVER_HPP_
#define HOLD_AND_WELD_APPLICATION__KINEMATICS__KINEMATICS_SOLVER_HPP_

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "hold_and_weld_application/kinematics/urdf_parser.hpp"

namespace hold_and_weld_application
{
namespace kinematics
{

/**
 * @brief Kinematics solver for 6-DOF manipulators
 *
 * Provides forward kinematics, Jacobian computation, and manipulability
 * analysis for a specific robot configuration parsed from URDF.
 */
class KinematicsSolver
{
public:
  /**
   * @brief Construct kinematics solver from parsed URDF chain
   * @param chain ParsedChain from URDFParser containing robot geometry
   * @throws std::runtime_error if chain is not 6-DOF
   */
  explicit KinematicsSolver(const ParsedChain & chain);

  /**
   * @brief Compute forward kinematics (joint angles → TCP pose)
   * @param q Joint angles [rad] (size must equal DOF)
   * @return TCP pose in base frame
   * @throws std::invalid_argument if q.size() != DOF
   */
  Eigen::Isometry3d compute_fk(const std::vector<double> & q) const;

  /**
   * @brief Compute geometric Jacobian (relates joint velocities to TCP velocity)
   * @param q Joint angles [rad]
   * @return 6×DOF Jacobian matrix [linear_velocity; angular_velocity]
   * @throws std::invalid_argument if q.size() != DOF
   */
  Eigen::MatrixXd compute_jacobian(const std::vector<double> & q) const;

  /**
   * @brief Compute Yoshikawa manipulability index
   * @param q Joint angles [rad]
   * @return Manipulability index (≥0, larger is better)
   * @throws std::invalid_argument if q.size() != DOF
   */
  double compute_yoshikawa_index(const std::vector<double> & q) const;

  /**
   * @brief Compute condition number of Jacobian
   * @param q Joint angles [rad]
   * @return Condition number (≥1, closer to 1 is better)
   * @throws std::invalid_argument if q.size() != DOF
   */
  double compute_condition_number(const std::vector<double> & q) const;

  /**
   * @brief Check if configuration is near singularity
   * @param q Joint angles [rad]
   * @param threshold Yoshikawa index threshold (default 0.01)
   * @return true if manipulability < threshold
   */
  bool is_near_singularity(
    const std::vector<double> & q,
    double threshold = 0.01) const;

  /**
   * @brief Check if joint angles are within limits (with margin)
   * @param q Joint angles [rad]
   * @param margin Safety margin from limits [rad] (default 0.05 = ~3°)
   * @return true if all joints within limits
   */
  bool check_joint_limits(
    const std::vector<double> & q,
    double margin = 0.05) const;

  /**
   * @brief Get degrees of freedom
   * @return Number of actuated joints (should be 6)
   */
  size_t dof() const {return dof_;}

  /**
   * @brief Get joint limits
   * @return Vector of (min, max) pairs for each joint
   */
  const std::vector<std::pair<double, double>> & joint_limits() const
  {
    return joint_limits_;
  }

private:
  // Robot structure (precomputed from URDF)
  size_t dof_;
  std::vector<Eigen::Isometry3d> joint_local_transforms_;
  std::vector<Eigen::Vector3d> joint_axes_;
  std::vector<bool> is_revolute_;
  std::vector<std::pair<double, double>> joint_limits_;
  Eigen::Isometry3d tool_transform_;
  std::string base_link_;
  std::string tip_link_;

  /**
   * @brief Validate joint angle vector size
   * @throws std::invalid_argument if size mismatch
   */
  void validate_joint_vector(const std::vector<double> & q) const;
};

}  // namespace kinematics
}  // namespace hold_and_weld_application

#endif  // HOLD_AND_WELD_APPLICATION__KINEMATICS__KINEMATICS_SOLVER_HPP_
