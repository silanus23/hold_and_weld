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

#ifndef HOLD_AND_WELD_APPLICATION__KINEMATICS__CERES_IK_SOLVER_HPP_
#define HOLD_AND_WELD_APPLICATION__KINEMATICS__CERES_IK_SOLVER_HPP_

#include <ceres/ceres.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <cmath>
#include <memory>
#include <vector>

#include "hold_and_weld_application/kinematics/kinematics_solver.hpp"

namespace hold_and_weld
{
namespace kinematics
{

/**
 * @brief Ceres cost functor for IK optimization
 *
 * Computes residuals for inverse kinematics optimization:
 * - Residuals 0-2: Position error (x, y, z)
 * - Residuals 3-5: Weighted orientation error (axis-angle representation)
 * - Residuals 6-11: Seed penalty to prevent configuration flips
 *
 * The seed penalty acts as a "restoring force" keeping the solution
 * near the initial guess, which is critical for trajectory continuity.
 */
class IKCostFunctor
{
public:
  /**
   * @brief Construct cost functor
   * @param solver Forward kinematics solver
   * @param target_pose Desired TCP pose in base frame
   * @param rotation_weight Weight for orientation error vs position error
   * @param seed_weight Weight for seed penalty (prevents configuration jumps)
   */
  IKCostFunctor(
    const KinematicsSolver * solver,
    const Eigen::Isometry3d & target_pose,
    double rotation_weight,
    double seed_weight);

  /**
   * @brief Set target seed for configuration penalty
   * @param seed Initial joint configuration to stay close to
   */
  void set_target_seed(const std::vector<double> & seed);

  /**
   * @brief Compute residuals for Ceres optimizer
   * @param q_array Joint angles (6-DOF)
   * @param residuals Output residuals (12 total: 6 pose + 6 seed penalty)
   * @return true if evaluation succeeded
   */
  template<typename T>
  bool operator()(const T * const q_array, T * residuals) const;

private:
  const KinematicsSolver * solver_;
  Eigen::Isometry3d target_pose_;
  double rotation_weight_;
  double seed_weight_;
  std::vector<double> q_target_seed_;
};

/**
 * @brief Fast inverse kinematics solver using Ceres optimization
 *
 * Uses numerical optimization to find joint angles that achieve a target pose.
 * Features:
 * - Warm-starting for fast convergence (1-5ms with good seed)
 * - Hard joint limits enforced via parameter bounds
 * - Seed penalty for configuration continuity (prevents flips)
 * - Configurable position and orientation tolerances
 *
 * Designed for validation pipelines where IK is called sequentially
 * along a trajectory, with each solution seeding the next.
 */
class CeresIKSolver
{
public:
  using Vector6d = Eigen::Matrix<double, 6, 1>;

  /**
   * @brief Construct IK solver
   * @param fk_solver Forward kinematics solver (must outlive this object)
   * @param rotation_weight Weight for orientation error vs position error (default 1.0)
   */
  explicit CeresIKSolver(
    std::shared_ptr<KinematicsSolver> fk_solver,
    double rotation_weight = 1.0);

  /**
   * @brief Solve inverse kinematics with warm start
   *
   * Solves IK using numerical optimization with:
   * - Position + orientation error minimization
   * - Seed penalty to maintain configuration continuity
   * - Hard joint limits via parameter bounds
   *
   * @param target_pose Desired TCP pose in base frame
   * @param q_seed Initial guess for joint angles (warm start, Eigen vector)
   * @param q_solution Output: joint angles achieving target pose (Eigen vector)
   * @param position_tolerance Position convergence tolerance [m] (default 0.1mm)
   * @param orientation_tolerance Orientation convergence tolerance [rad] (default ~0.057°)
   * @return true if IK converged and meets tolerances
   */
  bool solve(
    const Eigen::Isometry3d & target_pose,
    const Vector6d & q_seed,
    Vector6d & q_solution,
    double position_tolerance = 1e-4,
    double orientation_tolerance = 1e-3);

  /**
   * @brief Set maximum number of optimization iterations
   * @param max_iterations Maximum iterations (default 100)
   */
  void set_max_iterations(int max_iterations) {max_iterations_ = max_iterations;}

  /**
   * @brief Set rotation weight (balance position vs orientation errors)
   * @param weight Weight multiplier for rotation error (default 1.0)
   */
  void set_rotation_weight(double weight) {rotation_weight_ = weight;}

private:
  std::shared_ptr<KinematicsSolver> fk_solver_;
  double rotation_weight_;
  int max_iterations_;
};

}  // namespace kinematics
}  // namespace hold_and_weld

#endif  // HOLD_AND_WELD_APPLICATION__KINEMATICS__CERES_IK_SOLVER_HPP_
