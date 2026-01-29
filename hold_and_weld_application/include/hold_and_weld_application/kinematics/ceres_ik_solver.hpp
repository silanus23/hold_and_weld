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
 * @brief Fast inverse kinematics solver using Ceres optimization
 *
 * Uses numerical optimization to find joint angles that achieve a target pose.
 * Designed for validation pipelines with warm-starting for fast convergence.
 * Target solve time: 1-5ms with good initial guess.
 */
class CeresIKSolver
{
public:
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
   * @param target_pose Desired TCP pose in base frame
   * @param q_seed Initial guess for joint angles (warm start)
   * @param q_solution Output: joint angles achieving target pose
   * @param position_tolerance Position convergence tolerance [m] (default 1mm)
   * @param orientation_tolerance Orientation convergence tolerance [rad] (default ~0.57°)
   * @return true if IK converged successfully
   */
  bool solve(
    const Eigen::Isometry3d & target_pose,
    const std::vector<double> & q_seed,
    std::vector<double> & q_solution,
    double position_tolerance = 1e-3,
    double orientation_tolerance = 1e-2);

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

/**
 * @brief Ceres cost functor for IK optimization
 *
 * Computes pose error residuals: [position_error, weighted_rotation_error]
 */
class IKCostFunctor
{
public:
  IKCostFunctor(
    const KinematicsSolver * solver,
    const Eigen::Isometry3d & target_pose,
    double rotation_weight);

  template<typename T>
  bool operator()(const T * const q_array, T * residuals) const;

private:
  const KinematicsSolver * solver_;
  Eigen::Isometry3d target_pose_;
  double rotation_weight_;
};

}  // namespace kinematics
}  // namespace hold_and_weld

#endif  // HOLD_AND_WELD_APPLICATION__KINEMATICS__CERES_IK_SOLVER_HPP_
