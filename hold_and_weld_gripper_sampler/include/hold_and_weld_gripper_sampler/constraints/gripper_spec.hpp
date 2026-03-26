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

#ifndef HOLD_AND_WELD_GRIPPER_SAMPLER__CONSTRAINTS__GRIPPER_SPEC_HPP_
#define HOLD_AND_WELD_GRIPPER_SAMPLER__CONSTRAINTS__GRIPPER_SPEC_HPP_

namespace hold_and_weld_gripper_sampler
{

/**
 * @brief Physical specifications of the parallel-jaw gripper.
 *
 * All dimensions are in meters, in the gripper's local frame.
 * These constraints are used by the generator and post-filters to:
 * - Validate gripper opening is within mechanical limits
 * - Check gripper base collision (Phase 2+)
 * - Compute grasp quality scores
 */
struct GripperSpec
{
  /**
   * @brief Length of gripper fingers [m]
   *
   * Distance from finger base to finger tip.
   * Used for collision checking and reach validation.
   */
  double finger_length;

  /**
   * @brief Width of gripper base (perpendicular to finger motion) [m]
   *
   * Dimension of the gripper body in the direction parallel to the jaw axis.
   * Used for collision checking with object surfaces.
   */
  double base_width;

  /**
   * @brief Height of gripper base (parallel to finger motion) [m]
   *
   * Dimension of the gripper body in the direction the fingers open/close.
   * Used for collision checking and clearance validation.
   */
  double base_height;

  /**
   * @brief Minimum gripper opening distance [m]
   *
   * Smallest distance between finger contact points.
   * Mechanical limit of the gripper mechanism.
   */
  double min_opening;

  /**
   * @brief Maximum gripper opening distance [m]
   *
   * Largest distance between finger contact points.
   * Mechanical limit of the gripper mechanism.
   */
  double max_opening;

  /**
   * @brief Default constructor - initializes to invalid values
   *
   * User must set all fields explicitly from YAML config.
   */
  GripperSpec()
  : finger_length(0.0),
    base_width(0.0),
    base_height(0.0),
    min_opening(0.0),
    max_opening(0.0)
  {}

  /**
   * @brief Validate that gripper spec has reasonable values
   *
   * @return true if all dimensions are positive and min < max
   */
  bool is_valid() const
  {
    return finger_length > 0.0 &&
           base_width > 0.0 &&
           base_height > 0.0 &&
           min_opening > 0.0 &&
           max_opening > min_opening;
  }

  /**
   * @brief Check if a gripper opening is within mechanical limits
   *
   * @param opening Desired opening distance [m]
   * @return true if min_opening <= opening <= max_opening
   */
  bool is_opening_valid(double opening) const
  {
    return opening >= min_opening && opening <= max_opening;
  }
};

}  // namespace hold_and_weld_gripper_sampler

#endif  // HOLD_AND_WELD_GRIPPER_SAMPLER__CONSTRAINTS__GRIPPER_SPEC_HPP_
