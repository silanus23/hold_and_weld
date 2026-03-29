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

#ifndef HOLD_AND_WELD_GRIPPER_SAMPLER__CORE__GRASP_HPP_
#define HOLD_AND_WELD_GRIPPER_SAMPLER__CORE__GRASP_HPP_

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <algorithm>
#include <vector>

namespace hold_and_weld_gripper_sampler
{

/**
 * @brief Represents a single grasp configuration.
 *
 * All geometric data is in the world frame (after STEP file transformation).
 * This struct is used throughout the sampling pipeline:
 * - Pre-filters create "marker" grasps (surface_id_2 = -1) to encode allowed surfaces
 * - Generator creates real grasps with both contact points
 * - Post-filters validate and score real grasps
 */
struct Grasp
{
  /**
   * @brief TCP (Tool Center Point) position in world frame [m]
   *
   * This is the target pose for the gripper during execution.
   * Positioned at the midpoint between the two contact points.
   */
  Eigen::Vector3d tcp_position;

  /**
   * @brief TCP orientation in world frame (quaternion)
   *
   * Defines the gripper's approach direction and finger alignment.
   * Convention: Z-axis points along approach direction (toward object)
   */
  Eigen::Quaterniond tcp_orientation;

  /**
   * @brief Required gripper opening distance [m]
   *
   * Distance between finger contact points.
   * Must be within gripper's min/max opening range.
   */
  double gripper_opening;

  /**
   * @brief First contact point on object surface [m, world frame]
   *
   * Location where first gripper finger touches the object.
   */
  Eigen::Vector3d contact_point_1;

  /**
   * @brief Second contact point on object surface [m, world frame]
   *
   * Location where second gripper finger touches the object.
   */
  Eigen::Vector3d contact_point_2;

  /**
   * @brief Surface ID for first contact point (0-indexed)
   *
   * References Topology::Surface index.
   * Special value: -1 indicates this is a pre-filter marker grasp.
   */
  int surface_id_1;

  /**
   * @brief Surface ID for second contact point (0-indexed)
   *
   * References Topology::Surface index.
   * Special value: -1 indicates this is a pre-filter marker grasp
   * (only surface_id_1 is valid in that case).
   */
  int surface_id_2;

  /**
   * @brief Grasp quality score [0.0 - 1.0]
   *
   * Higher is better. Computed by quality filters based on:
   * - Antipodality (normal alignment)
   * - Distance from edges
   * - Gripper opening (prefer mid-range)
   * - Orientation alignment with constraints
   *
   * Default: 0.0 (unscored)
   */
  double quality_score;

  /**
   * @brief Default constructor - initializes to safe values
   */
  Grasp()
  : tcp_position(Eigen::Vector3d::Zero()),
    tcp_orientation(Eigen::Quaterniond::Identity()),
    gripper_opening(0.0),
    contact_point_1(Eigen::Vector3d::Zero()),
    contact_point_2(Eigen::Vector3d::Zero()),
    surface_id_1(-1),
    surface_id_2(-1),
    quality_score(0.0)
  {}

  /**
   * @brief Factory method to create Grasp from pre-converted values
   *
   * Use this with occt_utils conversion functions:
   * - geometry::to_eigen(gp_Pnt) for contact points
   * - geometry::extract_translation(gp_Trsf) for TCP position
   * - geometry::extract_quaternion(gp_Trsf) for TCP orientation
   *
   * @param tcp_pos TCP position in world frame
   * @param tcp_orient TCP orientation as quaternion
   * @param opening Gripper opening distance
   * @param contact_1 First contact point
   * @param contact_2 Second contact point
   * @param surf_id_1 Surface ID for first contact
   * @param surf_id_2 Surface ID for second contact
   * @param quality Quality score [0.0 - 1.0]
   * @return Constructed Grasp object
   */
  static Grasp create(
    const Eigen::Vector3d & tcp_pos,
    const Eigen::Quaterniond & tcp_orient,
    double opening,
    const Eigen::Vector3d & contact_1,
    const Eigen::Vector3d & contact_2,
    int surf_id_1,
    int surf_id_2,
    double quality)
  {
    Grasp g;
    g.tcp_position = tcp_pos;
    g.tcp_orientation = tcp_orient;
    g.gripper_opening = opening;
    g.contact_point_1 = contact_1;
    g.contact_point_2 = contact_2;
    g.surface_id_1 = surf_id_1;
    g.surface_id_2 = surf_id_2;
    g.quality_score = quality;
    return g;
  }
};

namespace angle_finding
{
struct GraspCandidate;
}  // namespace angle_finding

/**
 * @brief Convert GraspCandidate (OCCT types) to Grasp (Eigen types)
 *
 * This function is declared here but implemented in grasp_orientation_finder.cpp
 * to avoid OCCT header dependencies in this file.
 *
 * @param candidate GraspCandidate from orientation finding
 * @return Grasp with Eigen types ready for downstream use
 */
Grasp to_grasp(const angle_finding::GraspCandidate & candidate);

/**
 * @brief Sort grasps by quality score descending (best first)
 *
 * @param grasps Vector of grasps to sort in place
 */
inline void sort_by_quality(std::vector<Grasp> & grasps)
{
  std::sort(
    grasps.begin(), grasps.end(),
    [](const Grasp & a, const Grasp & b) {
      return a.quality_score > b.quality_score;
    });
}

}  // namespace hold_and_weld_gripper_sampler

#endif  // HOLD_AND_WELD_GRIPPER_SAMPLER__CORE__GRASP_HPP_
