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
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

namespace hold_and_weld_gripper_sampler
{

/**
 * @brief Represents a single grasp configuration in the world frame.
 *
 * Pre-filters use surface_id_2 = -1 as a marker for allowed-surface encoding.
 * Generator fills both contact points; post-filters validate and score them.
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
   * @brief TCP orientation in world frame (quaternion).
   *
   * Z-axis points along the approach direction (toward object).
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
   * @brief Grasp quality score [0.0 - 1.0]. Higher is better. Default: 0.0 (unscored).
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
   * @brief Factory method to create a Grasp from pre-converted Eigen values.
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

/**
 * @brief Diversity-aware reordering: interleave geometrically distant grasps.
 *
 * Uses greedy farthest-point selection so each successive grasp differs
 * maximally from all previously selected ones. Distance combines TCP position
 * (metres) and approach-direction angle (radians). The first element is always
 * the highest-quality grasp (seed); quality breaks ties elsewhere.
 *
 * @param grasps     Vector to reorder in place (quality-sorted before calling).
 * @param pos_weight Scale applied to TCP position distances [m].
 * @param ori_weight Scale applied to orientation distances [rad].
 */
inline void sort_by_diversity(
  std::vector<Grasp> & grasps,
  double pos_weight = 1.0,
  double ori_weight = 1.0)
{
  const std::size_t n = grasps.size();
  if (n < 2) {
    return;
  }

  std::vector<Eigen::Vector3d> approach(n);
  for (std::size_t i = 0; i < n; ++i) {
    approach[i] = grasps[i].tcp_orientation.toRotationMatrix().col(2);
  }

  std::vector<double> min_dist(n, std::numeric_limits<double>::max());
  std::vector<bool> selected(n, false);

  std::vector<Grasp> result;
  result.reserve(n);

  std::size_t pick = 0;

  for (std::size_t step = 0; step < n; ++step) {
    selected[pick] = true;
    result.push_back(grasps[pick]);

    const Eigen::Vector3d & p_pick = grasps[pick].tcp_position;
    const Eigen::Vector3d & a_pick = approach[pick];

    double best_dist = -1.0;
    std::size_t next_pick = 0;
    bool found_next = false;

    for (std::size_t i = 0; i < n; ++i) {
      if (selected[i]) {
        continue;
      }

      double d_pos = pos_weight * (grasps[i].tcp_position - p_pick).norm();

      // Angular distance between approach directions [rad], clamped to [0, π]
      double dot = approach[i].dot(a_pick);
      dot = std::max(-1.0, std::min(1.0, dot));
      double d_ori = ori_weight * std::acos(dot);

      double d = d_pos + d_ori;
      if (d < min_dist[i]) {
        min_dist[i] = d;
      }

      // Farthest-point: candidate with largest min_dist wins.
      // Tie-break by quality_score (higher is better).
      if (!found_next ||
        min_dist[i] > best_dist ||
        (std::abs(min_dist[i] - best_dist) < 1e-9 &&
        grasps[i].quality_score > grasps[next_pick].quality_score))
      {
        best_dist = min_dist[i];
        next_pick = i;
        found_next = true;
      }
    }

    if (!found_next) {
      break;
    }
    pick = next_pick;
  }

  grasps = std::move(result);
}

}  // namespace hold_and_weld_gripper_sampler

#endif  // HOLD_AND_WELD_GRIPPER_SAMPLER__CORE__GRASP_HPP_
