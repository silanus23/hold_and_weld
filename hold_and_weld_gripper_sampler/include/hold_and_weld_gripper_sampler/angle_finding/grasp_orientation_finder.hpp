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

#ifndef HOLD_AND_WELD_GRIPPER_SAMPLER__ANGLE_FINDING__GRASP_ORIENTATION_FINDER_HPP_
#define HOLD_AND_WELD_GRIPPER_SAMPLER__ANGLE_FINDING__GRASP_ORIENTATION_FINDER_HPP_

#include <memory>
#include <vector>

#include <gp_Ax1.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>
#include <gp_Trsf.hxx>
#include <gp_Vec.hxx>
#include <rclcpp/rclcpp.hpp>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Shape.hxx>

#include "hold_and_weld_gripper_sampler/geometry/fcl_collision_checker.hpp"
#include "hold_and_weld_gripper_sampler/core/grasp.hpp"
#include "hold_and_weld_gripper_sampler/core/gripper.hpp"
#include "hold_and_weld_gripper_sampler/geometry/topology.hpp"
#include "hold_and_weld_gripper_sampler/sampling/contact_point_sampler.hpp"
namespace hold_and_weld_gripper_sampler
{

namespace constraints
{
class ExclusionZoneConstraint;
class KissingSurfaceConstraint;
}

namespace angle_finding
{

/**
 * @brief Configuration for grasp orientation finding
 */
struct OrientationConfig
{
  double finger_length = 0.10;
  double finger_radius = 0.02;
  size_t max_orientations_per_pair = 0;
  size_t max_edge_candidates = 3;
  double dual_seed_dedup_tolerance_deg = 3.0;
  size_t max_edges_per_contact = 0;
  std::vector<double> angle_offsets = {-15.0, 0.0, 15.0};
  bool stop_on_first_valid = false;

  double collision_tolerance = 0.001;
};

/**
 * @brief Represents an edge within the finger circle
 */
struct EdgeConstraint
{
  TopoDS_Edge edge;
  gp_Pnt closest_point;
  double distance;
  double bearing_angle;
};

/**
 * @brief Represents a valid grasp candidate
 */
struct GraspCandidate
{
  gp_Pnt contact_1;
  gp_Pnt contact_2;
  gp_Vec approach_direction;   // Finger axis direction
  gp_Trsf gripper_transform;   // Full gripper placement transform
  int surface_id_1;
  int surface_id_2;
  double grip_distance;
  double quality_score;        // Distance from edges (higher = better)
};

/**
 * @brief Convert GraspCandidate (OCCT types) to Grasp (Eigen types)
 *
 * @param candidate GraspCandidate from orientation finding
 * @return Grasp with Eigen types ready for downstream use
 */
Grasp to_grasp(const GraspCandidate & candidate);

/**
 * @brief Finds valid gripper orientations for contact point pairs
 *
 * Algorithm:
 * 1. Find edges within finger circle around contact point
 * 2. Identify local minima (closest edge points)
 * 3. For each minimum, test multiple approach angles
 * 4. Validate collision with primary, exclusion zones, and secondaries
 */
class GraspOrientationFinder
{
public:
  GraspOrientationFinder(
    const TopoDS_Shape & primary_shape,
    const ParsedGripper & gripper,
    std::shared_ptr<const constraints::ExclusionZoneConstraint> exclusion_constraint,
    std::shared_ptr<const constraints::KissingSurfaceConstraint> kissing_constraint,
    const OrientationConfig & config = OrientationConfig{}
  );

  /**
   * @brief Set FCL collision checker for fast collision queries
   *
   * When set, collides_with_primary() will use FCL instead of OCCT.
   * This significantly improves performance for repeated collision checks.
   *
   * @param fcl_checker Shared pointer to FCL collision checker
   */
  void set_fcl_checker(std::shared_ptr<const geometry::FCLCollisionChecker> fcl_checker);

  /**
   * @brief Find all valid grasp orientations for contact pairs
   *
   * @param contact_pairs Contact point pairs from sampling
   * @param topology Primary shape topology
   * @return Vector of valid grasp candidates
   */
  std::vector<GraspCandidate> find_valid_grasps(
    const std::vector<sampling::ContactPair> & contact_pairs,
    const geometry::Topology & topology
  );

private:
  TopoDS_Shape primary_shape_;
  ParsedGripper gripper_;
  std::shared_ptr<const constraints::ExclusionZoneConstraint> exclusion_constraint_;
  std::shared_ptr<const constraints::KissingSurfaceConstraint> kissing_constraint_;
  OrientationConfig config_;

  // Optional FCL collision checker for fast queries
  std::shared_ptr<const geometry::FCLCollisionChecker> fcl_checker_;

  // Logger
  rclcpp::Logger logger_;

  /**
   * @brief Find edges within finger circle around contact point
   *
   * Only checks edges belonging to the contact surface (not all edges in topology).
   * For each edge, finds ALL local minima points (not just the global minimum).
   * Uses BRepExtrema_ExtPC to find orthogonal projections, then filters
   * to keep only local minima within finger reach.
   *
   * @param contact Contact point
   * @param surface_id ID of the surface where contact point lies
   * @param topology Topology for edge access
   * @return Vector of edge constraints (multiple per edge if curved)
   */
  std::vector<EdgeConstraint> find_edges_in_circle(
    const gp_Pnt & contact,
    int surface_id,
    const geometry::Topology & topology
  ) const;

  /**
   * @brief Find all local minima points on a single edge
   *
   * Uses BRepExtrema_ExtPC to find all extrema (points where the distance
   * derivative equals zero), then filters to keep only local minima within
   * finger reach.
   *
   * @param contact Contact point to measure distance from
   * @param edge Edge to analyze
   * @return Vector of EdgeConstraint for each local minimum found
   */
  std::vector<EdgeConstraint> find_local_minima_on_edge(
    const gp_Pnt & contact,
    const TopoDS_Edge & edge
  ) const;

  /**
   * @brief Find closest edges to contact point
   *
   * Sorts edges by distance and returns the top N closest edges
   * (limited by max_edge_candidates if set).
   *
   * @param edges Edge constraints (distance already computed)
   * @return Filtered edge constraints sorted by distance
   */
  std::vector<EdgeConstraint> find_local_minima(
    const std::vector<EdgeConstraint> & edges
  ) const;

  /**
   * @brief Compute bearing angle of edge relative to contact point
   *
   * Projects the direction from contact to edge onto the plane perpendicular
   * to the grip axis, then computes the angle in that plane.
   * Used for dual-seed deduplication.
   *
   * @param contact Contact point
   * @param edge_point Closest point on edge
   * @param grip_axis Grip direction (normal to the bearing plane)
   * @return Bearing angle in radians [-π, π]
   */
  double compute_bearing_angle(
    const gp_Pnt & contact,
    const gp_Pnt & edge_point,
    const gp_Vec & grip_axis
  ) const;

  /**
   * @brief Compute approach direction pointing away from edge constraint
   *
   * Computes the direction from the nearest edge toward the TCP (midpoint between contacts).
   * This direction is projected onto the plane perpendicular to the grip axis.
   * NOTE: Using TCP instead of individual contact points is semantically clearer,
   * though mathematically equivalent after projection (the component along grip_axis
   * is removed anyway).
   *
   * @param contact_1 First contact point
   * @param contact_2 Second contact point
   * @param edge_constraint Edge to orient away from
   * @param angle_offset Rotation offset in degrees (around grip axis)
   * @param grip_axis Direction from contact_1 to contact_2 (normalized)
   * @return Approach direction vector (perpendicular to grip axis, unit length)
   */
  gp_Vec compute_approach_direction(
    const gp_Pnt & contact_1,
    const gp_Pnt & contact_2,
    const EdgeConstraint & edge_constraint,
    double angle_offset,
    const gp_Vec & grip_axis
  ) const;

  /**
   * @brief Compute full gripper transform from contacts and approach
   *
   * @param contact_1 First contact point
   * @param contact_2 Second contact point
   * @param approach Approach direction
   * @return Gripper placement transformation
   */
  gp_Trsf compute_gripper_transform(
    const gp_Pnt & contact_1,
    const gp_Pnt & contact_2,
    const gp_Vec & approach
  ) const;

  /**
   * @brief Check if gripper collides with primary shape
   *
   * @param gripper_transform Gripper pose
   * @param grip_distance Finger opening distance
   * @return true if collision detected
   */
  bool collides_with_primary(
    const gp_Trsf & gripper_transform,
    double grip_distance
  ) const;

  /**
   * @brief Compute quality score for grasp (higher = better)
   *
   * Quality is based on minimum edge clearance at BOTH contact points.
   * Uses the smaller of the two clearances (weakest point determines quality).
   *
   * @param contact_1 First contact point
   * @param contact_2 Second contact point
   * @param edges_1 Edge constraints around contact_1
   * @param edges_2 Edge constraints around contact_2
   * @return Quality score based on minimum clearance at either contact
   */
  double compute_quality_score(
    const gp_Pnt & contact_1,
    const gp_Pnt & contact_2,
    const std::vector<EdgeConstraint> & edges_1,
    const std::vector<EdgeConstraint> & edges_2
  ) const;

  /**
   * @brief Compute closest point on edge to query point
   *
   * @param point Query point
   * @param edge Edge to measure against
   * @param closest_point Output closest point on edge
   * @return Distance from point to edge
   */
  double compute_closest_point_on_edge(
    const gp_Pnt & point,
    const TopoDS_Edge & edge,
    gp_Pnt & closest_point
  ) const;
};

}  // namespace angle_finding
}  // namespace hold_and_weld_gripper_sampler

#endif  // HOLD_AND_WELD_GRIPPER_SAMPLER__ANGLE_FINDING__GRASP_ORIENTATION_FINDER_HPP_
