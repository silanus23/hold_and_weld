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

#ifndef HOLD_AND_WELD_GRIPPER_SAMPLER__CONSTRAINTS__KISSING_SURFACE_CONSTRAINT_HPP_
#define HOLD_AND_WELD_GRIPPER_SAMPLER__CONSTRAINTS__KISSING_SURFACE_CONSTRAINT_HPP_

#include <Eigen/Dense>
#include <memory>
#include <string>
#include <vector>

#include <gp_Trsf.hxx>
#include <rclcpp/rclcpp.hpp>
#include <TopoDS_Shape.hxx>

#include "hold_and_weld_gripper_sampler/core/region_filter.hpp"
#include "hold_and_weld_gripper_sampler/geometry/fcl_collision_checker.hpp"
#include "hold_and_weld_gripper_sampler/geometry/geometry_mapper.hpp"
#include "hold_and_weld_gripper_sampler/core/gripper.hpp"
#include "hold_and_weld_gripper_sampler/geometry/topology.hpp"

namespace hold_and_weld_gripper_sampler
{
namespace constraints
{



/**
 * @brief Statistics for collision rejection tracking
 */
struct CollisionStats
{
  size_t total_checks = 0;
  size_t ground_plane_simple_rejections = 0;
  size_t fcl_ground_rejections = 0;
  size_t fcl_secondary_rejections = 0;
  size_t occt_secondary_rejections = 0;
  size_t occt_failures = 0;
  size_t mesh_failures = 0;

  void reset()
  {
    total_checks = 0;
    ground_plane_simple_rejections = 0;
    fcl_ground_rejections = 0;
    fcl_secondary_rejections = 0;
    occt_secondary_rejections = 0;
    occt_failures = 0;
    mesh_failures = 0;
  }
};

/**
 * @brief Constraint for handling fixture and ground plane collisions
 *
 * Performs three jobs:
 * 1. Identifies surfaces in full contact with secondaries (banned from sampling)
 * 2. Creates partial exclusion wires for surfaces partially touching secondaries
 * 3. Validates final gripper poses against secondary collision (Phase 5)
 */
class KissingSurfaceConstraint
{
public:
  /**
   * @brief Constructor
   *
   * @param mapper Shared geometry mapper for face lookups
   * @param gripper Parsed gripper kinematic information
   * @param secondary_shapes Fixtures, ground plane, obstacles
   * @param contact_threshold Surfaces with contact > this ratio are banned (default 0.8 = 80%)
   * @param collision_tolerance Distance threshold for collision detection in meters (default 1e-6)
   */
  KissingSurfaceConstraint(
    std::shared_ptr<const geometry::GeometryMapper> mapper,
    const ParsedGripper & gripper,
    const std::vector<TopoDS_Shape> & secondary_shapes,
    double contact_threshold = 0.8,
    double collision_tolerance = 1e-6
  );

  /**
   * @brief Set FCL collision checker for fast collision queries
   *
   * When set, intersects_secondary() will use FCL instead of OCCT.
   * The FCL checker must have secondary shapes added via add_secondary_shapes().
   *
   * @param fcl_checker Shared pointer to FCL collision checker
   */
  void set_fcl_checker(std::shared_ptr<const geometry::FCLCollisionChecker> fcl_checker);

  /**
   * @brief Analyze contact between primary and secondaries
   *
   * Must be called before get_banned_surface_ids() or get_sample_areas()
   *
   * @param topology Primary shape topology
   */
  void analyze_constraints(const geometry::Topology & topology);

  /**
   * @brief Get surface IDs that should not be sampled (full contact with secondaries)
   *
   * @return Vector of banned surface IDs (0-indexed)
   */
  std::vector<int> get_banned_surface_ids() const;

  /**
   * @brief Get partial exclusion wires for surfaces partially touching secondaries
   *
   * @return Vector of SampleArea objects with exclusion wires
   */
  std::vector<core::SampleArea> get_sample_areas() const;

  /**
   * @brief Check if gripper at specified pose and grip distance collides with secondaries
   *
   * This method:
   * 1. Configures gripper fingers to grip_distance
   * 2. Transforms configured gripper to grasp_pose
   * 3. Checks collision with secondary shapes
   *
   * @param grip_distance Distance between fingers (determines joint state)
   * @param grasp_pose 6-DOF pose of gripper (Eigen::Isometry3d)
   * @return true if collision detected, false otherwise
   */
  bool intersects_secondary(
    double grip_distance,
    const Eigen::Isometry3d & grasp_pose
  ) const;

  std::string get_name() const;

  /**
   * @brief Get secondary shapes for FCL checker initialization
   *
   * Returns the secondary shapes, suitable for adding to
   * FCLCollisionChecker via add_secondary_shapes().
   *
   * @return Vector of secondary shapes
   */
  const std::vector<TopoDS_Shape> & get_secondary_shapes() const;

  /**
   * @brief Get collision rejection statistics
   *
   * Returns statistics about why grasps were rejected during collision checking.
   * Useful for debugging and understanding collision patterns.
   *
   * @return Collision statistics structure
   */
  CollisionStats get_collision_stats() const;

  /**
   * @brief Reset collision statistics
   */
  void reset_collision_stats();

private:
  std::shared_ptr<const geometry::GeometryMapper> mapper_;
  ParsedGripper gripper_;
  std::vector<TopoDS_Shape> secondary_shapes_;
  double contact_threshold_;
  double collision_tolerance_;

  // Optional FCL collision checker for fast queries
  std::shared_ptr<const geometry::FCLCollisionChecker> fcl_checker_;

  // Results from analysis
  std::vector<int> banned_surface_ids_;
  std::vector<core::SampleArea> partial_exclusions_;

  // Collision rejection telemetry (mutable for const methods)
  mutable CollisionStats collision_stats_;

  // Logger
  rclcpp::Logger logger_;

  /**
   * @brief Measure contact ratio between a surface and all secondaries
   *
   * Uses existing mesh triangulation for area-uniform sampling.
   * Each triangle centroid is tested against secondary shapes.
   * Result is weighted by triangle area for correctness on curved surfaces.
   *
   * @param surface_id Surface to measure
   * @param topology Primary shape topology
   * @return Contact ratio (0.0 = no contact, 1.0 = full contact)
   */
  double measure_contact_ratio(
    int surface_id,
    const geometry::Topology & topology
  ) const;

  /**
   * @brief Extract boundary wire of contact region on a surface
   *
   * @param surface_id Surface with partial contact
   * @param topology Primary shape topology
   * @return Wire representing contact boundary
   */
  TopoDS_Wire extract_contact_boundary(
    int surface_id,
    const geometry::Topology & topology
  ) const;
};

}  // namespace constraints
}  // namespace hold_and_weld_gripper_sampler

#endif  // HOLD_AND_WELD_GRIPPER_SAMPLER__CONSTRAINTS__KISSING_SURFACE_CONSTRAINT_HPP_
