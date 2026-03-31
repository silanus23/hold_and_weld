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

#ifndef HOLD_AND_WELD_GRIPPER_SAMPLER__CORE__GRASP_FINDER_HPP_
#define HOLD_AND_WELD_GRIPPER_SAMPLER__CORE__GRASP_FINDER_HPP_

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <TopoDS_Shape.hxx>

#include "hold_and_weld_gripper_sampler/angle_finding/grasp_orientation_finder.hpp"
#include "hold_and_weld_gripper_sampler/geometry/shape_refiner.hpp"
#include "hold_and_weld_gripper_sampler/constraints/exclusion_zone_constraint.hpp"
#include "hold_and_weld_gripper_sampler/constraints/kissing_surface_constraint.hpp"
#include "hold_and_weld_gripper_sampler/core/grasp.hpp"
#include "hold_and_weld_gripper_sampler/core/region_filter.hpp"
#include "hold_and_weld_gripper_sampler/geometry/fcl_collision_checker.hpp"
#include "hold_and_weld_gripper_sampler/geometry/geometry_mapper.hpp"
#include "hold_and_weld_gripper_sampler/core/gripper.hpp"
#include "hold_and_weld_gripper_sampler/io/shape_loader.hpp"
#include "hold_and_weld_gripper_sampler/geometry/topology.hpp"
#include "hold_and_weld_gripper_sampler/sampling/contact_point_sampler.hpp"

namespace hold_and_weld_gripper_sampler
{
namespace core
{

/**
 * @brief Result from grasp finding operation
 */
struct GraspFinderResult
{
  std::vector<Grasp> grasps; // sorted by quality descending
  size_t num_contact_pairs = 0;
  size_t num_valid_surfaces = 0;
  size_t num_banned_surfaces = 0;
  size_t num_exclusion_areas = 0; // exclusion wires passed to contact sampler
  size_t num_candidates = 0;
  bool success = false;
  std::string error_message;

  /**
   * @brief Check whether any grasps were found
   *
   * @return true if the grasps vector is non-empty, false otherwise
   */
  bool has_grasps() const {return !grasps.empty();}

  /**
   * @brief Return a pointer to the highest-quality grasp
   *
   * Grasps are stored sorted by quality (descending), so the front element
   * is always the best one.
   *
   * @return Pointer to the best Grasp, or nullptr if no grasps exist
   */
  const Grasp * best_grasp() const
  {
    return grasps.empty() ? nullptr : &grasps.front();
  }
};

/**
 * @brief Shape refiner configuration
 */
struct ShapeRefinerConfig
{
  bool enabled = true;
  double max_cylinder_radius = 0.100; // meters, before radial splitting
  double max_arc_length = 0.200;      // meters, before surface splitting
  double enclave_area_ratio = 0.005;  // fraction of total area, e.g. 0.005 = 0.5%
  double enclave_angle_threshold = 45.0; // degrees, shallow walls below this are removed
};

/**
 * @brief Configuration for GraspFinder
 */
struct GraspFinderConfig
{
  sampling::SamplingConfig sampling;
  angle_finding::OrientationConfig orientation;
  ShapeRefinerConfig shape_refiner;

  double kissing_contact_threshold = 0.8; // contact ratio above which surface is banned
  double ground_normal_z_threshold = -0.9;
  double ground_safety_margin = 0.005; // meters above ground plane
  double ground_z = 0.0;

  // Collision tolerance for secondary/fixture checks. Kept tight (1e-6 m) — pre-computed
  // queries against known geometry. Separate from orientation.collision_tolerance (1mm)
  // which is looser to avoid false rejections during pose sampling.
  double collision_tolerance = 0.000001;

  bool use_fcl = true;
  bool enable_ground_plane_check = true;
  bool use_fcl_for_ground_plane = true;

  double triangulation_deflection = 0.0001; // FCL BVH construction precision (meters)
  double mesh_linear_deflection = 0.001;    // exclusion zone mesh precision (meters)
  double mesh_angular_deflection = 0.1;
};

/**
 * @brief Coordinator class that wires all grasp sampling components together
 *
 * GraspFinder is the main entry point for finding valid grasps on a workpiece.
 * Components are initialized lazily on the first find() call in this order:
 * 1. Analyze constraints (exclusion zones, kissing surfaces)
 * 2. Build FCL collision checker
 * 3. Wire FCL to constraints and orientation finder
 * 4. Sample contact points
 * 5. Find valid grasp orientations
 * 6. Return sorted results
 */
class GraspFinder
{
public:
  /**
   * @brief Construct GraspFinder with all required inputs
   *
   * @param primary_shape Primary workpiece shape (must be pre-refined and triangulated)
   * @param primary_topology Topology extracted from the refined primary shape
   * @param gripper Parsed gripper from URDF
   * @param secondary_shapes Fixture/ground shapes for collision
   * @param exclusion_circles Optional exclusion circles (weld points, etc.)
   * @param exclusion_polygons Optional exclusion polygons (forbidden regions)
   * @param exclusion_lines Optional exclusion lines (weld seams, etc.)
   * @param config Configuration for all sub-components
   */
  GraspFinder(
    const TopoDS_Shape & primary_shape,
    const geometry::Topology & primary_topology,
    const ParsedGripper & gripper,
    const std::vector<TopoDS_Shape> & secondary_shapes,
    const std::optional<std::vector<constraints::exclusion_circle>> & exclusion_circles =
      std::nullopt,
    const std::optional<std::vector<constraints::exclusion_polygon>> & exclusion_polygons =
      std::nullopt,
    const std::optional<std::vector<constraints::exclusion_line>> & exclusion_lines =
      std::nullopt,
    const GraspFinderConfig & config = GraspFinderConfig{}
  );

  /**
   * @brief Construct GraspFinder with shared GeometryMapper
   *
   * Use when the mapper is shared with other components or face ID lookups are needed.
   * The mapper must have been loaded from the same refined shape passed as primary_shape.
   *
   * @param mapper Shared geometry mapper
   * @param primary_shape Primary workpiece shape
   * @param primary_topology Topology matching the refined shape
   * @param gripper Parsed gripper
   * @param secondary_shapes Secondary collision shapes
   * @param exclusion_circles Optional exclusion circles
   * @param exclusion_polygons Optional exclusion polygons
   * @param exclusion_lines Optional exclusion lines
   * @param config Configuration
   */
  GraspFinder(
    std::shared_ptr<const geometry::GeometryMapper> mapper,
    const TopoDS_Shape & primary_shape,
    const geometry::Topology & primary_topology,
    const ParsedGripper & gripper,
    const std::vector<TopoDS_Shape> & secondary_shapes,
    const std::optional<std::vector<constraints::exclusion_circle>> & exclusion_circles =
      std::nullopt,
    const std::optional<std::vector<constraints::exclusion_polygon>> & exclusion_polygons =
      std::nullopt,
    const std::optional<std::vector<constraints::exclusion_line>> & exclusion_lines =
      std::nullopt,
    const GraspFinderConfig & config = GraspFinderConfig{}
  );

  /**
   * @brief Find all valid grasps
   *
   * @return GraspFinderResult with grasps and pipeline statistics
   */
  GraspFinderResult find();

  /**
   * @brief Get top N grasps by quality
   *
   * @param n Maximum number of grasps to return
   * @return Vector of top N grasps (may be fewer if not enough found)
   */
  std::vector<Grasp> find_top(size_t n);

  /**
   * @brief Get the best grasp
   *
   * @return Best grasp if found, std::nullopt otherwise
   */
  std::optional<Grasp> find_best();

private:
  std::shared_ptr<const geometry::GeometryMapper> mapper_;
  TopoDS_Shape primary_shape_;
  geometry::Topology primary_topology_;
  ParsedGripper gripper_;
  std::vector<TopoDS_Shape> secondary_shapes_;

  std::vector<constraints::exclusion_circle> exclusion_circles_;
  std::vector<constraints::exclusion_polygon> exclusion_polygons_;
  std::vector<constraints::exclusion_line> exclusion_lines_;

  GraspFinderConfig config_;
  rclcpp::Logger logger_;

  // Lazily initialized on first find() call
  bool initialized_ = false;
  std::shared_ptr<constraints::ExclusionZoneConstraint> exclusion_constraint_;
  std::shared_ptr<constraints::KissingSurfaceConstraint> kissing_constraint_;
  std::shared_ptr<geometry::FCLCollisionChecker> fcl_checker_;

  /**
   * @brief Lazily initialize all sub-components on the first find() call
   *
   * Constructs and wires together the ExclusionZoneConstraint,
   * KissingSurfaceConstraint, and FCLCollisionChecker in the correct order.
   *
   * @return Empty string on success, or a human-readable error message on failure
   */
  std::string initialize();

  /**
   * @brief Compute the set of surface IDs eligible for contact-point sampling
   *
   * Subtracts banned_ids (surfaces fully in contact with secondaries) from
   * the complete list of surface IDs in the primary topology.
   *
   * @param banned_ids Surface IDs that must be excluded from sampling
   * @return Sorted vector of surface IDs that are available for sampling
   */
  std::vector<int> compute_valid_surface_ids(const std::vector<int> & banned_ids) const;

  /**
   * @brief Merge exclusion SampleAreas from all active constraints
   *
   * Collects SampleArea objects produced by the ExclusionZoneConstraint and
   * the KissingSurfaceConstraint and concatenates them into a single vector
   * that is forwarded to the contact-point sampler.
   *
   * @return Combined vector of SampleArea objects from all constraints
   */
  std::vector<core::SampleArea> merge_sample_areas() const;
};

}  // namespace core
}  // namespace hold_and_weld_gripper_sampler

#endif  // HOLD_AND_WELD_GRIPPER_SAMPLER__CORE__GRASP_FINDER_HPP_
