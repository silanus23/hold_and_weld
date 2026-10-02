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
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <TopoDS_Shape.hxx>

#include "hold_and_weld_gripper_sampler/angle_finding/grasp_orientation_finder.hpp"
#include "hold_and_weld_gripper_sampler/collision/jaw_clearance_check.hpp"
#include "hold_and_weld_gripper_sampler/constraints/exclusion_zone_constraint.hpp"
#include "hold_and_weld_gripper_sampler/constraints/ground_constraint.hpp"
#include "hold_and_weld_gripper_sampler/constraints/kissing_surface_constraint.hpp"
#include "hold_and_weld_gripper_sampler/core/grasp.hpp"
#include "hold_and_weld_gripper_sampler/core/region_filter.hpp"
#include "hold_and_weld_gripper_sampler/collision/fcl_collision_checker.hpp"
#include "hold_and_weld_gripper_sampler/geometry/geometry_mapper.hpp"
#include "hold_and_weld_gripper_sampler/core/gripper.hpp"
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
  std::vector<Grasp> grasps;
  size_t num_contact_pairs = 0;
  size_t num_valid_surfaces = 0;
  size_t num_banned_surfaces = 0;
  size_t num_exclusion_areas = 0;
  size_t num_candidates = 0;
  /** Constraints or obstacles that were not (fully) enforced; the grasps may violate them. */
  std::vector<std::string> skipped_constraints;
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
 *
 * Mirrors the shape_refiner.* keys; see PARAMS.md.
 */
struct ShapeRefinerConfig
{
  bool enabled = true;
  double max_cylinder_radius = 0.100;
  double max_arc_length = 0.200;
  double enclave_area_ratio = 0.005;
  double enclave_angle_threshold = 45.0;
  double max_face_area_ratio = 0.3;
  double planarity_tolerance_deg = 1.0;
  int inflection_samples = 25;
};

/**
 * @brief Configuration for GraspFinder
 *
 * The kissing_* fields mirror the kissing.* keys and the ground_* fields the
 * ground_plane secondary; see PARAMS.md.
 */
struct GraspFinderConfig
{
  sampling::SamplingConfig sampling;
  angle_finding::OrientationConfig orientation;
  ShapeRefinerConfig shape_refiner;
  geometry::JawClearanceConfig jaw_clearance;

  /** Also used as the ground constraint's support_threshold. */
  double kissing_contact_threshold = 0.8;
  double kissing_contact_distance_threshold = 0.005;
  /** Spacing of the face samples that measure secondary and ground contact ratios [m]. */
  double kissing_sample_density = 0.005;

  /** Spacing of the face samples that find each exclusion zone's footprint [m]. */
  double exclusion_sample_density = 0.005;

  /**
   * TODO(silanus23): still unused. GroundConstraint decides support by measured
   * area fraction rather than by face normal, which handles faces that graze the
   * ground at an angle; a normal test would reject those. Kept in case explicit
   * normal-based filtering is wanted later.
   */
  double ground_normal_z_threshold = -0.9;

  /** A surface sample within this distance of ground_surface_z rests on the ground [m]. */
  double ground_safety_margin = 0.005;

  /** The ground is a finite footprint; see GroundConfig. */
  double ground_surface_z = 0.0;
  double ground_center_x = 0.0;
  double ground_center_y = 0.0;
  double ground_size_x = 10.0;
  double ground_size_y = 10.0;

  /**
   * Collision tolerance for secondary/fixture checks [m]. Kept tight: pre-computed
   * queries against known geometry. Separate from orientation.collision_tolerance.
   */
  double collision_tolerance = 0.000001;
  std::vector<TopoDS_Shape> ground_shapes;

  bool use_fcl = true;
  /** YAML implicit_ground: without a ground_plane secondary, assume the ground_* defaults. */
  bool enable_ground_plane_check = true;
  bool use_fcl_for_ground_plane = true;

  double triangulation_deflection = 0.0001;
  double mesh_linear_deflection = 0.001;
  double mesh_angular_deflection = 0.1;
};

/**
 * @brief Coordinator class that wires all grasp sampling components together
 *
 * GraspFinder is the main entry point for finding valid grasps on a workpiece.
 * Not thread-safe: find() and its wrappers must not be called concurrently.
 * The first find() call initializes lazily:
 * 1. Analyze constraints (exclusion zones, kissing surfaces, ground)
 * 2. Build the FCL collision checker and wire it to the constraints
 * 3. Build the jaw-clearance check, if enabled
 *
 * Every find() call then:
 * 1. Sample contact points
 * 2. Wire FCL and the checks into an orientation finder and find valid grasps
 * 3. Return results sorted by quality
 */
class GraspFinder
{
public:
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
   * @param fcl_primary_shape Unrefined primary shape for FCL collision meshes; null uses
   *   primary_shape
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
    const GraspFinderConfig & config = GraspFinderConfig{},
    const TopoDS_Shape & fcl_primary_shape = TopoDS_Shape{}
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
  TopoDS_Shape fcl_primary_shape_;
  geometry::Topology primary_topology_;
  ParsedGripper gripper_;
  std::vector<TopoDS_Shape> secondary_shapes_;

  std::vector<constraints::exclusion_circle> exclusion_circles_;
  std::vector<constraints::exclusion_polygon> exclusion_polygons_;
  std::vector<constraints::exclusion_line> exclusion_lines_;

  GraspFinderConfig config_;
  rclcpp::Logger logger_;

  mutable std::once_flag init_flag_;
  mutable std::string init_error_;
  /** Set by the first successful find(), so find_top() / find_best() don't rerun the pipeline. */
  mutable std::optional<GraspFinderResult> cached_result_;
  std::shared_ptr<constraints::ExclusionZoneConstraint> exclusion_constraint_;
  std::shared_ptr<constraints::KissingSurfaceConstraint> kissing_constraint_;
  std::shared_ptr<constraints::GroundConstraint> ground_constraint_;
  std::shared_ptr<geometry::JawClearanceCheck> jaw_clearance_check_;
  std::shared_ptr<geometry::FCLCollisionChecker> fcl_checker_;

  /**
   * @brief Lazily initialize all sub-components on the first find() call
   *
   * Constructs and wires together the ExclusionZoneConstraint,
   * KissingSurfaceConstraint, GroundConstraint, FCLCollisionChecker and
   * JawClearanceCheck in the correct order.
   *
   * @return Empty string on success, or a human-readable error message on failure
   */
  std::string initialize();

  /**
   * @brief Compute the sorted surface IDs eligible for contact-point sampling
   *
   * Subtracts banned_ids (surfaces fully in contact with secondaries) from
   * the complete list of surface IDs in the primary topology.
   */
  std::vector<int> compute_valid_surface_ids(const std::vector<int> & banned_ids) const;

  /**
   * @brief Merge exclusion SampleAreas from all active constraints
   *
   * Collects SampleArea objects produced by the ExclusionZoneConstraint,
   * the KissingSurfaceConstraint and, if present, the GroundConstraint and
   * concatenates them into a single vector that is forwarded to the
   * contact-point sampler.
   */
  std::vector<core::SampleArea> merge_sample_areas() const;

  /**
   * @brief List what initialize() could not enforce: exclusion zones, and obstacles
   * whose collision model failed to build
   *
   * @return One human-readable entry per skipped constraint
   */
  std::vector<std::string> collect_skipped_constraints() const;
};

}  // namespace core
}  // namespace hold_and_weld_gripper_sampler

#endif  // HOLD_AND_WELD_GRIPPER_SAMPLER__CORE__GRASP_FINDER_HPP_
