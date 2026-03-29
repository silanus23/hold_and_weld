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
#include "hold_and_weld_gripper_sampler/constraints/exclusion_zone_constraint.hpp"
#include "hold_and_weld_gripper_sampler/constraints/kissing_surface_constraint.hpp"
#include "hold_and_weld_gripper_sampler/core/grasp.hpp"
#include "hold_and_weld_gripper_sampler/core/region_filter.hpp"
#include "hold_and_weld_gripper_sampler/geometry/fcl_collision_checker.hpp"
#include "hold_and_weld_gripper_sampler/geometry/geometry_mapper.hpp"
#include "hold_and_weld_gripper_sampler/io/gripper_parser.hpp"
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
  /// Valid grasps found (sorted by quality, descending)
  std::vector<Grasp> grasps;
  size_t num_contact_pairs = 0;
  size_t num_valid_surfaces = 0;
  size_t num_banned_surfaces = 0;
  size_t num_exclusion_areas = 0;
  size_t num_candidates = 0;
  bool success = false;
  std::string error_message;

  /**
   * @brief Check if any grasps were found
   */
  bool has_grasps() const {return !grasps.empty();}

  /**
   * @brief Get best grasp (highest quality)
   *
   * @return Pointer to best grasp, or nullptr if no grasps found
   */
  const Grasp * best_grasp() const
  {
    return grasps.empty() ? nullptr : &grasps.front();
  }
};

/**
 * @brief Configuration for GraspFinder
 */
struct GraspFinderConfig
{
  sampling::SamplingConfig sampling;
  angle_finding::OrientationConfig orientation;

  double kissing_contact_threshold = 0.8;
  double ground_normal_z_threshold = -0.9;
  double collision_tolerance = 0.000001;
  double ground_safety_margin = 0.005;
  double ground_z = 0.0;

  bool use_fcl = true;
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
 * It handles all the wiring between components in the correct order:
 *
 * 1. Load and analyze geometry (primary shape, gripper, secondaries)
 * 2. Analyze constraints (exclusion zones, kissing surfaces)
 * 3. Build FCL collision checker with all collision volumes
 * 4. Wire FCL to constraints and orientation finder
 * 5. Sample contact points
 * 6. Find valid grasp orientations
 * 7. Return sorted results
 *
 * }
 * @endcode
 */
class GraspFinder
{
public:
  /**
   * @brief Construct GraspFinder with all required inputs
   *
   * @param primary_shape Primary workpiece shape (must be triangulated)
   * @param primary_topology Topology extracted from primary shape
   * @param gripper Parsed gripper from URDF
   * @param secondary_shapes Fixture/ground shapes for collision (already triangulated)
   * @param exclusion_circles Optional exclusion circles (weld points, etc.)
   * @param exclusion_polygons Optional exclusion polygons (forbidden regions)
   * @param exclusion_lines Optional exclusion lines (weld seams, etc.)
   * @param config Configuration for all sub-components
   */
  GraspFinder(
    const TopoDS_Shape & primary_shape,
    const geometry::Topology & primary_topology,
    const io::ParsedGripper & gripper,
    const std::vector<TopoDS_Shape> & secondary_shapes,
    const std::optional<std::vector<constraints::exclusion_circle>> & exclusion_circles =
    std::nullopt,
    const std::optional<std::vector<constraints::exclusion_polygon>> & exclusion_polygons =
    std::nullopt,
    const std::optional<std::vector<constraints::exclusion_line>> & exclusion_lines = std::nullopt,
    const GraspFinderConfig & config = GraspFinderConfig{}
  );

  /**
   * @brief Construct GraspFinder with shared GeometryMapper
   *
   * Use this constructor when you want to share the GeometryMapper with other components
   * or when you need access to face mapping for surface ID lookups.
   *
   * @param mapper Shared geometry mapper (must have loaded the primary shape)
   * @param primary_shape Primary workpiece shape
   * @param primary_topology Topology from mapper
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
    const io::ParsedGripper & gripper,
    const std::vector<TopoDS_Shape> & secondary_shapes,
    const std::optional<std::vector<constraints::exclusion_circle>> & exclusion_circles =
    std::nullopt,
    const std::optional<std::vector<constraints::exclusion_polygon>> & exclusion_polygons =
    std::nullopt,
    const std::optional<std::vector<constraints::exclusion_line>> & exclusion_lines = std::nullopt,
    const GraspFinderConfig & config = GraspFinderConfig{}
  );

  /**
   * @brief Find all valid grasps
   *
   * This is the main method. It:
   * 1. Analyzes constraints
   * 2. Builds and wires FCL (if enabled)
   * 3. Samples contact points
   * 4. Finds valid orientations
   * 5. Converts and sorts results
   *
   * @return GraspFinderResult with grasps and statistics
   */
  GraspFinderResult find();

  /**
   * @brief Get top N grasps by quality
   *
   * Convenience method that calls find() and returns only the top N.
   *
   * @param n Maximum number of grasps to return
   * @return Vector of top N grasps (may be fewer if not enough found)
   */
  std::vector<Grasp> find_top(size_t n);

  /**
   * @brief Get the best grasp
   *
   * Convenience method that calls find() and returns only the best.
   *
   * @return Best grasp if found, std::nullopt otherwise
   */
  std::optional<Grasp> find_best();

private:
  // Inputs (stored, not owned except mapper)
  std::shared_ptr<const geometry::GeometryMapper> mapper_;
  TopoDS_Shape primary_shape_;
  geometry::Topology primary_topology_;
  io::ParsedGripper gripper_;
  std::vector<TopoDS_Shape> secondary_shapes_;

  // Exclusion zone definitions
  std::vector<constraints::exclusion_circle> exclusion_circles_;
  std::vector<constraints::exclusion_polygon> exclusion_polygons_;
  std::vector<constraints::exclusion_line> exclusion_lines_;

  // Configuration
  GraspFinderConfig config_;

  // Lazily initialized components (built on first find() call)
  bool initialized_ = false;
  std::shared_ptr<constraints::ExclusionZoneConstraint> exclusion_constraint_;
  std::shared_ptr<constraints::KissingSurfaceConstraint> kissing_constraint_;
  std::shared_ptr<geometry::FCLCollisionChecker> fcl_checker_;

  /**
   * @brief Initialize all components and wire them together
   *
   * Called lazily on first find() call.
   *
   * @return Error message if initialization failed, empty string on success
   */
  std::string initialize();

  /**
   * @brief Compute valid surface IDs by removing banned surfaces
   *
   * @param banned_ids Surface IDs to exclude
   * @return Vector of valid surface IDs for sampling
   */
  std::vector<int> compute_valid_surface_ids(const std::vector<int> & banned_ids) const;

  /**
   * @brief Merge sample areas from multiple constraints
   *
   * @return Combined vector of sample areas
   */
  std::vector<core::SampleArea> merge_sample_areas() const;

  rclcpp::Logger logger_;
};

}  // namespace core
}  // namespace hold_and_weld_gripper_sampler

#endif  // HOLD_AND_WELD_GRIPPER_SAMPLER__CORE__GRASP_FINDER_HPP_
