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

#ifndef HOLD_AND_WELD_GRIPPER_SAMPLER__CONSTRAINTS__EXCLUSION_ZONE_CONSTRAINT_HPP_
#define HOLD_AND_WELD_GRIPPER_SAMPLER__CONSTRAINTS__EXCLUSION_ZONE_CONSTRAINT_HPP_

#include <Eigen/Dense>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <gp_Trsf.hxx>
#include <rclcpp/rclcpp.hpp>
#include <TopoDS_Shape.hxx>

#include "hold_and_weld_gripper_sampler/core/region_filter.hpp"
#include "hold_and_weld_gripper_sampler/geometry/fcl_collision_checker.hpp"
#include "hold_and_weld_gripper_sampler/geometry/geometry_mapper.hpp"
#include "hold_and_weld_gripper_sampler/io/gripper_parser.hpp"
#include "hold_and_weld_gripper_sampler/geometry/topology.hpp"

namespace hold_and_weld_gripper_sampler
{
namespace constraints
{

/**
 * @brief Exclusion zone defined as a line with radius
 */
struct exclusion_line
{
  Eigen::Vector3d start;
  Eigen::Vector3d end;
  double exclusion_radius;
  double clearance = 0.01;
};

/**
 * @brief Exclusion zone defined as a circle on a plane
 */
struct exclusion_circle
{
  Eigen::Vector3d center;
  Eigen::Vector3d normal;
  double radius;
  double projection_depth;
  double clearance = 0.01;
};

/**
 * @brief Exclusion zone defined as a polygon
 */
struct exclusion_polygon
{
  std::vector<Eigen::Vector3d> exclusion_corners;
  double projection_depth;
  double clearance = 0.01;
};

// Use ParsedGripper from gripper_parser.hpp
using io::ParsedGripper;

/**
 * @brief Constraint for user-defined exclusion zones (welds, screws, forbidden areas)
 *
 * Handles:
 * 1. Creates exclusion wires for sampling
 * 2. Configures gripper kinematics (opens fingers to grip distance)
 * 3. Validates gripper collision at configured state
 */
class ExclusionZoneConstraint
{
public:
  ExclusionZoneConstraint(
    std::shared_ptr<const geometry::GeometryMapper> mapper,
    const io::ParsedGripper & gripper,
    const std::optional<std::vector<exclusion_circle>> & circles = std::nullopt,
    const std::optional<std::vector<exclusion_polygon>> & polygons = std::nullopt,
    const std::optional<std::vector<exclusion_line>> & lines = std::nullopt,
    double mesh_linear_deflection = 0.001,
    double mesh_angular_deflection = 0.1
  );

  /**
   * @brief Set FCL collision checker for fast collision queries
   *
   * When set, intersects_exclusion_zone() will use FCL instead of OCCT.
   * The FCL checker must have exclusion volumes added via add_exclusion_volumes().
   *
   * @param fcl_checker Shared pointer to FCL collision checker
   */
  void set_fcl_checker(std::shared_ptr<const geometry::FCLCollisionChecker> fcl_checker);

  /**
   * @brief Analyze exclusion constraints against primary shape
   *
   * Creates constraint volumes, projects to get exclusion wires
   * Must be called before get_sample_areas()
   *
   * @param shape Primary shape to project constraints onto
   * @param topology Topology of the primary shape
   */
  void analyze_constraints(
    const TopoDS_Shape & shape,
    const geometry::Topology & topology
  );

  /**
   * @brief Get exclusion wires for sampling
   *
   * Call after analyze_constraints()
   *
   * @return Vector of SampleArea objects with exclusion wires
   */
  std::vector<core::SampleArea> get_sample_areas() const;


  /**
   * @brief Check if gripper at specified pose and grip distance collides with exclusion zones
   *
   * This method:
   * 1. Configures gripper fingers to grip_distance
   * 2. Transforms configured gripper to gripper_transform
   * 3. Checks collision with exclusion zone volumes
   *
   * @param gripper_transform 6-DOF pose of gripper
   * @param grip_distance Distance between fingers (determines joint state)
   * @param tolerance Distance threshold for collision detection (default 0.001m = 1mm)
   * @return true if collision detected, false otherwise
   */
  bool intersects_exclusion_zone(
    const gp_Trsf & gripper_transform,
    double grip_distance,
    double tolerance = 0.001
  ) const;

  std::string get_name() const;

  /**
   * @brief Get collision volumes for FCL checker initialization
   *
   * Returns the exclusion volumes with clearance applied, suitable for
   * adding to FCLCollisionChecker via add_exclusion_volumes().
   *
   * @return Vector of collision volume shapes
   */
  const std::vector<TopoDS_Shape> & get_collision_volumes() const;

private:
  std::shared_ptr<const geometry::GeometryMapper> mapper_;
  ParsedGripper gripper_;
  std::vector<exclusion_circle> circles_;
  std::vector<exclusion_polygon> polygons_;
  std::vector<exclusion_line> lines_;

  std::shared_ptr<const geometry::FCLCollisionChecker> fcl_checker_;

  std::vector<TopoDS_Shape> projection_volumes_;
  std::vector<TopoDS_Shape> collision_volumes_;

  std::vector<core::SampleArea> sample_areas_;

  // Mesh deflection parameters for OCCT triangulation
  // Linear deflection: maximum distance between mesh edge and actual curve
  // Angular deflection: maximum angular deviation in radians
  double mesh_linear_deflection_;
  double mesh_angular_deflection_;

  // Logger
  rclcpp::Logger logger_;

  /**
   * @brief Create cylindrical tube from line constraint
   *
   * @param line Line constraint definition
   * @param include_clearance If true, add clearance to radius
   * @return Triangulated tube shape
   */
  TopoDS_Shape create_tube_from_line(
    const exclusion_line & line,
    bool include_clearance
  ) const;

  /**
   * @brief Create thick disk from circle constraint
   *
   * @param circle Circle constraint definition
   * @param include_clearance If true, add clearance to radius
   * @return Triangulated disk shape
   */
  TopoDS_Shape create_volume_from_circle(
    const exclusion_circle & circle,
    bool include_clearance
  ) const;

  /**
   * @brief Create extruded prism from polygon constraint
   *
   * @param polygon Polygon constraint definition
   * @param include_clearance If true, offset polygon by clearance
   * @return Triangulated prism shape
   */
  TopoDS_Shape create_prism_from_polygon(
    const exclusion_polygon & polygon,
    bool include_clearance
  ) const;

  /**
   * @brief Section constraint volume with shape to extract exclusion wires
   *
   * @param constraint_volume Constraint geometry to project
   * @param shape Primary shape to section with
   * @return Vector of SampleArea objects with exclusion wires per surface
   */
  std::vector<core::SampleArea> process_constraint_volume(
    const TopoDS_Shape & constraint_volume,
    const TopoDS_Shape & shape
  ) const;
};

}  // namespace constraints
}  // namespace hold_and_weld_gripper_sampler

#endif  // HOLD_AND_WELD_GRIPPER_SAMPLER__CONSTRAINTS__EXCLUSION_ZONE_CONSTRAINT_HPP_
