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

#ifndef HOLD_AND_WELD_GRIPPER_SAMPLER__CONSTRAINTS__GROUND_CONSTRAINT_HPP_
#define HOLD_AND_WELD_GRIPPER_SAMPLER__CONSTRAINTS__GROUND_CONSTRAINT_HPP_

#include <memory>
#include <string>
#include <vector>

#include <gp_Trsf.hxx>
#include <rclcpp/rclcpp.hpp>
#include <TopoDS_Face.hxx>

#include "hold_and_weld_gripper_sampler/collision/fcl_collision_checker.hpp"
#include "hold_and_weld_gripper_sampler/core/region_filter.hpp"
#include "hold_and_weld_gripper_sampler/sampling/face_sampler.hpp"
#include "hold_and_weld_gripper_sampler/geometry/topology.hpp"

namespace hold_and_weld_gripper_sampler
{
namespace constraints
{

/**
 * @brief The ground the workpiece rests on: a finite, axis-aligned footprint.
 *
 * A weld setup is a bounded thing, so the ground is modelled as a rectangle of
 * size_x by size_y centred on (center_x, center_y) at height bottom_z rather
 * than as an infinite plane. Parts outside that rectangle are over open floor
 * and are not supported by it.
 */
struct GroundConfig
{
  double bottom_z = 0.0;
  double center_x = 0.0;
  double center_y = 0.0;
  double size_x = 10.0;
  double size_y = 10.0;
  double contact_band = 0.005;
  double support_threshold = 0.8;
  double sample_density = 0.005;
  double collision_tolerance = 1e-6;
};

/**
 * @brief Decides which workpiece surfaces rest on the ground, and whether a gripper pose hits it.
 *
 * Mostly-resting surfaces are banned outright, partly-resting ones get an exclusion wire
 * instead (same split as the kissing-surface constraint). intersects_ground() is the
 * separate pose-level check: surfaces ask what rests on the floor, poses ask what hits it.
 */
class GroundConstraint
{
public:
  /**
   * @brief Construct a ground constraint.
   *
   * @param config Ground footprint, contact band, and thresholds
   */
  explicit GroundConstraint(const GroundConfig & config);

  /**
   * @brief Set the FCL checker used for pose-level ground queries.
   * @param fcl_checker Checker holding the gripper and, if configured, the ground body
   */
  void set_fcl_checker(std::shared_ptr<const geometry::FCLCollisionChecker> fcl_checker);

  /**
   * @brief Classify every surface of the primary shape against the ground.
   *
   * @param topology Primary shape topology
   */
  void analyze_constraints(const geometry::Topology & topology);

  /**
   * @brief Get surface IDs that should not be sampled (resting fully on the ground)
   *
   * @return Vector of banned surface IDs (0-indexed)
   */
  std::vector<int> get_banned_surface_ids() const;

  /**
   * @brief Get partial exclusion wires for surfaces resting on part of their area
   *
   * @return Vector of SampleArea objects with exclusion wires
   */
  std::vector<core::SampleArea> get_sample_areas() const;

  /**
   * @brief Check if the gripper at a pose hits the ground.
   *
   * Uses GroundConfig::collision_tolerance. Returns false when the checker has
   * no ground body, and true (conservative reject) when no checker is set.
   *
   * @param gripper_transform 6-DOF pose of gripper
   * @param grip_distance Distance between fingers
   * @return True if the gripper collides with the ground
   */
  bool intersects_ground(const gp_Trsf & gripper_transform, double grip_distance) const;

  /**
   * @brief Get the human-readable name of this constraint
   *
   * @return Constraint name string ("GroundConstraint")
   */
  std::string get_name() const;

  /**
   * @brief True if the point lies within the ground footprint in XY.
   * @param x World-frame X coordinate [m]
   * @param y World-frame Y coordinate [m]
   * @return True if (x, y) falls inside the configured footprint
   */
  bool is_within_footprint(double x, double y) const;

private:
  /**
   * @brief Area fraction of a face resting on the ground.
   *
   * @param face Face to measure
   * @param resting_samples Optional output: the samples found to be resting
   * @return Fraction in [0, 1] of the face's area resting on the ground
   */
  double measure_ground_support(
    const TopoDS_Face & face,
    std::vector<sampling::FaceSample> * resting_samples = nullptr) const;

  GroundConfig config_;
  std::shared_ptr<const geometry::FCLCollisionChecker> fcl_checker_;

  std::vector<int> banned_surface_ids_;
  std::vector<core::SampleArea> partial_exclusions_;

  rclcpp::Logger logger_;
};

}  // namespace constraints
}  // namespace hold_and_weld_gripper_sampler

#endif  // HOLD_AND_WELD_GRIPPER_SAMPLER__CONSTRAINTS__GROUND_CONSTRAINT_HPP_
