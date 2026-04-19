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

#include <cstdint>
#include <memory>
#include <vector>

#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>
#include <gp_Trsf.hxx>
#include <gp_Vec.hxx>
#include <rclcpp/rclcpp.hpp>
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
  size_t max_orientations_per_pair = 3;
  double collision_tolerance = 0.001;
  bool stop_on_first_valid = false;
  double ring_step_size = 0.010;
  double angular_step_deg = 2.0;
  double flat_detection_tolerance_m = 0.003;
  double cliff_merge_tolerance_deg = 2.0;
  double min_cliff_width_deg = 5.0;

  // Kept for config-parser compatibility — not used by the radial-map algorithm
  size_t max_edge_candidates = 3;
  double dual_seed_dedup_tolerance_deg = 3.0;
  size_t max_edges_per_contact = 0;
  std::vector<double> angle_offsets = {-15.0, 0.0, 15.0};
};

/**
 * @brief Classification of a surface sample relative to the contact plane
 *
 *   FLAT — surface exists at the contact elevation (no edge here)
 *   HIGH — surface rises above the contact plane (wall, instant ban)
 *   LOW  — no hit (cliff / drop-off, graspable direction)
 */
enum class SurfaceState : uint8_t { FLAT, HIGH, LOW };

/**
 * @brief A contiguous arc of uniform SurfaceState on one ring
 */
struct RadialSegment
{
  double start_rad;
  double end_rad;
  double radius;
  SurfaceState state;
};

/**
 * @brief Complete radial surface map around one contact point
 */
struct RadialMaps
{
  std::vector<RadialSegment> flat;
  std::vector<RadialSegment> high;
  std::vector<RadialSegment> low;
};

/**
 * @brief Represents a valid grasp candidate
 */
struct GraspCandidate
{
  gp_Pnt contact_1;
  gp_Pnt contact_2;
  gp_Vec approach_direction;
  gp_Trsf gripper_transform;
  gp_Pnt base_position;
  int surface_id_1;
  int surface_id_2;
  double grip_distance;
  double quality_score;
};

/**
 * @brief Convert GraspCandidate (OCCT types) to Grasp (Eigen types)
 */
Grasp to_grasp(const GraspCandidate & candidate);

/**
 * @brief Finds valid gripper orientations for contact point pairs
 *
 * Algorithm:
 * 1. Build a local tangent frame per contact from its own surface normal.
 * 2. Sweep the outer ring (r = finger_length) using FCL raycasting in the
 *    local tangent plane. Miss = LOW (cliff), hit = FLAT or HIGH based on
 *    elevation relative to contact point.
 * 3. Calibrate each contact's angular segments to a shared reference frame
 *    before merging.
 * 4. If no LOW arcs on the outer ring → skip this contact pair (fully flat).
 * 5. Sweep inner rings (r < finger_length) only within the LOW arc ranges
 *    found on the outer ring. Look only for HIGHs — a HIGH at any inner
 *    radius trims that arc from the candidate set.
 * 6. Cluster surviving LOW arcs → one approach seed per cliff.
 * 7. Validate each seed against primary collision, exclusion zones, and
 *    secondary shapes via FCL.
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

  void set_fcl_checker(std::shared_ptr<const geometry::FCLCollisionChecker> fcl_checker);

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
  std::shared_ptr<const geometry::FCLCollisionChecker> fcl_checker_;
  rclcpp::Logger logger_;

  /**
   * @brief Build a RadialMaps for one contact point using FCL raycasting.
   *
   * Rays are cast in the local tangent plane defined by lx/ly (built from
   * the contact's own surface normal). Segments are stored in the calibrated
   * shared angular frame (local_angle - angle_offset), so both contacts in a
   * pair are comparable after this call.
   *
   * @param contact        Contact point on the workpiece surface
   * @param normal         Outward surface normal at the contact
   * @param lx             Local tangent frame X axis (0° in local frame)
   * @param ly             Local tangent frame Y axis (90° in local frame)
   * @param lifted_center  contact + normal * kCeilingOffset (pre-computed)
   * @param angle_offset   Rotation from local frame to shared reference [rad]
   */
  RadialMaps create_radial_maps(
    const gp_Pnt & contact,
    const gp_Dir & normal,
    const gp_Vec & lx,
    const gp_Vec & ly,
    const gp_Pnt & lifted_center,
    double angle_offset
  ) const;

  /**
   * @brief Classify a single FCL ray hit relative to the contact plane.
   *
   * @param hit_found   Whether the ray hit the shape
   * @param hit_point   Hit point (valid only if hit_found)
   * @param contact     Contact point (elevation reference)
   * @param normal_vec  Outward surface normal as gp_Vec
   * @param tol         Flat-detection tolerance [m]
   */
  SurfaceState classify_hit(
    bool hit_found,
    const gp_Pnt & hit_point,
    const gp_Pnt & contact,
    const gp_Vec & normal_vec,
    double tol
  ) const;

  /** @brief Intersect LOW segments from two contact RadialMaps. */
  std::vector<RadialSegment> merge_low_segments(
    const RadialMaps & maps_1,
    const RadialMaps & maps_2
  ) const;

  /**
   * @brief Remove segments covered by HIGH on the outermost ring in either map.
   *
   * FLAT at outer ring means open space (no geometry within finger_length)
   * and does NOT block a segment.
   */
  std::vector<RadialSegment> filter_by_outer_ring(
    const std::vector<RadialSegment> & segments,
    const RadialMaps & maps_1,
    const RadialMaps & maps_2
  ) const;

  /** @brief Remove segments that overlap any HIGH segment at any radius. */
  std::vector<RadialSegment> ban_high_angles(
    const std::vector<RadialSegment> & segments,
    const RadialMaps & maps_1,
    const RadialMaps & maps_2
  ) const;

  /**
   * @brief Group angularly close segments into clusters and discard narrow ones.
   */
  std::vector<std::vector<RadialSegment>> cluster_and_filter(
    const std::vector<RadialSegment> & segments
  ) const;

  /** @brief Return the midpoint angle of a cluster (handles 0/2π wrap-around). */
  double cluster_midpoint(const std::vector<RadialSegment> & cluster) const;

  /** @brief Top-level orchestrator: derive approach seeds from two RadialMaps. */
  std::vector<double> build_approach_seeds(
    const RadialMaps & maps_1,
    const RadialMaps & maps_2
  ) const;

  gp_Trsf compute_gripper_transform(
    const gp_Pnt & contact_1,
    const gp_Pnt & contact_2,
    const gp_Vec & approach,
    gp_Pnt & out_base
  ) const;

  bool collides_with_primary(const gp_Trsf & transform, double grip_distance) const;
};

}  // namespace angle_finding
}  // namespace hold_and_weld_gripper_sampler

#endif  // HOLD_AND_WELD_GRIPPER_SAMPLER__ANGLE_FINDING__GRASP_ORIENTATION_FINDER_HPP_
