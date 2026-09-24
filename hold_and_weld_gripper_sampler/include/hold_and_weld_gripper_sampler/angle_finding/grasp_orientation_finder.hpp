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

#include "hold_and_weld_gripper_sampler/collision/fcl_collision_checker.hpp"
#include "hold_and_weld_gripper_sampler/collision/embree_mesh_query.hpp"
#include "hold_and_weld_gripper_sampler/core/grasp.hpp"
#include "hold_and_weld_gripper_sampler/geometry/occt_utils.hpp"
#include "hold_and_weld_gripper_sampler/core/gripper.hpp"
#include "hold_and_weld_gripper_sampler/geometry/topology.hpp"
#include "hold_and_weld_gripper_sampler/sampling/contact_point_sampler.hpp"


namespace hold_and_weld_gripper_sampler
{

namespace geometry
{
class JawClearanceCheck;
}

namespace constraints
{
class ExclusionZoneConstraint;
class GroundConstraint;
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
  double collision_tolerance = 0.001;
  bool stop_on_first_valid = false;
  double ring_step_size = 0.010;
  double angular_step_deg = 2.0;
  double flat_detection_tolerance_m = 0.003;
  double cliff_merge_tolerance_deg = 2.0;
  double min_cliff_width_deg = 5.0;
  double ray_lift_offset = 0.010;
  double seed_step_deg = 15.0;

  bool randomize_seeds = false;

  bool debug_full_sweep = false;
  double debug_sweep_step_deg = 10.0;

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

  std::vector<RadialSegment> & segs_for(SurfaceState state)
  {
    if (state == SurfaceState::FLAT) {return flat;}
    if (state == SurfaceState::HIGH) {return high;}
    return low;
  }

  const std::vector<RadialSegment> & segs_for(SurfaceState state) const
  {
    if (state == SurfaceState::FLAT) {return flat;}
    if (state == SurfaceState::HIGH) {return high;}
    return low;
  }
};

inline constexpr std::array<SurfaceState, 3> kAllSurfaceStates = {
  SurfaceState::FLAT, SurfaceState::HIGH, SurfaceState::LOW};

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
inline Grasp to_grasp(const GraspCandidate & candidate)
{
  gp_Pnt tcp(
    (candidate.contact_1.X() + candidate.contact_2.X()) / 2.0,
    (candidate.contact_1.Y() + candidate.contact_2.Y()) / 2.0,
    (candidate.contact_1.Z() + candidate.contact_2.Z()) / 2.0);

  return Grasp::create(
    geometry::to_eigen(tcp),
    geometry::extract_quaternion(candidate.gripper_transform),
    candidate.grip_distance,
    geometry::to_eigen(candidate.contact_1),
    geometry::to_eigen(candidate.contact_2),
    candidate.surface_id_1,
    candidate.surface_id_2,
    candidate.quality_score
  );
}

/**
 * @brief Finds valid gripper orientations for contact point pairs
 *
 * Algorithm:
 * 1. Build a local tangent frame per contact from its own surface normal.
 * 2. Sweep the outer ring (r = finger_length) using Embree raycasting in the
 *    local tangent plane. Miss = LOW (cliff), hit = FLAT or HIGH based on
 *    elevation relative to contact point. FLAT and HIGH are stored for
 *    diagnostics; only LOW segments survive as grasp candidates.
 * 3. Calibrate each contact's angular segments to a shared reference frame
 *    before merging: a common zero direction, and both sweeps turning the same
 *    way about the grip axis.
 * 4. If no LOW arcs on the outer ring -> skip this contact pair (fully flat).
 * 5. Sweep inner rings (r = finger_length - ring_step_size down to
 *    finger_radius, stepping by ring_step_size). Only angles within surviving
 *    LOW segments are tested. A HIGH hit at any inner radius trims or splits
 *    the containing LOW segment; FLAT/LOW hits leave it unchanged. Segments
 *    too narrow after trimming (< min_cliff_width_deg) are discarded.
 * 6. Intersect surviving LOW arcs from both contacts, then cluster -> one
 *    approach seed per cliff.
 * 7. Validate each seed against primary collision, exclusion zones, the
 *    ground, and secondary shapes via FCL.
 */
class GraspOrientationFinder
{
public:
  /**
   * @brief Construct the finder; collision checkers are attached afterwards via the setters.
   *
   * Throws std::invalid_argument if a step size in @p config is not positive,
   * since the sweeps would never terminate.
   *
   * @param primary_shape        Workpiece the contacts lie on
   * @param gripper              Gripper geometry and finger axes
   * @param exclusion_constraint Exclusion zones to reject poses against, or nullptr
   * @param kissing_constraint   Secondary shapes to reject poses against, or nullptr
   * @param config               Orientation-finding parameters
   */
  GraspOrientationFinder(
    const TopoDS_Shape & primary_shape,
    const ParsedGripper & gripper,
    std::shared_ptr<const constraints::ExclusionZoneConstraint> exclusion_constraint,
    std::shared_ptr<const constraints::KissingSurfaceConstraint> kissing_constraint,
    const OrientationConfig & config = OrientationConfig{}
  );

  /**
   * @brief Attach the jaw-mouth broad-phase check, run before the exact pose checks.
   * @param jaw_clearance_check Check to use, or nullptr to skip it
   */
  void set_jaw_clearance_check(
    std::shared_ptr<const geometry::JawClearanceCheck> jaw_clearance_check);

  /**
   * @brief Attach the ground constraint every candidate pose is tested against.
   * @param ground_constraint Constraint to use, or nullptr to skip the ground check
   */
  void set_ground_constraint(
    std::shared_ptr<const constraints::GroundConstraint> ground_constraint);

  /**
   * @brief Attach the checker for gripper-vs-workpiece collision.
   *
   * Required: without a valid checker every pose counts as colliding, so
   * find_valid_grasps returns nothing.
   *
   * @param fcl_checker Checker built for this gripper and primary shape
   */
  void set_fcl_checker(std::shared_ptr<const geometry::FCLCollisionChecker> fcl_checker);

  /**
   * @brief Attach the ray-query scene used to build the radial maps.
   *
   * Without a valid scene every direction reads as a cliff, so every seed is
   * tried and every quality score is 1.0.
   *
   * @param embree_checker Scene built from the primary shape
   */
  void set_embree_checker(std::shared_ptr<const geometry::EmbreeMeshQuery> embree_checker);

  /**
   * @brief Find collision-free gripper poses for each contact pair.
   *
   * A pair that raises an exception is logged and skipped; the rest still run.
   *
   * @param contact_pairs Antipodal contact pairs from the contact sampler
   * @param topology      Unused; kept for interface stability
   * @return Every pose that passed all checks (at most max_orientations_per_pair per
   *         pair when that is set)
   */
  std::vector<GraspCandidate> find_valid_grasps(
    const std::vector<sampling::ContactPair> & contact_pairs,
    const geometry::Topology & topology
  );

  /**
   * @brief Build a RadialMaps for one contact point using Embree raycasting.
   *
   * Rays are cast in the local tangent plane defined by lx/ly (built from
   * the contact's own surface normal). Segments are stored in the calibrated
   * shared angular frame (local_angle - angle_offset), so both contacts in a
   * pair are comparable after this call.
   *
   * @param contact        Contact point on the workpiece surface
   * @param normal         Outward surface normal at the contact
   * @param tangent_axis_x Local tangent frame X axis — points in the angle=0° direction of the radial sweep
   * @param tangent_axis_y Local tangent frame Y axis — points in the angle=90° direction of the radial sweep
   * @param lifted_center  contact + normal * ray_lift_offset (pre-computed)
   * @param angle_offset   Rotation from local frame to shared reference [rad]
   * @return Arcs of each state; a LOW arc crossing the seam ends past 2π
   */
  RadialMaps create_radial_maps(
    const gp_Pnt & contact,
    const gp_Dir & normal,
    const gp_Vec & tangent_axis_x,
    const gp_Vec & tangent_axis_y,
    const gp_Pnt & lifted_center,
    double angle_offset
  ) const;

private:
  TopoDS_Shape primary_shape_;
  ParsedGripper gripper_;
  std::shared_ptr<const constraints::ExclusionZoneConstraint> exclusion_constraint_;
  std::shared_ptr<const constraints::KissingSurfaceConstraint> kissing_constraint_;
  std::shared_ptr<const constraints::GroundConstraint> ground_constraint_;
  std::shared_ptr<const geometry::JawClearanceCheck> jaw_clearance_check_;
  OrientationConfig config_;
  std::shared_ptr<const geometry::FCLCollisionChecker> fcl_checker_;
  std::shared_ptr<const geometry::EmbreeMeshQuery> embree_checker_;
  rclcpp::Logger logger_;

  /** @brief Classify a single ray hit by its elevation above the contact plane. */
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
   * @brief Group angularly close segments into clusters and discard narrow ones.
   */
  std::vector<std::vector<RadialSegment>> cluster_and_filter(
    const std::vector<RadialSegment> & segments
  ) const;

  /**
   * @brief Gripper base pose that puts the TCP midway between the contacts.
   *
   * @param contact_1 First contact; the jaw closes along contact_1 -> contact_2
   * @param contact_2 Second contact
   * @param approach  Approach direction; its component along the grip axis is dropped
   * @param out_base  Receives the base origin in world frame
   * @return Base pose in world frame
   */
  gp_Trsf compute_gripper_transform(
    const gp_Pnt & contact_1,
    const gp_Pnt & contact_2,
    const gp_Vec & approach,
    gp_Pnt & out_base
  ) const;

  /** @brief Primary-shape collision check; true (reject) when no valid checker is attached. */
  bool collides_with_primary(const gp_Trsf & transform, double grip_distance) const;
};

}  // namespace angle_finding
}  // namespace hold_and_weld_gripper_sampler

#endif  // HOLD_AND_WELD_GRIPPER_SAMPLER__ANGLE_FINDING__GRASP_ORIENTATION_FINDER_HPP_
