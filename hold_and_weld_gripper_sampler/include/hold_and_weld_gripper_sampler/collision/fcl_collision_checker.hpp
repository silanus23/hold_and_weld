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

#ifndef HOLD_AND_WELD_GRIPPER_SAMPLER__COLLISION__FCL_COLLISION_CHECKER_HPP_
#define HOLD_AND_WELD_GRIPPER_SAMPLER__COLLISION__FCL_COLLISION_CHECKER_HPP_

#include <Eigen/Dense>
#include <fcl/fcl.h>
#include <atomic>
#include <memory>
#include <unordered_map>
#include <vector>

#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>
#include <gp_Trsf.hxx>
#include <TopoDS_Shape.hxx>

#include "hold_and_weld_gripper_sampler/core/gripper.hpp"
#include "hold_and_weld_gripper_sampler/collision/embree_mesh_query.hpp"

namespace hold_and_weld_gripper_sampler
{
namespace geometry
{

/**
 * @brief FCL-based collision checker for gripper collision queries.
 *
 * Converts OCCT shapes to FCL BVH models once at construction; subsequent
 * queries use FCL's optimized algorithms. The gripper is decomposed into
 * finger_1, finger_2, and base, each with its own BVH so they can be
 * transformed independently per grip distance.
 */
class FCLCollisionChecker
{
public:
  using FCLScalar = double;
  using BVHModel = fcl::BVHModel<fcl::OBBRSS<FCLScalar>>;
  using CollisionObject = fcl::CollisionObject<FCLScalar>;
  using Transform3 = fcl::Transform3<FCLScalar>;

  /**
   * @brief Construct collision checker with gripper and primary shape only
   *
   * Use when exclusion volumes, secondaries and ground plane will be added
   * incrementally via the add_* methods (e.g. in tests).
   *
   * @param gripper Parsed gripper with finger and base shapes
   * @param primary_shape Primary workpiece shape for collision checking
   * @param linear_deflection Triangulation precision (default 0.1mm)
   */
  FCLCollisionChecker(
    const ParsedGripper & gripper,
    const TopoDS_Shape & primary_shape,
    double linear_deflection = 0.0001);

  /**
   * @brief Fully-wired constructor — all collision volumes provided upfront.
   *
   * @param gripper Parsed gripper with finger and base shapes
   * @param primary_shape Primary workpiece shape for collision checking
   * @param exclusion_volumes Exclusion zone shapes (with clearance already applied)
   * @param secondary_shapes Fixture / ground shapes
   * @param enable_ground_plane Whether to add an infinite ground plane
   * @param ground_z Z-coordinate of the ground plane surface
   * @param linear_deflection Triangulation precision (default 0.1mm)
   */
  FCLCollisionChecker(
    const ParsedGripper & gripper,
    const TopoDS_Shape & primary_shape,
    const std::vector<TopoDS_Shape> & exclusion_volumes,
    const std::vector<TopoDS_Shape> & secondary_shapes,
    bool enable_ground_plane,
    double ground_z,
    double linear_deflection = 0.0001);

  /**
   * @brief Add exclusion zone volumes for collision checking
   *
   * @param exclusion_volumes Vector of exclusion zone shapes (already expanded with clearance)
   */
  void add_exclusion_volumes(const std::vector<TopoDS_Shape> & exclusion_volumes);

  /**
   * @brief Add secondary shapes for collision checking
   *
   * @param secondary_shapes Vector of secondary workpiece shapes
   */
  void add_secondary_shapes(const std::vector<TopoDS_Shape> & secondary_shapes);

  /**
   * @brief Add ground plane as an FCL Halfspace (infinite, no size limits).
   *
   * @param normal Outward unit normal of the ground surface (default (0,0,1) = floor up).
   * @param plane_offset Signed distance from world origin along normal to the plane surface.
   */
  void add_ground_plane(
    const Eigen::Vector3d & normal = Eigen::Vector3d(0.0, 0.0, 1.0),
    double plane_offset = 0.0);

  /**
   * @brief Returns true if a ground plane BVH has been added to this checker.
   */
  bool has_ground_plane() const;


  /**
   * @brief Check if gripper collides with primary shape
   *
   * @param gripper_transform Transform placing gripper in world frame
   * @param grip_distance Distance between finger contact points
   * @param tolerance Collision tolerance (penetration allowed)
   * @return true if collision detected (distance < tolerance)
   */
  bool collides_with_primary(
    const gp_Trsf & gripper_transform,
    double grip_distance,
    double tolerance) const;

  /**
   * @brief Check if gripper collides with any exclusion volume
   *
   * @param gripper_transform Transform placing gripper in world frame
   * @param grip_distance Distance between finger contact points
   * @param tolerance Collision tolerance
   * @return true if collision detected with any exclusion volume
   */
  bool collides_with_exclusions(
    const gp_Trsf & gripper_transform,
    double grip_distance,
    double tolerance) const;

  /**
   * @brief Check if gripper collides with the ground plane
   *
   * @param gripper_transform Transform placing gripper in world frame
   * @param grip_distance Distance between finger contact points
   * @param tolerance Collision tolerance
   * @return true if collision detected with the ground plane
   */
  bool collides_with_ground(
    const gp_Trsf & gripper_transform,
    double grip_distance,
    double tolerance) const;

  /**
   * @brief Check if gripper collides with any secondary shape (excludes ground plane)
   *
   * @param gripper_transform Transform placing gripper in world frame
   * @param grip_distance Distance between finger contact points
   * @param tolerance Collision tolerance
   * @return true if collision detected with any secondary shape
   */
  bool collides_with_secondaries(
    const gp_Trsf & gripper_transform,
    double grip_distance,
    double tolerance) const;

  /**
   * @brief Get minimum distance from gripper to primary shape
   *
   * @param gripper_transform Transform placing gripper in world frame
   * @param grip_distance Distance between finger contact points
   * @return Minimum distance (negative if penetrating)
   */
  double distance_to_primary(
    const gp_Trsf & gripper_transform,
    double grip_distance) const;


  /**
   * @brief Check if collision checker is properly initialized
   *
   * @return true if all BVH models were built successfully
   */
  bool is_valid() const;

private:
  /**
   * @brief Convert OCCT shape to FCL BVH model
   */
  std::shared_ptr<BVHModel> shape_to_bvh(const TopoDS_Shape & shape) const;

  /**
   * @brief Convert gp_Trsf to FCL Transform3
   */
  Transform3 to_fcl_transform(const gp_Trsf & trsf) const;

  /**
   * @brief Compute finger transforms for given grip distance
   */
  void compute_finger_transforms(
    double grip_distance,
    Transform3 & finger_1_transform,
    Transform3 & finger_2_transform) const;

  enum class TargetKind { Primary, Ground, Secondary, Exclusion };

  /**
   * @brief Check collision between gripper components and a target BVH
   */
  bool check_gripper_collision(
    const gp_Trsf & gripper_transform,
    double grip_distance,
    const std::shared_ptr<BVHModel> & target_bvh,
    double tolerance,
    TargetKind kind,
    size_t secondary_index = 0) const;

  /**
   * @brief Check collision between gripper components and the ground halfspace
   */
  bool check_gripper_collision_halfspace(
    const gp_Trsf & gripper_transform,
    double grip_distance,
    double tolerance) const;

  /**
   * @brief Compute minimum distance between gripper components and target
   */
  double compute_gripper_distance(
    const gp_Trsf & gripper_transform,
    double grip_distance,
    const std::shared_ptr<BVHModel> & target_bvh) const;

  std::shared_ptr<BVHModel> finger_1_bvh_;
  std::shared_ptr<BVHModel> finger_2_bvh_;
  std::shared_ptr<BVHModel> base_bvh_;

  Eigen::Vector3d finger_1_axis_;
  Eigen::Vector3d finger_2_axis_;

  double rest_gap_;

  std::shared_ptr<BVHModel> primary_bvh_;

  // Embree query engines for watertight ray casting and point-in-solid checks.
  // Indices are kept in sync with exclusion_bvhs_ / secondary_bvhs_.
  // Each entry may be nullptr if Embree construction failed (falls back to FCL BVH).
  std::shared_ptr<EmbreeMeshQuery> embree_primary_;
  std::vector<std::shared_ptr<EmbreeMeshQuery>> embree_exclusions_;
  std::vector<std::shared_ptr<EmbreeMeshQuery>> embree_secondaries_;

  std::vector<std::shared_ptr<BVHModel>> exclusion_bvhs_;
  std::vector<std::shared_ptr<BVHModel>> secondary_bvhs_;

  // Ground plane as an FCL Halfspace — infinite, normal · x <= d is solid.
  using Halfspace = fcl::Halfspace<FCLScalar>;
  std::shared_ptr<Halfspace> ground_halfspace_;

  double linear_deflection_;
  bool valid_;

  struct CollisionStats
  {
    uint64_t total_checks{0};

    // primary
    uint64_t primary_base{0};
    uint64_t primary_f1{0};
    uint64_t primary_f2{0};

    // ground plane
    uint64_t ground_base{0};
    uint64_t ground_f1{0};
    uint64_t ground_f2{0};

    // secondaries — per-index vectors; resized once when secondary_bvhs_ is populated
    std::vector<uint64_t> sec_base;
    std::vector<uint64_t> sec_f1;
    std::vector<uint64_t> sec_f2;

    // exclusions — per-index vectors; resized once when exclusion_bvhs_ is populated
    std::vector<uint64_t> exc_base;
    std::vector<uint64_t> exc_f1;
    std::vector<uint64_t> exc_f2;

    // ground pass/fail Z tracking in microns (z * 1e6)
    int64_t ground_pass_count{0};
    int64_t ground_fail_count{0};
    int64_t ground_sum_z_pass_um{0};
    int64_t ground_sum_z_fail_um{0};
    int64_t ground_min_z_pass_um{INT64_MAX};
  };
  mutable CollisionStats stats_;

public:
  /**
   * @brief Log per-part collision statistics accumulated since construction.
   */
  void log_collision_stats() const;

  /**
   * @brief Get the primary shape BVH model (for diagnostics only)
   */
  std::shared_ptr<BVHModel> get_primary_bvh() const {return primary_bvh_;}

  /**
   * @brief Get the base BVH model (for diagnostics only)
   */
  std::shared_ptr<BVHModel> get_base_bvh() const {return base_bvh_;}
};

}  // namespace geometry
}  // namespace hold_and_weld_gripper_sampler

#endif  // HOLD_AND_WELD_GRIPPER_SAMPLER__COLLISION__FCL_COLLISION_CHECKER_HPP_
