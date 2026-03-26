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

#ifndef HOLD_AND_WELD_GRIPPER_SAMPLER__GEOMETRY__FCL_COLLISION_CHECKER_HPP_
#define HOLD_AND_WELD_GRIPPER_SAMPLER__GEOMETRY__FCL_COLLISION_CHECKER_HPP_

#include <Eigen/Dense>
#include <fcl/fcl.h>
#include <memory>
#include <unordered_map>
#include <vector>

#include <gp_Trsf.hxx>
#include <TopoDS_Shape.hxx>

#include "hold_and_weld_gripper_sampler/io/gripper_parser.hpp"

namespace hold_and_weld_gripper_sampler
{
namespace geometry
{

/**
 * @brief FCL-based collision checker for fast gripper collision queries
 *
 * This class provides high-performance collision detection by converting
 * OCCT shapes to FCL BVH (Bounding Volume Hierarchy) models. The conversion
 * is done once during initialization, and subsequent collision queries
 * use FCL's optimized algorithms.
 *
 * Usage:
 * 1. Create checker with gripper and primary shape
 * 2. Optionally add exclusion volumes and secondary shapes
 * 3. Call collision check methods in the hot loop
 *
 * The gripper is decomposed into finger_1, finger_2, and base components.
 * Each component gets its own BVH model that can be transformed independently,
 * avoiding the need to rebuild geometry for different grip distances.
 */
class FCLCollisionChecker
{
public:
  using FCLScalar = double;
  using BVHModel = fcl::BVHModel<fcl::OBBRSS<FCLScalar>>;
  using CollisionObject = fcl::CollisionObject<FCLScalar>;
  using Transform3 = fcl::Transform3<FCLScalar>;

  /**
   * @brief Construct collision checker with gripper and primary shape
   *
   * @param gripper Parsed gripper with finger and base shapes
   * @param primary_shape Primary workpiece shape for collision checking
   * @param linear_deflection Triangulation precision (default 0.1mm)
   */
  FCLCollisionChecker(
    const io::ParsedGripper & gripper,
    const TopoDS_Shape & primary_shape,
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
   * @brief Add ground plane as a collision object
   *
   * Creates a large flat box at specified Z height to represent the ground.
   * The box extends from (center_x - size/2, center_y - size/2, ground_z - thickness/2)
   * to (center_x + size/2, center_y + size/2, ground_z + thickness/2).
   *
   * @param ground_z Z-coordinate of ground plane surface (default 0.0)
   * @param size Lateral extent of ground plane in X and Y (default 100.0 meters)
   * @param thickness Thickness of ground plane box in Z (default 0.1 meters)
   * @param center_x X-coordinate of ground plane center (default 0.0)
   * @param center_y Y-coordinate of ground plane center (default 0.0)
   */
  void add_ground_plane(
    double ground_z = 0.0,
    double size = 100.0,
    double thickness = 0.1,
    double center_x = 0.0,
    double center_y = 0.0);

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
   * @brief Check if gripper collides with any secondary shape
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
   *
   * Extracts triangles from OCCT's internal triangulation and builds
   * an FCL BVH model for fast collision queries.
   *
   * @param shape OCCT shape (must be triangulated)
   * @return Shared pointer to BVH model, or nullptr if conversion failed
   */
  std::shared_ptr<BVHModel> shape_to_bvh(const TopoDS_Shape & shape) const;

  /**
   * @brief Convert gp_Trsf to FCL Transform3
   *
   * @param trsf OCCT transformation
   * @return FCL transformation
   */
  Transform3 to_fcl_transform(const gp_Trsf & trsf) const;

  /**
   * @brief Compute finger transforms for given grip distance
   *
   * @param grip_distance Distance between finger contact points
   * @param finger_1_transform Output transform for finger 1
   * @param finger_2_transform Output transform for finger 2
   */
  void compute_finger_transforms(
    double grip_distance,
    Transform3 & finger_1_transform,
    Transform3 & finger_2_transform) const;

  /**
   * @brief Check collision between gripper components and a target BVH
   *
   * @param gripper_transform Base gripper transform
   * @param grip_distance Finger opening distance
   * @param target_bvh Target shape BVH model
   * @param tolerance Collision tolerance
   * @return true if any gripper component collides with target
   */
  bool check_gripper_collision(
    const gp_Trsf & gripper_transform,
    double grip_distance,
    const std::shared_ptr<BVHModel> & target_bvh,
    double tolerance) const;

  /**
   * @brief Compute minimum distance between gripper components and target
   *
   * @param gripper_transform Base gripper transform
   * @param grip_distance Finger opening distance
   * @param target_bvh Target shape BVH model
   * @return Minimum distance from any gripper component to target
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

  std::shared_ptr<BVHModel> primary_bvh_;

  std::vector<std::shared_ptr<BVHModel>> exclusion_bvhs_;

  std::vector<std::shared_ptr<BVHModel>> secondary_bvhs_;
  std::shared_ptr<BVHModel> ground_plane_bvh_;

  double linear_deflection_;

  bool valid_;
};

}  // namespace geometry
}  // namespace hold_and_weld_gripper_sampler

#endif  // HOLD_AND_WELD_GRIPPER_SAMPLER__GEOMETRY__FCL_COLLISION_CHECKER_HPP_
