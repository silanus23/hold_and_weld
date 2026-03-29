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

#include "hold_and_weld_gripper_sampler/geometry/fcl_collision_checker.hpp"

#include <fcl/narrowphase/collision.h>
#include <fcl/narrowphase/distance.h>

#include <algorithm>
#include <memory>
#include <limits>
#include <vector>

#include <BRepMesh_IncrementalMesh.hxx>
#include <BRep_Tool.hxx>
#include <Poly_Triangulation.hxx>
#include <rclcpp/rclcpp.hpp>
#include <TopExp_Explorer.hxx>
#include <TopLoc_Location.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Face.hxx>

namespace hold_and_weld_gripper_sampler
{
namespace geometry
{

static const rclcpp::Logger logger_ = rclcpp::get_logger("gripper_sampler");

FCLCollisionChecker::FCLCollisionChecker(
  const ParsedGripper & gripper,
  const TopoDS_Shape & primary_shape,
  double linear_deflection)
: finger_1_axis_(gripper.finger_1_axis),
  finger_2_axis_(gripper.finger_2_axis),
  linear_deflection_(linear_deflection),
  valid_(false)
{
  RCLCPP_DEBUG(logger_, "Initializing FCL collision checker");

  try {
    Standard_Real ang_deflection = 0.5;

    BRepMesh_IncrementalMesh mesh_f1(gripper.finger_1, linear_deflection_, Standard_False,
      ang_deflection);
    BRepMesh_IncrementalMesh mesh_f2(gripper.finger_2, linear_deflection_, Standard_False,
      ang_deflection);
    BRepMesh_IncrementalMesh mesh_base(gripper.base, linear_deflection_, Standard_False,
      ang_deflection);
    BRepMesh_IncrementalMesh mesh_primary(primary_shape, linear_deflection_, Standard_False,
      ang_deflection);

    finger_1_bvh_ = shape_to_bvh(gripper.finger_1);
    finger_2_bvh_ = shape_to_bvh(gripper.finger_2);
    base_bvh_ = shape_to_bvh(gripper.base);

    if (!finger_1_bvh_ || !finger_2_bvh_ || !base_bvh_) {
      RCLCPP_ERROR(logger_, "Failed to build BVH models for gripper components");
      return;
    }

    RCLCPP_DEBUG(logger_, "Gripper BVH models built successfully");
    RCLCPP_DEBUG(logger_, "  Finger 1: %d triangles", finger_1_bvh_->num_tris);
    RCLCPP_DEBUG(logger_, "  Finger 2: %d triangles", finger_2_bvh_->num_tris);
    RCLCPP_DEBUG(logger_, "  Base: %d triangles", base_bvh_->num_tris);

    primary_bvh_ = shape_to_bvh(primary_shape);

    if (!primary_bvh_) {
      RCLCPP_ERROR(logger_, "Failed to build BVH model for primary shape");
      return;
    }

    RCLCPP_DEBUG(logger_, "Primary shape BVH built: %d triangles", primary_bvh_->num_tris);

    valid_ = true;
    RCLCPP_INFO(logger_, "FCL collision checker initialized successfully");
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger_, "Exception during FCL initialization: %s", e.what());
    valid_ = false;
  } catch (...) {
    RCLCPP_ERROR(logger_, "Unknown exception during FCL initialization");
    valid_ = false;
  }
}

void FCLCollisionChecker::add_exclusion_volumes(const std::vector<TopoDS_Shape> & exclusion_volumes)
{
  RCLCPP_DEBUG(logger_, "Adding %zu exclusion volumes", exclusion_volumes.size());

  Standard_Real ang_deflection = 0.5;

  for (size_t i = 0; i < exclusion_volumes.size(); ++i) {
    const auto & volume = exclusion_volumes[i];

    try {
      BRepMesh_IncrementalMesh mesher(volume, linear_deflection_, Standard_False, ang_deflection);

      auto bvh = shape_to_bvh(volume);

      if (bvh) {
        exclusion_bvhs_.push_back(bvh);
        RCLCPP_DEBUG(logger_, "  Exclusion volume %zu: %d triangles", i, bvh->num_tris);
      } else {
        RCLCPP_WARN(logger_, "  Exclusion volume %zu: failed to build BVH", i);
      }
    } catch (const std::exception & e) {
      RCLCPP_WARN(logger_, "  Exclusion volume %zu: exception: %s", i, e.what());
    } catch (...) {
      RCLCPP_WARN(logger_, "  Exclusion volume %zu: unknown exception", i);
    }
  }

  RCLCPP_DEBUG(logger_, "Added %zu exclusion BVH models", exclusion_bvhs_.size());
}

void FCLCollisionChecker::add_secondary_shapes(const std::vector<TopoDS_Shape> & secondary_shapes)
{
  RCLCPP_DEBUG(logger_, "Adding %zu secondary shapes", secondary_shapes.size());

  Standard_Real ang_deflection = 0.5;

  for (size_t i = 0; i < secondary_shapes.size(); ++i) {
    const auto & shape = secondary_shapes[i];

    try {
      BRepMesh_IncrementalMesh mesher(shape, linear_deflection_, Standard_False, ang_deflection);

      auto bvh = shape_to_bvh(shape);

      if (bvh) {
        secondary_bvhs_.push_back(bvh);
        RCLCPP_DEBUG(logger_, "  Secondary shape %zu: %d triangles", i, bvh->num_tris);
      } else {
        RCLCPP_WARN(logger_, "  Secondary shape %zu: failed to build BVH", i);
      }
    } catch (const std::exception & e) {
      RCLCPP_WARN(logger_, "  Secondary shape %zu: exception: %s", i, e.what());
    } catch (...) {
      RCLCPP_WARN(logger_, "  Secondary shape %zu: unknown exception", i);
    }
  }

  RCLCPP_DEBUG(logger_, "Added %zu secondary BVH models", secondary_bvhs_.size());
}

void FCLCollisionChecker::add_ground_plane(
  double ground_z,
  double size,
  double thickness,
  double center_x,
  double center_y)
{
  RCLCPP_DEBUG(logger_,
        "Adding ground plane at z=%.3f, size=%.1f, thickness=%.3f, center=(%.1f, %.1f)",
    ground_z, size, thickness, center_x, center_y);

  try {
    // Create a box BVH model for the ground plane
    // Box extends from -size/2 to +size/2 in X and Y, centered at (center_x, center_y)
    // and from ground_z - thickness/2 to ground_z + thickness/2 in Z
    auto ground_bvh = std::make_shared<BVHModel>();
    ground_bvh->beginModel();

    double half_size = size / 2.0;
    double half_thickness = thickness / 2.0;

    std::vector<fcl::Vector3<FCLScalar>> corners = {
      {center_x - half_size, center_y - half_size,
        ground_z - half_thickness},  // 0: bottom-left-bottom
      {center_x + half_size, center_y - half_size,
        ground_z - half_thickness},  // 1: bottom-right-bottom
      {center_x + half_size,
        center_y + half_size, ground_z - half_thickness},  // 2: top-right-bottom
      {center_x - half_size, center_y + half_size,
        ground_z - half_thickness},  // 3: top-left-bottom
      {center_x - half_size,
        center_y - half_size, ground_z + half_thickness},  // 4: bottom-left-top
      {center_x + half_size,
        center_y - half_size, ground_z + half_thickness},  // 5: bottom-right-top
      {center_x + half_size, center_y + half_size, ground_z + half_thickness},  // 6: top-right-top
      {center_x - half_size, center_y + half_size, ground_z + half_thickness}   // 7: top-left-top
    };

    // Define 12 triangles forming the box (2 per face, 6 faces)
    std::vector<fcl::Triangle> triangles = {
      // Bottom face (z = ground_z - half_thickness)
      {0, 1, 2}, {0, 2, 3},
      // Top face (z = ground_z + half_thickness)
      {4, 6, 5}, {4, 7, 6},
      // Front face (y = center_y - half_size)
      {0, 5, 1}, {0, 4, 5},
      // Back face (y = center_y + half_size)
      {2, 6, 7}, {2, 7, 3},
      // Left face (x = center_x - half_size)
      {0, 3, 7}, {0, 7, 4},
      // Right face (x = center_x + half_size)
      {1, 5, 6}, {1, 6, 2}
    };

    ground_bvh->addSubModel(corners, triangles);
    ground_bvh->endModel();

    ground_plane_bvh_ = ground_bvh;
    RCLCPP_DEBUG(logger_, "Ground plane BVH created with %d triangles", ground_bvh->num_tris);
  } catch (const std::exception & e) {
    RCLCPP_WARN(logger_, "Failed to create ground plane BVH: %s", e.what());
    ground_plane_bvh_ = nullptr;
  } catch (...) {
    RCLCPP_WARN(logger_, "Failed to create ground plane BVH: unknown exception");
    ground_plane_bvh_ = nullptr;
  }
}

std::shared_ptr<FCLCollisionChecker::BVHModel> FCLCollisionChecker::shape_to_bvh(
  const TopoDS_Shape & shape) const
{
  std::vector<fcl::Vector3<FCLScalar>> vertices;
  std::vector<fcl::Triangle> triangles;

  int skipped_faces = 0;

  try {
    for (TopExp_Explorer exp(shape, TopAbs_FACE); exp.More(); exp.Next()) {
      TopoDS_Face face = TopoDS::Face(exp.Current());

      TopLoc_Location loc;
      Handle(Poly_Triangulation) triangulation = BRep_Tool::Triangulation(face, loc);

      if (triangulation.IsNull()) {
        skipped_faces++;
        continue;
      }

      gp_Trsf face_transform = loc.Transformation();
      int vertex_offset = static_cast<int>(vertices.size());

      int nb_nodes = triangulation->NbNodes();
      for (int i = 1; i <= nb_nodes; ++i) {
        gp_Pnt p = triangulation->Node(i);
        p.Transform(face_transform);
        vertices.emplace_back(p.X(), p.Y(), p.Z());
      }

      int nb_triangles = triangulation->NbTriangles();
      for (int i = 1; i <= nb_triangles; ++i) {
        const Poly_Triangle & tri = triangulation->Triangle(i);

        int n1, n2, n3;
        tri.Get(n1, n2, n3);

        // Validate indices are in valid range
        if (n1 < 1 || n1 > nb_nodes || n2 < 1 || n2 > nb_nodes || n3 < 1 || n3 > nb_nodes) {
          RCLCPP_DEBUG(logger_, "Invalid triangle indices: %d, %d, %d (max: %d)",
            n1, n2, n3, nb_nodes);
          continue;
        }

        int v1 = vertex_offset + n1 - 1;
        int v2 = vertex_offset + n2 - 1;
        int v3 = vertex_offset + n3 - 1;

        // Swap vertices for reversed faces to maintain consistent winding order
        if (face.Orientation() == TopAbs_REVERSED) {
          std::swap(v2, v3);
        }

        triangles.emplace_back(v1, v2, v3);
      }
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger_, "Exception during BVH conversion: %s", e.what());
    return nullptr;
  } catch (...) {
    RCLCPP_ERROR(logger_, "Unknown exception during BVH conversion");
    return nullptr;
  }

  if (skipped_faces > 0) {
    RCLCPP_DEBUG(logger_, "Skipped %d non-triangulated faces", skipped_faces);
  }

  if (vertices.empty() || triangles.empty()) {
    RCLCPP_DEBUG(logger_, "Shape has no triangulation data");
    return nullptr;
  }

  try {
    auto bvh = std::make_shared<BVHModel>();
    bvh->beginModel();
    bvh->addSubModel(vertices, triangles);
    bvh->endModel();

    if (bvh->num_tris == 0) {
      RCLCPP_DEBUG(logger_, "BVH model has no triangles after build");
      return nullptr;
    }

    return bvh;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger_, "Exception during BVH build: %s", e.what());
    return nullptr;
  } catch (...) {
    RCLCPP_ERROR(logger_, "Unknown exception during BVH build");
    return nullptr;
  }
}

FCLCollisionChecker::Transform3 FCLCollisionChecker::to_fcl_transform(const gp_Trsf & trsf) const
{
  // Extract rotation matrix
  Eigen::Matrix3d rotation;
  rotation(0, 0) = trsf.Value(1, 1);
  rotation(0, 1) = trsf.Value(1, 2);
  rotation(0, 2) = trsf.Value(1, 3);
  rotation(1, 0) = trsf.Value(2, 1);
  rotation(1, 1) = trsf.Value(2, 2);
  rotation(1, 2) = trsf.Value(2, 3);
  rotation(2, 0) = trsf.Value(3, 1);
  rotation(2, 1) = trsf.Value(3, 2);
  rotation(2, 2) = trsf.Value(3, 3);

  // Extract translation
  Eigen::Vector3d translation(
    trsf.TranslationPart().X(),
    trsf.TranslationPart().Y(),
    trsf.TranslationPart().Z());

  Transform3 transform = Transform3::Identity();
  transform.linear() = rotation;
  transform.translation() = translation;

  return transform;
}

void FCLCollisionChecker::compute_finger_transforms(
  double grip_distance,
  Transform3 & finger_1_transform,
  Transform3 & finger_2_transform) const
{
  // Each finger moves half the grip distance along its axis
  double half_opening = grip_distance / 2.0;

  Eigen::Vector3d translation_1 = finger_1_axis_ * half_opening;
  Eigen::Vector3d translation_2 = finger_2_axis_ * half_opening;

  finger_1_transform = Transform3::Identity();
  finger_1_transform.translation() = translation_1;

  finger_2_transform = Transform3::Identity();
  finger_2_transform.translation() = translation_2;
}

bool FCLCollisionChecker::check_gripper_collision(
  const gp_Trsf & gripper_transform,
  double grip_distance,
  const std::shared_ptr<BVHModel> & target_bvh,
  double tolerance) const
{
  if (!target_bvh) {
    RCLCPP_DEBUG(logger_, "Null target BVH in collision check");
    return false;
  }

  // Convert gripper transform to FCL
  Transform3 base_transform = to_fcl_transform(gripper_transform);

  // Compute finger offsets for grip distance
  Transform3 finger_1_offset, finger_2_offset;
  compute_finger_transforms(grip_distance, finger_1_offset, finger_2_offset);

  // Combined transforms: gripper_transform * finger_offset
  Transform3 finger_1_world = base_transform * finger_1_offset;
  Transform3 finger_2_world = base_transform * finger_2_offset;

  // Create collision objects
  CollisionObject finger_1_obj(finger_1_bvh_, finger_1_world);
  CollisionObject finger_2_obj(finger_2_bvh_, finger_2_world);
  CollisionObject base_obj(base_bvh_, base_transform);
  CollisionObject target_obj(target_bvh, Transform3::Identity());

  // Collision request/result
  fcl::CollisionRequest<FCLScalar> request;
  request.enable_contact = false;  // We just need yes/no
  fcl::CollisionResult<FCLScalar> result;

  // If tolerance is positive, we need distance checking instead
  if (tolerance > 0) {
    fcl::DistanceRequest<FCLScalar> dist_request;
    fcl::DistanceResult<FCLScalar> dist_result;

    // Check finger 1
    dist_result.clear();
    fcl::distance(&finger_1_obj, &target_obj, dist_request, dist_result);
    if (dist_result.min_distance < tolerance) {
      return true;
    }

    // Check finger 2
    dist_result.clear();
    fcl::distance(&finger_2_obj, &target_obj, dist_request, dist_result);
    if (dist_result.min_distance < tolerance) {
      return true;
    }

    // Check base
    dist_result.clear();
    fcl::distance(&base_obj, &target_obj, dist_request, dist_result);
    if (dist_result.min_distance < tolerance) {
      return true;
    }

    return false;
  }

  // Zero tolerance: use collision detection
  // Check finger 1
  result.clear();
  fcl::collide(&finger_1_obj, &target_obj, request, result);
  if (result.isCollision()) {
    return true;
  }

  // Check finger 2
  result.clear();
  fcl::collide(&finger_2_obj, &target_obj, request, result);
  if (result.isCollision()) {
    return true;
  }

  // Check base
  result.clear();
  fcl::collide(&base_obj, &target_obj, request, result);
  if (result.isCollision()) {
    return true;
  }

  return false;
}

double FCLCollisionChecker::compute_gripper_distance(
  const gp_Trsf & gripper_transform,
  double grip_distance,
  const std::shared_ptr<BVHModel> & target_bvh) const
{
  if (!target_bvh) {
    return std::numeric_limits<double>::max();
  }

  // Convert gripper transform to FCL
  Transform3 base_transform = to_fcl_transform(gripper_transform);

  // Compute finger offsets for grip distance
  Transform3 finger_1_offset, finger_2_offset;
  compute_finger_transforms(grip_distance, finger_1_offset, finger_2_offset);

  // Combined transforms
  Transform3 finger_1_world = base_transform * finger_1_offset;
  Transform3 finger_2_world = base_transform * finger_2_offset;

  // Create collision objects
  CollisionObject finger_1_obj(finger_1_bvh_, finger_1_world);
  CollisionObject finger_2_obj(finger_2_bvh_, finger_2_world);
  CollisionObject base_obj(base_bvh_, base_transform);
  CollisionObject target_obj(target_bvh, Transform3::Identity());

  fcl::DistanceRequest<FCLScalar> request;
  fcl::DistanceResult<FCLScalar> result;

  double min_distance = std::numeric_limits<double>::max();

  // Check finger 1
  result.clear();
  fcl::distance(&finger_1_obj, &target_obj, request, result);
  min_distance = std::min(min_distance, result.min_distance);

  // Check finger 2
  result.clear();
  fcl::distance(&finger_2_obj, &target_obj, request, result);
  min_distance = std::min(min_distance, result.min_distance);

  // Check base
  result.clear();
  fcl::distance(&base_obj, &target_obj, request, result);
  min_distance = std::min(min_distance, result.min_distance);

  return min_distance;
}

bool FCLCollisionChecker::collides_with_primary(
  const gp_Trsf & gripper_transform,
  double grip_distance,
  double tolerance) const
{
  if (!valid_) {
    RCLCPP_WARN(logger_, "Collision checker not valid, assuming no collision");
    return false;
  }

  try {
    return check_gripper_collision(gripper_transform, grip_distance, primary_bvh_, tolerance);
  } catch (const std::exception & e) {
    RCLCPP_WARN(logger_, "Exception in primary collision check: %s", e.what());
    return true;
  } catch (...) {
    RCLCPP_WARN(logger_, "Unknown exception in primary collision check");
    return true;
  }
}

bool FCLCollisionChecker::collides_with_exclusions(
  const gp_Trsf & gripper_transform,
  double grip_distance,
  double tolerance) const
{
  if (!valid_ || exclusion_bvhs_.empty()) {
    return false;
  }

  try {
    for (const auto & exclusion_bvh : exclusion_bvhs_) {
      if (check_gripper_collision(gripper_transform, grip_distance, exclusion_bvh, tolerance)) {
        return true;
      }
    }
    return false;
  } catch (const std::exception & e) {
    RCLCPP_WARN(logger_, "Exception in exclusion collision check: %s", e.what());
    return true;
  } catch (...) {
    RCLCPP_WARN(logger_, "Unknown exception in exclusion collision check");
    return true;
  }
}

bool FCLCollisionChecker::collides_with_secondaries(
  const gp_Trsf & gripper_transform,
  double grip_distance,
  double tolerance) const
{
  if (!valid_) {
    return false;
  }

  try {
    // Check ground plane collision if present
    if (ground_plane_bvh_) {
      if (check_gripper_collision(gripper_transform, grip_distance, ground_plane_bvh_, tolerance)) {
        return true;
      }
    }

    // Check secondary shapes
    for (const auto & secondary_bvh : secondary_bvhs_) {
      if (check_gripper_collision(gripper_transform, grip_distance, secondary_bvh, tolerance)) {
        return true;
      }
    }
    return false;
  } catch (const std::exception & e) {
    RCLCPP_WARN(logger_, "Exception in secondary collision check: %s", e.what());
    return true;
  } catch (...) {
    RCLCPP_WARN(logger_, "Unknown exception in secondary collision check");
    return true;
  }
}

double FCLCollisionChecker::distance_to_primary(
  const gp_Trsf & gripper_transform,
  double grip_distance) const
{
  if (!valid_) {
    return std::numeric_limits<double>::max();
  }

  try {
    return compute_gripper_distance(gripper_transform, grip_distance, primary_bvh_);
  } catch (const std::exception & e) {
    RCLCPP_WARN(logger_, "Exception in distance computation: %s", e.what());
    return std::numeric_limits<double>::max();
  } catch (...) {
    RCLCPP_WARN(logger_, "Unknown exception in distance computation");
    return std::numeric_limits<double>::max();
  }
}

bool FCLCollisionChecker::is_valid() const
{
  return valid_;
}

}  // namespace geometry
}  // namespace hold_and_weld_gripper_sampler
