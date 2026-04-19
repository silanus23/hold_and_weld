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

#include <BRepMesh_IncrementalMesh.hxx>
#include <BRep_Tool.hxx>
#include <Poly_Triangulation.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Face.hxx>
#include <rclcpp/rclcpp.hpp>

#include <limits>
#include <stack>

namespace hold_and_weld_gripper_sampler
{
namespace geometry
{

static const rclcpp::Logger logger_ = rclcpp::get_logger("gripper_sampler");

static bool ray_aabb(
  const fcl::Vector3<double> & ro,
  const fcl::Vector3<double> & inv_rd,
  const fcl::Vector3<double> & aabb_min,
  const fcl::Vector3<double> & aabb_max,
  double max_t)
{
  double tmin = 0.0;
  double tmax = max_t;

  for (int i = 0; i < 3; ++i) {
    double t1 = (aabb_min[i] - ro[i]) * inv_rd[i];
    double t2 = (aabb_max[i] - ro[i]) * inv_rd[i];
    if (t1 > t2) {std::swap(t1, t2);}
    tmin = std::max(tmin, t1);
    tmax = std::min(tmax, t2);
    if (tmin > tmax) {return false;}
  }
  return true;
}

// Möller–Trumbore ray-triangle intersection
static double ray_triangle(
  const fcl::Vector3<double> & ro,
  const fcl::Vector3<double> & rd,
  const fcl::Vector3<double> & v0,
  const fcl::Vector3<double> & v1,
  const fcl::Vector3<double> & v2)
{
  constexpr double kEps = 1e-8;
  fcl::Vector3<double> e1 = v1 - v0;
  fcl::Vector3<double> e2 = v2 - v0;
  fcl::Vector3<double> h = rd.cross(e2);
  double a = e1.dot(h);
  if (std::abs(a) < kEps) {return -1.0;}

  double f = 1.0 / a;
  fcl::Vector3<double> s = ro - v0;
  double u = f * s.dot(h);
  if (u < 0.0 || u > 1.0) {return -1.0;}

  fcl::Vector3<double> q = s.cross(e1);
  double v = f * rd.dot(q);
  if (v < 0.0 || u + v > 1.0) {return -1.0;}

  double t = f * e2.dot(q);
  return (t > kEps) ? t : -1.0;
}

// BVH traversal ray cast — returns closest hit t and triangle index, or -1
static double bvh_ray_cast(
  const FCLCollisionChecker::BVHModel & bvh,
  const fcl::Vector3<double> & ro,
  const fcl::Vector3<double> & rd,
  double max_distance,
  int & hit_tri_out)
{
  hit_tri_out = -1;
  double closest_t = max_distance;

  fcl::Vector3<double> inv_rd(
    std::abs(rd[0]) > 1e-12 ? 1.0 / rd[0] : std::numeric_limits<double>::max(),
    std::abs(rd[1]) > 1e-12 ? 1.0 / rd[1] : std::numeric_limits<double>::max(),
    std::abs(rd[2]) > 1e-12 ? 1.0 / rd[2] : std::numeric_limits<double>::max());

  // FCL BVHModel stores the root at index 0, not getNumBVs()-1.
  // Starting from the wrong node causes most of the tree to be skipped,
  // resulting in all rays missing even when the geometry is correctly placed.
  std::stack<int> stack;
  stack.push(0);

  while (!stack.empty()) {
    int node_idx = stack.top();
    stack.pop();

    const auto & node = bvh.getBV(node_idx);
    const auto & bv = node.bv;

    // Extract AABB from OBBRSS via its RSS component's AABB
    const auto & obb = bv.obb;
    fcl::Vector3<double> center = obb.To;
    fcl::Vector3<double> half_extents = obb.extent;
    fcl::Vector3<double> aabb_min, aabb_max;
    for (int i = 0; i < 3; ++i) {
      double r = 0.0;
      for (int j = 0; j < 3; ++j) {
        r += std::abs(obb.axis.col(j)[i]) * half_extents[j];
      }
      aabb_min[i] = center[i] - r;
      aabb_max[i] = center[i] + r;
    }

    if (!ray_aabb(ro, inv_rd, aabb_min, aabb_max, closest_t)) {
      continue;
    }

    if (node.isLeaf()) {
      int tri_idx = node.primitiveId();
      const auto & tri = bvh.tri_indices[tri_idx];
      const auto & v0 = bvh.vertices[tri[0]];
      const auto & v1 = bvh.vertices[tri[1]];
      const auto & v2 = bvh.vertices[tri[2]];

      double t = ray_triangle(ro, rd, v0, v1, v2);
      if (t > 0.0 && t < closest_t) {
        closest_t = t;
        hit_tri_out = tri_idx;
      }
    } else {
      stack.push(node.leftChild());
      stack.push(node.rightChild());
    }
  }

  return (hit_tri_out >= 0) ? closest_t : -1.0;
}

FCLCollisionChecker::FCLCollisionChecker(
  const ParsedGripper & gripper,
  const TopoDS_Shape & primary_shape,
  double linear_deflection)
: finger_1_axis_(gripper.finger_1_axis),
  finger_2_axis_(gripper.finger_2_axis),
  rest_gap_(0.0),
  linear_deflection_(linear_deflection),
  valid_(false)
{
  finger_1_bvh_ = shape_to_bvh(gripper.finger_1);
  finger_2_bvh_ = shape_to_bvh(gripper.finger_2);
  base_bvh_ = shape_to_bvh(gripper.base);
  primary_bvh_ = shape_to_bvh(primary_shape);

  // Compute rest_gap_: distance between inner faces of the two fingers at rest (closed position).
  // Using a shared grip axis (finger_2_axis_), find:
  //   - finger_2 inner face = minimum projection onto finger_2_axis (face closest to center)
  //   - finger_1 inner face = maximum projection onto finger_2_axis (face closest to center)
  // rest_gap_ = finger_2_inner - finger_1_inner
  auto proj_on_axis = [](
    const std::shared_ptr<BVHModel> & bvh,
    const Eigen::Vector3d & axis,
    bool find_min) -> double
  {
    if (!bvh || bvh->num_vertices == 0) {return 0.0;}
    double result = find_min ? std::numeric_limits<double>::max()
                             : -std::numeric_limits<double>::max();
    for (int i = 0; i < bvh->num_vertices; ++i) {
      double p = Eigen::Vector3d(
        bvh->vertices[i][0],
        bvh->vertices[i][1],
        bvh->vertices[i][2]).dot(axis);
      result = find_min ? std::min(result, p) : std::max(result, p);
    }
    return result;
  };

  // finger_2_axis_ points from center outward toward finger_2
  // finger_1 inner face: max projection onto finger_2_axis (highest Y for +Y axis)
  // finger_2 inner face: min projection onto finger_2_axis (lowest Y for +Y axis)
  double f1_inner = proj_on_axis(finger_1_bvh_, finger_2_axis_, false);
  double f2_inner = proj_on_axis(finger_2_bvh_, finger_2_axis_, true);
  rest_gap_ = f2_inner - f1_inner;
  if (rest_gap_ < 0.0) {rest_gap_ = 0.0;}


  if (finger_1_bvh_ && finger_2_bvh_ && base_bvh_ && primary_bvh_) {
    RCLCPP_INFO(logger_, "primary_bvh_ triangle count: %d", primary_bvh_->num_tris);
    valid_ = true;
  } else {
    RCLCPP_ERROR(logger_, "FCL checker: failed to build one or more BVH models");
  }
}

FCLCollisionChecker::FCLCollisionChecker(
  const ParsedGripper & gripper,
  const TopoDS_Shape & primary_shape,
  const std::vector<TopoDS_Shape> & exclusion_volumes,
  const std::vector<TopoDS_Shape> & secondary_shapes,
  bool enable_ground_plane,
  double ground_z,
  double linear_deflection)
: FCLCollisionChecker(gripper, primary_shape, linear_deflection)
{
  add_exclusion_volumes(exclusion_volumes);
  add_secondary_shapes(secondary_shapes);
  if (enable_ground_plane) {
    add_ground_plane(ground_z);
  }
}

void FCLCollisionChecker::add_exclusion_volumes(
  const std::vector<TopoDS_Shape> & exclusion_volumes)
{
  for (const auto & shape : exclusion_volumes) {
    auto bvh = shape_to_bvh(shape);
    if (bvh) {exclusion_bvhs_.push_back(bvh);}
  }
}

void FCLCollisionChecker::add_secondary_shapes(
  const std::vector<TopoDS_Shape> & secondary_shapes)
{
  for (const auto & shape : secondary_shapes) {
    auto bvh = shape_to_bvh(shape);
    if (bvh) {secondary_bvhs_.push_back(bvh);}
  }
}

void FCLCollisionChecker::add_ground_plane(
  double ground_z, double size, double thickness,
  double center_x, double center_y)
{
  // Build ground plane as a flat box BVH
  auto bvh = std::make_shared<BVHModel>();
  bvh->beginModel();

  double hx = size / 2.0;
  double hy = size / 2.0;
  // double hz = thickness / 2.0;
  double cx = center_x, cy = center_y;
  double zlo = ground_z - thickness;
  double zhi = ground_z;

  // 8 corners
  std::vector<fcl::Vector3<FCLScalar>> verts = {
    {cx - hx, cy - hy, zlo}, {cx + hx, cy - hy, zlo},
    {cx + hx, cy + hy, zlo}, {cx - hx, cy + hy, zlo},
    {cx - hx, cy - hy, zhi}, {cx + hx, cy - hy, zhi},
    {cx + hx, cy + hy, zhi}, {cx - hx, cy + hy, zhi}
  };

  // 12 triangles (2 per face)
  std::vector<fcl::Triangle> tris = {
    {0, 1, 2}, {0, 2, 3},  // bottom
    {4, 6, 5}, {4, 7, 6},  // top
    {0, 5, 1}, {0, 4, 5},  // front
    {2, 6, 7}, {2, 7, 3},  // back
    {0, 3, 7}, {0, 7, 4},  // left
    {1, 5, 6}, {1, 6, 2}   // right
  };

  bvh->addSubModel(verts, tris);
  bvh->endModel();

  ground_plane_bvh_ = bvh;
  RCLCPP_DEBUG(logger_, "Ground plane BVH built: 12 triangles");
}

bool FCLCollisionChecker::collides_with_primary(
  const gp_Trsf & gripper_transform,
  double grip_distance,
  double tolerance) const
{
  if (!valid_) {return false;}
  return check_gripper_collision(
    gripper_transform, grip_distance, primary_bvh_, tolerance);
}

bool FCLCollisionChecker::collides_with_exclusions(
  const gp_Trsf & gripper_transform,
  double grip_distance,
  double tolerance) const
{
  if (!valid_) {return false;}
  for (const auto & bvh : exclusion_bvhs_) {
    if (check_gripper_collision(gripper_transform, grip_distance, bvh, tolerance)) {
      return true;
    }
  }
  return false;
}

bool FCLCollisionChecker::collides_with_secondaries(
  const gp_Trsf & gripper_transform,
  double grip_distance,
  double tolerance) const
{
  if (!valid_) {return false;}

  if (ground_plane_bvh_) {
    if (check_gripper_collision(
        gripper_transform, grip_distance, ground_plane_bvh_, tolerance))
    {
      return true;
    }
  }

  for (const auto & bvh : secondary_bvhs_) {
    if (check_gripper_collision(gripper_transform, grip_distance, bvh, tolerance)) {
      return true;
    }
  }
  return false;
}

double FCLCollisionChecker::distance_to_primary(
  const gp_Trsf & gripper_transform,
  double grip_distance) const
{
  if (!valid_ || !primary_bvh_) {
    return std::numeric_limits<double>::max();
  }
  return compute_gripper_distance(gripper_transform, grip_distance, primary_bvh_);
}

bool FCLCollisionChecker::ray_hits_primary(
  const gp_Pnt & origin,
  const gp_Dir & direction,
  double max_distance,
  gp_Pnt & hit_point_out) const
{
  if (!valid_ || !primary_bvh_) {return false;}

  const fcl::Vector3<double> ro(origin.X(), origin.Y(), origin.Z());
  const fcl::Vector3<double> rd(direction.X(), direction.Y(), direction.Z());

  int hit_tri = -1;
  double t = bvh_ray_cast(*primary_bvh_, ro, rd, max_distance, hit_tri);

  if (hit_tri >= 0) {
    hit_point_out = gp_Pnt(
      origin.X() + direction.X() * t,
      origin.Y() + direction.Y() * t,
      origin.Z() + direction.Z() * t);
    return true;
  }
  return false;
}

bool FCLCollisionChecker::is_valid() const
{
  return valid_;
}

bool FCLCollisionChecker::has_ground_plane() const
{
  return ground_plane_bvh_ != nullptr;
}

std::shared_ptr<FCLCollisionChecker::BVHModel> FCLCollisionChecker::shape_to_bvh(
  const TopoDS_Shape & shape) const
{
  if (shape.IsNull()) {
    RCLCPP_DEBUG(logger_, "shape_to_bvh: shape is null — returning nullptr");
    return nullptr;
  }

  BRepMesh_IncrementalMesh mesher(shape, linear_deflection_);

  std::vector<fcl::Vector3<FCLScalar>> vertices;
  std::vector<fcl::Triangle> triangles;

  int face_idx = 0;
  int faces_with_no_triangulation = 0;

  for (TopExp_Explorer exp(shape, TopAbs_FACE); exp.More(); exp.Next(), ++face_idx) {
    TopoDS_Face face = TopoDS::Face(exp.Current());
    TopLoc_Location loc;
    Handle(Poly_Triangulation) tri = BRep_Tool::Triangulation(face, loc);

    if (tri.IsNull()) {
      RCLCPP_DEBUG(logger_, "shape_to_bvh: face %d has no triangulation — skipping", face_idx);
      ++faces_with_no_triangulation;
      continue;
    }

    const gp_Trsf & trsf = loc.Transformation();

    // Log whether this face carries a non-identity location transform.
    // A non-identity transform here means the mesh nodes are in a local
    // frame and need Transformed() to reach world coords — if this is
    // unexpectedly identity the BVH will be built at the wrong position.
    const bool is_identity = loc.IsIdentity();
    RCLCPP_DEBUG(logger_,
      "shape_to_bvh: face %d — %d nodes, %d triangles, location_is_identity=%s, "
      "trsf_translation=(%.4f, %.4f, %.4f)",
      face_idx, tri->NbNodes(), tri->NbTriangles(),
      is_identity ? "YES" : "NO",
      trsf.Value(1, 4), trsf.Value(2, 4), trsf.Value(3, 4));

    int offset = static_cast<int>(vertices.size());

    for (int i = 1; i <= tri->NbNodes(); ++i) {
      gp_Pnt p = tri->Node(i).Transformed(trsf);
      vertices.emplace_back(p.X(), p.Y(), p.Z());
    }

    for (int i = 1; i <= tri->NbTriangles(); ++i) {
      int n1, n2, n3;
      tri->Triangle(i).Get(n1, n2, n3);
      if (face.Orientation() == TopAbs_REVERSED) {std::swap(n2, n3);}
      triangles.emplace_back(
        offset + n1 - 1,
        offset + n2 - 1,
        offset + n3 - 1);
    }
  }

  if (vertices.empty() || triangles.empty()) {
    RCLCPP_WARN(logger_,
      "shape_to_bvh: no geometry collected (%zu vertices, %zu triangles, "
      "%d faces skipped) — returning nullptr",
      vertices.size(), triangles.size(), faces_with_no_triangulation);
    return nullptr;
  }

  // Compute AABB of all collected vertices so we can verify world position.
  fcl::Vector3<FCLScalar> vmin = vertices[0];
  fcl::Vector3<FCLScalar> vmax = vertices[0];
  for (const auto & v : vertices) {
    for (int i = 0; i < 3; ++i) {
      vmin[i] = std::min(vmin[i], v[i]);
      vmax[i] = std::max(vmax[i], v[i]);
    }
  }
  RCLCPP_DEBUG(logger_,
    "shape_to_bvh: built BVH — %zu vertices, %zu triangles, %d faces skipped. "
    "AABB x=[%.4f, %.4f] y=[%.4f, %.4f] z=[%.4f, %.4f]",
    vertices.size(), triangles.size(), faces_with_no_triangulation,
    vmin[0], vmax[0], vmin[1], vmax[1], vmin[2], vmax[2]);

  auto model = std::make_shared<BVHModel>();
  model->beginModel();
  model->addSubModel(vertices, triangles);
  model->endModel();

  return model;
}

FCLCollisionChecker::Transform3 FCLCollisionChecker::to_fcl_transform(
  const gp_Trsf & trsf) const
{
  Transform3 tf;
  tf.setIdentity();
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      tf.linear()(i, j) = trsf.Value(i + 1, j + 1);
    }
    tf.translation()(i) = trsf.Value(i + 1, 4);
  }
  return tf;
}

void FCLCollisionChecker::compute_finger_transforms(
  double grip_distance,
  Transform3 & f1_tf,
  Transform3 & f2_tf) const
{
  f1_tf.setIdentity();
  f2_tf.setIdentity();
  f1_tf.translation() = finger_1_axis_ * (grip_distance / 2.0 + rest_gap_);
  f2_tf.translation() = finger_2_axis_ * (grip_distance / 2.0 + rest_gap_);
}

bool FCLCollisionChecker::check_gripper_collision(
  const gp_Trsf & gripper_transform,
  double grip_distance,
  const std::shared_ptr<BVHModel> & target_bvh,
  double tolerance) const
{
  if (!target_bvh) {return false;}

  Transform3 base_tf = to_fcl_transform(gripper_transform);
  Transform3 f1_local, f2_local;
  compute_finger_transforms(grip_distance, f1_local, f2_local);

  Transform3 f1_tf = base_tf * f1_local;
  Transform3 f2_tf = base_tf * f2_local;

  fcl::CollisionRequest<FCLScalar> request;
  fcl::CollisionResult<FCLScalar> result;

  CollisionObject target_obj(target_bvh, Transform3::Identity());

  if (tolerance > 0.0) {
    // Distance-based check
    fcl::DistanceRequest<FCLScalar> dist_req;
    fcl::DistanceResult<FCLScalar> dist_res;

    if (base_bvh_) {
      CollisionObject base_obj(base_bvh_, base_tf);
      fcl::distance(&base_obj, &target_obj, dist_req, dist_res);
      if (dist_res.min_distance < tolerance) {return true;}
    }
    if (finger_1_bvh_) {
      dist_res.clear();
      CollisionObject f1_obj(finger_1_bvh_, f1_tf);
      fcl::distance(&f1_obj, &target_obj, dist_req, dist_res);
      if (dist_res.min_distance < tolerance) {return true;}
    }
    if (finger_2_bvh_) {
      dist_res.clear();
      CollisionObject f2_obj(finger_2_bvh_, f2_tf);
      fcl::distance(&f2_obj, &target_obj, dist_req, dist_res);
      if (dist_res.min_distance < tolerance) {return true;}
    }
  } else {
    // Boolean collision check
    if (base_bvh_) {
      CollisionObject base_obj(base_bvh_, base_tf);
      fcl::collide(&base_obj, &target_obj, request, result);
      if (result.isCollision()) {return true;}
    }
    if (finger_1_bvh_) {
      result.clear();
      CollisionObject f1_obj(finger_1_bvh_, f1_tf);
      fcl::collide(&f1_obj, &target_obj, request, result);
      if (result.isCollision()) {return true;}
    }
    if (finger_2_bvh_) {
      result.clear();
      CollisionObject f2_obj(finger_2_bvh_, f2_tf);
      fcl::collide(&f2_obj, &target_obj, request, result);
      if (result.isCollision()) {return true;}
    }
  }

  return false;
}

double FCLCollisionChecker::compute_gripper_distance(
  const gp_Trsf & gripper_transform,
  double grip_distance,
  const std::shared_ptr<BVHModel> & target_bvh) const
{
  if (!target_bvh) {return std::numeric_limits<double>::max();}

  Transform3 base_tf = to_fcl_transform(gripper_transform);
  Transform3 f1_local, f2_local;
  compute_finger_transforms(grip_distance, f1_local, f2_local);

  Transform3 f1_tf = base_tf * f1_local;
  Transform3 f2_tf = base_tf * f2_local;

  fcl::DistanceRequest<FCLScalar> request;
  fcl::DistanceResult<FCLScalar> result;
  double min_dist = std::numeric_limits<double>::max();

  CollisionObject target_obj(target_bvh, Transform3::Identity());

  if (base_bvh_) {
    CollisionObject base_obj(base_bvh_, base_tf);
    fcl::distance(&base_obj, &target_obj, request, result);
    min_dist = std::min(min_dist, result.min_distance);
  }
  if (finger_1_bvh_) {
    result.clear();
    CollisionObject f1_obj(finger_1_bvh_, f1_tf);
    fcl::distance(&f1_obj, &target_obj, request, result);
    min_dist = std::min(min_dist, result.min_distance);
  }
  if (finger_2_bvh_) {
    result.clear();
    CollisionObject f2_obj(finger_2_bvh_, f2_tf);
    fcl::distance(&f2_obj, &target_obj, request, result);
    min_dist = std::min(min_dist, result.min_distance);
  }

  return min_dist;
}

}  // namespace geometry
}  // namespace hold_and_weld_gripper_sampler
