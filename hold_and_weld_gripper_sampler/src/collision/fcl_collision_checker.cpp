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

#include <limits>

#include <BRepMesh_IncrementalMesh.hxx>
#include <IMeshTools_Parameters.hxx>
#include <BRep_Tool.hxx>
#include <Poly_Triangulation.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Face.hxx>
#include <rclcpp/rclcpp.hpp>

#include "hold_and_weld_gripper_sampler/collision/fcl_collision_checker.hpp"
#include "hold_and_weld_gripper_sampler/collision/embree_mesh_query.hpp"

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
  rest_gap_(0.0),
  linear_deflection_(linear_deflection),
  valid_(false)
{
  finger_1_bvh_ = shape_to_bvh(gripper.finger_1);
  finger_2_bvh_ = shape_to_bvh(gripper.finger_2);
  base_bvh_ = shape_to_bvh(gripper.base);
  primary_bvh_ = shape_to_bvh(primary_shape);

  if (!primary_shape.IsNull()) {
    try {
      embree_primary_ = std::make_shared<EmbreeMeshQuery>(primary_shape, linear_deflection);
      RCLCPP_INFO(logger_,
        "EmbreeMeshQuery built: %u triangles, %u vertices",
        embree_primary_->num_triangles(), embree_primary_->num_vertices());
    } catch (const std::exception & e) {
      RCLCPP_ERROR(logger_, "EmbreeMeshQuery construction failed: %s", e.what());
      embree_primary_ = nullptr;
    }
  }

  // rest_gap_: gap between finger inner faces at closed position, measured along finger_2_axis_
  auto proj_on_axis = [](
    const std::shared_ptr<BVHModel> & bvh,
    const Eigen::Vector3d & axis,
    bool find_min) -> double
    {
      if (!bvh || bvh->num_vertices == 0) {return 0.0;}
      double result = find_min ? std::numeric_limits<double>::max() :
        -std::numeric_limits<double>::max();
      for (int i = 0; i < bvh->num_vertices; ++i) {
        double projection = Eigen::Vector3d(
        bvh->vertices[i][0],
        bvh->vertices[i][1],
        bvh->vertices[i][2]).dot(axis);
        result = find_min ? std::min(result, projection) : std::max(result, projection);
      }
      return result;
    };

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

  if (!embree_primary_ || !embree_primary_->is_valid()) {
    RCLCPP_WARN(logger_,
      "Embree primary query not available — Phase-0 containment checks will be skipped");
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
    add_ground_plane(Eigen::Vector3d(0.0, 0.0, 1.0), ground_z);
  }
}

void FCLCollisionChecker::add_exclusion_volumes(
  const std::vector<TopoDS_Shape> & exclusion_volumes)
{
  for (size_t i = 0; i < exclusion_volumes.size(); ++i) {
    auto bvh = shape_to_bvh(exclusion_volumes[i]);
    if (bvh) {
      exclusion_bvhs_.push_back(bvh);
    } else {
      RCLCPP_ERROR(logger_,
        "add_exclusion_volumes: failed to build BVH for exclusion volume [%zu] — "
        "this obstacle will NOT be checked for collisions", i);
    }
    // Build Embree scene for Phase 0 point-in-solid containment (push nullptr on
    // failure so indices stay in sync with exclusion_bvhs_).
    try {
      embree_exclusions_.push_back(
        std::make_shared<EmbreeMeshQuery>(exclusion_volumes[i], linear_deflection_));
    } catch (const std::exception & e) {
      RCLCPP_WARN(logger_,
        "add_exclusion_volumes: Embree build failed for exclusion [%zu]: %s — "
        "Phase 0 containment disabled for this volume", i, e.what());
      embree_exclusions_.push_back(nullptr);
    }
  }

  const size_t n = exclusion_bvhs_.size();
  stats_.exc_base = std::vector<uint64_t>(n);
  stats_.exc_f1 = std::vector<uint64_t>(n);
  stats_.exc_f2 = std::vector<uint64_t>(n);
}

void FCLCollisionChecker::add_secondary_shapes(
  const std::vector<TopoDS_Shape> & secondary_shapes)
{
  for (size_t i = 0; i < secondary_shapes.size(); ++i) {
    auto bvh = shape_to_bvh(secondary_shapes[i]);
    if (bvh) {
      secondary_bvhs_.push_back(bvh);
    } else {
      RCLCPP_ERROR(logger_,
        "add_secondary_shapes: failed to build BVH for secondary shape [%zu] — "
        "this obstacle will NOT be checked for collisions", i);
    }
    // Build Embree scene for Phase 0 point-in-solid containment (push nullptr on
    // failure so indices stay in sync with secondary_bvhs_).
    try {
      embree_secondaries_.push_back(
        std::make_shared<EmbreeMeshQuery>(secondary_shapes[i], linear_deflection_));
    } catch (const std::exception & e) {
      RCLCPP_WARN(logger_,
        "add_secondary_shapes: Embree build failed for secondary [%zu]: %s — "
        "Phase 0 containment disabled for this shape", i, e.what());
      embree_secondaries_.push_back(nullptr);
    }
  }

  const size_t n = secondary_bvhs_.size();
  stats_.sec_base = std::vector<uint64_t>(n);
  stats_.sec_f1 = std::vector<uint64_t>(n);
  stats_.sec_f2 = std::vector<uint64_t>(n);
}

void FCLCollisionChecker::add_ground_plane(
  const Eigen::Vector3d & normal,
  double plane_offset)
{
  double norm_mag = normal.norm();
  if (norm_mag < 1e-9) {
    RCLCPP_ERROR(logger_,
      "add_ground_plane: normal vector has near-zero magnitude — "
      "ground collision will NOT be checked");
    ground_halfspace_ = nullptr;
    return;
  }

  Eigen::Vector3d unit_normal = normal / norm_mag;

  // FCL Halfspace(n, d): solid is n·x < d — pass plane_offset directly, no negation needed.
  ground_halfspace_ = std::make_shared<Halfspace>(
    fcl::Vector3<FCLScalar>(unit_normal.x(), unit_normal.y(), unit_normal.z()),
    plane_offset);

  RCLCPP_DEBUG(logger_,
    "Ground halfspace set: normal=(%.4f,%.4f,%.4f) offset=%.4f",
    unit_normal.x(), unit_normal.y(), unit_normal.z(), plane_offset);
}

bool FCLCollisionChecker::collides_with_primary(
  const gp_Trsf & gripper_transform,
  double grip_distance,
  double tolerance) const
{
  if (!valid_) {return false;}
  return check_gripper_collision(
    gripper_transform, grip_distance, primary_bvh_, tolerance, TargetKind::Primary);
}

bool FCLCollisionChecker::collides_with_exclusions(
  const gp_Trsf & gripper_transform,
  double grip_distance,
  double tolerance) const
{
  if (!valid_) {return false;}
  for (size_t i = 0; i < exclusion_bvhs_.size(); ++i) {
    if (check_gripper_collision(
        gripper_transform, grip_distance, exclusion_bvhs_[i], tolerance,
        TargetKind::Exclusion, i))
    {
      return true;
    }
  }
  return false;
}

bool FCLCollisionChecker::collides_with_ground(
  const gp_Trsf & gripper_transform,
  double grip_distance,
  double tolerance) const
{
  if (!valid_ || !ground_halfspace_) {return false;}
  return check_gripper_collision_halfspace(gripper_transform, grip_distance, tolerance);
}

bool FCLCollisionChecker::collides_with_secondaries(
  const gp_Trsf & gripper_transform,
  double grip_distance,
  double tolerance) const
{
  if (!valid_) {return false;}
  for (size_t i = 0; i < secondary_bvhs_.size(); ++i) {
    if (check_gripper_collision(
        gripper_transform, grip_distance, secondary_bvhs_[i], tolerance,
        TargetKind::Secondary, i))
    {
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
    RCLCPP_WARN(logger_,
          "distance_to_primary: checker not valid or primary BVH missing — returning max distance");
    return std::numeric_limits<double>::max();
  }
  return compute_gripper_distance(gripper_transform, grip_distance, primary_bvh_);
}

bool FCLCollisionChecker::is_valid() const
{
  return valid_;
}

bool FCLCollisionChecker::has_ground_plane() const
{
  return ground_halfspace_ != nullptr;
}

void FCLCollisionChecker::log_collision_stats() const
{
  RCLCPP_INFO(logger_, "[FCL collision stats] total_checks=%lu", stats_.total_checks);
  RCLCPP_INFO(logger_, "  primary  : base=%lu  f1=%lu  f2=%lu",
    stats_.primary_base, stats_.primary_f1, stats_.primary_f2);
  RCLCPP_INFO(logger_, "  ground   : base=%lu  f1=%lu  f2=%lu",
    stats_.ground_base, stats_.ground_f1, stats_.ground_f2);

  for (size_t i = 0; i < stats_.exc_base.size(); ++i) {
    RCLCPP_INFO(logger_, "  exclusion[%zu]: base=%lu  f1=%lu  f2=%lu",
      i, stats_.exc_base[i], stats_.exc_f1[i], stats_.exc_f2[i]);
  }

  for (size_t i = 0; i < stats_.sec_base.size(); ++i) {
    RCLCPP_INFO(logger_, "  secondary[%zu]: base=%lu  f1=%lu  f2=%lu",
      i, stats_.sec_base[i], stats_.sec_f1[i], stats_.sec_f2[i]);
  }

  const int64_t gpass = stats_.ground_pass_count;
  const int64_t gfail = stats_.ground_fail_count;
  const double avg_z_pass = gpass > 0 ? (stats_.ground_sum_z_pass_um / 1e6 / gpass) : 0.0;
  const double avg_z_fail = gfail > 0 ? (stats_.ground_sum_z_fail_um / 1e6 / gfail) : 0.0;
  const double min_z_pass = stats_.ground_min_z_pass_um != INT64_MAX ?
    stats_.ground_min_z_pass_um / 1e6 : 0.0;
  RCLCPP_INFO(logger_, "  ground pass: count=%ld  avg_base_z=%.4fm  min_base_z=%.4fm",
    gpass, avg_z_pass, min_z_pass);
  RCLCPP_INFO(logger_, "  ground fail: count=%ld  avg_base_z=%.4fm",
    gfail, avg_z_fail);
}

std::shared_ptr<FCLCollisionChecker::BVHModel> FCLCollisionChecker::shape_to_bvh(
  const TopoDS_Shape & shape) const
{
  if (shape.IsNull()) {
    RCLCPP_DEBUG(logger_, "shape_to_bvh: shape is null — returning nullptr");
    return nullptr;
  }

  IMeshTools_Parameters mesh_params;
  mesh_params.Deflection = linear_deflection_;
  mesh_params.Angle = 0.5;
  mesh_params.DeflectionInterior = 0.01;  // 10mm interior deflection — forces flat face subdivision
  mesh_params.InParallel = true;

  try {
    BRepMesh_IncrementalMesh mesher(shape, mesh_params);
  } catch (const Standard_Failure & e) {
    RCLCPP_ERROR(logger_, "shape_to_bvh: meshing failed — %s", e.GetMessageString());
    return nullptr;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger_, "shape_to_bvh: meshing failed — %s", e.what());
    return nullptr;
  }

  std::vector<fcl::Vector3<FCLScalar>> vertices;
  std::vector<fcl::Triangle> triangles;

  int face_idx = 0;
  int faces_with_no_triangulation = 0;

  for (TopExp_Explorer exp(shape, TopAbs_FACE); exp.More(); exp.Next(), ++face_idx) {
    try {
      TopoDS_Face face = TopoDS::Face(exp.Current());
      TopLoc_Location loc;
      Handle(Poly_Triangulation) tri = BRep_Tool::Triangulation(face, loc);

      if (tri.IsNull()) {
        RCLCPP_DEBUG(logger_, "shape_to_bvh: face %d has no triangulation — skipping", face_idx);
        ++faces_with_no_triangulation;
        continue;
      }

      const gp_Trsf & trsf = loc.Transformation();

      const bool is_identity = loc.IsIdentity();
      RCLCPP_DEBUG(logger_,
        "shape_to_bvh: face %d — %d nodes, %d triangles, location_is_identity=%s, "
        "trsf_translation=(%.4f, %.4f, %.4f)",
        face_idx, tri->NbNodes(), tri->NbTriangles(),
        is_identity ? "YES" : "NO",
        trsf.Value(1, 4), trsf.Value(2, 4), trsf.Value(3, 4));

      size_t offset = vertices.size();

      for (int i = 1; i <= tri->NbNodes(); ++i) {
        gp_Pnt p = tri->Node(i).Transformed(trsf);
        vertices.emplace_back(p.X(), p.Y(), p.Z());
      }

      for (int i = 1; i <= tri->NbTriangles(); ++i) {
        int n1, n2, n3;
        tri->Triangle(i).Get(n1, n2, n3);
        if (exp.Current().Orientation() == TopAbs_REVERSED) {std::swap(n2, n3);}
        triangles.emplace_back(
          offset + n1 - 1,
          offset + n2 - 1,
          offset + n3 - 1);
      }
    } catch (const Standard_Failure & e) {
      RCLCPP_WARN(logger_, "shape_to_bvh: face %d failed — %s, skipping",
        face_idx, e.GetMessageString());
      ++faces_with_no_triangulation;
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
  RCLCPP_INFO(logger_,
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
  f1_tf.translation() = finger_1_axis_ * ((grip_distance + rest_gap_) / 2.0);
  f2_tf.translation() = finger_2_axis_ * ((grip_distance + rest_gap_) / 2.0);
}

bool FCLCollisionChecker::check_gripper_collision_halfspace(
  const gp_Trsf & gripper_transform,
  double grip_distance,
  double tolerance) const
{
  if (!ground_halfspace_) {return false;}

  ++stats_.total_checks;

  Transform3 base_tf = to_fcl_transform(gripper_transform);
  Transform3 f1_local, f2_local;
  compute_finger_transforms(grip_distance, f1_local, f2_local);
  Transform3 f1_tf = base_tf * f1_local;
  Transform3 f2_tf = base_tf * f2_local;

  std::shared_ptr<Halfspace> hs_for_check;
  if (tolerance > 0.0) {
    hs_for_check = std::make_shared<Halfspace>(
      ground_halfspace_->n,
      ground_halfspace_->d + static_cast<FCLScalar>(tolerance));
  } else {
    hs_for_check = ground_halfspace_;
  }

  CollisionObject hs_obj(hs_for_check, Transform3::Identity());

  fcl::CollisionRequest<FCLScalar> request;
  fcl::CollisionResult<FCLScalar> result;

  if (base_bvh_) {
    CollisionObject base_obj(base_bvh_, base_tf);
    fcl::collide(&base_obj, &hs_obj, request, result);
    if (result.isCollision()) {
      ++stats_.ground_base;
      ++stats_.ground_fail_count;
      stats_.ground_sum_z_fail_um += static_cast<int64_t>(base_tf.translation().z() * 1e6);
      return true;
    }
  }
  if (finger_1_bvh_) {
    result.clear();
    CollisionObject f1_obj(finger_1_bvh_, f1_tf);
    fcl::collide(&f1_obj, &hs_obj, request, result);
    if (result.isCollision()) {
      ++stats_.ground_f1;
      ++stats_.ground_fail_count;
      stats_.ground_sum_z_fail_um += static_cast<int64_t>(base_tf.translation().z() * 1e6);
      return true;
    }
  }
  if (finger_2_bvh_) {
    result.clear();
    CollisionObject f2_obj(finger_2_bvh_, f2_tf);
    fcl::collide(&f2_obj, &hs_obj, request, result);
    if (result.isCollision()) {
      ++stats_.ground_f2;
      ++stats_.ground_fail_count;
      stats_.ground_sum_z_fail_um += static_cast<int64_t>(base_tf.translation().z() * 1e6);
      return true;
    }
  }

  const int64_t z_um = static_cast<int64_t>(base_tf.translation().z() * 1e6);
  ++stats_.ground_pass_count;
  stats_.ground_sum_z_pass_um += z_um;
  if (z_um < stats_.ground_min_z_pass_um) {stats_.ground_min_z_pass_um = z_um;}

  return false;
}

bool FCLCollisionChecker::check_gripper_collision(
  const gp_Trsf & gripper_transform,
  double grip_distance,
  const std::shared_ptr<BVHModel> & target_bvh,
  double tolerance,
  TargetKind kind,
  size_t secondary_index) const
{
  if (!target_bvh) {return false;}

  ++stats_.total_checks;

  uint64_t * ctr_base = nullptr;
  uint64_t * ctr_f1 = nullptr;
  uint64_t * ctr_f2 = nullptr;

  switch (kind) {
    case TargetKind::Primary:
      ctr_base = &stats_.primary_base;
      ctr_f1 = &stats_.primary_f1;
      ctr_f2 = &stats_.primary_f2;
      break;
    case TargetKind::Ground:
      ctr_base = &stats_.ground_base;
      ctr_f1 = &stats_.ground_f1;
      ctr_f2 = &stats_.ground_f2;
      break;
    case TargetKind::Secondary:
      if (secondary_index < stats_.sec_base.size()) {
        ctr_base = &stats_.sec_base[secondary_index];
        ctr_f1 = &stats_.sec_f1[secondary_index];
        ctr_f2 = &stats_.sec_f2[secondary_index];
      }
      break;
    case TargetKind::Exclusion:
      if (secondary_index < stats_.exc_base.size()) {
        ctr_base = &stats_.exc_base[secondary_index];
        ctr_f1 = &stats_.exc_f1[secondary_index];
        ctr_f2 = &stats_.exc_f2[secondary_index];
      }
      break;
  }

  Transform3 base_tf = to_fcl_transform(gripper_transform);
  Transform3 f1_local, f2_local;
  compute_finger_transforms(grip_distance, f1_local, f2_local);

  Transform3 f1_tf = base_tf * f1_local;
  Transform3 f2_tf = base_tf * f2_local;

  // collide() first: distance() returns garbage when shapes intersect (GJK undefined on overlap).
  fcl::CollisionRequest<FCLScalar> col_req;
  fcl::DistanceRequest<FCLScalar> dist_req;

  CollisionObject target_obj(target_bvh, Transform3::Identity());

  std::shared_ptr<EmbreeMeshQuery> embree_for_check;
  switch (kind) {
    case TargetKind::Primary:
      embree_for_check = embree_primary_;
      break;
    case TargetKind::Secondary:
      if (secondary_index < embree_secondaries_.size()) {
        embree_for_check = embree_secondaries_[secondary_index];
      }
      break;
    case TargetKind::Exclusion:
      if (secondary_index < embree_exclusions_.size()) {
        embree_for_check = embree_exclusions_[secondary_index];
      }
      break;
    default:
      break;
  }
  const bool use_embree_containment = embree_for_check && embree_for_check->is_valid();

  auto check_part = [&](
    const std::shared_ptr<BVHModel> & part_bvh,
    const Transform3 & part_tf,
    uint64_t * ctr) -> bool
    {
      if (!part_bvh) {return false;}

      // Phase 0: FCL misses containment when one mesh is fully inside
      // another probe all vertices via Embree parity test.
      if (use_embree_containment) {
        for (int vi = 0; vi < part_bvh->num_vertices; ++vi) {
          Eigen::Vector3d local_v(
            part_bvh->vertices[vi][0],
            part_bvh->vertices[vi][1],
            part_bvh->vertices[vi][2]);
          Eigen::Vector3d world_v = part_tf * local_v;
          gp_Pnt probe(world_v[0], world_v[1], world_v[2]);
          if (embree_for_check->point_inside(probe)) {
            if (ctr) {++(*ctr);}
            return true;
          }
        }
      }

      CollisionObject part_obj(part_bvh, part_tf);

    // Phase 1: boolean collision (catches surface intersection)
      fcl::CollisionResult<FCLScalar> col_res;
      fcl::collide(&part_obj, &target_obj, col_req, col_res);
      if (col_res.isCollision()) {
        if (ctr) {++(*ctr);}
        return true;
      }

    // Phase 2: near-miss distance check
      if (tolerance > 0.0) {
        fcl::DistanceResult<FCLScalar> dist_res;
        fcl::distance(&part_obj, &target_obj, dist_req, dist_res);
        if (dist_res.min_distance < tolerance) {
          if (ctr) {++(*ctr);}
          return true;
        }
      }

      return false;
    };

  if (check_part(base_bvh_, base_tf, ctr_base)) {return true;}
  if (check_part(finger_1_bvh_, f1_tf, ctr_f1)) {return true;}
  if (check_part(finger_2_bvh_, f2_tf, ctr_f2)) {return true;}

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
    result.clear();
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
