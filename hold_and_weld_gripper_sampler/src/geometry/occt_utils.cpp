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

#include "hold_and_weld_gripper_sampler/geometry/occt_utils.hpp"

#include <cmath>
#include <limits>
#include <optional>
#include <stdexcept>

#include <rclcpp/rclcpp.hpp>

#include <BRepBuilderAPI_Transform.hxx>
#include <BRepExtrema_DistShapeShape.hxx>
#include <BRepGProp.hxx>
#include <BRepTools.hxx>
#include <BRep_Tool.hxx>

#include <GeomAPI_ProjectPointOnSurf.hxx>
#include <GeomLProp_SLProps.hxx>
#include <Geom_Surface.hxx>
#include <GProp_GProps.hxx>
#include <Standard_Failure.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <TopoDS.hxx>

namespace hold_and_weld_gripper_sampler
{
namespace geometry
{

static const rclcpp::Logger logger_ = rclcpp::get_logger("gripper_sampler");

Eigen::Vector3d to_eigen(const gp_Pnt & pnt)
{
  return Eigen::Vector3d(pnt.X(), pnt.Y(), pnt.Z());
}

gp_Pnt to_occt_point(const Eigen::Vector3d & vec)
{
  return gp_Pnt(vec.x(), vec.y(), vec.z());
}

Eigen::Vector3d to_eigen(const gp_Vec & vec)
{
  return Eigen::Vector3d(vec.X(), vec.Y(), vec.Z());
}

gp_Vec to_occt_vec(const Eigen::Vector3d & vec)
{
  return gp_Vec(vec.x(), vec.y(), vec.z());
}

Eigen::Vector3d to_eigen(const gp_Dir & dir)
{
  return Eigen::Vector3d(dir.X(), dir.Y(), dir.Z());
}

gp_Trsf create_transform(
  const Eigen::Vector3d & translation,
  const Eigen::Quaterniond & quaternion)
{
  gp_Trsf rot_transform;
  rot_transform.SetRotation(
    gp_Quaternion(quaternion.x(), quaternion.y(), quaternion.z(), quaternion.w()));

  gp_Trsf trans_transform;
  trans_transform.SetTranslation(gp_Vec(translation.x(), translation.y(), translation.z()));

  return trans_transform * rot_transform;
}

TopoDS_Shape apply_transform(
  const TopoDS_Shape & shape,
  const gp_Trsf & transform)
{
  try {
    BRepBuilderAPI_Transform transformer(shape, transform, Standard_True);
    if (!transformer.IsDone()) {
      throw std::runtime_error("Failed to apply transform: IsDone() returned false");
    }
    return transformer.Shape();
  } catch (Standard_Failure & e) {
    throw std::runtime_error(
      std::string("Failed to apply transform: ") + e.GetMessageString());
  }
}

gp_Vec extract_surface_normal(const TopoDS_Face & face)
{
  try {
    Handle(Geom_Surface) surf = BRep_Tool::Surface(face);
    if (surf.IsNull()) {
      throw std::runtime_error("Failed to extract surface from face");
    }

    Standard_Real u_min, u_max, v_min, v_max;
    BRepTools::UVBounds(face, u_min, u_max, v_min, v_max);

    GeomLProp_SLProps props(
      surf, (u_min + u_max) / 2.0, (v_min + v_max) / 2.0, 1, 1e-6);

    if (!props.IsNormalDefined()) {
      throw std::runtime_error("Surface normal not defined at face center");
    }

    gp_Vec normal = props.Normal();
    if (face.Orientation() == TopAbs_REVERSED) {
      normal.Reverse();
    }

    if (normal.Magnitude() < 1e-9) {
      throw std::runtime_error("Surface normal has near-zero magnitude");
    }
    normal.Normalize();

    return normal;
  } catch (Standard_Failure & e) {
    throw std::runtime_error(
      std::string("Failed to extract surface normal: ") + e.GetMessageString());
  }
}

gp_Pnt extract_surface_center(const TopoDS_Face & face)
{
  try {
    GProp_GProps props;
    BRepGProp::SurfaceProperties(face, props);
    return props.CentreOfMass();
  } catch (Standard_Failure & e) {
    throw std::runtime_error(
      std::string("Failed to extract surface center: ") + e.GetMessageString());
  }
}

bool has_inner_holes(const TopoDS_Face & face)
{
  int wire_count = 0;
  for (TopExp_Explorer exp(face, TopAbs_WIRE); exp.More(); exp.Next()) {
    wire_count++;
  }
  return wire_count > 1;
}

std::vector<Eigen::Vector3d> extract_corners_from_wire(const TopoDS_Wire & wire)
{
  try {
    TopTools_IndexedMapOfShape vertex_map;
    TopExp::MapShapes(wire, TopAbs_VERTEX, vertex_map);

    std::vector<Eigen::Vector3d> corners;
    corners.reserve(vertex_map.Extent());

    for (int i = 1; i <= vertex_map.Extent(); ++i) {
      gp_Pnt point = BRep_Tool::Pnt(TopoDS::Vertex(vertex_map(i)));
      corners.emplace_back(point.X(), point.Y(), point.Z());
    }

    return corners;
  } catch (Standard_Failure & e) {
    throw std::runtime_error(
      std::string("Failed to extract corners from wire: ") + e.GetMessageString());
  }
}

Eigen::Vector3d extract_translation(const gp_Trsf & transform)
{
  gp_XYZ t = transform.TranslationPart();
  return Eigen::Vector3d(t.X(), t.Y(), t.Z());
}

Eigen::Quaterniond extract_quaternion(const gp_Trsf & transform)
{
  gp_Mat r = transform.VectorialPart();
  Eigen::Matrix3d eigen_rot;
  eigen_rot(0, 0) = r.Value(1, 1); eigen_rot(0, 1) = r.Value(1, 2); eigen_rot(0, 2) = r.Value(1, 3);
  eigen_rot(1, 0) = r.Value(2, 1); eigen_rot(1, 1) = r.Value(2, 2); eigen_rot(1, 2) = r.Value(2, 3);
  eigen_rot(2, 0) = r.Value(3, 1); eigen_rot(2, 1) = r.Value(3, 2); eigen_rot(2, 2) = r.Value(3, 3);
  return Eigen::Quaterniond(eigen_rot).normalized();
}

double face_min_distance(const TopoDS_Face & face_1, const TopoDS_Face & face_2)
{
  if (face_1.IsNull() || face_2.IsNull()) {
    return std::numeric_limits<double>::max();
  }

  BRepExtrema_DistShapeShape dist(face_1, face_2);
  dist.Perform();

  if (dist.NbSolution() > 0) {
    return dist.Value();
  }

  return std::numeric_limits<double>::max();
}

std::optional<gp_Vec> surface_normal_at_point(const gp_Pnt & point, const TopoDS_Face & face)
{
  if (face.IsNull()) {
    return std::nullopt;
  }

  Handle(Geom_Surface) surf = BRep_Tool::Surface(face);

  GeomAPI_ProjectPointOnSurf projector(point, surf);

  double u, v;
  if (projector.NbPoints() == 0) {
    Standard_Real u_min, u_max, v_min, v_max;
    BRepTools::UVBounds(face, u_min, u_max, v_min, v_max);
    u = (u_min + u_max) / 2.0;
    v = (v_min + v_max) / 2.0;
  } else {
    projector.Parameters(1, u, v);
  }

  GeomLProp_SLProps props(surf, u, v, 1, 1e-6);

  if (!props.IsNormalDefined()) {
    return std::nullopt;
  }

  gp_Vec normal = props.Normal();
  if (face.Orientation() == TopAbs_REVERSED) {
    normal.Reverse();
  }

  return normal;
}

}  // namespace geometry
}  // namespace hold_and_weld_gripper_sampler
