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
#include <GeomAbs_CurveType.hxx>
#include <GeomAdaptor_Curve.hxx>
#include <GeomAPI_ProjectPointOnSurf.hxx>
#include <GeomLProp_SLProps.hxx>
#include <Geom_Surface.hxx>
#include <GProp_GProps.hxx>
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
  try {
    gp_Trsf transform;

    transform.SetTranslation(gp_Vec(translation.x(), translation.y(), translation.z()));

    gp_Quaternion quat(quaternion.x(), quaternion.y(), quaternion.z(), quaternion.w());
    gp_Trsf rot_transform;
    rot_transform.SetRotation(quat);

    gp_Trsf combined_transform = transform * rot_transform;

    return combined_transform;
  } catch (const std::exception & e) {
    throw std::runtime_error("Failed to create transform: " + std::string(e.what()));
  } catch (...) {
    throw std::runtime_error("Failed to create transform: unknown error");
  }
}

TopoDS_Shape apply_transform(
  const TopoDS_Shape & shape,
  const gp_Trsf & transform)
{
  try {
    BRepBuilderAPI_Transform transformer(shape, transform, Standard_True);
    return transformer.Shape();
  } catch (const std::exception & e) {
    throw std::runtime_error("Failed to apply transform: " + std::string(e.what()));
  } catch (...) {
    throw std::runtime_error("Failed to apply transform: unknown error");
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

    Standard_Real u_mid = (u_min + u_max) / 2.0;
    Standard_Real v_mid = (v_min + v_max) / 2.0;

    // Parameters: surface, u, v, derivative_order=1, tolerance=1e-6
    GeomLProp_SLProps props(surf, u_mid, v_mid, 1, 1e-6);

    if (!props.IsNormalDefined()) {
      throw std::runtime_error("Surface normal is not defined at center point");
    }

    gp_Vec normal = props.Normal();

    if (face.Orientation() == TopAbs_REVERSED) {
      normal.Reverse();
    }

    if (normal.Magnitude() < 1e-10) {
      throw std::runtime_error("Surface normal has near-zero magnitude");
    }
    normal.Normalize();

    return normal;
  } catch (const std::exception & e) {
    throw std::runtime_error("Failed to extract surface normal: " + std::string(e.what()));
  } catch (...) {
    throw std::runtime_error("Failed to extract surface normal: unknown error");
  }
}

gp_Pnt extract_surface_center(const TopoDS_Face & face)
{
  try {
    GProp_GProps props;
    BRepGProp::SurfaceProperties(face, props);
    return props.CentreOfMass();
  } catch (const std::exception & e) {
    throw std::runtime_error("Failed to extract surface center: " + std::string(e.what()));
  } catch (...) {
    throw std::runtime_error("Failed to extract surface center: unknown error");
  }
}

double dot_product(const gp_Vec & v1, const gp_Vec & v2)
{
  return v1.Dot(v2);
}

bool are_normals_aligned(
  const gp_Vec & normal1,
  const gp_Vec & normal2,
  double tolerance)
{
  // Normalize vectors
  gp_Vec n1 = normal1.Normalized();
  gp_Vec n2 = normal2.Normalized();

  // Compute dot product
  double dot = n1.Dot(n2);

  // For aligned normals: dot ≈ 1
  // angle = acos(dot)
  // We want angle < tolerance
  // So: acos(dot) < tolerance
  // Therefore: dot > cos(tolerance)

  double threshold = std::cos(tolerance);
  return dot > threshold;
}

bool are_normals_opposite(
  const gp_Vec & normal1,
  const gp_Vec & normal2,
  double tolerance)
{
  // Normalize vectors
  gp_Vec n1 = normal1.Normalized();
  gp_Vec n2 = normal2.Normalized();

  // Compute dot product
  double dot = n1.Dot(n2);

  // For opposite normals: dot ≈ -1
  // angle = π - acos(-dot) = π - acos(|dot|)
  // We want |angle - π| < tolerance
  // So: acos(-dot) < tolerance
  // Therefore: dot < -cos(tolerance)

  double threshold = -std::cos(tolerance);
  return dot < threshold;
}

double distance(const gp_Pnt & p1, const gp_Pnt & p2)
{
  return p1.Distance(p2);
}

EdgeType classify_edge(const TopoDS_Edge & edge)
{
  try {
    Standard_Real first, last;
    Handle(Geom_Curve) curve = BRep_Tool::Curve(edge, first, last);

    if (curve.IsNull()) {
      return EdgeType::LINE;
    }

    GeomAdaptor_Curve adaptor(curve);
    GeomAbs_CurveType curve_type = adaptor.GetType();

    if (curve_type == GeomAbs_Line) {
      return EdgeType::LINE;
    } else if (curve_type == GeomAbs_Circle) {
      return EdgeType::CIRCLE;
    } else {
      return EdgeType::SPLINE;
    }
  } catch (...) {
    return EdgeType::LINE;
  }
}

bool has_inner_holes(const TopoDS_Face & face)
{
  // Count the number of wires in the face
  int wire_count = 0;

  for (TopExp_Explorer exp(face, TopAbs_WIRE); exp.More(); exp.Next()) {
    wire_count++;
  }

  // If more than one wire, face has inner holes
  // (1 outer wire + N inner wires where N >= 1)
  return wire_count > 1;
}

std::vector<Eigen::Vector3d> extract_corners_from_wire(const TopoDS_Wire & wire)
{
  try {
    std::vector<Eigen::Vector3d> corners;

    TopTools_IndexedMapOfShape vertex_map;
    TopExp::MapShapes(wire, TopAbs_VERTEX, vertex_map);

    corners.reserve(vertex_map.Extent());

    for (int i = 1; i <= vertex_map.Extent(); ++i) {
      const TopoDS_Vertex & vertex = TopoDS::Vertex(vertex_map(i));
      gp_Pnt point = BRep_Tool::Pnt(vertex);
      corners.push_back(Eigen::Vector3d(point.X(), point.Y(), point.Z()));
    }

    return corners;
  } catch (const std::exception & e) {
    throw std::runtime_error("Failed to extract corners from wire: " + std::string(e.what()));
  } catch (...) {
    throw std::runtime_error("Failed to extract corners from wire: unknown error");
  }
}

Eigen::Vector3d extract_translation(const gp_Trsf & transform)
{
  try {
    gp_XYZ translation = transform.TranslationPart();
    return Eigen::Vector3d(translation.X(), translation.Y(), translation.Z());
  } catch (const std::exception & e) {
    throw std::runtime_error("Failed to extract translation: " + std::string(e.what()));
  } catch (...) {
    throw std::runtime_error("Failed to extract translation: unknown error");
  }
}

Eigen::Quaterniond extract_quaternion(const gp_Trsf & transform)
{
  try {
    gp_Mat rotation_matrix = transform.VectorialPart();

    Eigen::Matrix3d eigen_rot;
    eigen_rot(0, 0) = rotation_matrix.Value(1, 1);
    eigen_rot(0, 1) = rotation_matrix.Value(1, 2);
    eigen_rot(0, 2) = rotation_matrix.Value(1, 3);
    eigen_rot(1, 0) = rotation_matrix.Value(2, 1);
    eigen_rot(1, 1) = rotation_matrix.Value(2, 2);
    eigen_rot(1, 2) = rotation_matrix.Value(2, 3);
    eigen_rot(2, 0) = rotation_matrix.Value(3, 1);
    eigen_rot(2, 1) = rotation_matrix.Value(3, 2);
    eigen_rot(2, 2) = rotation_matrix.Value(3, 3);

    Eigen::Quaterniond quat(eigen_rot);
    quat.normalize();

    return quat;
  } catch (const std::exception & e) {
    throw std::runtime_error("Failed to extract quaternion: " + std::string(e.what()));
  } catch (...) {
    throw std::runtime_error("Failed to extract quaternion: unknown error");
  }
}

double face_min_distance(const TopoDS_Face & face_1, const TopoDS_Face & face_2)
{
  try {
    BRepExtrema_DistShapeShape dist(face_1, face_2);
    dist.Perform();

    if (dist.IsDone() && dist.NbSolution() > 0) {
      return dist.Value();
    }

    return std::numeric_limits<double>::max();
  } catch (const std::exception & e) {
    RCLCPP_DEBUG(logger_, "Exception in face_min_distance: %s", e.what());
    return std::numeric_limits<double>::max();
  } catch (...) {
    RCLCPP_DEBUG(logger_, "Unknown exception in face_min_distance");
    return std::numeric_limits<double>::max();
  }
}

std::optional<gp_Vec> surface_normal_at_point(const gp_Pnt & point, const TopoDS_Face & face)
{
  try {
    Handle(Geom_Surface) surf = BRep_Tool::Surface(face);

    GeomAPI_ProjectPointOnSurf projector(point, surf);

    if (projector.NbPoints() == 0) {
      Standard_Real u_min, u_max, v_min, v_max;
      BRepTools::UVBounds(face, u_min, u_max, v_min, v_max);
      double u_mid = (u_min + u_max) / 2.0;
      double v_mid = (v_min + v_max) / 2.0;

      GeomLProp_SLProps props(surf, u_mid, v_mid, 1, 1e-6);

      if (!props.IsNormalDefined()) {
        RCLCPP_DEBUG(logger_, "Normal undefined at face center - returning nullopt");
        return std::nullopt;
      }

      gp_Vec normal = props.Normal();

      if (face.Orientation() == TopAbs_REVERSED) {
        normal.Reverse();
      }

      return normal;
    }

    double u, v;
    projector.Parameters(1, u, v);

    GeomLProp_SLProps props(surf, u, v, 1, 1e-6);

    if (!props.IsNormalDefined()) {
      RCLCPP_DEBUG(logger_, "Normal undefined at projected point - returning nullopt");
      return std::nullopt;
    }

    gp_Vec normal = props.Normal();

    if (face.Orientation() == TopAbs_REVERSED) {
      normal.Reverse();
    }

    return normal;
  } catch (const std::exception & e) {
    RCLCPP_DEBUG(logger_, "Exception in surface_normal_at_point: %s", e.what());
    return std::nullopt;
  } catch (...) {
    RCLCPP_DEBUG(logger_, "Unknown exception in surface_normal_at_point");
    return std::nullopt;
  }
}

}  // namespace geometry
}  // namespace hold_and_weld_gripper_sampler
