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

#include "hold_and_weld_gripper_sampler/constraints/kissing_surface_constraint.hpp"

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <BRepAlgoAPI_Section.hxx>
#include <BRepBuilderAPI_MakeVertex.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <BRepBuilderAPI_WireError.hxx>
#include <BRepExtrema_DistShapeShape.hxx>
#include <BRepGProp.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <BRepTools.hxx>
#include <BRep_Builder.hxx>
#include <BRep_Tool.hxx>
#include <Geom_Surface.hxx>
#include <GProp_GProps.hxx>
#include <Poly_Triangulation.hxx>
#include <rclcpp/rclcpp.hpp>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopLoc_Location.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Compound.hxx>
#include <TopTools_IndexedDataMapOfShapeListOfShape.hxx>
#include <TopTools_ListIteratorOfListOfShape.hxx>
#include <gp_Vec.hxx>

#include "hold_and_weld_gripper_sampler/core/region_filter.hpp"
#include "hold_and_weld_gripper_sampler/geometry/topology.hpp"

namespace hold_and_weld_gripper_sampler
{
namespace constraints
{

KissingSurfaceConstraint::KissingSurfaceConstraint(
  std::shared_ptr<const geometry::GeometryMapper> mapper,
  const ParsedGripper & gripper,
  const std::vector<TopoDS_Shape> & secondary_shapes,
  double contact_threshold,
  double collision_tolerance,
  double contact_distance_threshold,
  double mesh_linear_deflection,
  double mesh_angular_deflection)
: mapper_(mapper),
  gripper_(gripper),
  secondary_shapes_(secondary_shapes),
  contact_threshold_(contact_threshold),
  collision_tolerance_(collision_tolerance),
  contact_distance_threshold_(contact_distance_threshold),
  mesh_linear_deflection_(mesh_linear_deflection),
  mesh_angular_deflection_(mesh_angular_deflection),
  fcl_checker_(nullptr),
  logger_(rclcpp::get_logger("gripper_sampler"))
{
  RCLCPP_DEBUG(logger_, "KissingSurfaceConstraint: %zu secondaries, "
    "contact_threshold=%.1f%%, collision_tolerance=%.6f m",
    secondary_shapes_.size(), contact_threshold_ * 100.0, collision_tolerance_);

  try {
    if (!gripper_.finger_1.IsNull()) {
      BRepMesh_IncrementalMesh(gripper_.finger_1, mesh_linear_deflection_, Standard_False,
        mesh_angular_deflection_);
    }
    if (!gripper_.finger_2.IsNull()) {
      BRepMesh_IncrementalMesh(gripper_.finger_2, mesh_linear_deflection_, Standard_False,
        mesh_angular_deflection_);
    }
    if (!gripper_.base.IsNull()) {
      BRepMesh_IncrementalMesh(gripper_.base, mesh_linear_deflection_, Standard_False,
        mesh_angular_deflection_);
    }
  } catch (const Standard_Failure & e) {
    RCLCPP_ERROR(logger_, "Critical gripper meshing failure: %s", e.GetMessageString());
    throw std::runtime_error("Gripper geometry is unmeshable.");
  }

  for (size_t i = 0; i < secondary_shapes_.size(); ++i) {
    if (secondary_shapes_[i].IsNull()) {continue;}
    try {
      BRepMesh_IncrementalMesh(secondary_shapes_[i], mesh_linear_deflection_, Standard_False,
            mesh_angular_deflection_);
    } catch (const Standard_Failure & e) {
      RCLCPP_WARN(logger_, "Secondary shape %zu failed to mesh: %s — results may be inaccurate",
        i, e.GetMessageString());
    }
  }
}

void KissingSurfaceConstraint::analyze_constraints(const geometry::Topology & topology)
{
  banned_surface_ids_.clear();
  partial_exclusions_.clear();

  if (secondary_shapes_.empty()) {
    RCLCPP_WARN(logger_, "No secondary shapes defined - skipping kissing surface analysis");
    return;
  }

  const auto & all_surfaces = topology.get_all_surfaces();

  RCLCPP_INFO(logger_, "Analyzing kissing surfaces: %zu surface(s), "
    "contact_threshold=%.1f%%",
    all_surfaces.size(), contact_threshold_ * 100.0);

  // Mesh all primary faces before contact ratio sampling — topology faces are
  // not guaranteed to have triangulation until explicitly meshed here.
  for (const auto & surface : all_surfaces) {
    try {
      BRepMesh_IncrementalMesh(surface.face, mesh_linear_deflection_, Standard_False,
            mesh_angular_deflection_);
    } catch (const Standard_Failure & e) {
      RCLCPP_WARN(logger_, "Failed to mesh surface %d - contact ratio may be zero",
            mapper_->find_topology_surface_id(surface.face));
    }
  }

  for (size_t i = 0; i < all_surfaces.size(); i++) {
    int surface_id = static_cast<int>(i);
    double contact_ratio = measure_contact_ratio(surface_id, topology);

    if (contact_ratio < 1e-9) {
      continue;
    }

    RCLCPP_DEBUG(logger_, "Surface %d: contact_ratio=%.1f%%",
      surface_id, contact_ratio * 100.0);

    if (contact_ratio > contact_threshold_) {
      banned_surface_ids_.push_back(surface_id);
      RCLCPP_DEBUG(logger_, "  -> banned (%.1f%% > threshold %.1f%%)",
        contact_ratio * 100.0, contact_threshold_ * 100.0);
    } else {
      TopoDS_Wire boundary = extract_contact_boundary(surface_id, topology);
      if (!boundary.IsNull()) {
        core::SampleArea area;
        area.surface_id = surface_id;
        area.wire = boundary;
        area.is_exclusion = true;
        partial_exclusions_.push_back(area);
        RCLCPP_DEBUG(logger_, "  -> partial exclusion wire created (%.1f%% contact)",
          contact_ratio * 100.0);
      } else {
        RCLCPP_DEBUG(logger_, "  -> partial contact (%.1f%%) but wire extraction failed",
          contact_ratio * 100.0);
      }
    }
  }

  RCLCPP_INFO(logger_, "Kissing surface analysis complete: %zu banned, %zu partial exclusions",
    banned_surface_ids_.size(), partial_exclusions_.size());

  if (!banned_surface_ids_.empty()) {
    std::string banned_str;
    for (size_t i = 0; i < banned_surface_ids_.size(); i++) {
      if (i > 0) {banned_str += ", ";}
      banned_str += std::to_string(banned_surface_ids_[i]);
    }
    RCLCPP_DEBUG(logger_, "Banned surface IDs: [%s]", banned_str.c_str());
  }
}

double KissingSurfaceConstraint::measure_contact_ratio(
  int surface_id,
  const geometry::Topology & topology) const
{
  const auto & surface = topology.get_surface(surface_id);
  TopoDS_Face face = surface.face;

  TopLoc_Location location;
  Handle(Poly_Triangulation) triangulation = BRep_Tool::Triangulation(face, location);

  if (triangulation.IsNull() || triangulation->NbTriangles() == 0) {
    RCLCPP_DEBUG(logger_, "Surface %d: no triangulation available", surface_id);
    return 0.0;
  }

  const gp_Trsf & trsf = location.Transformation();
  const int num_triangles = triangulation->NbTriangles();

  double total_area = 0.0;
  double contact_area = 0.0;

  for (int tri_idx = 1; tri_idx <= num_triangles; ++tri_idx) {
    int n1, n2, n3;
    triangulation->Triangle(tri_idx).Get(n1, n2, n3);

    gp_Pnt p1 = triangulation->Node(n1).Transformed(trsf);
    gp_Pnt p2 = triangulation->Node(n2).Transformed(trsf);
    gp_Pnt p3 = triangulation->Node(n3).Transformed(trsf);

    // Area-weighted contact ratio: weight each sample by its triangle area
    gp_Vec v1(p1, p2);
    gp_Vec v2(p1, p3);
    double tri_area = 0.5 * v1.Crossed(v2).Magnitude();

    if (tri_area < 1e-12) {
      continue;
    }

    total_area += tri_area;

    gp_Pnt centroid(
      (p1.X() + p2.X() + p3.X()) / 3.0,
      (p1.Y() + p2.Y() + p3.Y()) / 3.0,
      (p1.Z() + p2.Z() + p3.Z()) / 3.0);

    for (const auto & secondary : secondary_shapes_) {
      if (secondary.IsNull()) {continue;}
      try {
        TopoDS_Vertex v = BRepBuilderAPI_MakeVertex(centroid);
        BRepExtrema_DistShapeShape dist(v, secondary);

        if (dist.Value() <= contact_distance_threshold_) {
          contact_area += tri_area;
          break;
        }
      } catch (const Standard_Failure & e) {
        continue;
      }
    }
  }

  if (total_area < 1e-9) {
    return 0.0;
  }

  double contact_ratio = contact_area / total_area;
  RCLCPP_DEBUG(logger_, "Surface %d: contact_ratio=%.1f%% (%d triangles)",
    surface_id, contact_ratio * 100.0, num_triangles);

  return contact_ratio;
}

TopoDS_Wire KissingSurfaceConstraint::extract_contact_boundary(
  int surface_id,
  const geometry::Topology & topology) const
{
  const auto & surface = topology.get_surface(surface_id);
  TopoDS_Face face = surface.face;

  std::vector<TopoDS_Edge> boundary_edges;

  for (size_t i = 0; i < secondary_shapes_.size(); ++i) {
    if (secondary_shapes_[i].IsNull()) {continue;}
    try {
      BRepAlgoAPI_Section section(face, secondary_shapes_[i]);
      section.Build();

      if (!section.IsDone() || section.Shape().IsNull()) {continue;}

      for (TopExp_Explorer exp(section.Shape(), TopAbs_EDGE); exp.More(); exp.Next()) {
        boundary_edges.push_back(TopoDS::Edge(exp.Current()));
      }
    } catch (const Standard_Failure & e) {
      RCLCPP_DEBUG(logger_, "Section failed for surface %d: %s", surface_id, e.GetMessageString());
    }
  }

  if (boundary_edges.empty()) {
    return TopoDS_Wire();
  }

  try {
    BRepBuilderAPI_MakeWire wire_builder;
    for (const auto & edge : boundary_edges) {
      wire_builder.Add(edge);
    }

    if (!wire_builder.IsDone()) {
      RCLCPP_WARN(logger_,
        "MakeWire failed for surface %d — contact boundary wire skipped", surface_id);
      return TopoDS_Wire();
    }

    TopoDS_Wire boundary_wire = wire_builder.Wire();
    return boundary_wire;
  } catch (const Standard_Failure & e) {
    RCLCPP_ERROR(logger_, "Wire building failure on surface %d: %s", surface_id,
          e.GetMessageString());
    return TopoDS_Wire();
  }
}

bool KissingSurfaceConstraint::intersects_secondary(
  double grip_distance,
  const gp_Trsf & grasp_transform) const
{
  collision_stats_.total_checks++;

  if (secondary_shapes_.empty() && (!fcl_checker_ || !fcl_checker_->has_ground_plane())) {
    return false;
  }

  if (!fcl_checker_ || !fcl_checker_->is_valid()) {
    RCLCPP_ERROR(logger_, "FCL checker not available — rejecting grasp conservatively");
    return true;
  }

  if (fcl_checker_->collides_with_ground(grasp_transform, grip_distance, collision_tolerance_)) {
    collision_stats_.fcl_rejections++;
    return true;
  }
  bool collision = fcl_checker_->collides_with_secondaries(
    grasp_transform, grip_distance, collision_tolerance_);

  if (collision) {
    collision_stats_.fcl_rejections++;
  }

  return collision;
}

void KissingSurfaceConstraint::set_fcl_checker(
  std::shared_ptr<const geometry::FCLCollisionChecker> fcl_checker)
{
  fcl_checker_ = fcl_checker;
  RCLCPP_DEBUG(logger_, "FCL checker set");
}

std::vector<int> KissingSurfaceConstraint::get_banned_surface_ids() const
{
  return banned_surface_ids_;
}

std::vector<core::SampleArea> KissingSurfaceConstraint::get_sample_areas() const
{
  return partial_exclusions_;
}

const std::vector<TopoDS_Shape> & KissingSurfaceConstraint::get_secondary_shapes() const
{
  return secondary_shapes_;
}

GraspCollisionStats KissingSurfaceConstraint::get_collision_stats() const
{
  return collision_stats_;
}

void KissingSurfaceConstraint::reset_collision_stats()
{
  collision_stats_.reset();
}

std::string KissingSurfaceConstraint::get_name() const
{
  return "KissingSurfaceConstraint";
}

}  // namespace constraints
}  // namespace hold_and_weld_gripper_sampler
