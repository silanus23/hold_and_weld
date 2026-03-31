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

#include "hold_and_weld_gripper_sampler/constraints/exclusion_zone_constraint.hpp"

#include <Eigen/Dense>

#include <algorithm>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <BRepAlgoAPI_Section.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakePolygon.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <BRepOffsetAPI_MakeOffset.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRepPrimAPI_MakePrism.hxx>
#include <BRep_Builder.hxx>
#include <GeomAbs_JoinType.hxx>
#include <gp_Ax2.hxx>
#include <gp_Circ.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>
#include <rclcpp/rclcpp.hpp>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Compound.hxx>
#include <TopTools_IndexedDataMapOfShapeListOfShape.hxx>
#include <TopTools_ListIteratorOfListOfShape.hxx>

#include "hold_and_weld_gripper_sampler/core/region_filter.hpp"
#include "hold_and_weld_gripper_sampler/geometry/topology.hpp"

namespace hold_and_weld_gripper_sampler
{
namespace constraints
{

ExclusionZoneConstraint::ExclusionZoneConstraint(
  std::shared_ptr<const geometry::GeometryMapper> mapper,
  const ParsedGripper & gripper,
  const std::optional<std::vector<exclusion_circle>> & circles,
  const std::optional<std::vector<exclusion_polygon>> & polygons,
  const std::optional<std::vector<exclusion_line>> & lines,
  double mesh_linear_deflection,
  double mesh_angular_deflection)
: mapper_(mapper),
  gripper_(gripper),
  circles_(circles.value_or(std::vector<exclusion_circle>{})),
  polygons_(polygons.value_or(std::vector<exclusion_polygon>{})),
  lines_(lines.value_or(std::vector<exclusion_line>{})),
  fcl_checker_(nullptr),
  mesh_linear_deflection_(mesh_linear_deflection),
  mesh_angular_deflection_(mesh_angular_deflection),
  logger_(rclcpp::get_logger("gripper_sampler"))
{
  RCLCPP_DEBUG(logger_, "ExclusionZoneConstraint: %zu lines, %zu circles, %zu polygons",
    lines_.size(), circles_.size(), polygons_.size());

  BRepMesh_IncrementalMesh(gripper_.finger_1, mesh_linear_deflection_, Standard_False,
    mesh_angular_deflection_);
  BRepMesh_IncrementalMesh(gripper_.finger_2, mesh_linear_deflection_, Standard_False,
    mesh_angular_deflection_);
  BRepMesh_IncrementalMesh(gripper_.base, mesh_linear_deflection_, Standard_False,
    mesh_angular_deflection_);
}

TopoDS_Shape ExclusionZoneConstraint::create_tube_from_line(
  const exclusion_line & line,
  bool include_clearance) const
{
  gp_Pnt start(line.start.x(), line.start.y(), line.start.z());
  gp_Pnt end(line.end.x(), line.end.y(), line.end.z());

  gp_Vec direction(start, end);
  double length = direction.Magnitude();

  if (length < 1e-6) {
    RCLCPP_WARN(logger_, "Line exclusion has zero length - skipping");
    return TopoDS_Shape();
  }

  gp_Dir axis_dir(direction);
  double radius = line.exclusion_radius + (include_clearance ? line.clearance : 0.0);
  double actual_length = length + (include_clearance ? 2.0 * line.clearance : 0.0);

  gp_Pnt actual_start = start;
  if (include_clearance) {
    // Extend tube symmetrically beyond endpoints for complete coverage
    gp_Vec back_shift(axis_dir);
    back_shift.Scale(-line.clearance);
    actual_start.Translate(back_shift);
  }

  try {
    TopoDS_Shape tube = BRepPrimAPI_MakeCylinder(
      gp_Ax2(actual_start, axis_dir), radius, actual_length).Shape();
    BRepMesh_IncrementalMesh(tube, mesh_linear_deflection_, Standard_False,
      mesh_angular_deflection_);
    return tube;
  } catch (Standard_Failure & e) {
    RCLCPP_ERROR(logger_, "Tube creation failed: %s", e.GetMessageString());
    return TopoDS_Shape();
  }
}

TopoDS_Shape ExclusionZoneConstraint::create_volume_from_circle(
  const exclusion_circle & circle,
  bool include_clearance) const
{
  if (circle.normal.norm() < 1e-6) {
    RCLCPP_WARN(logger_, "Circle exclusion has zero-length normal - skipping");
    return TopoDS_Shape();
  }

  gp_Pnt center(circle.center.x(), circle.center.y(), circle.center.z());
  gp_Dir normal(circle.normal.x(), circle.normal.y(), circle.normal.z());
  gp_Ax2 axis(center, normal);

  double radius = circle.radius + (include_clearance ? circle.clearance : 0.0);
  double extrusion_depth = circle.projection_depth +
    (include_clearance ? 2.0 * circle.clearance : 0.0);

  try {
    TopoDS_Edge circle_edge = BRepBuilderAPI_MakeEdge(gp_Circ(axis, radius));
    TopoDS_Wire circle_wire = BRepBuilderAPI_MakeWire(circle_edge);
    TopoDS_Face disk = BRepBuilderAPI_MakeFace(circle_wire);

    TopoDS_Shape thick_disk = BRepPrimAPI_MakePrism(
      disk, gp_Vec(normal.XYZ() * extrusion_depth)).Shape();

    if (include_clearance) {
      // Shift back so volume straddles the original surface plane
      gp_Trsf shift_back;
      shift_back.SetTranslation(gp_Vec(
        -normal.X() * circle.clearance,
        -normal.Y() * circle.clearance,
        -normal.Z() * circle.clearance));
      thick_disk = BRepBuilderAPI_Transform(thick_disk, shift_back, Standard_True).Shape();
    }

    BRepMesh_IncrementalMesh(thick_disk, mesh_linear_deflection_, Standard_False,
      mesh_angular_deflection_);
    return thick_disk;
  } catch (Standard_Failure & e) {
    RCLCPP_ERROR(logger_, "Circle volume creation failed: %s", e.GetMessageString());
    return TopoDS_Shape();
  }
}

TopoDS_Shape ExclusionZoneConstraint::create_prism_from_polygon(
  const exclusion_polygon & polygon,
  bool include_clearance) const
{
  if (polygon.exclusion_corners.size() < 3) {
    RCLCPP_WARN(logger_, "Polygon exclusion has fewer than 3 corners (%zu) - skipping",
      polygon.exclusion_corners.size());
    return TopoDS_Shape();
  }

  // Compute face normal from first two edges (right-hand rule)
  Eigen::Vector3d v1 = polygon.exclusion_corners[1] - polygon.exclusion_corners[0];
  Eigen::Vector3d v2 = polygon.exclusion_corners[2] - polygon.exclusion_corners[0];
  Eigen::Vector3d normal = v1.cross(v2);

  if (normal.norm() < 1e-6) {
    RCLCPP_WARN(logger_, "Polygon has collinear points - cannot compute normal");
    return TopoDS_Shape();
  }
  normal.normalize();

  double extrusion_depth = polygon.projection_depth +
    (include_clearance ? 2.0 * polygon.clearance : 0.0);

  gp_Vec extrusion(
    normal.x() * extrusion_depth,
    normal.y() * extrusion_depth,
    normal.z() * extrusion_depth);

  try {
    BRepBuilderAPI_MakePolygon poly_builder;
    for (const auto & corner : polygon.exclusion_corners) {
      poly_builder.Add(gp_Pnt(corner.x(), corner.y(), corner.z()));
    }
    poly_builder.Close();

    if (!poly_builder.IsDone()) {
      RCLCPP_ERROR(logger_, "Failed to create polygon wire");
      return TopoDS_Shape();
    }

    TopoDS_Face poly_face;

    if (include_clearance) {
      TopoDS_Face temp_face = BRepBuilderAPI_MakeFace(poly_builder.Wire());
      BRepOffsetAPI_MakeOffset offset_maker(temp_face, GeomAbs_Arc);
      offset_maker.Perform(polygon.clearance);

      if (offset_maker.IsDone()) {
        poly_face = BRepBuilderAPI_MakeFace(TopoDS::Wire(offset_maker.Shape()));
      } else {
        RCLCPP_WARN(logger_, "Polygon offset failed - using original polygon");
        poly_face = temp_face;
      }
    } else {
      poly_face = BRepBuilderAPI_MakeFace(poly_builder.Wire());
    }

    TopoDS_Shape prism = BRepPrimAPI_MakePrism(poly_face, extrusion).Shape();

    if (include_clearance) {
      gp_Trsf shift_back;
      shift_back.SetTranslation(gp_Vec(
        -normal.x() * polygon.clearance,
        -normal.y() * polygon.clearance,
        -normal.z() * polygon.clearance));
      prism = BRepBuilderAPI_Transform(prism, shift_back, Standard_True).Shape();
    }

    BRepMesh_IncrementalMesh(prism, mesh_linear_deflection_, Standard_False,
      mesh_angular_deflection_);
    return prism;
  } catch (Standard_Failure & e) {
    RCLCPP_ERROR(logger_, "Prism creation failed: %s", e.GetMessageString());
    return TopoDS_Shape();
  }
}

std::vector<core::SampleArea> ExclusionZoneConstraint::process_constraint_volume(
  const TopoDS_Shape & constraint_volume,
  const TopoDS_Shape & shape) const
{
  std::vector<core::SampleArea> sample_areas;

  BRepAlgoAPI_Section section(constraint_volume, shape);

  try {
    section.Build();
  } catch (Standard_Failure & e) {
    RCLCPP_WARN(logger_, "Section operation failed: %s", e.GetMessageString());
    return sample_areas;
  }

  if (!section.IsDone() || section.Shape().IsNull()) {
    return sample_areas;
  }

  // Map edges back to primary shape faces to identify which surface each
  // intersection edge belongs to
  TopTools_IndexedDataMapOfShapeListOfShape edge_to_faces;
  TopExp::MapShapesAndAncestors(shape, TopAbs_EDGE, TopAbs_FACE, edge_to_faces);

  std::map<int, std::vector<TopoDS_Edge>> surface_edges;

  for (TopExp_Explorer exp(section.Shape(), TopAbs_EDGE); exp.More(); exp.Next()) {
    TopoDS_Edge edge = TopoDS::Edge(exp.Current());

    if (!edge_to_faces.Contains(edge)) {
      continue;
    }

    for (TopTools_ListIteratorOfListOfShape it(edge_to_faces.FindFromKey(edge));
      it.More(); it.Next())
    {
      try {
        int surface_id = mapper_->find_topology_surface_id(TopoDS::Face(it.Value()));
        surface_edges[surface_id].push_back(edge);
      } catch (const std::runtime_error &) {
        continue;
      }
    }
  }

  for (const auto & [surface_id, edges] : surface_edges) {
    BRepBuilderAPI_MakeWire wire_builder;
    for (const auto & edge : edges) {
      wire_builder.Add(edge);
    }

    if (!wire_builder.IsDone()) {
      RCLCPP_WARN(logger_, "Failed to build exclusion wire for surface %d (%zu edges) "
        "- edges may be disconnected", surface_id, edges.size());
      continue;
    }

    TopoDS_Wire wire = wire_builder.Wire();
    wire.Reverse();  // Reversed orientation marks interior as exclusion zone

    core::SampleArea area;
    area.surface_id = surface_id;
    area.wire = wire;
    sample_areas.push_back(area);

    RCLCPP_DEBUG(logger_, "Exclusion wire created for surface %d (%zu edges)",
      surface_id, edges.size());
  }

  return sample_areas;
}

void ExclusionZoneConstraint::analyze_constraints(
  const TopoDS_Shape & shape,
  const geometry::Topology & topology)
{
  (void)topology;

  sample_areas_.clear();
  projection_volumes_.clear();
  collision_volumes_.clear();

  const size_t total = lines_.size() + circles_.size() + polygons_.size();

  if (total == 0) {
    RCLCPP_DEBUG(logger_, "No exclusion zones defined - skipping analysis");
    return;
  }

  RCLCPP_INFO(logger_, "Analyzing %zu exclusion constraint(s): %zu line(s), "
    "%zu circle(s), %zu polygon(s)",
    total, lines_.size(), circles_.size(), polygons_.size());

  for (size_t i = 0; i < lines_.size(); ++i) {
    const auto & line = lines_[i];
    RCLCPP_DEBUG(logger_, "Line exclusion %zu: start=(%.4f,%.4f,%.4f) end=(%.4f,%.4f,%.4f)",
      i, line.start.x(), line.start.y(), line.start.z(),
      line.end.x(), line.end.y(), line.end.z());

    TopoDS_Shape proj = create_tube_from_line(line, false);
    TopoDS_Shape coll = create_tube_from_line(line, true);
    projection_volumes_.push_back(proj);
    collision_volumes_.push_back(coll);

    auto areas = process_constraint_volume(proj, shape);
    sample_areas_.insert(sample_areas_.end(), areas.begin(), areas.end());
    RCLCPP_DEBUG(logger_, "  -> %zu exclusion wire(s) extracted", areas.size());
  }

  for (size_t i = 0; i < circles_.size(); ++i) {
    const auto & circle = circles_[i];
    RCLCPP_DEBUG(logger_, "Circle exclusion %zu: center=(%.4f,%.4f,%.4f) r=%.4f m",
      i, circle.center.x(), circle.center.y(), circle.center.z(), circle.radius);

    TopoDS_Shape proj = create_volume_from_circle(circle, false);
    TopoDS_Shape coll = create_volume_from_circle(circle, true);
    projection_volumes_.push_back(proj);
    collision_volumes_.push_back(coll);

    auto areas = process_constraint_volume(proj, shape);
    sample_areas_.insert(sample_areas_.end(), areas.begin(), areas.end());
    RCLCPP_DEBUG(logger_, "  -> %zu exclusion wire(s) extracted", areas.size());
  }

  for (size_t i = 0; i < polygons_.size(); ++i) {
    const auto & polygon = polygons_[i];
    RCLCPP_DEBUG(logger_, "Polygon exclusion %zu: %zu corners, depth=%.4f m",
      i, polygon.exclusion_corners.size(), polygon.projection_depth);

    TopoDS_Shape proj = create_prism_from_polygon(polygon, false);
    TopoDS_Shape coll = create_prism_from_polygon(polygon, true);
    projection_volumes_.push_back(proj);
    collision_volumes_.push_back(coll);

    auto areas = process_constraint_volume(proj, shape);
    sample_areas_.insert(sample_areas_.end(), areas.begin(), areas.end());
    RCLCPP_DEBUG(logger_, "  -> %zu exclusion wire(s) extracted", areas.size());
  }

  // Count unique surfaces affected by exclusion wires
  const size_t affected_surfaces = [&]() {
    std::map<int, int> counts;
    for (const auto & a : sample_areas_) {counts[a.surface_id]++;}
    return counts.size();
  }();

  RCLCPP_INFO(logger_, "Exclusion analysis complete: %zu volume(s), %zu wire(s) "
    "on %zu surface(s)",
    collision_volumes_.size(), sample_areas_.size(), affected_surfaces);
}

std::vector<core::SampleArea> ExclusionZoneConstraint::get_sample_areas() const
{
  return sample_areas_;
}

void ExclusionZoneConstraint::set_fcl_checker(
  std::shared_ptr<const geometry::FCLCollisionChecker> fcl_checker)
{
  fcl_checker_ = fcl_checker;
}

bool ExclusionZoneConstraint::intersects_exclusion_zone(
  const gp_Trsf & gripper_transform,
  double grip_distance,
  double tolerance) const
{
  if (collision_volumes_.empty()) {
    return false;
  }

  if (!fcl_checker_ || !fcl_checker_->is_valid()) {
    RCLCPP_WARN(logger_, "FCL checker not available - rejecting grasp conservatively");
    return true;
  }

  return fcl_checker_->collides_with_exclusions(gripper_transform, grip_distance, tolerance);
}

std::string ExclusionZoneConstraint::get_name() const
{
  return "ExclusionZoneConstraint";
}

const std::vector<TopoDS_Shape> & ExclusionZoneConstraint::get_collision_volumes() const
{
  return collision_volumes_;
}

}  // namespace constraints
}  // namespace hold_and_weld_gripper_sampler
