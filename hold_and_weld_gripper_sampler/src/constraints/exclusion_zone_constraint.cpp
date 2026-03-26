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
#include <BRepExtrema_DistShapeShape.hxx>
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
  circles_(circles.value_or(std::vector<exclusion_circle> {})),
  polygons_(polygons.value_or(std::vector<exclusion_polygon> {})),
  lines_(lines.value_or(std::vector<exclusion_line> {})),
  fcl_checker_(nullptr),
  mesh_linear_deflection_(mesh_linear_deflection),
  mesh_angular_deflection_(mesh_angular_deflection),
  logger_(rclcpp::get_logger("gripper_sampler"))
{
  RCLCPP_DEBUG(logger_, "Initializing ExclusionZoneConstraint");
  RCLCPP_DEBUG(logger_, "  Line exclusions: %zu", lines_.size());
  RCLCPP_DEBUG(logger_, "  Circle exclusions: %zu", circles_.size());
  RCLCPP_DEBUG(logger_, "  Polygon exclusions: %zu", polygons_.size());

  // Triangulate gripper components for OCCT fallback collision detection
  // (FCL will re-use OCCT's cached triangulation)
  BRepMesh_IncrementalMesh mesher_f1(gripper_.finger_1, mesh_linear_deflection_, Standard_False,
    mesh_angular_deflection_);
  BRepMesh_IncrementalMesh mesher_f2(gripper_.finger_2, mesh_linear_deflection_, Standard_False,
    mesh_angular_deflection_);
  BRepMesh_IncrementalMesh mesher_base(gripper_.base, mesh_linear_deflection_, Standard_False,
    mesh_angular_deflection_);

  RCLCPP_DEBUG(logger_, "Gripper triangulation complete");
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
    RCLCPP_ERROR(logger_, "Line exclusion has zero length - skipping");
    return TopoDS_Shape();
  }

  gp_Dir axis_dir(direction);

  double radius = line.exclusion_radius;
  if (include_clearance) {
    radius += line.clearance;
  }

  double actual_length = length;
  gp_Pnt actual_start = start;

  if (include_clearance) {
    // Extend tube beyond line endpoints to ensure complete coverage
    actual_length += 2.0 * line.clearance;

    // Shift start point backward so tube is centered on original line
    gp_Vec back_shift(axis_dir);
    back_shift.Scale(-line.clearance);
    actual_start.Translate(back_shift);
  }

  // Create axis at (possibly shifted) start point
  gp_Ax2 axis(actual_start, axis_dir);

  TopoDS_Shape tube;
  try {
    tube = BRepPrimAPI_MakeCylinder(axis, radius, actual_length).Shape();
    // Pre-mesh for fast distance queries (required by BRepExtrema_DistShapeShape)
    BRepMesh_IncrementalMesh mesher(tube, mesh_linear_deflection_, Standard_False,
      mesh_angular_deflection_);
  } catch (Standard_Failure & e) {
    RCLCPP_ERROR(logger_, "OCCT exception in tube creation: %s", e.GetMessageString());
    return TopoDS_Shape();
  }

  return tube;
}

TopoDS_Shape ExclusionZoneConstraint::create_volume_from_circle(
  const exclusion_circle & circle,
  bool include_clearance) const
{
  gp_Pnt center(circle.center.x(), circle.center.y(), circle.center.z());

  Eigen::Vector3d normal_vec = circle.normal;
  if (normal_vec.norm() < 1e-6) {
    RCLCPP_ERROR(logger_, "Circle exclusion has zero-length normal - skipping");
    return TopoDS_Shape();
  }

  gp_Dir normal(circle.normal.x(), circle.normal.y(), circle.normal.z());
  gp_Ax2 axis(center, normal);

  double radius = circle.radius;
  if (include_clearance) {
    radius += circle.clearance;
  }

  TopoDS_Face disk;
  try {
    gp_Circ circ(axis, radius);
    TopoDS_Edge circle_edge = BRepBuilderAPI_MakeEdge(circ);
    TopoDS_Wire circle_wire = BRepBuilderAPI_MakeWire(circle_edge);
    disk = BRepBuilderAPI_MakeFace(circle_wire);
  } catch (Standard_Failure & e) {
    RCLCPP_ERROR(logger_, "OCCT exception in circle disk creation: %s", e.GetMessageString());
    return TopoDS_Shape();
  }

  double extrusion_depth = circle.projection_depth;
  if (include_clearance) {
    extrusion_depth += 2.0 * circle.clearance;
  }

  // Extrude disk into the part (backward along normal)
  // This creates a volume projecting into the workpiece surface
  gp_Vec extrusion(normal.XYZ() * extrusion_depth);
  TopoDS_Shape thick_disk;

  try {
    thick_disk = BRepPrimAPI_MakePrism(disk, extrusion).Shape();

    // Shift back so volume is centered on original surface plane
    if (include_clearance) {
      gp_Trsf shift_back;
      shift_back.SetTranslation(gp_Vec(
        -normal.X() * circle.clearance,
        -normal.Y() * circle.clearance,
        -normal.Z() * circle.clearance));
      thick_disk = BRepBuilderAPI_Transform(thick_disk, shift_back, Standard_True).Shape();
    }

    BRepMesh_IncrementalMesh mesher(thick_disk, mesh_linear_deflection_, Standard_False,
      mesh_angular_deflection_);
  } catch (Standard_Failure & e) {
    RCLCPP_ERROR(logger_, "OCCT exception in circle extrusion: %s", e.GetMessageString());
    return TopoDS_Shape();
  }

  return thick_disk;
}

TopoDS_Shape ExclusionZoneConstraint::create_prism_from_polygon(
  const exclusion_polygon & polygon,
  bool include_clearance) const
{
  if (polygon.exclusion_corners.size() < 3) {
    RCLCPP_ERROR(logger_, "Polygon exclusion has fewer than 3 corners (%zu) - skipping",
      polygon.exclusion_corners.size());
    return TopoDS_Shape();
  }

  BRepBuilderAPI_MakePolygon poly_builder;
  for (const auto & corner : polygon.exclusion_corners) {
    gp_Pnt point(corner.x(), corner.y(), corner.z());
    poly_builder.Add(point);
  }
  poly_builder.Close();

  if (!poly_builder.IsDone()) {
    RCLCPP_ERROR(logger_, "Failed to create polygon wire");
    return TopoDS_Shape();
  }

  TopoDS_Wire poly_wire = poly_builder.Wire();

  // Calculate polygon plane normal using cross product of first two edges
  Eigen::Vector3d v1 = polygon.exclusion_corners[1] - polygon.exclusion_corners[0];
  Eigen::Vector3d v2 = polygon.exclusion_corners[2] - polygon.exclusion_corners[0];
  Eigen::Vector3d normal = v1.cross(v2);  // Right-hand rule determines direction

  // Check for collinear points (degenerate polygon)
  if (normal.norm() < 1e-6) {
    RCLCPP_ERROR(logger_, "Polygon has collinear points - cannot compute normal");
    return TopoDS_Shape();
  }

  normal.normalize();

  TopoDS_Face poly_face;

  try {
    if (include_clearance) {
      // Create temporary face for offset reference plane
      TopoDS_Face temp_face = BRepBuilderAPI_MakeFace(poly_wire);

      // Offset polygon outward by clearance to create safety margin
      // GeomAbs_Arc produces rounded corners
      BRepOffsetAPI_MakeOffset offset_maker(temp_face, GeomAbs_Arc);
      offset_maker.Perform(polygon.clearance);

      if (offset_maker.IsDone()) {
        TopoDS_Wire offset_wire = TopoDS::Wire(offset_maker.Shape());
        poly_face = BRepBuilderAPI_MakeFace(offset_wire);
      } else {
        RCLCPP_WARN(logger_, "Polygon offset failed - using original polygon");
        poly_face = temp_face;
      }
    } else {
      poly_face = BRepBuilderAPI_MakeFace(poly_wire);
    }
  } catch (Standard_Failure & e) {
    RCLCPP_ERROR(logger_, "OCCT exception in polygon face creation: %s",
      e.GetMessageString());
    return TopoDS_Shape();
  }

  double extrusion_depth = polygon.projection_depth;
  if (include_clearance) {
    extrusion_depth += 2.0 * polygon.clearance;
  }

  gp_Vec extrusion(
    normal.x() * extrusion_depth,
    normal.y() * extrusion_depth,
    normal.z() * extrusion_depth);

  TopoDS_Shape prism;
  try {
    prism = BRepPrimAPI_MakePrism(poly_face, extrusion).Shape();
  } catch (Standard_Failure & e) {
    RCLCPP_ERROR(logger_, "OCCT exception in prism creation: %s", e.GetMessageString());
    return TopoDS_Shape();
  }

  // Shift prism back so it's centered on the original polygon plane
  if (include_clearance) {
    gp_Trsf shift_back;
    shift_back.SetTranslation(gp_Vec(
      -normal.x() * polygon.clearance,
      -normal.y() * polygon.clearance,
      -normal.z() * polygon.clearance));
    prism = BRepBuilderAPI_Transform(prism, shift_back, Standard_True).Shape();
  }

  BRepMesh_IncrementalMesh mesher(prism, mesh_linear_deflection_, Standard_False,
    mesh_angular_deflection_);

  return prism;
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
    RCLCPP_WARN(logger_, "OCCT exception in section operation: %s", e.GetMessageString());
    return sample_areas;
  }

  if (!section.IsDone()) {
    RCLCPP_DEBUG(logger_, "Section operation failed or produced no result");
    return sample_areas;
  }

  TopoDS_Shape section_result = section.Shape();

  if (section_result.IsNull()) {
    RCLCPP_DEBUG(logger_, "Section result is null - no intersection");
    return sample_areas;
  }

  // Build map: edge → faces that share that edge (for finding which surface each edge belongs to)
  TopTools_IndexedDataMapOfShapeListOfShape edge_to_faces;
  TopExp::MapShapesAndAncestors(shape, TopAbs_EDGE, TopAbs_FACE, edge_to_faces);

  // Group intersection edges by which primary surface they belong to
  std::map<int, std::vector<TopoDS_Edge>> surface_edges;

  for (TopExp_Explorer exp(section_result, TopAbs_EDGE); exp.More(); exp.Next()) {
    TopoDS_Edge edge = TopoDS::Edge(exp.Current());

    if (edge_to_faces.Contains(edge)) {
      const TopTools_ListOfShape & faces = edge_to_faces.FindFromKey(edge);

      for (TopTools_ListIteratorOfListOfShape it(faces); it.More(); it.Next()) {
        TopoDS_Face face = TopoDS::Face(it.Value());

        try {
          // Look up which surface ID this face corresponds to in the topology
          int surface_id = mapper_->find_topology_surface_id(face);

          if (surface_id < 0) {
            RCLCPP_DEBUG(logger_,
              "Face from section not found in topology (ID=%d) - skipping", surface_id);
            continue;
          }

          surface_edges[surface_id].push_back(edge);
        } catch (const std::runtime_error & e) {
          RCLCPP_DEBUG(logger_, "Exception finding surface ID: %s", e.what());
          continue;
        }
      }
    }
  }

  for (const auto & [surface_id, edges] : surface_edges) {
    if (edges.empty()) {
      continue;
    }

    BRepBuilderAPI_MakeWire wire_builder;

    for (const auto & edge : edges) {
      wire_builder.Add(edge);
    }

    if (wire_builder.IsDone()) {
      TopoDS_Wire wire = wire_builder.Wire();

      // Reverse wire orientation so interior of wire = exclusion zone
      wire.Reverse();

      core::SampleArea area;
      area.surface_id = surface_id;
      area.wire = wire;
      sample_areas.push_back(area);

      RCLCPP_DEBUG(logger_, "Created exclusion wire for surface %d with %zu edges",
        surface_id, edges.size());
    } else {
      RCLCPP_WARN(logger_,
        "Failed to build wire for surface %d from %zu edges - edges may be disconnected",
        surface_id, edges.size());
    }
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

  size_t total_constraints = lines_.size() + circles_.size() + polygons_.size();
  RCLCPP_INFO(logger_, "Total exclusion constraints defined: %zu", total_constraints);
  RCLCPP_INFO(logger_, "  Line exclusions: %zu", lines_.size());
  RCLCPP_INFO(logger_, "  Circle exclusions: %zu", circles_.size());
  RCLCPP_INFO(logger_, "  Polygon exclusions: %zu", polygons_.size());

  if (total_constraints == 0) {
    RCLCPP_INFO(logger_, "No exclusion zones defined - skipping analysis");
    return;
  }

  for (size_t i = 0; i < lines_.size(); ++i) {
    const auto & line = lines_[i];
    RCLCPP_INFO(logger_, "Processing LINE exclusion %zu:", i);
    RCLCPP_INFO(logger_, "  Start: (%.4f, %.4f, %.4f)", line.start.x(), line.start.y(),
          line.start.z());
    RCLCPP_INFO(logger_, "  End: (%.4f, %.4f, %.4f)", line.end.x(), line.end.y(), line.end.z());

    TopoDS_Shape proj_tube = create_tube_from_line(line, false);
    TopoDS_Shape coll_tube = create_tube_from_line(line, true);

    RCLCPP_INFO(logger_, "  Projection tube null: %s, Collision tube null: %s",
      proj_tube.IsNull() ? "YES" : "NO", coll_tube.IsNull() ? "YES" : "NO");

    projection_volumes_.push_back(proj_tube);
    collision_volumes_.push_back(coll_tube);

    // Extract sample areas using projection volume
    auto areas = process_constraint_volume(proj_tube, shape);
    RCLCPP_INFO(logger_, "  Extracted %zu sample areas (exclusion wires)", areas.size());
    for (const auto & area : areas) {
      RCLCPP_INFO(logger_, "    -> Surface %d affected", area.surface_id);
    }
    sample_areas_.insert(sample_areas_.end(), areas.begin(), areas.end());
  }

  for (size_t i = 0; i < circles_.size(); ++i) {
    const auto & circle = circles_[i];
    RCLCPP_INFO(logger_, "Processing CIRCLE exclusion %zu:", i);
    RCLCPP_INFO(logger_, "  Center: (%.4f, %.4f, %.4f)", circle.center.x(), circle.center.y(),
          circle.center.z());
    RCLCPP_INFO(logger_, "  Normal: (%.4f, %.4f, %.4f)", circle.normal.x(), circle.normal.y(),
          circle.normal.z());
    RCLCPP_INFO(logger_, "  Radius: %.4f m, Depth: %.4f m, Clearance: %.4f m",
      circle.radius, circle.projection_depth, circle.clearance);

    TopoDS_Shape proj_disk = create_volume_from_circle(circle, false);
    TopoDS_Shape coll_disk = create_volume_from_circle(circle, true);

    RCLCPP_INFO(logger_, "  Projection disk null: %s, Collision disk null: %s",
      proj_disk.IsNull() ? "YES" : "NO", coll_disk.IsNull() ? "YES" : "NO");

    projection_volumes_.push_back(proj_disk);
    collision_volumes_.push_back(coll_disk);

    auto areas = process_constraint_volume(proj_disk, shape);
    RCLCPP_INFO(logger_, "  Extracted %zu sample areas (exclusion wires)", areas.size());
    for (const auto & area : areas) {
      RCLCPP_INFO(logger_, "    -> Surface %d affected", area.surface_id);
    }
    sample_areas_.insert(sample_areas_.end(), areas.begin(), areas.end());
  }

  // Process polygons
  for (size_t i = 0; i < polygons_.size(); ++i) {
    const auto & polygon = polygons_[i];
    RCLCPP_INFO(logger_, "Processing POLYGON exclusion %zu:", i);
    RCLCPP_INFO(logger_, "  Corners: %zu, Depth: %.4f m, Clearance: %.4f m",
      polygon.exclusion_corners.size(), polygon.projection_depth, polygon.clearance);
    for (size_t j = 0; j < polygon.exclusion_corners.size(); ++j) {
      const auto & corner = polygon.exclusion_corners[j];
      RCLCPP_INFO(logger_, "    Corner %zu: (%.4f, %.4f, %.4f)", j, corner.x(), corner.y(),
            corner.z());
    }

    TopoDS_Shape proj_prism = create_prism_from_polygon(polygon, false);
    TopoDS_Shape coll_prism = create_prism_from_polygon(polygon, true);

    RCLCPP_INFO(logger_, "  Projection prism null: %s, Collision prism null: %s",
      proj_prism.IsNull() ? "YES" : "NO", coll_prism.IsNull() ? "YES" : "NO");

    projection_volumes_.push_back(proj_prism);
    collision_volumes_.push_back(coll_prism);

    auto areas = process_constraint_volume(proj_prism, shape);
    RCLCPP_INFO(logger_, "  Extracted %zu sample areas (exclusion wires)", areas.size());
    for (const auto & area : areas) {
      RCLCPP_INFO(logger_, "    -> Surface %d affected", area.surface_id);
    }
    sample_areas_.insert(sample_areas_.end(), areas.begin(), areas.end());
  }

  RCLCPP_INFO(logger_, "Total exclusion volumes created: %zu", projection_volumes_.size());
  RCLCPP_INFO(logger_, "  Total collision volumes created: %zu", collision_volumes_.size());

  if (!sample_areas_.empty()) {
    std::map<int, int> surface_wire_count;
    for (const auto & area : sample_areas_) {
      surface_wire_count[area.surface_id]++;
    }

    if (!surface_wire_count.empty()) {
      RCLCPP_INFO(logger_, "  Affected surfaces:");
      for (const auto & [surface_id, count] : surface_wire_count) {
        RCLCPP_INFO(logger_, "    Surface %d: %d exclusion wire(s)", surface_id, count);
      }
    }
  }
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
    RCLCPP_DEBUG(logger_, "intersects_exclusion_zone: No collision volumes - returning false");
    return false;
  }

  RCLCPP_DEBUG(logger_,
        "intersects_exclusion_zone: Checking %zu collision volumes, grip_distance=%.4f m",
    collision_volumes_.size(), grip_distance);

  if (fcl_checker_ && fcl_checker_->is_valid()) {
    bool result = fcl_checker_->collides_with_exclusions(gripper_transform, grip_distance,
          tolerance);
    RCLCPP_DEBUG(logger_, "intersects_exclusion_zone: FCL check result = %s",
          result ? "COLLISION" : "clear");
    return result;
  }

  RCLCPP_DEBUG(logger_, "intersects_exclusion_zone: Using OCCT fallback (FCL not available)");

  TopoDS_Shape configured_gripper = io::configure_gripper(gripper_, grip_distance);

  BRepBuilderAPI_Transform transformer(configured_gripper, gripper_transform, Standard_True);
  TopoDS_Shape placed_gripper = transformer.Shape();

  // Mesh gripper for distance computation (coarser mesh OK for collision checks)
  BRepMesh_IncrementalMesh mesher(placed_gripper, mesh_linear_deflection_, Standard_False,
    mesh_angular_deflection_);

  // Check distance to each exclusion volume
  for (size_t i = 0; i < collision_volumes_.size(); ++i) {
    const auto & volume = collision_volumes_[i];
    BRepExtrema_DistShapeShape dist(placed_gripper, volume);

    if (dist.IsDone()) {
      RCLCPP_DEBUG(logger_, "Volume %zu: distance = %.6f m (tolerance = %.4f m)",
        i, dist.Value(), tolerance);
      if (dist.Value() <= tolerance) {
        RCLCPP_DEBUG(logger_, "-> COLLISION detected with exclusion volume %zu", i);
        return true;
      }
    } else {
      RCLCPP_WARN(logger_, "Volume %zu: Distance computation failed!", i);
    }
  }

  RCLCPP_DEBUG(logger_, "intersects_exclusion_zone: All volumes clear");
  return false;
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
