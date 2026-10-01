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
#include <limits>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <Bnd_Box.hxx>
#include <BRepBndLib.hxx>
#include <BRepClass3d_SolidClassifier.hxx>
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
#include <TopAbs_State.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Compound.hxx>
#include <TopTools_IndexedDataMapOfShapeListOfShape.hxx>
#include <TopTools_ListIteratorOfListOfShape.hxx>

#include "hold_and_weld_gripper_sampler/core/region_filter.hpp"
#include "hold_and_weld_gripper_sampler/geometry/topology.hpp"
#include "hold_and_weld_gripper_sampler/sampling/face_sampler.hpp"

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
  double mesh_angular_deflection,
  double sample_density)
: mapper_(mapper),
  gripper_(gripper),
  circles_(circles.value_or(std::vector<exclusion_circle> {})),
  polygons_(polygons.value_or(std::vector<exclusion_polygon> {})),
  lines_(lines.value_or(std::vector<exclusion_line> {})),
  fcl_checker_(nullptr),
  mesh_linear_deflection_(mesh_linear_deflection),
  mesh_angular_deflection_(mesh_angular_deflection),
  sample_density_(sample_density),
  logger_(rclcpp::get_logger("gripper_sampler"))
{
  RCLCPP_DEBUG(logger_, "ExclusionZoneConstraint: %zu lines, %zu circles, %zu polygons",
    lines_.size(), circles_.size(), polygons_.size());
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
    RCLCPP_WARN(logger_, "Line too short - skipping");
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
    // Build tube as a swept disk: circle edge -> wire -> face -> prism along axis.
    // This mirrors create_volume_from_circle and avoids BRepPrimAPI_MakeCylinder issues.
    gp_Vec arb = (std::abs(axis_dir.X()) < 0.9) ? gp_Vec(1, 0, 0) : gp_Vec(0, 1, 0);
    gp_Vec cross = gp_Vec(axis_dir.XYZ()).Crossed(arb);
    gp_Dir ref_dir(cross);
    gp_Ax2 disk_ax2(actual_start, axis_dir, ref_dir);

    BRepBuilderAPI_MakeEdge edge_maker(gp_Circ(disk_ax2, radius));
    BRepBuilderAPI_MakeWire wire_maker(edge_maker.Edge());
    BRepBuilderAPI_MakeFace disk_maker(wire_maker.Wire());
    gp_Vec extrude(axis_dir.X() * actual_length,
      axis_dir.Y() * actual_length,
      axis_dir.Z() * actual_length);
    TopoDS_Shape tube = BRepPrimAPI_MakePrism(disk_maker.Face(), extrude).Shape();
    if (tube.IsNull()) {
      RCLCPP_ERROR(logger_, "Tube: prism shape is null");
      return TopoDS_Shape();
    }
    BRepMesh_IncrementalMesh(tube, mesh_linear_deflection_, Standard_False,
          mesh_angular_deflection_);
    return tube;
  } catch (const Standard_Failure & e) {
    RCLCPP_ERROR(logger_, "Tube creation failed: %s", e.GetMessageString());
    return TopoDS_Shape();
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger_, "Standard exception in geometry creation: %s", e.what());
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
  // Both volumes start `clearance` below the zone's plane, so on a curved face the
  // surface that falls away from the plane is still inside.
  double extrusion_depth = circle.projection_depth + circle.clearance +
    (include_clearance ? circle.clearance : 0.0);

  try {
    BRepBuilderAPI_MakeEdge edge_maker(gp_Circ(axis, radius));
    BRepBuilderAPI_MakeWire wire_maker(edge_maker.Edge());
    BRepBuilderAPI_MakeFace disk_maker(wire_maker.Wire());

    TopoDS_Shape thick_disk = BRepPrimAPI_MakePrism(disk_maker.Face(),
          gp_Vec(normal.XYZ() * extrusion_depth)).Shape();

    gp_Trsf shift_back;
    shift_back.SetTranslation(gp_Vec(-normal.X() * circle.clearance,
          -normal.Y() * circle.clearance, -normal.Z() * circle.clearance));
    BRepBuilderAPI_Transform circle_transformer(thick_disk, shift_back, Standard_True);
    if (!circle_transformer.IsDone()) {
      RCLCPP_ERROR(logger_, "Circle volume: shift-back transform failed");
      return TopoDS_Shape();
    }
    thick_disk = circle_transformer.Shape();

    BRepMesh_IncrementalMesh(thick_disk, mesh_linear_deflection_, Standard_False,
          mesh_angular_deflection_);
    return thick_disk;
  } catch (const Standard_Failure & e) {
    RCLCPP_ERROR(logger_, "Circle volume creation failed: %s", e.GetMessageString());
    return TopoDS_Shape();
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger_, "Standard exception in circle volume creation: %s", e.what());
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
    RCLCPP_WARN(logger_, "Polygon exclusion corners 0-2 are collinear - cannot compute "
      "normal; start the corner list at a real corner");
    return TopoDS_Shape();
  }
  normal.normalize();

  try {
    BRepBuilderAPI_MakePolygon poly_builder;
    for (const auto & corner : polygon.exclusion_corners) {
      poly_builder.Add(gp_Pnt(corner.x(), corner.y(), corner.z()));
    }
    poly_builder.Close();

    TopoDS_Face poly_face;
    if (include_clearance) {
      BRepBuilderAPI_MakeFace temp_face(poly_builder.Wire());
      BRepOffsetAPI_MakeOffset offset_maker(temp_face, GeomAbs_Arc);
      offset_maker.Perform(polygon.clearance);

      // If offset collapses or fails, fallback to original polygon face
      if (offset_maker.IsDone() && !offset_maker.Shape().IsNull()) {
        poly_face = BRepBuilderAPI_MakeFace(TopoDS::Wire(offset_maker.Shape()));
      } else {
        RCLCPP_WARN(logger_, "Polygon offset failed - using original polygon");
        poly_face = temp_face;
      }
    } else {
      poly_face = BRepBuilderAPI_MakeFace(poly_builder.Wire());
    }

    // Starts `clearance` below the corners' plane, as in create_volume_from_circle.
    double depth = polygon.projection_depth + polygon.clearance +
      (include_clearance ? polygon.clearance : 0.0);
    gp_Vec extrusion = gp_Vec(normal.x(), normal.y(), normal.z()) * depth;

    TopoDS_Shape prism = BRepPrimAPI_MakePrism(poly_face, extrusion).Shape();

    gp_Trsf shift_back;
    shift_back.SetTranslation(gp_Vec(
      -normal.x() * polygon.clearance,
      -normal.y() * polygon.clearance,
      -normal.z() * polygon.clearance));
    BRepBuilderAPI_Transform prism_transformer(prism, shift_back, Standard_True);
    if (!prism_transformer.IsDone()) {
      RCLCPP_ERROR(logger_, "Polygon prism: shift-back transform failed");
      return TopoDS_Shape();
    }
    prism = prism_transformer.Shape();

    BRepMesh_IncrementalMesh(prism, mesh_linear_deflection_, Standard_False,
      mesh_angular_deflection_);
    return prism;
  } catch (const Standard_Failure & e) {
    RCLCPP_ERROR(logger_, "Prism creation failed: %s", e.GetMessageString());
    return TopoDS_Shape();
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger_, "Standard exception in geometry creation: %s", e.what());
    return TopoDS_Shape();
  }
}

std::vector<core::SampleArea> ExclusionZoneConstraint::process_constraint_volume(
  const TopoDS_Shape & constraint_volume,
  const geometry::Topology & topology,
  const std::string & zone_label,
  std::vector<std::string> & skipped) const
{
  constexpr double kOnVolumeTolerance = 1e-6;
  std::vector<core::SampleArea> sample_areas;

  if (constraint_volume.IsNull()) {
    return sample_areas;
  }

  try {
    Bnd_Box volume_box;
    BRepBndLib::Add(constraint_volume, volume_box);
    if (volume_box.IsVoid()) {
      return sample_areas;
    }
    volume_box.Enlarge(kOnVolumeTolerance);

    BRepClass3d_SolidClassifier classifier(constraint_volume);

    sampling::FaceSamplingConfig sampling_config;
    sampling_config.sample_density = sample_density_;
    sampling_config.layout = sampling::GridLayout::kCellCentres;

    // TODO(silanus23): linear scan over all_surfaces with a fresh BRepBndLib::Add per face,
    // repeated per constraint volume/zone. A spatial index (BVH/AABB tree) built once
    // over the surfaces and reused across zones would avoid both the O(n) rescan and
    // the redundant per-zone bbox recomputation. Not a bottleneck at current PoC scale.
    const auto & all_surfaces = topology.get_all_surfaces();
    for (size_t i = 0; i < all_surfaces.size(); ++i) {
      const int surface_id = static_cast<int>(i);
      const TopoDS_Face & face = all_surfaces[i].face;

      // One face failing must not drop the rest of this zone.
      try {
        Bnd_Box face_box;
        BRepBndLib::Add(face, face_box);
        if (face_box.IsVoid() || face_box.IsOut(volume_box)) {
          continue;
        }

        const auto samples = sampling::sample_face_region(face, sampling_config);
        if (samples.empty()) {
          RCLCPP_DEBUG(logger_, "Surface %d: no samples produced", surface_id);
          continue;
        }

        std::vector<sampling::FaceSample> excluded;
        for (const auto & sample : samples) {
          if (volume_box.IsOut(sample.point)) {continue;}
          classifier.Perform(sample.point, kOnVolumeTolerance);
          const TopAbs_State state = classifier.State();
          if (state == TopAbs_IN || state == TopAbs_ON) {
            excluded.push_back(sample);
          }
        }

        if (excluded.empty()) {
          continue;
        }

        const TopoDS_Wire boundary = sampling::bounding_wire_in_uv(face, excluded);
        if (boundary.IsNull()) {
          RCLCPP_WARN(logger_,
            "Surface %d: %zu sample(s) in exclusion zone but wire extraction failed — "
            "zone not excluded from sampling on this face", surface_id, excluded.size());
          skipped.push_back(
            zone_label + ": not excluded from sampling on surface " +
            std::to_string(surface_id) + " (wire extraction failed)");
          continue;
        }

        core::SampleArea area;
        area.surface_id = surface_id;
        area.wire = boundary;
        area.is_exclusion = true;
        sample_areas.push_back(area);

        RCLCPP_DEBUG(logger_, "Exclusion wire created for surface %d (%zu of %zu samples)",
          surface_id, excluded.size(), samples.size());
      } catch (const Standard_Failure & e) {
        RCLCPP_WARN(logger_, "Surface %d: exclusion footprint measurement failed: %s — "
          "zone not excluded from sampling on this face", surface_id, e.GetMessageString());
        skipped.push_back(
          zone_label + ": not excluded from sampling on surface " + std::to_string(surface_id) +
          " (" + e.GetMessageString() + ")");
      } catch (const std::exception & e) {
        RCLCPP_WARN(logger_, "Surface %d: exception in exclusion processing: %s — "
          "zone not excluded from sampling on this face", surface_id, e.what());
        skipped.push_back(
          zone_label + ": not excluded from sampling on surface " + std::to_string(surface_id) +
          " (" + e.what() + ")");
      }
    }
  } catch (const Standard_Failure & e) {
    RCLCPP_ERROR(logger_, "Exclusion footprint measurement failed: %s", e.GetMessageString());
    skipped.push_back(
      zone_label + ": not excluded from sampling (" + e.GetMessageString() + ")");
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger_, "Exception in constraint processing: %s", e.what());
    skipped.push_back(zone_label + ": not excluded from sampling (" + e.what() + ")");
  }

  return sample_areas;
}

void ExclusionZoneConstraint::analyze_constraints(
  const TopoDS_Shape & shape,
  const geometry::Topology & topology)
{
  (void)shape;

  sample_areas_.clear();
  projection_volumes_.clear();
  collision_volumes_.clear();
  collision_volume_labels_.clear();
  skipped_.clear();

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
    RCLCPP_DEBUG(logger_, "Line exclusion %zu%s: start=(%.4f,%.4f,%.4f) end=(%.4f,%.4f,%.4f)",
      i, id_suffix(line.id).c_str(),
      line.start.x(), line.start.y(), line.start.z(),
      line.end.x(), line.end.y(), line.end.z());
    if (2.0 * line.exclusion_radius < sample_density_) {
      RCLCPP_WARN(logger_, "Line exclusion %zu%s is %.4f m wide, narrower than "
        "exclusion_zones.sample_density (%.4f m) — it may fall between face samples and "
        "go unexcluded; lower exclusion_zones.sample_density below half this width to fix",
        i, id_suffix(line.id).c_str(), 2.0 * line.exclusion_radius, sample_density_);
    }

    TopoDS_Shape proj = create_tube_from_line(line, false);
    TopoDS_Shape coll = create_tube_from_line(line, true);

    const std::string label = "line exclusion zone " + std::to_string(i) + id_suffix(line.id);
    add_zone(proj, coll, label, topology);
  }

  for (size_t i = 0; i < circles_.size(); ++i) {
    const auto & circle = circles_[i];
    RCLCPP_DEBUG(logger_, "Circle exclusion %zu%s: center=(%.4f,%.4f,%.4f) r=%.4f m",
      i, id_suffix(circle.id).c_str(),
      circle.center.x(), circle.center.y(), circle.center.z(), circle.radius);
    if (2.0 * circle.radius < sample_density_) {
      RCLCPP_WARN(logger_, "Circle exclusion %zu%s is %.4f m across, narrower than "
        "exclusion_zones.sample_density (%.4f m) — it may fall between face samples and "
        "go unexcluded; lower exclusion_zones.sample_density below half this width to fix",
        i, id_suffix(circle.id).c_str(), 2.0 * circle.radius, sample_density_);
    }

    TopoDS_Shape proj = create_volume_from_circle(circle, false);
    TopoDS_Shape coll = create_volume_from_circle(circle, true);

    const std::string label = "circle exclusion zone " + std::to_string(i) + id_suffix(circle.id);
    add_zone(proj, coll, label, topology);
  }

  for (size_t i = 0; i < polygons_.size(); ++i) {
    const auto & polygon = polygons_[i];
    RCLCPP_DEBUG(logger_, "Polygon exclusion %zu%s: %zu corners, depth=%.4f m",
      i, id_suffix(polygon.id).c_str(),
      polygon.exclusion_corners.size(), polygon.projection_depth);

    // Min edge length is a cheap proxy for "narrowest dimension", a long thin polygon
    // can still have long edges while being narrow across, so this can under-warn.
    double min_edge_length = std::numeric_limits<double>::max();
    for (size_t c = 0; c < polygon.exclusion_corners.size(); ++c) {
      const auto & a = polygon.exclusion_corners[c];
      const auto & b = polygon.exclusion_corners[(c + 1) % polygon.exclusion_corners.size()];
      min_edge_length = std::min(min_edge_length, (b - a).norm());
    }
    if (!polygon.exclusion_corners.empty() && min_edge_length < sample_density_) {
      RCLCPP_WARN(logger_, "Polygon exclusion %zu%s has an edge %.4f m long, narrower than "
        "exclusion_zones.sample_density (%.4f m) — it may fall between face samples and "
        "go unexcluded; lower exclusion_zones.sample_density below this width to fix",
        i, id_suffix(polygon.id).c_str(), min_edge_length, sample_density_);
    }

    TopoDS_Shape proj = create_prism_from_polygon(polygon, false);
    TopoDS_Shape coll = create_prism_from_polygon(polygon, true);

    const std::string label = "polygon exclusion zone " + std::to_string(i) + id_suffix(polygon.id);
    add_zone(proj, coll, label, topology);
  }

  std::map<int, int> surface_counts;
  for (const auto & a : sample_areas_) {
    surface_counts[a.surface_id]++;
  }
  const size_t affected_surfaces = surface_counts.size();

  RCLCPP_INFO(logger_, "Exclusion analysis complete: %zu volume(s), %zu wire(s) "
    "on %zu surface(s)",
    collision_volumes_.size(), sample_areas_.size(), affected_surfaces);
}

void ExclusionZoneConstraint::add_zone(
  const TopoDS_Shape & projection_volume,
  const TopoDS_Shape & collision_volume,
  const std::string & label,
  const geometry::Topology & topology)
{
  if (projection_volume.IsNull() || collision_volume.IsNull()) {
    RCLCPP_ERROR(logger_, "Skipping %s due to geometry failure — this zone will NOT be enforced",
      label.c_str());
    skipped_.push_back(label + ": not enforced (geometry failure)");
    return;
  }
  projection_volumes_.push_back(projection_volume);
  collision_volumes_.push_back(collision_volume);
  collision_volume_labels_.push_back(label);
  auto areas = process_constraint_volume(projection_volume, topology, label, skipped_);
  sample_areas_.insert(sample_areas_.end(), areas.begin(), areas.end());
  RCLCPP_DEBUG(logger_, "  -> %zu exclusion wire(s) extracted", areas.size());
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
    RCLCPP_ERROR_ONCE(logger_, "FCL checker not available — rejecting every grasp conservatively");
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

const std::vector<std::string> & ExclusionZoneConstraint::get_collision_volume_labels() const
{
  return collision_volume_labels_;
}

const std::vector<std::string> & ExclusionZoneConstraint::get_skipped() const
{
  return skipped_;
}

}  // namespace constraints
}  // namespace hold_and_weld_gripper_sampler
