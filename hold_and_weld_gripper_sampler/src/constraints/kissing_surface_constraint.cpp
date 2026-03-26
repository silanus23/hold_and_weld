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
  const io::ParsedGripper & gripper,
  const std::vector<TopoDS_Shape> & secondary_shapes,
  double contact_threshold,
  double collision_tolerance)
: mapper_(mapper),
  gripper_(gripper),
  secondary_shapes_(secondary_shapes),
  contact_threshold_(contact_threshold),
  collision_tolerance_(collision_tolerance),
  fcl_checker_(nullptr),
  logger_(rclcpp::get_logger("gripper_sampler"))
{
  RCLCPP_DEBUG(logger_, "Initializing KissingSurfaceConstraint");
  RCLCPP_DEBUG(logger_, "Secondary shapes: %zu", secondary_shapes_.size());
  RCLCPP_DEBUG(logger_, "Contact threshold: %.1f%%", contact_threshold_ * 100.0);
  RCLCPP_DEBUG(logger_, "Collision tolerance: %.6f m", collision_tolerance_);

  Standard_Real lin_deflection = 0.0001;
  Standard_Real ang_deflection = 0.5;

  // Mesh secondary shapes with validation
  size_t mesh_failures = 0;
  for (size_t i = 0; i < secondary_shapes_.size(); ++i) {
    auto & secondary = secondary_shapes_[i];
    BRepMesh_IncrementalMesh mesher(secondary, lin_deflection, Standard_False, ang_deflection);
    if (!mesher.IsDone()) {
      RCLCPP_WARN(logger_, "Failed to mesh secondary shape %zu", i);
      mesh_failures++;
    }
  }

  // Mesh gripper components with validation
  BRepMesh_IncrementalMesh mesher_f1(gripper_.finger_1, lin_deflection, Standard_False,
    ang_deflection);
  if (!mesher_f1.IsDone()) {
    RCLCPP_WARN(logger_, "Failed to mesh gripper finger 1");
    mesh_failures++;
  }

  BRepMesh_IncrementalMesh mesher_f2(gripper_.finger_2, lin_deflection, Standard_False,
    ang_deflection);
  if (!mesher_f2.IsDone()) {
    RCLCPP_WARN(logger_, "Failed to mesh gripper finger 2");
    mesh_failures++;
  }

  BRepMesh_IncrementalMesh mesher_base(gripper_.base, lin_deflection, Standard_False,
    ang_deflection);
  if (!mesher_base.IsDone()) {
    RCLCPP_WARN(logger_, "Failed to mesh gripper base");
    mesh_failures++;
  }

  if (mesh_failures > 0) {
    RCLCPP_WARN(logger_, "Total meshing failures: %zu (FCL may fall back to OCCT)", mesh_failures);
  }
  RCLCPP_DEBUG(logger_, "Gripper and secondary shapes triangulated");
}


void KissingSurfaceConstraint::analyze_constraints(const geometry::Topology & topology)
{
  RCLCPP_INFO(logger_, "Analyzing kissing surface constraints");
  RCLCPP_INFO(logger_, "  Contact threshold: %.1f%%", contact_threshold_ * 100.0);
  RCLCPP_INFO(logger_, "  Secondary shapes count: %zu", secondary_shapes_.size());

  banned_surface_ids_.clear();
  partial_exclusions_.clear();

  if (secondary_shapes_.empty()) {
    RCLCPP_WARN(logger_, "[!] No secondary shapes defined - skipping analysis");
    return;
  }

  const auto & all_surfaces = topology.get_all_surfaces();
  RCLCPP_INFO(logger_, "Total surfaces to check: %zu", all_surfaces.size());

  size_t surfaces_with_contact = 0;
  size_t surfaces_no_contact = 0;

  for (size_t i = 0; i < all_surfaces.size(); i++) {
    int surface_id = static_cast<int>(i);

    const auto & surface = all_surfaces[i];
    gp_Pnt center = surface.center;

    RCLCPP_INFO(logger_, "Checking Surface %d:", surface_id);
    RCLCPP_INFO(logger_, "Center: (%.4f, %.4f, %.4f)",
      center.X(), center.Y(), center.Z());
    RCLCPP_INFO(logger_, "Normal: (%.3f, %.3f, %.3f)",
      surface.normal.X(), surface.normal.Y(), surface.normal.Z());

    double contact_ratio = measure_contact_ratio(surface_id, topology);

    if (contact_ratio < 1e-9) {
      surfaces_no_contact++;
      RCLCPP_INFO(logger_, "Result: NO CONTACT");
      continue;
    }

    surfaces_with_contact++;

    RCLCPP_INFO(logger_, "Surface %d: contact_ratio=%.1f%%",
      surface_id, contact_ratio * 100.0);

    if (contact_ratio > contact_threshold_) {
      banned_surface_ids_.push_back(surface_id);
      RCLCPP_INFO(logger_, "[BANNED] %.1f%% > threshold %.1f%%",
        contact_ratio * 100.0, contact_threshold_ * 100.0);
    } else {
      TopoDS_Wire boundary = extract_contact_boundary(surface_id, topology);

      if (!boundary.IsNull()) {
        core::SampleArea area;
        area.surface_id = surface_id;
        area.wire = boundary;
        // Reverse wire orientation so interior is excluded from sampling
        area.wire.Reverse();
        partial_exclusions_.push_back(area);
        RCLCPP_INFO(logger_, "[PARTIAL] %.1f%% contact - exclusion wire created",
          contact_ratio * 100.0);
      } else {
        RCLCPP_INFO(logger_, "[PARTIAL] %.1f%% contact - but wire extraction failed",
          contact_ratio * 100.0);
      }
    }
  }

  RCLCPP_INFO(logger_, "Kissing surface analysis SUMMARY:");
  RCLCPP_INFO(logger_, "Surfaces with NO contact: %zu", surfaces_no_contact);
  RCLCPP_INFO(logger_, "Surfaces with contact: %zu", surfaces_with_contact);
  RCLCPP_INFO(logger_, "-> BANNED (full contact): %zu", banned_surface_ids_.size());
  RCLCPP_INFO(logger_, "-> PARTIAL exclusions: %zu", partial_exclusions_.size());

  if (!banned_surface_ids_.empty()) {
    std::string banned_str = "  Banned surface IDs: [";
    for (size_t i = 0; i < banned_surface_ids_.size(); i++) {
      banned_str += std::to_string(banned_surface_ids_[i]);
      if (i < banned_surface_ids_.size() - 1) {
        banned_str += ", ";
      }
    }
    banned_str += "]";
    RCLCPP_INFO(logger_, "%s", banned_str.c_str());
  }
}

double KissingSurfaceConstraint::measure_contact_ratio(
  int surface_id,
  const geometry::Topology & topology) const
{
  const auto & surface = topology.get_surface(surface_id);
  TopoDS_Face face = surface.face;

  // Get the existing triangulation (meshed in constructor)
  TopLoc_Location location;
  Handle(Poly_Triangulation) triangulation = BRep_Tool::Triangulation(face, location);

  if (triangulation.IsNull() || triangulation->NbTriangles() == 0) {
    RCLCPP_DEBUG(logger_, "[measure_contact] Surface %d: no triangulation available", surface_id);
    return 0.0;
  }

  // Distance threshold for contact detection
  constexpr double contact_distance_threshold = 0.005;

  const gp_Trsf & trsf = location.Transformation();
  int num_triangles = triangulation->NbTriangles();

  double total_area = 0.0;
  double contact_area = 0.0;
  int contact_triangles = 0;

  double min_distance_found = std::numeric_limits<double>::max();
  gp_Pnt closest_centroid;
  size_t closest_secondary_idx = 0;

  RCLCPP_INFO(logger_,
        "[measure_contact] Surface %d: sampling %d mesh triangles (threshold: %.4f m)",
    surface_id, num_triangles, contact_distance_threshold);

  for (int tri_idx = 1; tri_idx <= num_triangles; ++tri_idx) {
    const Poly_Triangle & triangle = triangulation->Triangle(tri_idx);

    int n1, n2, n3;
    triangle.Get(n1, n2, n3);

    // Get triangle vertices in world space
    gp_Pnt p1 = triangulation->Node(n1).Transformed(trsf);
    gp_Pnt p2 = triangulation->Node(n2).Transformed(trsf);
    gp_Pnt p3 = triangulation->Node(n3).Transformed(trsf);

    // Compute triangle area via cross product: 0.5 * ||(p2-p1) x (p3-p1)||
    gp_Vec v1(p1, p2);
    gp_Vec v2(p1, p3);
    gp_Vec cross = v1.Crossed(v2);
    double tri_area = 0.5 * cross.Magnitude();

    if (tri_area < 1e-12) {
      continue;  // Skip degenerate triangles
    }

    total_area += tri_area;

    // Use triangle centroid as sample point
    gp_Pnt centroid(
      (p1.X() + p2.X() + p3.X()) / 3.0,
      (p1.Y() + p2.Y() + p3.Y()) / 3.0,
      (p1.Z() + p2.Z() + p3.Z()) / 3.0);

    // Check distance from centroid to each secondary shape
    bool in_contact = false;
    for (size_t sec_idx = 0; sec_idx < secondary_shapes_.size(); ++sec_idx) {
      const auto & secondary = secondary_shapes_[sec_idx];
      try {
        BRepExtrema_DistShapeShape dist(
          BRepBuilderAPI_MakeVertex(centroid).Shape(), secondary);

        if (dist.IsDone()) {
          double d = dist.Value();
          if (d < min_distance_found) {
            min_distance_found = d;
            closest_centroid = centroid;
            closest_secondary_idx = sec_idx;
          }
          if (d <= contact_distance_threshold) {
            in_contact = true;
            break;
          }
        }
      } catch (...) {
        RCLCPP_DEBUG(logger_, "[measure_contact] Distance query failed for secondary %zu",
          sec_idx);
        continue;
      }
    }

    if (in_contact) {
      contact_area += tri_area;
      contact_triangles++;
    }
  }

  if (total_area < 1e-9) {
    RCLCPP_DEBUG(logger_, "[measure_contact] Surface %d: degenerate (mesh area ~0)", surface_id);
    return 0.0;
  }

  double contact_ratio = contact_area / total_area;

  RCLCPP_INFO(logger_,
    "[measure_contact] Result: %d/%d triangles in contact, area-weighted ratio=%.1f%%",
    contact_triangles, num_triangles, contact_ratio * 100.0);
  RCLCPP_INFO(logger_,
    "[measure_contact] Contact area: %.6f m², Total area: %.6f m²",
    contact_area, total_area);
  RCLCPP_INFO(logger_,
    "[measure_contact] Min distance to secondaries: %.6f m",
    min_distance_found);
  if (min_distance_found < std::numeric_limits<double>::max()) {
    RCLCPP_INFO(logger_,
      "[measure_contact] Closest centroid: (%.4f, %.4f, %.4f) to secondary %zu",
      closest_centroid.X(), closest_centroid.Y(), closest_centroid.Z(),
      closest_secondary_idx);
  }

  return contact_ratio;
}

TopoDS_Wire KissingSurfaceConstraint::extract_contact_boundary(
  int surface_id,
  const geometry::Topology & topology) const
{
  const auto & surface = topology.get_surface(surface_id);
  TopoDS_Face face = surface.face;

  std::vector<TopoDS_Edge> boundary_edges;

  for (const auto & secondary : secondary_shapes_) {
    try {
      BRepAlgoAPI_Section section(face, secondary);
      section.Build();

      if (!section.IsDone()) {
        continue;
      }

      TopoDS_Shape section_result = section.Shape();

      if (section_result.IsNull()) {
        continue;
      }

      for (TopExp_Explorer exp(section_result, TopAbs_EDGE); exp.More(); exp.Next()) {
        TopoDS_Edge edge = TopoDS::Edge(exp.Current());
        boundary_edges.push_back(edge);
      }
    } catch (...) {
      RCLCPP_DEBUG(logger_, "Surface %d: Section operation failed for a secondary", surface_id);
      continue;
    }
  }

  if (!boundary_edges.empty()) {
    BRepBuilderAPI_MakeWire wire_builder;

    for (const auto & edge : boundary_edges) {
      wire_builder.Add(edge);

      if (wire_builder.Error() == BRepBuilderAPI_DisconnectedWire) {
        RCLCPP_DEBUG(logger_, "Surface %d: Disconnected edges in contact boundary", surface_id);
      }
    }

    if (wire_builder.IsDone()) {
      return wire_builder.Wire();
    } else {
      RCLCPP_DEBUG(logger_, "Surface %d: Failed to build wire from %zu edges",
        surface_id, boundary_edges.size());
    }
  }

  return TopoDS_Wire();
}

std::vector<int> KissingSurfaceConstraint::get_banned_surface_ids() const
{
  return banned_surface_ids_;
}

std::vector<core::SampleArea> KissingSurfaceConstraint::get_sample_areas() const
{
  return partial_exclusions_;
}

CollisionStats KissingSurfaceConstraint::get_collision_stats() const
{
  return collision_stats_;
}

void KissingSurfaceConstraint::reset_collision_stats()
{
  collision_stats_.reset();
}

void KissingSurfaceConstraint::set_fcl_checker(
  std::shared_ptr<const geometry::FCLCollisionChecker> fcl_checker)
{
  fcl_checker_ = fcl_checker;

  // Note: Ground plane must be added to FCL checker externally via add_ground_plane()
  // The FCL checker should have add_ground_plane() called on it before being passed here
  if (fcl_checker_) {
    RCLCPP_DEBUG(logger_, "FCL checker set for collision detection");
  }
}

bool KissingSurfaceConstraint::intersects_secondary(
  double grip_distance,
  const Eigen::Isometry3d & grasp_pose) const
{
  collision_stats_.total_checks++;

  // Ground plane collision is handled by FCL ground plane BVH (if added to FCL checker)
  // This provides accurate collision detection that accounts for gripper rotation and geometry
  // Simple approximate checks are not needed - FCL is fast and accurate

  if (secondary_shapes_.empty()) {
    return false;
  }

  try {
    gp_Trsf grasp_transform;
    grasp_transform.SetValues(
      grasp_pose(0, 0), grasp_pose(0, 1), grasp_pose(0, 2), grasp_pose(0, 3),
      grasp_pose(1, 0), grasp_pose(1, 1), grasp_pose(1, 2), grasp_pose(1, 3),
      grasp_pose(2, 0), grasp_pose(2, 1), grasp_pose(2, 2), grasp_pose(2, 3)
    );

    // Use FCL if available (fast path)
    if (fcl_checker_ && fcl_checker_->is_valid()) {
      bool collision = fcl_checker_->collides_with_secondaries(grasp_transform, grip_distance,
            collision_tolerance_);
      if (collision) {
        RCLCPP_DEBUG(logger_,
              "Grasp rejected: FCL detected collision with secondaries/ground (tolerance=%.6f m)",
                     collision_tolerance_);
        // Note: FCL checks both ground plane and secondaries, we can't distinguish here
        // This counts both ground and secondary collisions detected via FCL
        collision_stats_.fcl_secondary_rejections++;
      }
      return collision;
    }

    // Fallback to OCCT (slow path)
    RCLCPP_DEBUG(logger_, "Using OCCT fallback for collision checking (FCL unavailable)");

    TopoDS_Shape configured_gripper = io::configure_gripper(gripper_, grip_distance);

    BRepBuilderAPI_Transform transformer(configured_gripper, grasp_transform, Standard_True);
    TopoDS_Shape placed_gripper = transformer.Shape();

    Standard_Real lin_deflection = 0.0001;
    Standard_Real ang_deflection = 0.5;
    BRepMesh_IncrementalMesh mesher(placed_gripper, lin_deflection, Standard_False, ang_deflection);

    // Validate meshing succeeded
    if (!mesher.IsDone()) {
      RCLCPP_WARN(logger_, "Failed to mesh placed gripper for collision check - rejecting grasp");
      collision_stats_.mesh_failures++;
      return true;  // Conservative: reject if we can't verify
    }

    for (size_t i = 0; i < secondary_shapes_.size(); ++i) {
      const auto & secondary = secondary_shapes_[i];

      try {
        BRepExtrema_DistShapeShape dist(placed_gripper, secondary);

        // Validate distance computation succeeded
        if (!dist.IsDone()) {
          RCLCPP_WARN(logger_, "Distance computation failed for secondary %zu - rejecting grasp",
                i);
          collision_stats_.occt_failures++;
          return true;  // Conservative: reject if we can't verify
        }

        if (dist.Value() < collision_tolerance_) {
          RCLCPP_DEBUG(logger_,
                       "Grasp rejected: OCCT detected collision with secondary"
                       "%zu (distance=%.6f m < tolerance=%.6f m)",
                       i, dist.Value(), collision_tolerance_);
          collision_stats_.occt_secondary_rejections++;
          return true;
        }
      } catch (const Standard_Failure & e) {
        RCLCPP_WARN(logger_, "OCCT exception checking secondary %zu: %s - rejecting grasp",
                    i, e.GetMessageString());
        collision_stats_.occt_failures++;
        return true;
      }
    }

    return false;
  } catch (const Standard_Failure & e) {
    RCLCPP_WARN(logger_, "OCCT exception during collision check: %s", e.GetMessageString());
    collision_stats_.occt_failures++;
    return true;  // Conservative: reject on error
  } catch (const std::exception & e) {
    RCLCPP_WARN(logger_, "STL exception during collision check: %s", e.what());
    collision_stats_.occt_failures++;
    return true;  // Conservative: reject on error
  } catch (...) {
    RCLCPP_WARN(logger_, "Unknown exception during collision check");
    collision_stats_.occt_failures++;
    return true;  // Conservative: reject on error
  }
}

std::string KissingSurfaceConstraint::get_name() const
{
  return "KissingSurfaceConstraint";
}

const std::vector<TopoDS_Shape> & KissingSurfaceConstraint::get_secondary_shapes() const
{
  return secondary_shapes_;
}


}  // namespace constraints
}  // namespace hold_and_weld_gripper_sampler
