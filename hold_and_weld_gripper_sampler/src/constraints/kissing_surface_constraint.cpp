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
#include <limits>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <Bnd_Box.hxx>
#include <BRepBndLib.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeVertex.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <BRepBuilderAPI_WireError.hxx>
#include <BRepExtrema_DistShapeShape.hxx>
#include <BRepGProp.hxx>
#include <BRepLib.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <BRepTools.hxx>
#include <BRep_Builder.hxx>
#include <BRep_Tool.hxx>
#include <GCE2d_MakeSegment.hxx>
#include <Geom2d_TrimmedCurve.hxx>
#include <Geom_Surface.hxx>
#include <GProp_GProps.hxx>
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
#include "hold_and_weld_gripper_sampler/sampling/face_sampler.hpp"
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
  double mesh_angular_deflection,
  double contact_sample_density)
: mapper_(mapper),
  gripper_(gripper),
  secondary_shapes_(secondary_shapes),
  contact_threshold_(contact_threshold),
  collision_tolerance_(collision_tolerance),
  contact_distance_threshold_(contact_distance_threshold),
  mesh_linear_deflection_(mesh_linear_deflection),
  mesh_angular_deflection_(mesh_angular_deflection),
  contact_sample_density_(contact_sample_density),
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
    // Normal for a setup with no fixtures: ground is handled by GroundConstraint.
    RCLCPP_DEBUG(logger_, "No fixture shapes defined - skipping kissing surface analysis");
    return;
  }

  const auto & all_surfaces = topology.get_all_surfaces();

  RCLCPP_INFO(logger_, "Analyzing kissing surfaces: %zu surface(s), "
    "contact_threshold=%.1f%%",
    all_surfaces.size(), contact_threshold_ * 100.0);

  // No meshing pass here any more: measure_contact_ratio samples the face's
  // own UV domain, so primary triangulation is irrelevant to contact ratio.

  for (size_t i = 0; i < all_surfaces.size(); i++) {
    int surface_id = static_cast<int>(i);
    std::vector<sampling::FaceSample> contact_samples;
    double contact_ratio = measure_contact_ratio(surface_id, topology, &contact_samples);

    if (contact_ratio < 1e-9) {
      continue;
    }

    if (contact_ratio > contact_threshold_) {
      banned_surface_ids_.push_back(surface_id);
      RCLCPP_DEBUG(logger_, "  -> banned (%.1f%% > threshold %.1f%%)",
        contact_ratio * 100.0, contact_threshold_ * 100.0);
    } else {
      TopoDS_Wire boundary = sampling::bounding_wire_in_uv(
        all_surfaces[i].face, contact_samples);
      if (!boundary.IsNull()) {
        core::SampleArea area;
        area.surface_id = surface_id;
        area.wire = boundary;
        area.is_exclusion = true;
        partial_exclusions_.push_back(area);
        RCLCPP_DEBUG(logger_, "  -> partial exclusion wire created (%.1f%% contact)",
          contact_ratio * 100.0);
      } else {
        RCLCPP_WARN(logger_, "Surface %d: %.1f%% contact but wire extraction failed — "
          "fixture contact not excluded from sampling on this face",
          surface_id, contact_ratio * 100.0);
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
  const geometry::Topology & topology,
  std::vector<sampling::FaceSample> * contact_samples) const
{
  if (contact_samples != nullptr) {
    contact_samples->clear();
  }

  const auto & surface = topology.get_surface(surface_id);
  const TopoDS_Face & face = surface.face;

  // Cheap reject, if inflated not touching any other object it's safe
  Bnd_Box face_box;
  BRepBndLib::Add(face, face_box);
  if (face_box.IsVoid()) {
    return 0.0;
  }
  face_box.Enlarge(contact_distance_threshold_);

  std::vector<std::pair<const TopoDS_Shape *, Bnd_Box>> near_secondaries;
  for (const auto & secondary : secondary_shapes_) {
    if (secondary.IsNull()) {continue;}
    Bnd_Box secondary_box;
    BRepBndLib::Add(secondary, secondary_box);
    if (secondary_box.IsVoid() || face_box.IsOut(secondary_box)) {continue;}
    secondary_box.Enlarge(contact_distance_threshold_);
    near_secondaries.emplace_back(&secondary, secondary_box);
  }

  if (near_secondaries.empty()) {
    return 0.0;
  }

  // Sample the face itself rather than its triangulation. A planar face is two
  // triangles at any mesh deflection, so triangle-centroid sampling quantized
  // this ratio to {0, 0.5, 1.0} and reported 0% for real partial contact.
  sampling::FaceSamplingConfig sampling_config;
  sampling_config.sample_density = contact_sample_density_;
  sampling_config.layout = sampling::GridLayout::kCellCentres;

  const auto samples = sampling::sample_face_region(face, sampling_config);
  if (samples.empty()) {
    RCLCPP_DEBUG(logger_, "Surface %d: no samples produced", surface_id);
    return 0.0;
  }

  const double contact_ratio = sampling::area_fraction(
    samples,
    [this, &near_secondaries, contact_samples](const sampling::FaceSample & sample) {
      for (const auto & [secondary, secondary_box] : near_secondaries) {
        if (secondary_box.IsOut(sample.point)) {continue;}
        try {
          BRepExtrema_DistShapeShape dist(
            BRepBuilderAPI_MakeVertex(sample.point), *secondary);
          if (dist.Value() <= contact_distance_threshold_) {
            if (contact_samples != nullptr) {
              contact_samples->push_back(sample);
            }
            return true;
          }
        } catch (const Standard_Failure &) {
          continue;
        }
      }
      return false;
    });

  RCLCPP_DEBUG(logger_, "Surface %d: contact_ratio=%.1f%% (%zu samples, %zu secondaries near)",
    surface_id, contact_ratio * 100.0, samples.size(), near_secondaries.size());

  return contact_ratio;
}

bool KissingSurfaceConstraint::intersects_secondary(
  double grip_distance,
  const gp_Trsf & grasp_transform) const
{
  collision_stats_.total_checks++;

  if (secondary_shapes_.empty()) {
    return false;
  }

  if (!fcl_checker_ || !fcl_checker_->is_valid()) {
    RCLCPP_ERROR(logger_, "FCL checker not available — rejecting grasp conservatively");
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
