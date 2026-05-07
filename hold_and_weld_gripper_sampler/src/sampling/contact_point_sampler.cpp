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

#include "hold_and_weld_gripper_sampler/sampling/contact_point_sampler.hpp"

#include <algorithm>
#include <cmath>
#include <optional>
#include <tuple>
#include <unordered_map>
#include <vector>

#include <BRep_Tool.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakeVertex.hxx>
#include <BRepClass_FaceClassifier.hxx>
#include <BRepExtrema_DistShapeShape.hxx>
#include <BRepGProp.hxx>
#include <BRepTools.hxx>
#include <Geom_Surface.hxx>
#include <Geom2d_Curve.hxx>
#include <GeomLProp_SLProps.hxx>
#include <gp_Lin.hxx>
#include <gp_Pnt.hxx>
#include <gp_Pnt2d.hxx>
#include <gp_Vec.hxx>
#include <GProp_GProps.hxx>
#include <IntCurvesFace_ShapeIntersector.hxx>
#include <Standard_Failure.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <rclcpp/rclcpp.hpp>

#include "hold_and_weld_gripper_sampler/geometry/occt_utils.hpp"

// TODO(@silanus23): Surface pairing uses face_min_distance as a pre-filter which false-rejects
// valid pairs on organic/curved shapes where face edges are close but centers are graspable.
// Fix: remove distance check from find_surface_pairs, keep only normal angle pre-filter,
// let contact sampling handle distance validation entirely.
// This is required before using the system on non-prismatic workpieces.

namespace hold_and_weld_gripper_sampler
{
namespace sampling
{

static const rclcpp::Logger logger_ = rclcpp::get_logger("gripper_sampler");

ContactPointSampler::ContactPointSampler(const SamplingConfig & config)
: config_(config)
{
  const double raw = config_.alignment_threshold;
  config_.alignment_threshold = std::clamp(config_.alignment_threshold, 0.0, 1.0);
  if (config_.alignment_threshold != raw) {
    RCLCPP_WARN(
      logger_,
      "ContactPointSampler: alignment_threshold %.4f clamped to %.4f",
      raw, config_.alignment_threshold);
  }
}

std::vector<ContactPair> ContactPointSampler::generate_contact_pairs(
  const geometry::Topology & topology,
  const std::vector<int> & valid_surface_ids,
  const std::vector<core::SampleArea> & exclusion_areas) const
{
  std::vector<ContactPair> contact_pairs;

  size_t total_samples = 0;
  size_t rejected_no_opposing = 0;
  size_t rejected_exclusion = 0;
  size_t rejected_not_in_allowed_area = 0;
  size_t rejected_diagonal = 0;
  size_t rejected_grip_distance = 0;

  RCLCPP_INFO(logger_, "Starting contact point sampling on %zu surfaces, %zu exclusion areas",
    valid_surface_ids.size(), exclusion_areas.size());

  if (valid_surface_ids.empty()) {
    RCLCPP_WARN(logger_, "No valid surfaces provided - returning empty contact pairs");
    return contact_pairs;
  }

  auto surface_pairs = find_surface_pairs(topology, valid_surface_ids, exclusion_areas);

  RCLCPP_INFO(logger_, "Found %zu valid surface pairs", surface_pairs.size());

  if (surface_pairs.empty()) {
    RCLCPP_WARN(logger_, "No valid surface pairs found - check gripper opening range "
      "and surface normal requirements");
    return contact_pairs;
  }

  for (const auto & pair : surface_pairs) {
    auto contacts_1 = sample_surface(pair.face_1, pair.surface_id_1, exclusion_areas);

    RCLCPP_DEBUG(logger_, "Surface pair %d-%d: sampled %zu points on surface 1",
      pair.surface_id_1, pair.surface_id_2, contacts_1.size());

    for (const auto & contact_1 : contacts_1) {
      total_samples++;
      gp_Pnt contact_2;

      if (!find_opposing_contact(contact_1, pair.face_1, pair.face_2, contact_2)) {
        rejected_no_opposing++;
        continue;
      }

      if (is_point_in_exclusion(contact_2, pair.face_2, pair.surface_id_2, exclusion_areas)) {
        rejected_exclusion++;
        continue;
      }

      if (!is_point_in_allowed_area(contact_2, pair.face_2, pair.surface_id_2, exclusion_areas)) {
        rejected_not_in_allowed_area++;
        continue;
      }

      if (!is_valid_pairing(contact_1, contact_2, pair.face_1, pair.face_2)) {
        rejected_diagonal++;
        continue;
      }

      double grip_distance = contact_1.Distance(contact_2);

      if (grip_distance < config_.min_gripper_opening ||
        grip_distance > config_.max_gripper_opening)
      {
        rejected_grip_distance++;
        continue;
      }

      auto normal_1_opt = geometry::surface_normal_at_point(contact_1, pair.face_1);
      auto normal_2_opt = geometry::surface_normal_at_point(contact_2, pair.face_2);

      if (!normal_1_opt.has_value() || !normal_2_opt.has_value()) {
        rejected_no_opposing++;
        continue;
      }

      ContactPair cp;
      cp.contact_1 = contact_1;
      cp.contact_2 = contact_2;
      cp.surface_id_1 = pair.surface_id_1;
      cp.surface_id_2 = pair.surface_id_2;
      cp.face_1 = pair.face_1;
      cp.face_2 = pair.face_2;
      cp.normal_1 = normal_1_opt.value();
      cp.normal_2 = normal_2_opt.value();
      cp.grip_distance = grip_distance;

      contact_pairs.push_back(cp);
    }

    auto contacts_2 = sample_surface(pair.face_2, pair.surface_id_2, exclusion_areas);

    RCLCPP_DEBUG(logger_, "Surface pair %d-%d: sampled %zu points on surface 2",
      pair.surface_id_1, pair.surface_id_2, contacts_2.size());

    for (const auto & contact_2 : contacts_2) {
      total_samples++;
      gp_Pnt contact_1;

      if (!find_opposing_contact(contact_2, pair.face_2, pair.face_1, contact_1)) {
        rejected_no_opposing++;
        continue;
      }

      if (is_point_in_exclusion(contact_1, pair.face_1, pair.surface_id_1, exclusion_areas)) {
        rejected_exclusion++;
        continue;
      }

      if (!is_point_in_allowed_area(contact_1, pair.face_1, pair.surface_id_1, exclusion_areas)) {
        rejected_not_in_allowed_area++;
        continue;
      }

      if (!is_valid_pairing(contact_1, contact_2, pair.face_1, pair.face_2)) {
        rejected_diagonal++;
        continue;
      }

      double grip_distance = contact_1.Distance(contact_2);

      if (grip_distance < config_.min_gripper_opening ||
        grip_distance > config_.max_gripper_opening)
      {
        rejected_grip_distance++;
        continue;
      }

      auto normal_1_opt = geometry::surface_normal_at_point(contact_1, pair.face_1);
      auto normal_2_opt = geometry::surface_normal_at_point(contact_2, pair.face_2);

      if (!normal_1_opt.has_value() || !normal_2_opt.has_value()) {
        rejected_no_opposing++;
        continue;
      }

      ContactPair cp;
      cp.contact_1 = contact_1;
      cp.contact_2 = contact_2;
      cp.surface_id_1 = pair.surface_id_1;
      cp.surface_id_2 = pair.surface_id_2;
      cp.face_1 = pair.face_1;
      cp.face_2 = pair.face_2;
      cp.normal_1 = normal_1_opt.value();
      cp.normal_2 = normal_2_opt.value();
      cp.grip_distance = grip_distance;

      contact_pairs.push_back(cp);
    }
  }

  // Deduplicate contact pairs using grid-based bucketing
  size_t pairs_before_dedup = contact_pairs.size();
  contact_pairs = deduplicate_contact_pairs(contact_pairs, config_.sample_density / 2.0);
  size_t rejected_duplicate = pairs_before_dedup - contact_pairs.size();

  RCLCPP_INFO(logger_,
    "Contact point sampling complete: %zu samples tested, %zu valid pairs (after dedup)",
    total_samples, contact_pairs.size());

  if (total_samples > 0) {
    RCLCPP_DEBUG(logger_, "Rejection breakdown: no_opposing=%zu, exclusion=%zu, "
      "not_allowed=%zu, diagonal=%zu, grip_distance=%zu, duplicates=%zu",
      rejected_no_opposing, rejected_exclusion, rejected_not_in_allowed_area,
      rejected_diagonal, rejected_grip_distance, rejected_duplicate);
  }

  if (contact_pairs.empty()) {
    RCLCPP_WARN(logger_, "No valid contact pairs found - check surface geometry, "
      "exclusion zones, and gripper configuration");
  }

  return contact_pairs;
}

std::vector<SurfacePair> ContactPointSampler::find_surface_pairs(
  const geometry::Topology & topology,
  const std::vector<int> & valid_surface_ids,
  const std::vector<core::SampleArea> & exclusion_areas) const
{
  std::vector<SurfacePair> pairs;

  double min_dot = std::cos(config_.max_angle_deg * M_PI / 180.0);
  double max_dot = std::cos(config_.min_angle_deg * M_PI / 180.0);

  size_t total_pairs_checked = 0;
  size_t rejected_normals_not_antiparallel = 0;

  RCLCPP_INFO(logger_, "Surface pair analysis: %zu surfaces, angle [%.1f°, %.1f°]",
    valid_surface_ids.size(), config_.min_angle_deg, config_.max_angle_deg);

  for (size_t i = 0; i < valid_surface_ids.size(); i++) {
    for (size_t j = i + 1; j < valid_surface_ids.size(); j++) {
      int id1 = valid_surface_ids[i];
      int id2 = valid_surface_ids[j];
      total_pairs_checked++;

      const auto & s1 = topology.get_surface(id1);
      const auto & s2 = topology.get_surface(id2);

      if (!has_antiparallel_local_normals(s1.face, s2.face, id1, id2,
        exclusion_areas, min_dot, max_dot))
      {
        rejected_normals_not_antiparallel++;
        continue;
      }

      SurfacePair pair;
      pair.surface_id_1 = id1;
      pair.surface_id_2 = id2;
      pair.face_1 = s1.face;
      pair.face_2 = s2.face;
      pair.normal_1 = s1.normal;
      pair.normal_2 = s2.normal;

      pairs.push_back(pair);
    }
  }

  RCLCPP_INFO(logger_,
    "Surface pair analysis complete: %zu checked, %zu valid, rejected: normals=%zu",
    total_pairs_checked, pairs.size(), rejected_normals_not_antiparallel);

  return pairs;
}

std::vector<gp_Pnt> ContactPointSampler::sample_surface(
  const TopoDS_Face & face,
  int surface_id,
  const std::vector<core::SampleArea> & exclusion_areas) const
{
  RCLCPP_DEBUG(logger_, "Sampling surface %d with density %.4f m",
    surface_id, config_.sample_density);

  std::vector<std::pair<TopoDS_Wire, bool>> wires_with_flags;
  for (const auto & area : exclusion_areas) {
    if (area.surface_id == surface_id) {
      wires_with_flags.emplace_back(area.wire, area.is_exclusion);
    }
  }

  if (wires_with_flags.empty()) {
    auto points = sample_full_face(face);
    RCLCPP_DEBUG(logger_, "Surface %d: %zu points sampled", surface_id, points.size());
    return points;
  } else {
    auto points = sample_with_exclusions(face, wires_with_flags);
    RCLCPP_DEBUG(logger_, "Surface %d: %zu points sampled (with exclusions)", surface_id,
          points.size());
    return points;
  }
}

std::vector<gp_Pnt> ContactPointSampler::sample_full_face(const TopoDS_Face & face) const
{
  std::vector<gp_Pnt> points;

  Standard_Real u_min, u_max, v_min, v_max;
  BRepTools::UVBounds(face, u_min, u_max, v_min, v_max);

  double u_range = u_max - u_min;
  double v_range = v_max - v_min;

  Handle(Geom_Surface) surf = BRep_Tool::Surface(face);

  // Estimate physical arc length per UV unit at the face midpoint so that
  // sample_density is respected in metres regardless of surface parameterisation
  double u_mid = u_min + u_range * 0.5;
  double v_mid = v_min + v_range * 0.5;
  constexpr double kProbeEps = 1e-4;
  gp_Pnt p0 = surf->Value(u_mid, v_mid);
  gp_Pnt pu = surf->Value(std::min(u_mid + kProbeEps, u_max), v_mid);
  gp_Pnt pv = surf->Value(u_mid, std::min(v_mid + kProbeEps, v_max));
  double du_scale = p0.Distance(pu) / kProbeEps;   // metres per U-unit
  double dv_scale = p0.Distance(pv) / kProbeEps;   // metres per V-unit
  if (du_scale < 1e-9) {du_scale = 1.0;}
  if (dv_scale < 1e-9) {dv_scale = 1.0;}

  int u_steps = static_cast<int>(std::ceil(u_range * du_scale / config_.sample_density));
  int v_steps = static_cast<int>(std::ceil(v_range * dv_scale / config_.sample_density));

  if (u_steps < 1) {u_steps = 1;}
  if (v_steps < 1) {v_steps = 1;}

  for (int i = 0; i <= u_steps; i++) {
    for (int j = 0; j <= v_steps; j++) {
      double u = u_min + i * u_range / u_steps;
      double v = v_min + j * v_range / v_steps;

      gp_Pnt2d uv_point(u, v);
      BRepClass_FaceClassifier classifier(face, uv_point, 1e-6);
      TopAbs_State state = classifier.State();

      if (state != TopAbs_IN && state != TopAbs_ON) {
        continue;
      }

      gp_Pnt point = surf->Value(u, v);
      points.push_back(point);
    }
  }

  return points;
}

std::vector<gp_Pnt> ContactPointSampler::sample_with_exclusions(
  const TopoDS_Face & face,
  const std::vector<std::pair<TopoDS_Wire, bool>> & wires_with_flags) const
{
  std::vector<gp_Pnt> points;

  Standard_Real u_min, u_max, v_min, v_max;
  BRepTools::UVBounds(face, u_min, u_max, v_min, v_max);

  double u_range = u_max - u_min;
  double v_range = v_max - v_min;

  Handle(Geom_Surface) surf = BRep_Tool::Surface(face);

  // Estimate physical arc length per UV unit at the face midpoint — same fix
  // as sample_full_face so exclusion-zone sampling also honours sample_density in metres.
  double u_mid = u_min + u_range * 0.5;
  double v_mid = v_min + v_range * 0.5;
  constexpr double kProbeEps = 1e-4;
  gp_Pnt p0 = surf->Value(u_mid, v_mid);
  gp_Pnt pu = surf->Value(std::min(u_mid + kProbeEps, u_max), v_mid);
  gp_Pnt pv = surf->Value(u_mid, std::min(v_mid + kProbeEps, v_max));
  double du_scale = p0.Distance(pu) / kProbeEps;   // metres per U-unit
  double dv_scale = p0.Distance(pv) / kProbeEps;   // metres per V-unit
  if (du_scale < 1e-9) {du_scale = 1.0;}
  if (dv_scale < 1e-9) {dv_scale = 1.0;}

  int u_steps = static_cast<int>(std::ceil(u_range * du_scale / config_.sample_density));
  int v_steps = static_cast<int>(std::ceil(v_range * dv_scale / config_.sample_density));

  if (u_steps < 1) {u_steps = 1;}
  if (v_steps < 1) {v_steps = 1;}

  std::vector<std::pair<TopoDS_Face, bool>> wire_faces;
  wire_faces.reserve(wires_with_flags.size());
  for (const auto & [wire, is_excl] : wires_with_flags) {
    BRepBuilderAPI_MakeFace maker(surf, wire, Standard_True);
    if (maker.IsDone()) {
      wire_faces.emplace_back(maker.Face(), is_excl);
    } else {
      RCLCPP_WARN(logger_, "sample_with_exclusions: failed to build face for wire, skipping");
    }
  }

  for (int i = 0; i <= u_steps; i++) {
    for (int j = 0; j <= v_steps; j++) {
      double u = u_min + i * u_range / u_steps;
      double v = v_min + j * v_range / v_steps;

      gp_Pnt2d point_2d(u, v);

      BRepClass_FaceClassifier face_classifier(face, point_2d, 1e-6);
      TopAbs_State face_state = face_classifier.State();

      if (face_state != TopAbs_IN && face_state != TopAbs_ON) {
        continue;
      }

      bool is_excluded = false;
      for (const auto & [wire_face, is_exclusion_zone] : wire_faces) {
        bool inside_wire = is_point_inside_wire(point_2d, wire_face);

        if (is_exclusion_zone && inside_wire) {
          is_excluded = true;
          break;
        } else if (!is_exclusion_zone && !inside_wire) {
          is_excluded = true;
          break;
        }
      }

      if (is_excluded) {
        continue;
      }

      gp_Pnt point = surf->Value(u, v);
      points.push_back(point);
    }
  }

  return points;
}

bool ContactPointSampler::is_point_inside_wire(
  const gp_Pnt2d & point_2d,
  const TopoDS_Wire & wire,
  const TopoDS_Face & face) const
{
  Handle(Geom_Surface) surf = BRep_Tool::Surface(face);
  if (surf.IsNull()) {return false;}

  BRepBuilderAPI_MakeFace maker(surf, wire, Standard_True);
  if (!maker.IsDone()) {return false;}

  BRepClass_FaceClassifier classifier(maker.Face(), point_2d, 1e-6);
  const TopAbs_State state = classifier.State();
  return state == TopAbs_IN || state == TopAbs_ON;
}

bool ContactPointSampler::is_point_inside_wire(
  const gp_Pnt2d & point_2d,
  const TopoDS_Face & wire_face) const
{
  BRepClass_FaceClassifier classifier(wire_face, point_2d, 1e-6);
  const TopAbs_State state = classifier.State();
  return state == TopAbs_IN || state == TopAbs_ON;
}

bool ContactPointSampler::is_point_in_exclusion(
  const gp_Pnt & point_3d,
  const TopoDS_Face & face,
  int surface_id,
  const std::vector<core::SampleArea> & exclusion_areas) const
{
  std::vector<std::pair<TopoDS_Wire, bool>> wires_with_flags;
  for (const auto & area : exclusion_areas) {
    if (area.surface_id == surface_id) {
      wires_with_flags.emplace_back(area.wire, area.is_exclusion);
    }
  }

  if (wires_with_flags.empty()) {
    return false;
  }

  try {
    Handle(Geom_Surface) surf = BRep_Tool::Surface(face);
    GeomAPI_ProjectPointOnSurf projector(point_3d, surf);

    if (projector.NbPoints() == 0) {
      return false;
    }

    double u, v;
    projector.Parameters(1, u, v);
    gp_Pnt2d point_2d(u, v);

    for (const auto & [wire, is_exclusion_zone] : wires_with_flags) {
      bool inside_wire = is_point_inside_wire(point_2d, wire, face);

      if (is_exclusion_zone && inside_wire) {
        return true;
      } else if (!is_exclusion_zone && !inside_wire) {
        return true;
      }
    }
  } catch (const Standard_Failure & e) {
    RCLCPP_DEBUG(logger_, "Projection failed in exclusion check: %s", e.GetMessageString());
  }

  return false;
}

bool ContactPointSampler::is_point_in_allowed_area(
  const gp_Pnt & point_3d,
  const TopoDS_Face & face,
  int surface_id,
  const std::vector<core::SampleArea> & exclusion_areas) const
{
  std::vector<std::pair<TopoDS_Wire, bool>> wires_with_flags;
  for (const auto & area : exclusion_areas) {
    if (area.surface_id == surface_id) {
      wires_with_flags.emplace_back(area.wire, area.is_exclusion);
    }
  }

  if (wires_with_flags.empty()) {
    return true;
  }

  try {
    Handle(Geom_Surface) surf = BRep_Tool::Surface(face);
    GeomAPI_ProjectPointOnSurf projector(point_3d, surf);

    if (projector.NbPoints() == 0) {
      return false;
    }

    double u, v;
    projector.Parameters(1, u, v);
    gp_Pnt2d point_2d(u, v);

    for (const auto & [wire, is_exclusion_zone] : wires_with_flags) {
      bool inside_wire = is_point_inside_wire(point_2d, wire, face);

      if (is_exclusion_zone && inside_wire) {
        return false;
      } else if (!is_exclusion_zone && !inside_wire) {
        return false;
      }
    }
  } catch (const Standard_Failure & e) {
    RCLCPP_DEBUG(logger_, "Projection failed in allowed area check: %s", e.GetMessageString());
    return false;
  }

  return true;
}

bool ContactPointSampler::find_opposing_contact(
  const gp_Pnt & contact_1,
  const TopoDS_Face & face_1,
  const TopoDS_Face & face_2,
  gp_Pnt & opposing_contact) const
{
  if (!std::isfinite(contact_1.X()) ||
    !std::isfinite(contact_1.Y()) ||
    !std::isfinite(contact_1.Z()))
  {
    return false;
  }

  // Helper: shoot a ray from contact_1 in direction dir, return closest hit on face_2.
  auto try_ray = [&](const gp_Dir & dir) -> bool {
      IntCurvesFace_ShapeIntersector intersector;
      intersector.Load(face_2, 1e-6);
      intersector.Perform(gp_Lin(contact_1, dir), 1e-6, 1e10);
      if (intersector.NbPnt() == 0) {return false;}
      double best_dist = std::numeric_limits<double>::max();
      gp_Pnt best_pt;
      for (int k = 1; k <= intersector.NbPnt(); ++k) {
        double d = contact_1.Distance(intersector.Pnt(k));
        if (d > 1e-6 && d < best_dist) {
          best_dist = d;
          best_pt = intersector.Pnt(k);
        }
      }
      if (best_dist < std::numeric_limits<double>::max()) {
        opposing_contact = best_pt;
        return true;
      }
      return false;
    };

  try {
    // Primary: shoot along the surface normal at contact_1 (both directions).
    // This guarantees the ray is perpendicular to face_1, so the resulting
    // pair passes alignment checks regardless of where on the face contact_1 is.
    auto normal_opt = geometry::surface_normal_at_point(contact_1, face_1);
    if (normal_opt.has_value()) {
      gp_Vec n = normal_opt.value();
      n.Normalize();
      if (try_ray(gp_Dir(n.X(), n.Y(), n.Z()))) {return true;}
      if (try_ray(gp_Dir(-n.X(), -n.Y(), -n.Z()))) {return true;}
    }

    // Fallback: centroid-aimed ray (works for compact symmetric faces).
    GProp_GProps props;
    BRepGProp::SurfaceProperties(face_2, props);
    gp_Pnt centroid_2 = props.CentreOfMass();
    gp_Vec approach(centroid_2, contact_1);
    if (approach.Magnitude() > 1e-6) {
      approach.Normalize();
      if (try_ray(gp_Dir(-approach.X(), -approach.Y(), -approach.Z()))) {return true;}
      if (try_ray(gp_Dir(approach.X(), approach.Y(), approach.Z()))) {return true;}
    }
  } catch (const Standard_Failure &) {
  }

  RCLCPP_DEBUG(logger_, "find_opposing_contact: all ray attempts failed, skipping pair");
  return false;
}

bool ContactPointSampler::is_valid_pairing(
  const gp_Pnt & contact_1,
  const gp_Pnt & contact_2,
  const TopoDS_Face & face_1,
  const TopoDS_Face & face_2) const
{
  gp_Vec projection_vec(contact_1, contact_2);
  double projection_length = projection_vec.Magnitude();

  if (projection_length < 1e-6) {
    return false;
  }

  auto normal_1_opt = geometry::surface_normal_at_point(contact_1, face_1);
  auto normal_2_opt = geometry::surface_normal_at_point(contact_2, face_2);

  if (!normal_1_opt.has_value() || !normal_2_opt.has_value()) {
    return false;
  }

  gp_Vec normal_1 = normal_1_opt.value();
  gp_Vec normal_2 = normal_2_opt.value();

  projection_vec.Normalize();
  normal_1.Normalize();
  normal_2.Normalize();

  double alignment_1 = std::abs(projection_vec.Dot(normal_1));
  double alignment_2 = std::abs(projection_vec.Dot(normal_2));

  if (alignment_1 < config_.alignment_threshold) {
    return false;
  }

  if (alignment_2 < config_.alignment_threshold) {
    return false;
  }

  double min_alignment = std::min(alignment_1, alignment_2);
  // clamp guards sqrt of values slightly above 1.0 due to float precision
  double alignment_squared = std::min(1.0, min_alignment * min_alignment);
  double lateral_component = projection_length * std::sqrt(1.0 - alignment_squared);
  if (lateral_component > config_.max_lateral_deviation) {
    return false;
  }

  return true;
}

bool ContactPointSampler::has_antiparallel_local_normals(
  const TopoDS_Face & face_1,
  const TopoDS_Face & face_2,
  int surface_id_1,
  int surface_id_2,
  const std::vector<core::SampleArea> & exclusion_areas,
  double min_dot,
  double max_dot) const
{
  if (face_1.IsNull() || face_2.IsNull()) {
    return false;
  }

  Handle(Geom_Surface) surf_1 = BRep_Tool::Surface(face_1);
  Handle(Geom_Surface) surf_2 = BRep_Tool::Surface(face_2);

  std::vector<std::pair<TopoDS_Wire, bool>> wires_1, wires_2;
  for (const auto & area : exclusion_areas) {
    if (area.surface_id == surface_id_1) {
      wires_1.emplace_back(area.wire, area.is_exclusion);
    } else if (area.surface_id == surface_id_2) {
      wires_2.emplace_back(area.wire, area.is_exclusion);
    }
  }

  // Size both grids from the larger face so the large face is covered and
  // the small face gets extra samples — over-sampling the small side is harmless.
  GProp_GProps area_props_1, area_props_2;
  BRepGProp::SurfaceProperties(face_1, area_props_1);
  BRepGProp::SurfaceProperties(face_2, area_props_2);
  double max_area_cm2 = std::max(area_props_1.Mass(), area_props_2.Mass()) * 10000.0;
  int target_samples = std::max(10, std::min(100,
    static_cast<int>(max_area_cm2 * config_.normal_sample_density)));

  auto normals_1 = sample_normals_from_allowed_region(face_1, surf_1, wires_1, target_samples);
  auto normals_2 = sample_normals_from_allowed_region(face_2, surf_2, wires_2, target_samples);

  if (normals_1.empty() || normals_2.empty()) {
    RCLCPP_DEBUG(logger_,
      "No normals sampled from allowed regions - surfaces [%d, %d]",
      surface_id_1, surface_id_2);
    return false;
  }

  for (const auto & n1 : normals_1) {
    for (const auto & n2 : normals_2) {
      double dot = n1.Dot(n2);
      if (dot >= min_dot && dot <= max_dot) {
        RCLCPP_DEBUG(logger_,
          "Found antiparallel normals in allowed regions - surfaces [%d, %d], dot=%.3f",
          surface_id_1, surface_id_2, dot);
        return true;
      }
    }
  }

  return false;
}

std::vector<gp_Vec> ContactPointSampler::sample_normals_from_allowed_region(
  const TopoDS_Face & face,
  const Handle(Geom_Surface) & surf,
  const std::vector<std::pair<TopoDS_Wire, bool>> & wires_with_flags,
  int target_samples) const
{
  std::vector<gp_Vec> normals;

  Standard_Real u_min, u_max, v_min, v_max;
  BRepTools::UVBounds(face, u_min, u_max, v_min, v_max);

  // Oversample by 2x so the actual count exceeds target after exclusion filtering.
  int samples_per_dim = std::max(1, static_cast<int>(std::sqrt(target_samples * 2.0)));

  RCLCPP_DEBUG(logger_, "Sampling %d normals (target=%d, grid=%dx%d)",
    samples_per_dim * samples_per_dim, target_samples,
    samples_per_dim, samples_per_dim);

  std::vector<std::pair<TopoDS_Face, bool>> wire_faces;
  wire_faces.reserve(wires_with_flags.size());
  for (const auto & [wire, is_excl] : wires_with_flags) {
    BRepBuilderAPI_MakeFace maker(surf, wire, Standard_True);
    if (maker.IsDone()) {
      wire_faces.emplace_back(maker.Face(), is_excl);
    } else {
      RCLCPP_WARN(logger_,
            "sample_normals_from_allowed_region: failed to build face for wire, skipping");
    }
  }

  for (int i = 0; i < samples_per_dim; ++i) {
    for (int j = 0; j < samples_per_dim; ++j) {
      double u = u_min + (u_max - u_min) * (i + 0.5) / samples_per_dim;
      double v = v_min + (v_max - v_min) * (j + 0.5) / samples_per_dim;
      gp_Pnt2d point_2d(u, v);

      BRepClass_FaceClassifier classifier(face, point_2d, 1e-6);
      if (classifier.State() != TopAbs_IN && classifier.State() != TopAbs_ON) {
        continue;
      }

      if (!wires_with_flags.empty()) {
        bool is_allowed = true;

        for (const auto & [wire_face, is_exclusion_zone] : wire_faces) {
          bool inside_wire = is_point_inside_wire(point_2d, wire_face);

          if (is_exclusion_zone && inside_wire) {
            is_allowed = false;
            break;
          } else if (!is_exclusion_zone && !inside_wire) {
            is_allowed = false;
            break;
          }
        }

        if (!is_allowed) {
          continue;
        }
      }

      GeomLProp_SLProps props(surf, u, v, 1, 1e-6);
      if (props.IsNormalDefined()) {
        gp_Vec normal = props.Normal();
        if (face.Orientation() == TopAbs_REVERSED) {
          normal.Reverse();
        }
        normals.push_back(normal);
      }
    }
  }

  RCLCPP_DEBUG(logger_, "Sampled %zu normals from allowed region", normals.size());

  return normals;
}

std::vector<ContactPair> ContactPointSampler::deduplicate_contact_pairs(
  const std::vector<ContactPair> & pairs,
  double tolerance) const
{
  if (pairs.empty()) {
    return {};
  }

  // Grid-based spatial hashing to avoid domino effect
  // Key = (grid_cell_x1, grid_cell_y1, grid_cell_z1, grid_cell_x2, grid_cell_y2, grid_cell_z2)
  struct TupleHash
  {
    size_t operator()(const std::tuple<int, int, int, int, int, int> & t) const
    {
      size_t seed = 0;
      auto hash_combine = [&seed](int v) {
          seed ^= std::hash<int>{}(v) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        };
      hash_combine(std::get<0>(t)); hash_combine(std::get<1>(t));
      hash_combine(std::get<2>(t)); hash_combine(std::get<3>(t));
      hash_combine(std::get<4>(t)); hash_combine(std::get<5>(t));
      return seed;
    }
  };
  std::unordered_map<std::tuple<int, int, int, int, int, int>, ContactPair, TupleHash> grid_map;

  for (const auto & pair : pairs) {
    int x1 = static_cast<int>(std::floor(pair.contact_1.X() / tolerance));
    int y1 = static_cast<int>(std::floor(pair.contact_1.Y() / tolerance));
    int z1 = static_cast<int>(std::floor(pair.contact_1.Z() / tolerance));

    int x2 = static_cast<int>(std::floor(pair.contact_2.X() / tolerance));
    int y2 = static_cast<int>(std::floor(pair.contact_2.Y() / tolerance));
    int z2 = static_cast<int>(std::floor(pair.contact_2.Z() / tolerance));

    auto key_fwd = std::make_tuple(x1, y1, z1, x2, y2, z2);
    auto key_rev = std::make_tuple(x2, y2, z2, x1, y1, z1);

    if (grid_map.find(key_fwd) == grid_map.end() &&
      grid_map.find(key_rev) == grid_map.end())
    {
      grid_map[key_fwd] = pair;
    }
  }

  std::vector<ContactPair> result;
  result.reserve(grid_map.size());
  for (const auto & [key, pair] : grid_map) {
    result.push_back(pair);
  }

  RCLCPP_DEBUG(logger_, "Deduplication: %zu pairs -> %zu unique pairs (tolerance=%.4f m)",
    pairs.size(), result.size(), tolerance);

  return result;
}

}  // namespace sampling
}  // namespace hold_and_weld_gripper_sampler
