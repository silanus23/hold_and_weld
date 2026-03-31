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

#include "hold_and_weld_gripper_sampler/geometry/occt_utils.hpp"

#include <algorithm>
#include <cmath>
#include <map>
#include <optional>
#include <tuple>
#include <vector>

#include <BRepBuilderAPI_MakeVertex.hxx>
#include <BRepClass_FaceClassifier.hxx>
#include <BRepExtrema_DistShapeShape.hxx>
#include <BRepGProp.hxx>
#include <BRepTools.hxx>
#include <BRep_Tool.hxx>
#include <Geom2d_Curve.hxx>
#include <GeomLProp_SLProps.hxx>
#include <Geom_Surface.hxx>
#include <gp_Pnt.hxx>
#include <gp_Pnt2d.hxx>
#include <gp_Vec.hxx>
#include <GProp_GProps.hxx>
#include <rclcpp/rclcpp.hpp>
#include <Standard_Failure.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>

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
}

std::vector<ContactPair> ContactPointSampler::generate_contact_pairs(
  const geometry::Topology & topology,
  const std::vector<int> & valid_surface_ids,
  const std::vector<core::SampleArea> & exclusion_areas)
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

  // For each surface pair, generate contact point pairs bidirectionally
  for (const auto & pair : surface_pairs) {
    // Direction 1: Sample from surface_1, project to surface_2
    auto contacts_1 = sample_surface(pair.face_1, pair.surface_id_1, exclusion_areas);

    RCLCPP_DEBUG(logger_, "Surface pair %d-%d: sampled %zu points on surface 1",
      pair.surface_id_1, pair.surface_id_2, contacts_1.size());

    for (const auto & contact_1 : contacts_1) {
      total_samples++;
      gp_Pnt contact_2;

      if (!find_opposing_contact(contact_1, pair.face_2, contact_2)) {
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

    // Direction 2: Sample from surface_2, project to surface_1
    auto contacts_2 = sample_surface(pair.face_2, pair.surface_id_2, exclusion_areas);

    RCLCPP_DEBUG(logger_, "Surface pair %d-%d: sampled %zu points on surface 2",
      pair.surface_id_1, pair.surface_id_2, contacts_2.size());

    for (const auto & contact_2 : contacts_2) {
      total_samples++;
      gp_Pnt contact_1;

      if (!find_opposing_contact(contact_2, pair.face_1, contact_1)) {
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

  // Convert angle limits to dot product range
  double min_dot = std::cos(config_.max_angle_deg * M_PI / 180.0);
  double max_dot = std::cos(config_.min_angle_deg * M_PI / 180.0);

  size_t total_pairs_checked = 0;
  size_t rejected_distance_too_small = 0;
  size_t rejected_distance_too_large = 0;
  size_t rejected_normals_not_antiparallel = 0;

  RCLCPP_INFO(logger_, "Surface pair analysis: %zu surfaces, angle [%.1f°, %.1f°], "
    "distance [%.4f, %.4f] m",
    valid_surface_ids.size(),
    config_.min_angle_deg, config_.max_angle_deg,
    config_.min_gripper_opening, config_.max_gripper_opening);

  for (size_t i = 0; i < valid_surface_ids.size(); i++) {
    for (size_t j = i + 1; j < valid_surface_ids.size(); j++) {
      int id1 = valid_surface_ids[i];
      int id2 = valid_surface_ids[j];
      total_pairs_checked++;

      const auto & s1 = topology.get_surface(id1);
      const auto & s2 = topology.get_surface(id2);

      // Check 1: Distance is within gripper range (CHEAP - do first)
      double min_distance = geometry::face_min_distance(s1.face, s2.face);
      if (min_distance < config_.min_gripper_opening) {
        rejected_distance_too_small++;
        RCLCPP_DEBUG(logger_, "Pair [%d, %d]: REJECTED - distance %.4f m < min %.4f m",
          id1, id2, min_distance, config_.min_gripper_opening);
        continue;
      }
      if (min_distance > config_.max_gripper_opening) {
        rejected_distance_too_large++;
        RCLCPP_DEBUG(logger_, "Pair [%d, %d]: REJECTED - distance %.4f m > max %.4f m",
          id1, id2, min_distance, config_.max_gripper_opening);
        continue;
      }

      // Check 2: Sample-based normal validation within allowed regions
      if (!has_antiparallel_local_normals(s1.face, s2.face, id1, id2,
        exclusion_areas, min_dot, max_dot))
      {
        rejected_normals_not_antiparallel++;
        RCLCPP_DEBUG(logger_, "Pair [%d, %d]: REJECTED - no antiparallel normals in allowed regions",
          id1, id2);
        continue;
      }

      SurfacePair pair;
      pair.surface_id_1 = id1;
      pair.surface_id_2 = id2;
      pair.face_1 = s1.face;
      pair.face_2 = s2.face;
      pair.normal_1 = s1.normal;
      pair.normal_2 = s2.normal;
      pair.min_distance = min_distance;

      pairs.push_back(pair);

      RCLCPP_DEBUG(logger_, "Pair [%d, %d]: VALID - distance=%.4f m",
        id1, id2, min_distance);
    }
  }

  RCLCPP_INFO(logger_,
    "Surface pair analysis complete: %zu checked, %zu valid, "
    "rejected: dist_small=%zu, dist_large=%zu, normals=%zu",
    total_pairs_checked, pairs.size(),
    rejected_distance_too_small, rejected_distance_too_large,
    rejected_normals_not_antiparallel);

  return pairs;
}

std::vector<gp_Pnt> ContactPointSampler::sample_surface(
  const TopoDS_Face & face,
  int surface_id,
  const std::vector<core::SampleArea> & exclusion_areas) const
{
  RCLCPP_DEBUG(logger_, "Sampling surface %d with density %.4f m",
    surface_id, config_.sample_density);

  std::vector<TopoDS_Wire> wires;
  for (const auto & area : exclusion_areas) {
    if (area.surface_id == surface_id) {
      wires.push_back(area.wire);
    }
  }

  if (wires.empty()) {
    auto points = sample_full_face(face);
    RCLCPP_DEBUG(logger_, "Surface %d: %zu points sampled", surface_id, points.size());
    return points;
  } else {
    auto points = sample_with_exclusions(face, wires);
    RCLCPP_DEBUG(logger_, "Surface %d: %zu points sampled (with exclusions)", surface_id, points.size());
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
  int u_steps = static_cast<int>(std::ceil(u_range / config_.sample_density));
  int v_steps = static_cast<int>(std::ceil(v_range / config_.sample_density));

  if (u_steps < 1) {u_steps = 1;}
  if (v_steps < 1) {v_steps = 1;}

  Handle(Geom_Surface) surf = BRep_Tool::Surface(face);

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
  const std::vector<TopoDS_Wire> & wires) const
{
  std::vector<gp_Pnt> points;

  Standard_Real u_min, u_max, v_min, v_max;
  BRepTools::UVBounds(face, u_min, u_max, v_min, v_max);

  double u_range = u_max - u_min;
  double v_range = v_max - v_min;
  int u_steps = static_cast<int>(std::ceil(u_range / config_.sample_density));
  int v_steps = static_cast<int>(std::ceil(v_range / config_.sample_density));

  if (u_steps < 1) {u_steps = 1;}
  if (v_steps < 1) {v_steps = 1;}

  Handle(Geom_Surface) surf = BRep_Tool::Surface(face);

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
      for (const auto & wire : wires) {
        bool inside_wire = is_point_inside_wire(point_2d, wire, face);
        bool is_exclusion_zone = (wire.Orientation() == TopAbs_REVERSED);

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
  int crossings = 0;
  double ray_y = point_2d.Y();
  double point_x = point_2d.X();

  for (TopExp_Explorer exp(wire, TopAbs_EDGE); exp.More(); exp.Next()) {
    TopoDS_Edge edge = TopoDS::Edge(exp.Current());

    Standard_Real first, last;
    Handle(Geom2d_Curve) curve_2d = BRep_Tool::CurveOnSurface(edge, face, first, last);

    if (curve_2d.IsNull()) {
      continue;
    }

    double param_range = std::abs(last - first);
    int num_samples = std::max(10, std::min(100, static_cast<int>(param_range * 50)));

    for (int i = 0; i < num_samples; i++) {
      double t1 = first + i * (last - first) / num_samples;
      double t2 = first + (i + 1) * (last - first) / num_samples;

      gp_Pnt2d p1 = curve_2d->Value(t1);
      gp_Pnt2d p2 = curve_2d->Value(t2);

      double y1 = p1.Y();
      double y2 = p2.Y();

      if ((y1 <= ray_y && y2 > ray_y) || (y2 <= ray_y && y1 > ray_y)) {
        double x_intercept = p1.X() + (ray_y - y1) * (p2.X() - p1.X()) / (y2 - y1);

        if (x_intercept > point_x) {
          crossings++;
        }
      }
    }
  }

  return crossings % 2 == 1;
}

bool ContactPointSampler::is_point_in_exclusion(
  const gp_Pnt & point_3d,
  const TopoDS_Face & face,
  int surface_id,
  const std::vector<core::SampleArea> & exclusion_areas) const
{
  std::vector<TopoDS_Wire> wires;
  for (const auto & area : exclusion_areas) {
    if (area.surface_id == surface_id) {
      wires.push_back(area.wire);
    }
  }

  if (wires.empty()) {
    return false;
  }

  Handle(Geom_Surface) surf = BRep_Tool::Surface(face);
  GeomAPI_ProjectPointOnSurf projector(point_3d, surf);

  if (projector.NbPoints() == 0) {
    return false;
  }

  double u, v;
  projector.Parameters(1, u, v);
  gp_Pnt2d point_2d(u, v);

  for (const auto & wire : wires) {
    bool inside_wire = is_point_inside_wire(point_2d, wire, face);
    bool is_exclusion_zone = (wire.Orientation() == TopAbs_REVERSED);

    if (is_exclusion_zone && inside_wire) {
      return true;
    } else if (!is_exclusion_zone && !inside_wire) {
      return true;
    }
  }

  return false;
}

bool ContactPointSampler::is_point_in_allowed_area(
  const gp_Pnt & point_3d,
  const TopoDS_Face & face,
  int surface_id,
  const std::vector<core::SampleArea> & exclusion_areas) const
{
  std::vector<TopoDS_Wire> wires;
  for (const auto & area : exclusion_areas) {
    if (area.surface_id == surface_id) {
      wires.push_back(area.wire);
    }
  }

  if (wires.empty()) {
    return true;
  }

  Handle(Geom_Surface) surf = BRep_Tool::Surface(face);
  GeomAPI_ProjectPointOnSurf projector(point_3d, surf);

  if (projector.NbPoints() == 0) {
    return false;
  }

  double u, v;
  projector.Parameters(1, u, v);
  gp_Pnt2d point_2d(u, v);

  for (const auto & wire : wires) {
    bool inside_wire = is_point_inside_wire(point_2d, wire, face);
    bool is_exclusion_zone = (wire.Orientation() == TopAbs_REVERSED);

    if (is_exclusion_zone && inside_wire) {
      return false;
    } else if (!is_exclusion_zone && !inside_wire) {
      return false;
    }
  }

  return true;
}

bool ContactPointSampler::find_opposing_contact(
  const gp_Pnt & contact_1,
  const TopoDS_Face & face_2,
  gp_Pnt & opposing_contact) const
{
  // Guard: reject degenerate input coordinates
  if (!std::isfinite(contact_1.X()) ||
    !std::isfinite(contact_1.Y()) ||
    !std::isfinite(contact_1.Z()))
  {
    return false;
  }

  // Build a vertex from contact_1 and find the nearest point on the
  // *trimmed* face_2 (not the infinite underlying surface).
  BRepBuilderAPI_MakeVertex vertex_maker(contact_1);
  if (!vertex_maker.IsDone()) {
    return false;
  }

  BRepExtrema_DistShapeShape dist(vertex_maker.Vertex(), face_2);
  if (!dist.IsDone() || dist.NbSolution() == 0) {
    return false;
  }

  opposing_contact = dist.PointOnShape2(1);
  return true;
}

bool ContactPointSampler::is_valid_pairing(
  const gp_Pnt & contact_1,
  const gp_Pnt & contact_2,
  const TopoDS_Face & face_1,
  const TopoDS_Face & face_2) const
{
  gp_Vec projection_vec(contact_1, contact_2);
  double projection_length = projection_vec.Magnitude();

  if (projection_length < 1e-9) {
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

  std::vector<TopoDS_Wire> wires_1, wires_2;
  for (const auto & area : exclusion_areas) {
    if (area.surface_id == surface_id_1) {
      wires_1.push_back(area.wire);
    } else if (area.surface_id == surface_id_2) {
      wires_2.push_back(area.wire);
    }
  }

  auto normals_1 = sample_normals_from_allowed_region(
    face_1, surf_1, wires_1, 10, 100, config_.normal_sample_density);
  auto normals_2 = sample_normals_from_allowed_region(
    face_2, surf_2, wires_2, 10, 100, config_.normal_sample_density);

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
  const std::vector<TopoDS_Wire> & wires,
  int min_samples,
  int max_samples,
  double samples_per_cm2) const
{
  std::vector<gp_Vec> normals;

  Standard_Real u_min, u_max, v_min, v_max;
  BRepTools::UVBounds(face, u_min, u_max, v_min, v_max);

  int target_samples = min_samples;

  if (wires.empty()) {
    GProp_GProps props;
    BRepGProp::SurfaceProperties(face, props);
    double area_cm2 = props.Mass() * 10000.0;
    target_samples = static_cast<int>(area_cm2 * samples_per_cm2);
  } else {
    int coarse_u = 20;
    int coarse_v = 20;
    int allowed_count = 0;
    int total_count = 0;

    for (int i = 0; i <= coarse_u; i++) {
      for (int j = 0; j <= coarse_v; j++) {
        double u = u_min + (u_max - u_min) * i / coarse_u;
        double v = v_min + (v_max - v_min) * j / coarse_v;
        gp_Pnt2d point_2d(u, v);

        BRepClass_FaceClassifier classifier(face, point_2d, 1e-6);
        if (classifier.State() != TopAbs_IN && classifier.State() != TopAbs_ON) {
          continue;
        }

        total_count++;
        bool is_allowed = true;

        for (const auto & wire : wires) {
          bool inside_wire = is_point_inside_wire(point_2d, wire, face);
          bool is_exclusion_zone = (wire.Orientation() == TopAbs_REVERSED);

          if (is_exclusion_zone && inside_wire) {
            is_allowed = false;
            break;
          } else if (!is_exclusion_zone && !inside_wire) {
            is_allowed = false;
            break;
          }
        }

        if (is_allowed) {
          allowed_count++;
        }
      }
    }

    if (total_count > 0) {
      GProp_GProps props;
      BRepGProp::SurfaceProperties(face, props);
      double total_area_cm2 = props.Mass() * 10000.0;
      double allowed_fraction = static_cast<double>(allowed_count) / total_count;
      double allowed_area_cm2 = total_area_cm2 * allowed_fraction;
      target_samples = static_cast<int>(allowed_area_cm2 * samples_per_cm2);
    }
  }

  target_samples = std::max(min_samples, std::min(max_samples, target_samples));

  int samples_per_dim = std::max(1, static_cast<int>(std::sqrt(target_samples * 2.0)));

  RCLCPP_DEBUG(logger_, "Sampling %d normals (target=%d, grid=%dx%d)",
    samples_per_dim * samples_per_dim, target_samples,
    samples_per_dim, samples_per_dim);

  for (int i = 0; i < samples_per_dim; ++i) {
    for (int j = 0; j < samples_per_dim; ++j) {
      double u = u_min + (u_max - u_min) * (i + 0.5) / samples_per_dim;
      double v = v_min + (v_max - v_min) * (j + 0.5) / samples_per_dim;
      gp_Pnt2d point_2d(u, v);

      BRepClass_FaceClassifier classifier(face, point_2d, 1e-6);
      if (classifier.State() != TopAbs_IN && classifier.State() != TopAbs_ON) {
        continue;
      }

      if (!wires.empty()) {
        bool is_allowed = true;

        for (const auto & wire : wires) {
          bool inside_wire = is_point_inside_wire(point_2d, wire, face);
          bool is_exclusion_zone = (wire.Orientation() == TopAbs_REVERSED);

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
  std::map<std::tuple<int, int, int, int, int, int>, ContactPair> grid_map;

  for (const auto & pair : pairs) {
    int x1 = static_cast<int>(std::floor(pair.contact_1.X() / tolerance));
    int y1 = static_cast<int>(std::floor(pair.contact_1.Y() / tolerance));
    int z1 = static_cast<int>(std::floor(pair.contact_1.Z() / tolerance));

    int x2 = static_cast<int>(std::floor(pair.contact_2.X() / tolerance));
    int y2 = static_cast<int>(std::floor(pair.contact_2.Y() / tolerance));
    int z2 = static_cast<int>(std::floor(pair.contact_2.Z() / tolerance));

    auto key = std::make_tuple(x1, y1, z1, x2, y2, z2);

    if (grid_map.find(key) == grid_map.end()) {
      grid_map[key] = pair;
    }
  }

  std::vector<ContactPair> result;
  result.reserve(grid_map.size());
  for (const auto & [key, pair] : grid_map) {
    result.push_back(pair);
  }

  RCLCPP_DEBUG(logger_, "Deduplication: %zu pairs → %zu unique pairs (tolerance=%.4f m)",
    pairs.size(), result.size(), tolerance);

  return result;
}

}  // namespace sampling
}  // namespace hold_and_weld_gripper_sampler
