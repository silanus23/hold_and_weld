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

#include <algorithm>
#include <cassert>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <random>
#include <stdexcept>
#include <vector>

#include <gp_Ax3.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>
#include <gp_Trsf.hxx>
#include <gp_Vec.hxx>
#include <rclcpp/rclcpp.hpp>

#include "hold_and_weld_gripper_sampler/angle_finding/grasp_orientation_finder.hpp"
#include "hold_and_weld_gripper_sampler/constraints/exclusion_zone_constraint.hpp"
#include "hold_and_weld_gripper_sampler/constraints/kissing_surface_constraint.hpp"
#include "hold_and_weld_gripper_sampler/core/grasp.hpp"
#include "hold_and_weld_gripper_sampler/core/gripper.hpp"
#include "hold_and_weld_gripper_sampler/geometry/occt_utils.hpp"

namespace hold_and_weld_gripper_sampler
{
namespace angle_finding
{

// Finds all overlapping pieces between two arcs, each of which may wrap the 0/2pi seam.
static bool angular_overlap(
  double a_start, double a_end,
  double b_start, double b_end,
  std::vector<std::pair<double, double>> & out_overlaps)
{
  auto split = [](double s, double e) -> std::vector<std::pair<double, double>> {
      if (s <= e) {return {{s, e}};}
      return {{s, 2.0 * M_PI}, {0.0, e}};
    };

  bool found = false;
  for (const auto & [a_piece_start, a_piece_end] : split(a_start, a_end)) {
    if (a_piece_start >= a_piece_end) {break;}
    for (const auto & [b_piece_start, b_piece_end] : split(b_start, b_end)) {
      if (b_piece_start >= b_piece_end) {break;}
      const double overlap_start = std::max(a_piece_start, b_piece_start);
      const double overlap_end = std::min(a_piece_end, b_piece_end);
      if (overlap_start < overlap_end) {
        out_overlaps.emplace_back(overlap_start, overlap_end);
        found = true;
      }
    }
  }
  return found;
}

// Orthonormal basis so stable for all normal orientations including +-Z.
static void build_tangent_frame(const gp_Vec & normal, gp_Vec & out_lx, gp_Vec & out_ly)
{
  const double sign = (normal.Z() >= 0.0) ? 1.0 : -1.0;
  const double inv_denom = -1.0 / (sign + normal.Z());
  const double xy_cross_term = normal.X() * normal.Y() * inv_denom;
  out_lx = gp_Vec(
    1.0 + sign * normal.X() * normal.X() * inv_denom,
    sign * xy_cross_term,
    -sign * normal.X());
  out_ly = gp_Vec(
    xy_cross_term,
    sign + normal.Y() * normal.Y() * inv_denom,
    -normal.Y());
}

static gp_Pnt compute_ring_point(
  const gp_Pnt & contact,
  const gp_Vec & tangent_axis_x,
  const gp_Vec & tangent_axis_y,
  double radius,
  double angle)
{
  const double cos_angle = std::cos(angle);
  const double sin_angle = std::sin(angle);
  return gp_Pnt(
    contact.X() + tangent_axis_x.X() * radius * cos_angle + tangent_axis_y.X() * radius * sin_angle,
    contact.Y() + tangent_axis_x.Y() * radius * cos_angle + tangent_axis_y.Y() * radius * sin_angle,
    contact.Z() + tangent_axis_x.Z() * radius * cos_angle + tangent_axis_y.Z() * radius *
        sin_angle);
}

// Merges a seam-split arc: the first and last segments of a sweep that
// started and ended in the same state are one arc split across the 0/2π seam.
// Extend front's end_rad by the back segment's width, then discard the back.
// Example: back=[350°,360°], front=[0°,10°] → front becomes [0°,20°].
// end_rad > 2π is fine; the sweep loop condition handles it correctly.
static void rejoin_wraparound_arc(
  RadialMaps & maps,
  SurfaceState first_state,
  double wrap_tol)
{
  for (const SurfaceState state : kAllSurfaceStates) {
    auto & segs = maps.segs_for(state);
    if (segs.size() < 2) {continue;}
    const double front_start = segs.front().start_rad;
    const double back_start = segs.back().start_rad;
    const double back_end = segs.back().end_rad;
    if (front_start < wrap_tol && first_state == state) {
      const double back_arc = back_end - back_start;
      segs.front().end_rad += back_arc;
      segs.pop_back();
    }
  }
}

GraspOrientationFinder::GraspOrientationFinder(
  const TopoDS_Shape & primary_shape,
  const ParsedGripper & gripper,
  std::shared_ptr<const constraints::ExclusionZoneConstraint> exclusion_constraint,
  std::shared_ptr<const constraints::KissingSurfaceConstraint> kissing_constraint,
  const OrientationConfig & config)
: primary_shape_(primary_shape),
  gripper_(gripper),
  exclusion_constraint_(exclusion_constraint),
  kissing_constraint_(kissing_constraint),
  config_(config),
  fcl_checker_(nullptr),
  logger_(rclcpp::get_logger("gripper_sampler"))
{
}

void GraspOrientationFinder::set_fcl_checker(
  std::shared_ptr<const geometry::FCLCollisionChecker> fcl_checker)
{
  fcl_checker_ = fcl_checker;
}

void GraspOrientationFinder::set_embree_checker(
  std::shared_ptr<const geometry::EmbreeMeshQuery> embree_checker)
{
  embree_checker_ = embree_checker;
}

SurfaceState GraspOrientationFinder::classify_hit(
  bool hit_found,
  const gp_Pnt & hit_point,
  const gp_Pnt & contact,
  const gp_Vec & normal_vec,
  double tol) const
{
  if (!hit_found) {return SurfaceState::LOW;}
  gp_Vec hit_vec(contact, hit_point);
  double elevation = hit_vec.Dot(normal_vec);
  if (elevation > tol) {return SurfaceState::HIGH;}
  if (elevation < -tol) {return SurfaceState::LOW;}
  return SurfaceState::FLAT;
}


RadialMaps GraspOrientationFinder::create_radial_maps(
  const gp_Pnt & contact,
  const gp_Dir & normal,
  const gp_Vec & tangent_axis_x,
  const gp_Vec & tangent_axis_y,
  const gp_Pnt & lifted_center,
  double angle_offset) const
{
  RadialMaps maps;
  const double flat_tol = config_.flat_detection_tolerance_m;
  const double step_rad = config_.angular_step_deg * M_PI / 180.0;
  const double min_arc_width = config_.min_cliff_width_deg * M_PI / 180.0;
  const gp_Vec normal_vec(normal);
  const double outer_radius = config_.finger_length;
  const double inner_radius = config_.finger_radius;
  const double ring_step = config_.ring_step_size;

  SurfaceState first_state = SurfaceState::FLAT;
  SurfaceState prev_state = SurfaceState::FLAT;
  bool first = true;

  for (double local_angle = angle_offset;
    local_angle < angle_offset + 2.0 * M_PI;
    local_angle += step_rad)
  {
    const gp_Pnt ring_point = compute_ring_point(
      contact, tangent_axis_x, tangent_axis_y, outer_radius, local_angle);

    const gp_Vec ray_vec(lifted_center, ring_point);
    const double ray_length = ray_vec.Magnitude();
    if (ray_length < 1e-6) {continue;}

    // Avoids spurious LOW classifications from rays falling short at grazing angles.
    gp_Pnt hit_point;
    bool hit = false;
    static std::once_flag embree_null_warned;
    if (embree_checker_ && embree_checker_->is_valid()) {
      auto embree_hit = embree_checker_->ray_intersect(lifted_center, gp_Dir(ray_vec), ray_length);
      if (embree_hit.has_value()) {hit_point = embree_hit.value(); hit = true;}
    } else if (!embree_checker_) {
      std::call_once(embree_null_warned, [this]() {
          RCLCPP_WARN(logger_,
          "GraspOrientationFinder: Embree checker is null — all radial directions "
          "classified as unobstructed. Quality scores will be 1.0 for all candidates.");
      });
    }

    const SurfaceState state = classify_hit(hit, hit_point, contact, normal_vec, flat_tol);
    const double shared_angle = local_angle - angle_offset;

    if (first) {
      first_state = state;
      maps.segs_for(state).push_back({shared_angle, shared_angle, outer_radius, state});
      prev_state = state;
      first = false;
      continue;
    }

    if (state != prev_state) {
      auto & prev_segs = maps.segs_for(prev_state);
      if (!prev_segs.empty()) {prev_segs.back().end_rad = shared_angle;}
      maps.segs_for(state).push_back({shared_angle, shared_angle, outer_radius, state});
      prev_state = state;
    } else {
      auto & cur_segs = maps.segs_for(state);
      if (!cur_segs.empty()) {cur_segs.back().end_rad = shared_angle;}
    }
  }

  // Close the last open segment at exactly 2pi.
  auto & prev_segs = maps.segs_for(prev_state);
  if (!prev_segs.empty()) {prev_segs.back().end_rad = 2.0 * M_PI;}

  rejoin_wraparound_arc(maps, first_state, step_rad);

  for (double current_radius = outer_radius - ring_step;
    current_radius >= inner_radius - 1e-9;
    current_radius -= ring_step)
  {
    std::vector<RadialSegment> new_low;

    for (const RadialSegment & seg : maps.low) {
      SurfaceState last_state = SurfaceState::FLAT;
      double piece_start = seg.start_rad;

      for (double sweep_angle = seg.start_rad;
        sweep_angle <= seg.end_rad + 1e-9;
        sweep_angle += step_rad)
      {
        const double angle = std::min(sweep_angle, seg.end_rad);
        const double local_angle = angle + angle_offset;
        const gp_Pnt ring_point = compute_ring_point(
          contact, tangent_axis_x, tangent_axis_y, current_radius, local_angle);

        const gp_Vec ray_vec(lifted_center, ring_point);
        const double ray_length = ray_vec.Magnitude();
        if (ray_length < 1e-6) {continue;}

        gp_Pnt hit_point;
        bool hit = false;
        if (embree_checker_ && embree_checker_->is_valid()) {
          auto embree_hit = embree_checker_->ray_intersect(
            lifted_center, gp_Dir(ray_vec), ray_length);
          if (embree_hit.has_value()) {hit_point = embree_hit.value(); hit = true;}
        }
        // Note: null embree_checker_ already warned in the outer ring loop above.
        const SurfaceState current_state = classify_hit(
          hit, hit_point, contact, normal_vec, flat_tol);

        if (current_state != last_state) {
          if (last_state != SurfaceState::HIGH && (angle - piece_start) >= min_arc_width) {
            new_low.push_back({piece_start, angle, current_radius, SurfaceState::LOW});
          }
          piece_start = angle;
          last_state = current_state;
        }
      }

      if (last_state != SurfaceState::HIGH && (seg.end_rad - piece_start) >= min_arc_width) {
        new_low.push_back({piece_start, seg.end_rad, current_radius, SurfaceState::LOW});
      }
    }

    maps.low = new_low;
    if (maps.low.empty()) {break;}
  }

  return maps;
}

std::vector<RadialSegment> GraspOrientationFinder::merge_low_segments(
  const RadialMaps & maps_1,
  const RadialMaps & maps_2) const
{
  std::vector<RadialSegment> result;
  for (const auto & seg_1 : maps_1.low) {
    for (const auto & seg_2 : maps_2.low) {
      std::vector<std::pair<double, double>> overlaps;
      if (angular_overlap(
          seg_1.start_rad, seg_1.end_rad,
          seg_2.start_rad, seg_2.end_rad, overlaps))
      {
        for (const auto & [ov_start, ov_end] : overlaps) {
          result.push_back(
            {ov_start, ov_end, std::min(seg_1.radius, seg_2.radius), SurfaceState::LOW});
        }
      }
    }
  }
  return result;
}

std::vector<std::vector<RadialSegment>> GraspOrientationFinder::cluster_and_filter(
  const std::vector<RadialSegment> & segments) const
{
  if (segments.empty()) {return {};}

  const double merge_tol = config_.cliff_merge_tolerance_deg * M_PI / 180.0;
  const double min_width = config_.min_cliff_width_deg * M_PI / 180.0;

  std::vector<std::vector<RadialSegment>> clusters;
  clusters.push_back({segments.front()});
  double cluster_end = segments.front().end_rad;

  // assumes segments are sorted by start_rad — guaranteed by the sweep order in create_radial_maps
  assert(std::is_sorted(segments.begin(), segments.end(),
    [](const auto & a, const auto & b) {return a.start_rad < b.start_rad;}) &&
    "cluster_and_filter: segments must be sorted by start_rad");
  for (size_t seg_idx = 1; seg_idx < segments.size(); ++seg_idx) {
    const RadialSegment & seg = segments[seg_idx];
    if (seg.start_rad - cluster_end <= merge_tol) {
      clusters.back().push_back(seg);
      cluster_end = std::max(cluster_end, seg.end_rad);
    } else {
      clusters.push_back({seg});
      cluster_end = seg.end_rad;
    }
  }

  // Check wrap-around: join last and first cluster if they touch across 2pi.
  if (clusters.size() > 1) {
    const double last_end = clusters.back().back().end_rad;
    const double first_start = clusters.front().front().start_rad;
    if ((2.0 * M_PI - last_end) + first_start <= merge_tol) {
      auto & first_cluster = clusters.front();
      for (auto & seg : clusters.back()) {
        first_cluster.push_back(seg);
      }
      clusters.pop_back();
    }
  }

  std::vector<std::vector<RadialSegment>> result;
  for (auto & cluster : clusters) {
    double total_arc = 0.0;
    for (const auto & seg : cluster) {
      total_arc += seg.end_rad - seg.start_rad;
    }
    if (std::min(total_arc, 2.0 * M_PI) >= min_width) {
      result.push_back(std::move(cluster));
    }
  }
  return result;
}

gp_Trsf GraspOrientationFinder::compute_gripper_transform(
  const gp_Pnt & contact_1,
  const gp_Pnt & contact_2,
  const gp_Vec & approach,
  gp_Pnt & out_base) const
{
  gp_Pnt tcp(
    (contact_1.X() + contact_2.X()) / 2.0,
    (contact_1.Y() + contact_2.Y()) / 2.0,
    (contact_1.Z() + contact_2.Z()) / 2.0);

  gp_Vec y_axis(contact_1, contact_2);
  if (y_axis.Magnitude() < 1e-6) {
    throw std::invalid_argument("compute_gripper_transform: coincident contact points");
  }
  y_axis.Normalize();

  gp_Vec z_axis = approach;
  z_axis.Normalize();
  z_axis = z_axis - (z_axis.Dot(y_axis)) * y_axis;
  if (z_axis.Magnitude() < 1e-6) {
    z_axis = (std::abs(y_axis.Z()) < 0.9) ?
      y_axis.Crossed(gp_Vec(0, 0, 1)) :
      y_axis.Crossed(gp_Vec(1, 0, 0));
  }
  z_axis.Normalize();

  gp_Vec x_axis = y_axis.Crossed(z_axis);
  x_axis.Normalize();

  gp_Vec offset_local(
    gripper_.tcp_offset.x(),
    gripper_.tcp_offset.y(),
    gripper_.tcp_offset.z());

  gp_Vec offset_world(
    x_axis.X() * offset_local.X() + y_axis.X() * offset_local.Y() + z_axis.X() * offset_local.Z(),
    x_axis.Y() * offset_local.X() + y_axis.Y() * offset_local.Y() + z_axis.Y() * offset_local.Z(),
    x_axis.Z() * offset_local.X() + y_axis.Z() * offset_local.Y() + z_axis.Z() * offset_local.Z());

  out_base = tcp.Translated(-offset_world);

  gp_Mat rot(
    x_axis.X(), y_axis.X(), z_axis.X(),
    x_axis.Y(), y_axis.Y(), z_axis.Y(),
    x_axis.Z(), y_axis.Z(), z_axis.Z());
  gp_Trsf transform;
  transform.SetValues(
    rot.Value(1, 1), rot.Value(1, 2), rot.Value(1, 3), out_base.X(),
    rot.Value(2, 1), rot.Value(2, 2), rot.Value(2, 3), out_base.Y(),
    rot.Value(3, 1), rot.Value(3, 2), rot.Value(3, 3), out_base.Z());
  return transform;
}

bool GraspOrientationFinder::collides_with_primary(
  const gp_Trsf & transform,
  double grip_distance) const
{
  if (!fcl_checker_ || !fcl_checker_->is_valid()) {return true;}
  return fcl_checker_->collides_with_primary(
    transform, grip_distance, config_.collision_tolerance);
}

std::vector<GraspCandidate> GraspOrientationFinder::find_valid_grasps(
  const std::vector<sampling::ContactPair> & contact_pairs,
  [[maybe_unused]] const geometry::Topology & topology)
{
  thread_local std::mt19937 rng(
    static_cast<uint32_t>(
      std::chrono::steady_clock::now().time_since_epoch().count()));

  std::vector<GraspCandidate> valid_grasps;
  const size_t max_orientations = config_.max_orientations_per_pair > 0 ?
    config_.max_orientations_per_pair : 16;
  valid_grasps.reserve(contact_pairs.size() * max_orientations);

  if (contact_pairs.empty()) {
    RCLCPP_WARN(logger_, "No contact pairs provided — returning empty grasp list");
    return valid_grasps;
  }

  size_t total_orientations_tested = 0;
  size_t rejected_by_primary = 0;
  size_t rejected_by_exclusion = 0;
  size_t rejected_by_secondary = 0;
  size_t pairs_skipped_flat = 0;
  size_t pairs_no_seeds = 0;
  size_t pairs_merged_empty = 0;
  size_t pairs_killed_cluster = 0;
  size_t total_seeds_before_cap = 0;
  size_t total_seeds_after_cap = 0;

  RCLCPP_INFO(logger_, "Processing %zu contact pairs (radial-map orientation finding)",
    contact_pairs.size());

  for (const auto & pair : contact_pairs) {
    try {
      gp_Vec normal_1 = pair.normal_1;
      gp_Vec normal_2 = pair.normal_2;

      if (normal_1.Magnitude() < 1e-6 || normal_2.Magnitude() < 1e-6) {
        RCLCPP_WARN(logger_, "Zero surface normal for pair [%d-%d] — skipping",
        pair.surface_id_1, pair.surface_id_2);
        continue;
      }
      normal_1.Normalize();
      normal_2.Normalize();

      gp_Vec grip_axis(pair.contact_1, pair.contact_2);
      if (grip_axis.Magnitude() < 1e-6) {
        RCLCPP_WARN(logger_, "Coincident contact points for surfaces [%d-%d] — skipping",
        pair.surface_id_1, pair.surface_id_2);
        continue;
      }
      grip_axis.Normalize();

      gp_Vec cal_ref = normal_1 - grip_axis * grip_axis.Dot(normal_1);

      if (cal_ref.Magnitude() < 1e-6) {
      // normal_1 is parallel to grip_axis — fall back to an arbitrary perpendicular
        gp_Vec unused_ly;
        build_tangent_frame(grip_axis, cal_ref, unused_ly);
      }
      cal_ref.Normalize();

      gp_Vec tangent_x_1, tangent_y_1, tangent_x_2, tangent_y_2;
      build_tangent_frame(normal_1, tangent_x_1, tangent_y_1);
      build_tangent_frame(normal_2, tangent_x_2, tangent_y_2);

    // Angle of cal_ref in each tangent frame — aligns both radial maps to a common reference.
      double offset_1 = std::atan2(cal_ref.Dot(tangent_y_1), cal_ref.Dot(tangent_x_1));
      double offset_2 = std::atan2(cal_ref.Dot(tangent_y_2), cal_ref.Dot(tangent_x_2));

      gp_Pnt lifted_1(
        pair.contact_1.X() + normal_1.X() * config_.ray_lift_offset,
        pair.contact_1.Y() + normal_1.Y() * config_.ray_lift_offset,
        pair.contact_1.Z() + normal_1.Z() * config_.ray_lift_offset);

      gp_Pnt lifted_2(
        pair.contact_2.X() + normal_2.X() * config_.ray_lift_offset,
        pair.contact_2.Y() + normal_2.Y() * config_.ray_lift_offset,
        pair.contact_2.Z() + normal_2.Z() * config_.ray_lift_offset);

      auto maps_1 = create_radial_maps(
      pair.contact_1, gp_Dir(normal_1), tangent_x_1, tangent_y_1, lifted_1, offset_1);

      auto maps_2 = create_radial_maps(
      pair.contact_2, gp_Dir(normal_2), tangent_x_2, tangent_y_2, lifted_2, offset_2);

      if (maps_1.low.empty() && maps_2.low.empty()) {
        pairs_skipped_flat++;
        continue;
      }

    // Reused for both quality scoring and seed generation.
      auto merged = merge_low_segments(maps_1, maps_2);
      std::sort(merged.begin(), merged.end(),
        [](const RadialSegment & a, const RadialSegment & b) {
          return a.start_rad < b.start_rad;
        });

      double grippable_rad = 0.0;
      for (const auto & seg : merged) {
        grippable_rad += seg.end_rad - seg.start_rad;
      }
      double quality = std::min(grippable_rad / (2.0 * M_PI), 1.0);

      std::vector<double> seeds;
      if (config_.debug_full_sweep) {
        const double sweep_step_rad = config_.debug_sweep_step_deg * M_PI / 180.0;
        const int    num_sweep_steps = static_cast<int>(std::round(2.0 * M_PI / sweep_step_rad));
        seeds.reserve(num_sweep_steps);
        for (int step_idx = 0; step_idx < num_sweep_steps; ++step_idx) {
          seeds.push_back(step_idx * sweep_step_rad);
        }
      } else {
        if (merged.empty()) {
          pairs_merged_empty++;
          continue;
        } else {
          auto clusters = cluster_and_filter(merged);
          if (clusters.empty()) {
            pairs_killed_cluster++;
            continue;
          } else {
            for (const auto & cluster : clusters) {
              double span_start = cluster.front().start_rad;
              double span_end = cluster.front().end_rad;
              for (const auto & seg : cluster) {
                span_start = std::min(span_start, seg.start_rad);
                span_end = std::max(span_end, seg.end_rad);
              }
              const double cluster_arc_span = span_end - span_start;

              if (config_.randomize_seeds) {
                const size_t n_clusters = clusters.size();
                const size_t n_samples = (config_.max_orientations_per_pair > 0) ?
                  std::max(size_t{1}, config_.max_orientations_per_pair / n_clusters) :
                  1;
                std::uniform_real_distribution<double> dist(0.0, cluster_arc_span);
                for (size_t sample_idx = 0; sample_idx < n_samples; ++sample_idx) {
                  seeds.push_back(span_start + dist(rng));
                }
              } else {
                const double seed_step_rad = config_.seed_step_deg * M_PI / 180.0;
                if (cluster_arc_span <= seed_step_rad) {
                  seeds.push_back(span_start + cluster_arc_span * 0.5);
                } else {
                  const int num_steps = static_cast<int>(std::floor(cluster_arc_span /
                      seed_step_rad));
                  const double actual_step = cluster_arc_span / num_steps;
                  for (int step_idx = 0; step_idx < num_steps; ++step_idx) {
                    seeds.push_back(span_start + (step_idx + 0.5) * actual_step);
                  }
                }
              }
            }
          }
        }
      }

      if (seeds.empty()) {
        pairs_no_seeds++;
        continue;
      }

      total_seeds_before_cap += seeds.size();
      if (config_.max_orientations_per_pair > 0 &&
        seeds.size() > config_.max_orientations_per_pair)
      {
        seeds.resize(config_.max_orientations_per_pair);
      }
      total_seeds_after_cap += seeds.size();

      gp_Vec perp_to_cal = grip_axis.Crossed(cal_ref);
      perp_to_cal.Normalize();

      for (double angle_rad : seeds) {
        gp_Vec approach = -(cal_ref * std::cos(angle_rad) + perp_to_cal * std::sin(angle_rad));

        gp_Pnt base_pos;
        gp_Trsf transform = compute_gripper_transform(
        pair.contact_1, pair.contact_2, approach, base_pos);

        total_orientations_tested++;

        if (collides_with_primary(transform, pair.grip_distance)) {
          rejected_by_primary++;
          continue;
        }

        if (exclusion_constraint_ && exclusion_constraint_->intersects_exclusion_zone(
          transform, pair.grip_distance, config_.collision_tolerance))
        {
          rejected_by_exclusion++;
          continue;
        }

        if (kissing_constraint_ &&
          kissing_constraint_->intersects_secondary(pair.grip_distance, transform))
        {
          rejected_by_secondary++;
          continue;
        }

        GraspCandidate candidate;
        candidate.contact_1 = pair.contact_1;
        candidate.contact_2 = pair.contact_2;
        candidate.approach_direction = approach;
        candidate.gripper_transform = transform;
        candidate.base_position = base_pos;
        candidate.surface_id_1 = pair.surface_id_1;
        candidate.surface_id_2 = pair.surface_id_2;
        candidate.grip_distance = pair.grip_distance;
        candidate.quality_score = quality;

        valid_grasps.push_back(candidate);

        if (config_.stop_on_first_valid) {break;}
      }
    } catch (const Standard_Failure & e) {
      RCLCPP_WARN(logger_, "OCCT error on pair [%d-%d]: %s — skipping",
        pair.surface_id_1, pair.surface_id_2, e.GetMessageString());
      continue;
    } catch (const std::exception & e) {
      RCLCPP_WARN(logger_, "Error on pair [%d-%d]: %s — skipping",
        pair.surface_id_1, pair.surface_id_2, e.what());
      continue;
    }
  }

  const size_t total_pairs = contact_pairs.size();
  const size_t pairs_with_seeds = total_pairs -
    pairs_skipped_flat - pairs_merged_empty - pairs_killed_cluster - pairs_no_seeds;

  // Pipeline stage breakdown as percentage of total pairs.
  auto pct = [&](size_t n) {
      return total_pairs > 0 ? 100.0 * n / static_cast<double>(total_pairs) : 0.0;
    };

  const double avg_seeds = pairs_with_seeds > 0 ?
    static_cast<double>(total_seeds_before_cap) / static_cast<double>(pairs_with_seeds) : 0.0;
  const double avg_seeds_capped = pairs_with_seeds > 0 ?
    static_cast<double>(total_seeds_after_cap) / static_cast<double>(pairs_with_seeds) : 0.0;

  RCLCPP_INFO(logger_,
    "Orientation finding complete: %zu valid grasps from %zu pairs "
    "(%zu flat-skipped, %zu no-seeds, %zu tested, "
    "rejected: %zu primary / %zu exclusion / %zu secondary)",
    valid_grasps.size(), total_pairs,
    pairs_skipped_flat, pairs_no_seeds, total_orientations_tested,
    rejected_by_primary, rejected_by_exclusion, rejected_by_secondary);

  RCLCPP_DEBUG(logger_,
    "[Radial pipeline] flat=%.1f%%  merged_empty=%.1f%%  "
    "killed_cluster=%.1f%%  no_seeds=%.1f%%  with_seeds=%.1f%%  "
    "avg_seeds=%.2f (capped=%.2f)",
    pct(pairs_skipped_flat), pct(pairs_merged_empty),
    pct(pairs_killed_cluster), pct(pairs_no_seeds), pct(pairs_with_seeds),
    avg_seeds, avg_seeds_capped);

  if (valid_grasps.empty()) {
    RCLCPP_WARN(logger_, "No valid grasps found");
  }

  return valid_grasps;
}

}  // namespace angle_finding
}  // namespace hold_and_weld_gripper_sampler
