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
#include <cmath>
#include <memory>
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

static constexpr double kCeilingOffset = 0.010;

Grasp to_grasp(const GraspCandidate & candidate)
{
  // TCP is the midpoint between the two contact points — not the gripper base.
  gp_Pnt tcp(
    (candidate.contact_1.X() + candidate.contact_2.X()) / 2.0,
    (candidate.contact_1.Y() + candidate.contact_2.Y()) / 2.0,
    (candidate.contact_1.Z() + candidate.contact_2.Z()) / 2.0);

  return Grasp::create(
    geometry::to_eigen(tcp),
    geometry::extract_quaternion(candidate.gripper_transform),
    candidate.grip_distance,
    geometry::to_eigen(candidate.contact_1),
    geometry::to_eigen(candidate.contact_2),
    candidate.surface_id_1,
    candidate.surface_id_2,
    candidate.quality_score
  );
}

static bool angular_overlap(
  double a_start, double a_end,
  double b_start, double b_end,
  double & out_start, double & out_end)
{
  constexpr double k2Pi = 2.0 * M_PI;

  struct Piece { double s, e; };
  auto pieces = [&](double s, double e) -> std::vector<Piece> {
      if (s <= e) {return {{s, e}};}
      return {{s, k2Pi}, {0.0, e}};
    };

  for (const auto & ap : pieces(a_start, a_end)) {
    for (const auto & bp : pieces(b_start, b_end)) {
      double os = std::max(ap.s, bp.s);
      double oe = std::min(ap.e, bp.e);
      if (os < oe) {out_start = os; out_end = oe; return true;}
    }
  }
  return false;
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

std::vector<GraspCandidate> GraspOrientationFinder::find_valid_grasps(
  const std::vector<sampling::ContactPair> & contact_pairs,
  [[maybe_unused]] const geometry::Topology & topology)
{
  std::vector<GraspCandidate> valid_grasps;

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

  RCLCPP_INFO(logger_, "Processing %zu contact pairs (FCL radial-map orientation finding)",
    contact_pairs.size());
  RCLCPP_DEBUG(logger_,
    "Config: finger_length=%.4f m, ring_step=%.4f m, angular_step=%.1f° "
    "flat_tol=%.4f m, cliff_merge=%.1f°, min_cliff=%.1f°",
    config_.finger_length, config_.ring_step_size, config_.angular_step_deg,
    config_.flat_detection_tolerance_m,
    config_.cliff_merge_tolerance_deg, config_.min_cliff_width_deg);

  auto make_tangent_frame = [](const gp_Vec & n, gp_Vec & lx, gp_Vec & ly) {
      double s = (n.Z() >= 0.0) ? 1.0 : -1.0;
      double a = -1.0 / (s + n.Z());
      double b = n.X() * n.Y() * a;
      lx = gp_Vec(1.0 + s * n.X() * n.X() * a, s * b, -s * n.X());
      ly = gp_Vec(b, s + n.Y() * n.Y() * a, -n.Y());
    };

  for (const auto & pair : contact_pairs) {
    gp_Vec normal_1 = pair.normal_1;
    gp_Vec normal_2 = pair.normal_2;

    if (normal_1.Magnitude() < 1e-9 || normal_2.Magnitude() < 1e-9) {
      RCLCPP_WARN(logger_, "Zero surface normal for pair [%d-%d] — skipping",
        pair.surface_id_1, pair.surface_id_2);
      continue;
    }
    normal_1.Normalize();
    normal_2.Normalize();

    gp_Vec grip_axis(pair.contact_1, pair.contact_2);
    if (grip_axis.Magnitude() < 1e-9) {
      RCLCPP_WARN(logger_, "Coincident contact points for surfaces [%d-%d] — skipping",
        pair.surface_id_1, pair.surface_id_2);
      continue;
    }
    grip_axis.Normalize();

    gp_Vec cal_ref = normal_1 - grip_axis * grip_axis.Dot(normal_1);
    if (cal_ref.Magnitude() < 1e-9) {
      // normal_1 is parallel to grip_axis — use Duff et al. to get a perpendicular
      double s = (grip_axis.Z() >= 0.0) ? 1.0 : -1.0;
      double a = -1.0 / (s + grip_axis.Z());
      double b = grip_axis.X() * grip_axis.Y() * a;
      cal_ref = gp_Vec(1.0 + s * grip_axis.X() * grip_axis.X() * a, s * b, -s * grip_axis.X());
    }
    cal_ref.Normalize();

    gp_Vec lx1, ly1, lx2, ly2;
    make_tangent_frame(normal_1, lx1, ly1);
    make_tangent_frame(normal_2, lx2, ly2);

    double offset_1 = std::atan2(cal_ref.Dot(ly1), cal_ref.Dot(lx1));
    double offset_2 = std::atan2(cal_ref.Dot(ly2), cal_ref.Dot(lx2));

    gp_Pnt lifted_1(
      pair.contact_1.X() + normal_1.X() * kCeilingOffset,
      pair.contact_1.Y() + normal_1.Y() * kCeilingOffset,
      pair.contact_1.Z() + normal_1.Z() * kCeilingOffset);

    gp_Pnt lifted_2(
      pair.contact_2.X() + normal_2.X() * kCeilingOffset,
      pair.contact_2.Y() + normal_2.Y() * kCeilingOffset,
      pair.contact_2.Z() + normal_2.Z() * kCeilingOffset);

    RCLCPP_DEBUG_EXPRESSION(logger_, []{static size_t _n = 0; return ++_n % 3 == 1;}(),
      "Pair [%d-%d]: normal_1=(%.3f,%.3f,%.3f) lx1=(%.3f,%.3f,%.3f) ly1=(%.3f,%.3f,%.3f) offset_1=%.1f°",
      pair.surface_id_1, pair.surface_id_2,
      normal_1.X(), normal_1.Y(), normal_1.Z(),
      lx1.X(), lx1.Y(), lx1.Z(),
      ly1.X(), ly1.Y(), ly1.Z(),
      offset_1 * 180.0 / M_PI);
    RCLCPP_DEBUG_EXPRESSION(logger_, []{static size_t _n = 0; return ++_n % 3 == 1;}(),
      "Pair [%d-%d]: normal_2=(%.3f,%.3f,%.3f) lx2=(%.3f,%.3f,%.3f) ly2=(%.3f,%.3f,%.3f) offset_2=%.1f°",
      pair.surface_id_1, pair.surface_id_2,
      normal_2.X(), normal_2.Y(), normal_2.Z(),
      lx2.X(), lx2.Y(), lx2.Z(),
      ly2.X(), ly2.Y(), ly2.Z(),
      offset_2 * 180.0 / M_PI);
    RCLCPP_DEBUG_EXPRESSION(logger_, []{static size_t _n = 0; return ++_n % 3 == 1;}(),
      "Pair [%d-%d]: cal_ref=(%.3f,%.3f,%.3f)",
      pair.surface_id_1, pair.surface_id_2,
      cal_ref.X(), cal_ref.Y(), cal_ref.Z());

    auto maps_1 = create_radial_maps(
      pair.contact_1, gp_Dir(normal_1), lx1, ly1, lifted_1, offset_1);

    auto maps_2 = create_radial_maps(
      pair.contact_2, gp_Dir(normal_2), lx2, ly2, lifted_2, offset_2);

    double grippable_rad = 0.0;
    for (const auto & seg : merge_low_segments(maps_1, maps_2)) {
      grippable_rad += seg.end_rad - seg.start_rad;
    }
    double quality = std::min(grippable_rad / (2.0 * M_PI), 1.0);

    if (maps_1.low.empty() && maps_2.low.empty()) {
      pairs_skipped_flat++;
      RCLCPP_DEBUG_EXPRESSION(logger_, []{static size_t _n = 0; return ++_n % 3 == 1;}(),
        "Pair [%d-%d]: outer ring fully FLAT on both contacts — skipping",
        pair.surface_id_1, pair.surface_id_2);
      continue;
    }

    std::vector<double> seeds;
    if (true) {
      auto merged = merge_low_segments(maps_1, maps_2);
      RCLCPP_DEBUG(logger_,
        "Pair [%d-%d]: grip=%.4f m, quality=%.3f "
        "(flat_1=%zu high_1=%zu low_1=%zu | flat_2=%zu high_2=%zu low_2=%zu) "
        "→ merged_low=%zu",
        pair.surface_id_1, pair.surface_id_2,
        pair.grip_distance, quality,
        maps_1.flat.size(), maps_1.high.size(), maps_1.low.size(),
        maps_2.flat.size(), maps_2.high.size(), maps_2.low.size(),
        merged.size());

      for (const auto & seg : maps_1.low) {
        RCLCPP_DEBUG(logger_, "  maps_1 LOW: [%.1f°, %.1f°]",
          seg.start_rad * 180.0 / M_PI, seg.end_rad * 180.0 / M_PI);
      }
      for (const auto & seg : maps_2.low) {
        RCLCPP_DEBUG(logger_, "  maps_2 LOW: [%.1f°, %.1f°]",
          seg.start_rad * 180.0 / M_PI, seg.end_rad * 180.0 / M_PI);
      }

      if (!merged.empty()) {
        auto after_outer = filter_by_outer_ring(merged, maps_1, maps_2);
        RCLCPP_DEBUG(logger_,
          "  [%d-%d] after filter_by_outer_ring: %zu → %zu segment(s)",
          pair.surface_id_1, pair.surface_id_2,
          merged.size(), after_outer.size());

        if (!after_outer.empty()) {
          auto after_high = ban_high_angles(after_outer, maps_1, maps_2);
          RCLCPP_DEBUG(logger_,
            "  [%d-%d] after ban_high_angles: %zu → %zu segment(s)",
            pair.surface_id_1, pair.surface_id_2,
            after_outer.size(), after_high.size());

          if (!after_high.empty()) {
            auto clusters = cluster_and_filter(after_high);
            RCLCPP_DEBUG(logger_,
              "  [%d-%d] clusters after filter: %zu (min_cliff=%.1f°)",
              pair.surface_id_1, pair.surface_id_2,
              clusters.size(), config_.min_cliff_width_deg);

            seeds.reserve(clusters.size());
            for (const auto & cluster : clusters) {
              seeds.push_back(cluster_midpoint(cluster));
            }
          }
        }
      }
    } else {
      seeds = build_approach_seeds(maps_1, maps_2);
    }

    if (seeds.empty()) {
      pairs_no_seeds++;
      RCLCPP_DEBUG_EXPRESSION(logger_, []{static size_t _n = 0; return ++_n % 3 == 1;}(),
        "  [%d-%d] No grippable cliff survived filtering — skipping pair",
        pair.surface_id_1, pair.surface_id_2);
      continue;
    }

    if (config_.max_orientations_per_pair > 0 &&
      seeds.size() > config_.max_orientations_per_pair)
    {
      seeds.resize(config_.max_orientations_per_pair);
    }

    for (double angle_rad : seeds) {
      gp_Vec perp_to_cal = grip_axis.Crossed(cal_ref);
      perp_to_cal.Normalize();
      gp_Vec approach = cal_ref * std::cos(angle_rad) + perp_to_cal * std::sin(angle_rad);

      gp_Pnt base_pos;
      gp_Trsf transform = compute_gripper_transform(
        pair.contact_1, pair.contact_2, approach, base_pos);

      total_orientations_tested++;

      if (collides_with_primary(transform, pair.grip_distance)) {
        rejected_by_primary++;
        RCLCPP_DEBUG_EXPRESSION(logger_, []{
            static size_t _n = 0; return ++_n % 3 == 1;
              }(), "  angle=%.1f°: REJECTED primary collision",
          angle_rad * 180.0 / M_PI);
        continue;
      }

      if (exclusion_constraint_ && exclusion_constraint_->intersects_exclusion_zone(
          transform, pair.grip_distance, config_.collision_tolerance))
      {
        rejected_by_exclusion++;
        RCLCPP_DEBUG_EXPRESSION(logger_, []{
            static size_t _n = 0; return ++_n % 3 == 1;
              }(), "  angle=%.1f°: REJECTED exclusion zone",
          angle_rad * 180.0 / M_PI);
        continue;
      }

      if (kissing_constraint_) {
        Eigen::Isometry3d grasp_pose = Eigen::Isometry3d::Identity();
        grasp_pose.translation() = Eigen::Vector3d(
          base_pos.X(), base_pos.Y(), base_pos.Z());
        grasp_pose.linear() =
          geometry::extract_quaternion(transform).toRotationMatrix();

        if (kissing_constraint_->intersects_secondary(pair.grip_distance, grasp_pose)) {
          rejected_by_secondary++;
          RCLCPP_DEBUG_EXPRESSION(logger_, []{
              static size_t _n = 0; return ++_n % 3 == 1;
                }(), "  angle=%.1f°: REJECTED secondary collision",
            angle_rad * 180.0 / M_PI);
          continue;
        }
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

      RCLCPP_DEBUG_EXPRESSION(logger_, []{
          static size_t _n = 0; return ++_n % 3 == 1;
            }(), "  angle=%.1f°: valid (quality=%.3f)",
        angle_rad * 180.0 / M_PI, quality);

      if (config_.stop_on_first_valid) {break;}
    }

  }

  RCLCPP_INFO(logger_,
    "Orientation finding complete: %zu valid grasps from %zu pairs "
    "(%zu flat-skipped, %zu no-seeds, %zu tested, "
    "rejected: %zu primary / %zu exclusion / %zu secondary)",
    valid_grasps.size(), contact_pairs.size(),
    pairs_skipped_flat, pairs_no_seeds, total_orientations_tested,
    rejected_by_primary, rejected_by_exclusion, rejected_by_secondary);

  if (valid_grasps.empty()) {
    RCLCPP_WARN(logger_, "No valid grasps found");
  }

  return valid_grasps;
}

SurfaceState GraspOrientationFinder::classify_hit(
  bool hit_found,
  const gp_Pnt & hit_point,
  const gp_Pnt & contact,
  const gp_Vec & normal_vec,
  double tol) const
{
  if (!hit_found) {
    return SurfaceState::LOW;
  }
  gp_Vec hit_vec(contact, hit_point);
  double elevation = hit_vec.Dot(normal_vec);

  if (elevation > tol) {return SurfaceState::HIGH;}
  if (elevation < -tol) {return SurfaceState::LOW;}
  return SurfaceState::FLAT;
}

RadialMaps GraspOrientationFinder::create_radial_maps(
  const gp_Pnt & contact,
  const gp_Dir & normal,
  const gp_Vec & lx,
  const gp_Vec & ly,
  const gp_Pnt & lifted_center,
  double angle_offset) const
{
  RadialMaps maps;
  const double tol = config_.flat_detection_tolerance_m;
  const double step_rad = config_.angular_step_deg * M_PI / 180.0;
  const gp_Vec normal_vec(normal);
  const double outer_r = config_.finger_length;

  {
    size_t flat_start = maps.flat.size();
    size_t high_start = maps.high.size();
    size_t low_start = maps.low.size();

    SurfaceState first_state = SurfaceState::FLAT;
    SurfaceState prev_state = SurfaceState::FLAT;
    bool first = true;

    for (double local_angle = angle_offset; local_angle < angle_offset + 2.0 * M_PI;
      local_angle += step_rad)
    {
      double cos_a = std::cos(local_angle);
      double sin_a = std::sin(local_angle);

      gp_Pnt ring_point(
        contact.X() + lx.X() * outer_r * cos_a + ly.X() * outer_r * sin_a,
        contact.Y() + lx.Y() * outer_r * cos_a + ly.Y() * outer_r * sin_a,
        contact.Z() + lx.Z() * outer_r * cos_a + ly.Z() * outer_r * sin_a);

      gp_Vec ray_vec(lifted_center, ring_point);
      double ray_len = ray_vec.Magnitude();
      if (ray_len < 1e-9) {continue;}
      gp_Dir ray_dir(ray_vec);

      gp_Pnt hit_point;
      bool hit = fcl_checker_ ?
        fcl_checker_->ray_hits_primary(lifted_center, ray_dir, ray_len * 2.0, hit_point) :
        false;

      SurfaceState state = classify_hit(hit, hit_point, contact, normal_vec, tol);

      double shared_angle = local_angle - angle_offset;
      if (first) {

        first_state = state;
        switch (state) {
          case SurfaceState::FLAT: maps.flat.push_back({shared_angle, shared_angle, outer_r,
                state}); break;
          case SurfaceState::HIGH: maps.high.push_back({shared_angle, shared_angle, outer_r,
                state}); break;
          case SurfaceState::LOW:  maps.low.push_back({shared_angle, shared_angle, outer_r, state});
            break;
        }
        prev_state = state;
        first = false;
        continue;
      }

      if (state != prev_state) {
        switch (prev_state) {
          case SurfaceState::FLAT: if (!maps.flat.empty()) {
              maps.flat.back().end_rad = shared_angle;
          }
            break;
          case SurfaceState::HIGH: if (!maps.high.empty()) {
              maps.high.back().end_rad = shared_angle;
          }
            break;
          case SurfaceState::LOW:  if (!maps.low.empty()) {
              maps.low.back().end_rad = shared_angle;
          }
            break;
        }
        switch (state) {
          case SurfaceState::FLAT: maps.flat.push_back({shared_angle, shared_angle, outer_r,
                state}); break;
          case SurfaceState::HIGH: maps.high.push_back({shared_angle, shared_angle, outer_r,
                state}); break;
          case SurfaceState::LOW:  maps.low.push_back({shared_angle, shared_angle, outer_r, state});
            break;
        }
        prev_state = state;
      } else {
        switch (state) {
          case SurfaceState::FLAT: if (!maps.flat.empty()) {
              maps.flat.back().end_rad = shared_angle;
          }
            break;
          case SurfaceState::HIGH: if (!maps.high.empty()) {
              maps.high.back().end_rad = shared_angle;
          }
            break;
          case SurfaceState::LOW:  if (!maps.low.empty()) {
              maps.low.back().end_rad = shared_angle;
          }
            break;
        }
      }
    }

    double end_angle = 2.0 * M_PI;
    switch (prev_state) {
      case SurfaceState::FLAT: if (!maps.flat.empty()) {
          maps.flat.back().end_rad = end_angle;
      }
        break;
      case SurfaceState::HIGH: if (!maps.high.empty()) {
          maps.high.back().end_rad = end_angle;
      }
        break;
      case SurfaceState::LOW:  if (!maps.low.empty()) {maps.low.back().end_rad = end_angle;} break;
    }

    const double wrap_tol = step_rad;
    auto try_wrap_merge = [&](
      std::vector<RadialSegment> & segs,
      size_t ring_start,
      SurfaceState state)
      {
        if (segs.size() < ring_start + 2) {return;}
        auto & first_seg = segs[ring_start];
        auto & last_seg = segs.back();
        if (last_seg.radius != outer_r || last_seg.state != state) {return;}
        if (first_seg.radius != outer_r || first_seg.state != state) {return;}
        if (first_seg.start_rad < wrap_tol && first_state == state) {
          double saved_start = last_seg.start_rad;
          segs.pop_back();
          segs[ring_start].start_rad = saved_start;
        }
      };

    try_wrap_merge(maps.flat, flat_start, SurfaceState::FLAT);
    try_wrap_merge(maps.high, high_start, SurfaceState::HIGH);
    try_wrap_merge(maps.low, low_start, SurfaceState::LOW);
  }

  return maps;
}

std::vector<RadialSegment> GraspOrientationFinder::merge_low_segments(
  const RadialMaps & maps_1,
  const RadialMaps & maps_2) const
{
  std::vector<RadialSegment> result;
  for (const auto & s1 : maps_1.low) {
    for (const auto & s2 : maps_2.low) {
      double os, oe;
      if (angular_overlap(s1.start_rad, s1.end_rad, s2.start_rad, s2.end_rad, os, oe)) {
        result.push_back({os, oe, std::min(s1.radius, s2.radius), SurfaceState::LOW});
      }
    }
  }
  return result;
}

std::vector<RadialSegment> GraspOrientationFinder::filter_by_outer_ring(
  const std::vector<RadialSegment> & segments,
  const RadialMaps & maps_1,
  const RadialMaps & maps_2) const
{
  const double outer_r = config_.finger_length;
  constexpr double kRadiusTol = 1e-6;

  auto is_outer_blocked = [&](const RadialMaps & maps,
    double seg_start, double seg_end) -> bool {
      for (const auto & seg : maps.high) {
        if (std::abs(seg.radius - outer_r) > kRadiusTol) {continue;}
        double os, oe;
        if (angular_overlap(seg.start_rad, seg.end_rad, seg_start, seg_end, os, oe)) {
          return true;
        }
      }
      for (const auto & seg : maps.flat) {
        if (std::abs(seg.radius - outer_r) > kRadiusTol) {continue;}
        double os, oe;
        if (angular_overlap(seg.start_rad, seg.end_rad, seg_start, seg_end, os, oe)) {
          return true;
        }
      }
      return false;
    };

  std::vector<RadialSegment> result;
  for (const auto & seg : segments) {
    if (!is_outer_blocked(maps_1, seg.start_rad, seg.end_rad) &&
      !is_outer_blocked(maps_2, seg.start_rad, seg.end_rad))
    {
      result.push_back(seg);
    }
  }
  return result;
}

std::vector<RadialSegment> GraspOrientationFinder::ban_high_angles(
  const std::vector<RadialSegment> & segments,
  const RadialMaps & maps_1,
  const RadialMaps & maps_2) const
{
  auto overlaps_any_high = [&](const RadialMaps & maps,
    double seg_start, double seg_end) -> bool {
      for (const auto & high : maps.high) {
        double os, oe;
        if (angular_overlap(high.start_rad, high.end_rad, seg_start, seg_end, os, oe)) {
          return true;
        }
      }
      return false;
    };

  std::vector<RadialSegment> result;
  for (const auto & seg : segments) {
    if (!overlaps_any_high(maps_1, seg.start_rad, seg.end_rad) &&
      !overlaps_any_high(maps_2, seg.start_rad, seg.end_rad))
    {
      result.push_back(seg);
    }
  }
  return result;
}

std::vector<std::vector<RadialSegment>> GraspOrientationFinder::cluster_and_filter(
  const std::vector<RadialSegment> & segments) const
{
  const double merge_tol = config_.cliff_merge_tolerance_deg * M_PI / 180.0;
  const double min_width = config_.min_cliff_width_deg * M_PI / 180.0;
  constexpr double k2Pi = 2.0 * M_PI;

  std::vector<std::vector<RadialSegment>> clusters;
  std::vector<bool> assigned(segments.size(), false);

  for (size_t i = 0; i < segments.size(); ++i) {
    if (assigned[i]) {continue;}

    std::vector<RadialSegment> cluster;
    cluster.push_back(segments[i]);
    assigned[i] = true;

    bool grew = true;
    while (grew) {
      grew = false;
      for (size_t j = 0; j < segments.size(); ++j) {
        if (assigned[j]) {continue;}
        for (const auto & cs : cluster) {
          auto ang_dist = [](double a, double b) {
              double d = std::abs(a - b);
              return std::min(d, 2.0 * M_PI - d);
            };
          double d = std::min({
                ang_dist(segments[j].start_rad, cs.start_rad),
                ang_dist(segments[j].start_rad, cs.end_rad),
                ang_dist(segments[j].end_rad, cs.start_rad),
                ang_dist(segments[j].end_rad, cs.end_rad)});
          if (d <= merge_tol) {
            cluster.push_back(segments[j]);
            assigned[j] = true;
            grew = true;
            break;
          }
        }
      }
    }

    clusters.push_back(std::move(cluster));
  }

  auto span_of = [&](const std::vector<RadialSegment> & segs, double offset) {
      double s = k2Pi, e = 0.0;
      for (const auto & seg : segs) {
        if (seg.end_rad - seg.start_rad >= k2Pi - 1e-9) {return k2Pi;}
        double a = std::fmod(seg.start_rad + offset, k2Pi);
        double b = std::fmod(seg.end_rad + offset, k2Pi);
        if (a < 0.0) {a += k2Pi;}
        if (b < 0.0) {b += k2Pi;}
        if (b < a) {b += k2Pi;}
        s = std::min(s, a);
        e = std::max(e, b);
      }
      return e - s;
    };

  std::vector<std::vector<RadialSegment>> result;
  for (auto & cluster : clusters) {
    double span = std::min(span_of(cluster, 0.0), span_of(cluster, M_PI));
    if (span >= min_width) {
      result.push_back(std::move(cluster));
    }
  }
  return result;
}

double GraspOrientationFinder::cluster_midpoint(
  const std::vector<RadialSegment> & cluster) const
{
  constexpr double k2Pi = 2.0 * M_PI;

  auto seg_width = [&](const RadialSegment & seg) -> double {
      if (seg.end_rad >= seg.start_rad) {
        return seg.end_rad - seg.start_rad;
      }
      return (k2Pi - seg.start_rad) + seg.end_rad;
    };

  auto seg_mid = [&](const RadialSegment & seg) -> double {
      if (seg.end_rad >= seg.start_rad) {
        return (seg.start_rad + seg.end_rad) / 2.0;
      }
      double mid = seg.start_rad + seg_width(seg) / 2.0;
      return std::fmod(mid, k2Pi);
    };

  const RadialSegment * widest = &cluster[0];
  double max_width = seg_width(cluster[0]);

  for (const auto & seg : cluster) {
    double w = seg_width(seg);
    if (w > max_width) {
      max_width = w;
      widest = &seg;
    }
  }

  return seg_mid(*widest);
}

std::vector<double> GraspOrientationFinder::build_approach_seeds(
  const RadialMaps & maps_1,
  const RadialMaps & maps_2) const
{
  auto merged = merge_low_segments(maps_1, maps_2);
  if (merged.empty()) {return {};}

  auto filtered = filter_by_outer_ring(merged, maps_1, maps_2);
  if (filtered.empty()) {return {};}

  filtered = ban_high_angles(filtered, maps_1, maps_2);
  if (filtered.empty()) {return {};}

  auto clusters = cluster_and_filter(filtered);
  if (clusters.empty()) {return {};}

  std::vector<double> seeds;
  seeds.reserve(clusters.size());
  for (const auto & cluster : clusters) {
    seeds.push_back(cluster_midpoint(cluster));
  }
  return seeds;
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
    x_axis.X() * offset_local.X() + y_axis.X() * offset_local.Y() +
    z_axis.X() * offset_local.Z(),
    x_axis.Y() * offset_local.X() + y_axis.Y() * offset_local.Y() +
    z_axis.Y() * offset_local.Z(),
    x_axis.Z() * offset_local.X() + y_axis.Z() * offset_local.Y() +
    z_axis.Z() * offset_local.Z());

  out_base = tcp.Translated(-offset_world);

  gp_Ax3 gripper_frame(
    out_base,
    gp_Dir(z_axis.X(), z_axis.Y(), z_axis.Z()),
    gp_Dir(x_axis.X(), x_axis.Y(), x_axis.Z()));

  gp_Trsf transform;
  transform.SetTransformation(gripper_frame, gp_Ax3());
  return transform;
}

bool GraspOrientationFinder::collides_with_primary(
  const gp_Trsf & transform,
  double grip_distance) const
{
  if (!fcl_checker_ || !fcl_checker_->is_valid()) {
    return true;
  }
  return fcl_checker_->collides_with_primary(
    transform, grip_distance, config_.collision_tolerance);
}

}  // namespace angle_finding
}  // namespace hold_and_weld_gripper_sampler
