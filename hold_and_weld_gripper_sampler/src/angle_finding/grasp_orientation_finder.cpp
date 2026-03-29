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
#include <limits>
#include <memory>
#include <vector>

#include <BRep_Builder.hxx>
#include <BRepBuilderAPI_MakeVertex.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <BRepExtrema_DistShapeShape.hxx>
#include <BRepExtrema_ExtPC.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <gp_Ax1.hxx>
#include <gp_Ax2.hxx>
#include <gp_Ax3.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>
#include <gp_Trsf.hxx>
#include <gp_Vec.hxx>
#include <rclcpp/rclcpp.hpp>
#include <Standard_Failure.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Compound.hxx>

#include "hold_and_weld_gripper_sampler/angle_finding/grasp_orientation_finder.hpp"
#include "hold_and_weld_gripper_sampler/constraints/exclusion_zone_constraint.hpp"
#include "hold_and_weld_gripper_sampler/constraints/kissing_surface_constraint.hpp"
#include "hold_and_weld_gripper_sampler/core/gripper.hpp"
#include "hold_and_weld_gripper_sampler/core/grasp.hpp"
#include "hold_and_weld_gripper_sampler/geometry/occt_utils.hpp"

namespace hold_and_weld_gripper_sampler
{

Grasp to_grasp(const angle_finding::GraspCandidate & candidate)
{
  return Grasp::create(
    geometry::extract_translation(candidate.gripper_transform),
    geometry::extract_quaternion(candidate.gripper_transform),
    candidate.grip_distance,
    geometry::to_eigen(candidate.contact_1),
    geometry::to_eigen(candidate.contact_2),
    candidate.surface_id_1,
    candidate.surface_id_2,
    candidate.quality_score
  );
}

namespace angle_finding
{
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
  Standard_Real lin_deflection = 0.0001;
  Standard_Real ang_deflection = 0.5;
  BRepMesh_IncrementalMesh mesher(primary_shape_, lin_deflection, Standard_False, ang_deflection);
}

std::vector<GraspCandidate> GraspOrientationFinder::find_valid_grasps(
  const std::vector<sampling::ContactPair> & contact_pairs,
  const geometry::Topology & topology)
{
  std::vector<GraspCandidate> valid_grasps;

  size_t total_orientations_tested = 0;
  size_t rejected_by_primary = 0;
  size_t rejected_by_exclusion = 0;
  size_t rejected_by_secondary = 0;
  size_t pairs_with_no_edges = 0;

  RCLCPP_INFO(logger_, "Processing %zu contact pairs for orientation finding",
    contact_pairs.size());
  RCLCPP_INFO(logger_, "Configuration:");
  RCLCPP_INFO(logger_, "  finger_length: %.4f m", config_.finger_length);
  RCLCPP_INFO(logger_, "  finger_radius: %.4f m", config_.finger_radius);
  RCLCPP_INFO(logger_, "  max_edge_candidates: %zu", config_.max_edge_candidates);
  RCLCPP_INFO(logger_, "  dual_seed_dedup_tolerance_deg: %.1f",
        config_.dual_seed_dedup_tolerance_deg);
  RCLCPP_INFO(logger_, "  angle_offsets: [");
  for (double offset : config_.angle_offsets) {
    RCLCPP_INFO(logger_, "    %.1f°", offset);
  }
  RCLCPP_INFO(logger_, "  ]");
  RCLCPP_INFO(logger_, "  collision_tolerance: %.4f m", config_.collision_tolerance);
  RCLCPP_INFO(logger_, "  FCL checker: %s", fcl_checker_ ? "ENABLED" : "DISABLED (using OCCT)");

  if (contact_pairs.empty()) {
    RCLCPP_WARN(logger_, "No contact pairs provided - returning empty grasp list");
    return valid_grasps;
  }

  RCLCPP_DEBUG(logger_, "Exclusion constraint: %s",
    exclusion_constraint_ ? "enabled" : "disabled");
  RCLCPP_DEBUG(logger_, "Kissing surface constraint: %s",
    kissing_constraint_ ? "enabled" : "disabled");

  // Get angle offsets to test
  std::vector<double> offsets_to_test = config_.angle_offsets;
  if (offsets_to_test.empty()) {
    offsets_to_test.push_back(0.0);
  }

  for (const auto & pair : contact_pairs) {
    // Compute grip axis (Y axis in gripper frame convention)
    gp_Vec grip_axis(pair.contact_1, pair.contact_2);

    // Guard against coincident contact points (would cause normalization failure)
    if (grip_axis.Magnitude() < 1e-9) {
      RCLCPP_WARN(logger_, "Contact points coincide for pair (surfaces %d-%d) - skipping",
        pair.surface_id_1, pair.surface_id_2);
      continue;
    }
    grip_axis.Normalize();

    auto edges_1 = find_edges_in_circle(pair.contact_1, pair.surface_id_1, topology);
    auto edges_2 = find_edges_in_circle(pair.contact_2, pair.surface_id_2, topology);

    RCLCPP_DEBUG(logger_, "Contact Pair: surfaces [%d, %d]",
      pair.surface_id_1, pair.surface_id_2);
    RCLCPP_DEBUG(logger_, "  Contact 1: (%.4f, %.4f, %.4f)",
      pair.contact_1.X(), pair.contact_1.Y(), pair.contact_1.Z());
    RCLCPP_DEBUG(logger_, "  Contact 2: (%.4f, %.4f, %.4f)",
      pair.contact_2.X(), pair.contact_2.Y(), pair.contact_2.Z());
    RCLCPP_DEBUG(logger_, "  Grip distance: %.4f m", pair.grip_distance);
    RCLCPP_DEBUG(logger_, "  Edges found: %zu (contact_1), %zu (contact_2)",
      edges_1.size(), edges_2.size());

    // Log individual edge details
    for (size_t i = 0; i < edges_1.size() && i < 5; ++i) {
      RCLCPP_DEBUG(logger_, "    Edge_1[%zu]: dist=%.4f m, bearing=%.1f°",
        i, edges_1[i].distance, edges_1[i].bearing_angle * 180.0 / M_PI);
    }
    for (size_t i = 0; i < edges_2.size() && i < 5; ++i) {
      RCLCPP_DEBUG(logger_, "    Edge_2[%zu]: dist=%.4f m, bearing=%.1f°",
        i, edges_2[i].distance, edges_2[i].bearing_angle * 180.0 / M_PI);
    }

    if (edges_1.empty() && edges_2.empty()) {
      pairs_with_no_edges++;
      RCLCPP_DEBUG(logger_, "  REJECTED: no edges within finger_length (%.4f m) for either contact",
        config_.finger_length);
      continue;
    }

    // Find edge minima from both contact points, then merge them.
    // Since the gripper has 180 rotational symmetry (finger 1 and finger 2
    // are interchangeable), an orientation that points finger 1 away from an
    // edge at contact_1 is equivalent to pointing finger 2 away from an edge
    // at contact_2 rotated by 180.

    // Therefore, we shift contact_2's bearing angles by pi before merging,
    // expressing everything in "finger 1's coordinate system".

    auto minima_1 = find_local_minima(edges_1);
    auto minima_2 = find_local_minima(edges_2);

    // Compute bearing angles for deduplication
    for (auto & edge : minima_1) {
      edge.bearing_angle = compute_bearing_angle(pair.contact_1, edge.closest_point, grip_axis);
    }
    for (auto & edge : minima_2) {
      edge.bearing_angle = compute_bearing_angle(pair.contact_2, edge.closest_point, grip_axis);
    }

    RCLCPP_DEBUG(logger_, "  After distance sort: %zu edges (contact_1), %zu edges (contact_2)",
      minima_1.size(), minima_2.size());

    for (size_t i = 0; i < minima_1.size(); ++i) {
      RCLCPP_DEBUG(logger_, "    Edge_1[%zu]: dist=%.4f m, bearing=%.1f",
        i, minima_1[i].distance, minima_1[i].bearing_angle * 180.0 / M_PI);
    }
    for (size_t i = 0; i < minima_2.size(); ++i) {
      RCLCPP_DEBUG(logger_, "    Edge_2[%zu]: dist=%.4f m, bearing=%.1f",
        i, minima_2[i].distance, minima_2[i].bearing_angle * 180.0 / M_PI);
    }
    std::vector<EdgeConstraint> master_seeds = minima_1;

    for (auto m2 : minima_2) {  // Intentional copy - we modify m2.bearing_angle before inserting
      // Shift by 180 to express in finger 1's coordinate system
      m2.bearing_angle += M_PI;
      if (m2.bearing_angle > M_PI) {
        m2.bearing_angle -= 2.0 * M_PI;
      }

      // Deduplicate: skip if too close to existing seed
      double dedup_tolerance = config_.dual_seed_dedup_tolerance_deg * M_PI / 180.0;
      bool is_duplicate = false;
      for (const auto & m1 : master_seeds) {
        double angle_diff = std::abs(m1.bearing_angle - m2.bearing_angle);
        if (angle_diff > M_PI) {
          angle_diff = 2.0 * M_PI - angle_diff;
        }
        if (angle_diff < dedup_tolerance) {
          is_duplicate = true;
          break;
        }
      }
      if (!is_duplicate) {
        master_seeds.push_back(m2);
      }
    }

    RCLCPP_DEBUG(logger_, "  Master seeds (merged with dedup): %zu", master_seeds.size());
    for (size_t i = 0; i < master_seeds.size(); ++i) {
      RCLCPP_DEBUG(logger_, "    Seed[%zu]: bearing=%.1f°, dist=%.4f m",
        i, master_seeds[i].bearing_angle * 180.0 / M_PI, master_seeds[i].distance);
    }

    // Each approach direction is a (gp_Vec approach, double bearing_angle) pair
    std::vector<std::pair<gp_Vec, double>> approaches_to_test;

    for (const auto & edge_constraint : master_seeds) {
      for (double angle_offset : offsets_to_test) {
        gp_Vec approach = compute_approach_direction(
          pair.contact_1, pair.contact_2, edge_constraint, angle_offset, grip_axis);
        double total_angle = edge_constraint.bearing_angle * 180.0 / M_PI + angle_offset;
        approaches_to_test.emplace_back(approach, total_angle);
      }
    }

    // Limit orientations per pair for performance control
    if (config_.max_orientations_per_pair > 0 &&
      approaches_to_test.size() > config_.max_orientations_per_pair)
    {
      approaches_to_test.resize(config_.max_orientations_per_pair);
    }

    RCLCPP_DEBUG(logger_, "Found %zu/%zu edges, %zu/%zu minima, %zu master seeds, "
      "testing %zu orientations for surfaces %d-%d",
      edges_1.size(), edges_2.size(),
      minima_1.size(), minima_2.size(),
      master_seeds.size(), approaches_to_test.size(),
      pair.surface_id_1, pair.surface_id_2);

    RCLCPP_DEBUG(logger_, "Testing %zu orientations", approaches_to_test.size());

    size_t pair_rejected_primary = 0;
    size_t pair_rejected_exclusion = 0;
    size_t pair_rejected_secondary = 0;
    size_t pair_valid = 0;

    // Look if approach rejected by any constraint
    // TODO(@silanus23): Make constraints pluginized for here except primary shape
    for (const auto & [approach, angle_for_log] : approaches_to_test) {
      total_orientations_tested++;

      gp_Trsf transform = compute_gripper_transform(
        pair.contact_1, pair.contact_2, approach);

      RCLCPP_DEBUG(logger_, "  Testing angle %.1f°: approach=(%.3f, %.3f, %.3f)",
        angle_for_log, approach.X(), approach.Y(), approach.Z());

      // Check collision with shape itself
      if (collides_with_primary(transform, pair.grip_distance)) {
        rejected_by_primary++;
        pair_rejected_primary++;
        RCLCPP_DEBUG(logger_, "REJECTED: collision with PRIMARY shape");
        continue;
      }

      // Check collision with exclusion zones
      if (exclusion_constraint_ && exclusion_constraint_->intersects_exclusion_zone(
        transform, pair.grip_distance, config_.collision_tolerance))
      {
        rejected_by_exclusion++;
        pair_rejected_exclusion++;
        RCLCPP_DEBUG(logger_, "REJECTED: collision with EXCLUSION zone");
        continue;
      }

      // Check collision with secondary shapes (fixtures, ground)
      if (kissing_constraint_) {
        Eigen::Isometry3d grasp_pose = Eigen::Isometry3d::Identity();
        grasp_pose.translation() = geometry::extract_translation(transform);
        Eigen::Quaterniond q = geometry::extract_quaternion(transform);
        grasp_pose.linear() = q.toRotationMatrix();

        if (kissing_constraint_->intersects_secondary(pair.grip_distance, grasp_pose)) {
          rejected_by_secondary++;
          pair_rejected_secondary++;
          RCLCPP_DEBUG(logger_, "REJECTED: collision with SECONDARY shape");
          continue;
        }
      }

      // All checks passed - create valid grasp candidate
      GraspCandidate candidate;
      candidate.contact_1 = pair.contact_1;
      candidate.contact_2 = pair.contact_2;
      candidate.approach_direction = approach;
      candidate.gripper_transform = transform;
      candidate.surface_id_1 = pair.surface_id_1;
      candidate.surface_id_2 = pair.surface_id_2;
      candidate.grip_distance = pair.grip_distance;
      candidate.quality_score = compute_quality_score(
        pair.contact_1, pair.contact_2, edges_1, edges_2);

      valid_grasps.push_back(candidate);
      pair_valid++;
      RCLCPP_DEBUG(logger_, "VALID: quality=%.3f", candidate.quality_score);

      if (config_.stop_on_first_valid) {
        RCLCPP_DEBUG(logger_, "stop_on_first_valid enabled - skipping remaining orientations");
        break;
      }
    }

    // Per-pair summary (throttled to avoid flooding logs)
    RCLCPP_DEBUG(logger_,
          "  Pair result: %zu valid, rejected: %zu primary, %zu exclusion, %zu secondary",
      pair_valid, pair_rejected_primary, pair_rejected_exclusion, pair_rejected_secondary);
  }

  RCLCPP_INFO(logger_, "Summary:");
  RCLCPP_INFO(logger_, "  Contact pairs processed: %zu", contact_pairs.size());
  RCLCPP_INFO(logger_, "  Pairs rejected (unreachable - no edges): %zu", pairs_with_no_edges);
  RCLCPP_INFO(logger_, "  Total orientations tested: %zu", total_orientations_tested);
  RCLCPP_INFO(logger_, "  Valid grasps found: %zu", valid_grasps.size());

  if (total_orientations_tested > 0) {
    RCLCPP_INFO(logger_, "Rejection breakdown:");
    RCLCPP_INFO(logger_, "  Primary collision: %zu (%.1f%%)",
      rejected_by_primary,
      100.0 * rejected_by_primary / total_orientations_tested);
    RCLCPP_INFO(logger_, "  Exclusion zone: %zu (%.1f%%)",
      rejected_by_exclusion,
      100.0 * rejected_by_exclusion / total_orientations_tested);
    RCLCPP_INFO(logger_, "  Secondary collision: %zu (%.1f%%)",
      rejected_by_secondary,
      100.0 * rejected_by_secondary / total_orientations_tested);

    double success_rate = 100.0 * valid_grasps.size() / total_orientations_tested;
    RCLCPP_INFO(logger_, "  Success rate: %.1f%%", success_rate);
  } else {
    RCLCPP_WARN(logger_, "No orientations were tested! Check:");
  }

  if (valid_grasps.empty()) {
    RCLCPP_WARN(logger_, "NO VALID GRASPS FOUND!");
    RCLCPP_WARN(logger_, "Troubleshooting tips:");
    RCLCPP_WARN(logger_, "  1. Check gripper dimensions vs workpiece size");
    RCLCPP_WARN(logger_, "  2. Verify exclusion zones are not too large");
    RCLCPP_WARN(logger_, "  3. Check secondary shapes (table/fixture) placement");
    RCLCPP_WARN(logger_, "  4. Try increasing finger_length or collision_tolerance");
  }

  return valid_grasps;
}


std::vector<EdgeConstraint> GraspOrientationFinder::find_edges_in_circle(
  const gp_Pnt & contact,
  int surface_id,
  const geometry::Topology & topology) const
{
  std::vector<EdgeConstraint> all_constraints;

  // Check edges belonging to the contact surface
  const auto & surface = topology.get_surface(surface_id);
  const auto & surface_edge_ids = surface.edge_ids;

  // Store all minima that are within finger_length
  for (int edge_id : surface_edge_ids) {
    const auto & edge_data = topology.get_edge(edge_id);
    TopoDS_Edge edge = edge_data.edge;
    auto minima_on_edge = find_local_minima_on_edge(contact, edge);

    for (const auto & constraint : minima_on_edge) {
      all_constraints.push_back(constraint);
    }
  }

  // Apply max_edges_per_contact limit: sort by distance and truncate
  if (config_.max_edges_per_contact > 0 &&
    all_constraints.size() > config_.max_edges_per_contact)
  {
    std::sort(all_constraints.begin(), all_constraints.end(),
      [](const EdgeConstraint & a, const EdgeConstraint & b) {
        return a.distance < b.distance;
      });
    all_constraints.resize(config_.max_edges_per_contact);
    RCLCPP_DEBUG(logger_, "Truncated edge constraints to %zu (closest by distance)",
      config_.max_edges_per_contact);
  }

  return all_constraints;
}

std::vector<EdgeConstraint> GraspOrientationFinder::find_local_minima_on_edge(
  const gp_Pnt & contact,
  const TopoDS_Edge & edge) const
{
  // TODO(@silanus23): Cache extrema results using mutable member
  // This function is called repeatedly for the same edge/contact combinations.

  std::vector<EdgeConstraint> minima;

  // Use BRepExtrema_ExtPC to find all extrema (points where distance derivative = 0)
  // Extrema = critical points where distance function has local min/max (not just endpoints)
  // Then filter to keep only local minima within finger reach
  // NOTE: For straight line edges, ExtPC finds endpoints (not the perpendicular projection).
  // The fallback (lines 471-483) handles this case using BRepExtrema_DistShapeShape.
  BRepBuilderAPI_MakeVertex vertex_maker(contact);
  TopoDS_Vertex contact_vertex = vertex_maker.Vertex();

  try {
    BRepExtrema_ExtPC extrema(contact_vertex, edge);

    if (!extrema.IsDone()) {
      // Fallback: use single closest point method
      gp_Pnt closest_point;
      double distance = compute_closest_point_on_edge(contact, edge, closest_point);

      if (distance <= config_.finger_length && distance > 1e-6) {
        EdgeConstraint constraint;
        constraint.edge = edge;
        constraint.closest_point = closest_point;
        constraint.distance = distance;
        minima.push_back(constraint);
      }
      return minima;
    }

    const Standard_Integer nb_extrema = extrema.NbExt();

    for (Standard_Integer i = 1; i <= nb_extrema; ++i) {
      // Only keep local minima
      if (!extrema.IsMin(i)) {
        continue;
      }

      double distance = std::sqrt(extrema.SquareDistance(i));

      // Filter by finger reach
      if (distance > config_.finger_length || distance <= 1e-6) {
        continue;
      }

      EdgeConstraint constraint;
      constraint.edge = edge;
      constraint.closest_point = extrema.Point(i);
      constraint.distance = distance;
      minima.push_back(constraint);
    }

    // If no interior minima found (straight edge or monotonic curve),
    // fall back to global closest point.
    // For straight lines: this finds the perpendicular projection (the actual minimum).
    // For curves: this finds the global minimum when no local minima exist.
    if (minima.empty()) {
      gp_Pnt closest_point;
      double distance = compute_closest_point_on_edge(contact, edge, closest_point);

      if (distance <= config_.finger_length && distance > 1e-6) {
        EdgeConstraint constraint;
        constraint.edge = edge;
        constraint.closest_point = closest_point;
        constraint.distance = distance;
        minima.push_back(constraint);
      }
    }
  } catch (Standard_Failure & f) {
    RCLCPP_DEBUG(logger_, "OCCT ExtPC exception on edge: %s - using fallback",
          f.GetMessageString());
    // Fallback: use single closest point method
    gp_Pnt closest_point;
    double distance = compute_closest_point_on_edge(contact, edge, closest_point);

    if (distance <= config_.finger_length && distance > 1e-6) {
      EdgeConstraint constraint;
      constraint.edge = edge;
      constraint.closest_point = closest_point;
      constraint.distance = distance;
      minima.push_back(constraint);
    }
  }

  return minima;
}

std::vector<EdgeConstraint> GraspOrientationFinder::find_local_minima(
  const std::vector<EdgeConstraint> & edges) const
{
  if (edges.empty()) {
    return {};
  }

  // Sort edges by distance (closest first)
  std::vector<EdgeConstraint> result = edges;
  std::sort(result.begin(), result.end(),
    [](const EdgeConstraint & a, const EdgeConstraint & b) {
      return a.distance < b.distance;
    });

  // Limit to max_edge_candidates if specified
  if (config_.max_edge_candidates > 0 && result.size() > config_.max_edge_candidates) {
    result.resize(config_.max_edge_candidates);
  }

  return result;
}

double GraspOrientationFinder::compute_bearing_angle(
  const gp_Pnt & contact,
  const gp_Pnt & edge_point,
  const gp_Vec & grip_axis) const
{
  // Direction from contact to edge point
  gp_Vec to_edge(contact, edge_point);

  if (to_edge.Magnitude() < 1e-9) {
    return 0.0;
  }

  // Project onto plane perpendicular to grip axis (measure angle in 2D plane,
  // ignore component along grip)
  gp_Vec projected = to_edge - (to_edge.Dot(grip_axis)) * grip_axis;

  if (projected.Magnitude() < 1e-9) {
    return 0.0;
  }

  projected.Normalize();  // Unit vector needed for angle computation

  // ref_x is an arbitrary but consistent direction perpendicular to grip_axis
  gp_Vec ref_x;
  if (std::abs(grip_axis.Z()) < 0.9) {
    ref_x = grip_axis.Crossed(gp_Vec(0, 0, 1));
  } else {
    ref_x = grip_axis.Crossed(gp_Vec(1, 0, 0));
  }
  ref_x.Normalize();  // Ensure unit vector for consistent angle measurement

  // ref_y completes the right-handed system
  gp_Vec ref_y = grip_axis.Crossed(ref_x);
  ref_y.Normalize();  // Cross product may not be unit length

  double x_component = projected.Dot(ref_x);
  double y_component = projected.Dot(ref_y);

  return std::atan2(y_component, x_component);
}

gp_Vec GraspOrientationFinder::compute_approach_direction(
  const gp_Pnt & contact_1,
  const gp_Pnt & contact_2,
  const EdgeConstraint & edge_constraint,
  double angle_offset,
  const gp_Vec & grip_axis) const
{
  // Compute TCP (Tool Center Point) as midpoint between contacts
  gp_Pnt tcp(
    (contact_1.X() + contact_2.X()) / 2.0,
    (contact_1.Y() + contact_2.Y()) / 2.0,
    (contact_1.Z() + contact_2.Z()) / 2.0
  );

  // Direction from edge toward TCP (gripper approaches from this direction)
  // NOTE: Using TCP is semantically correct - the gripper approaches as a unit,
  // not individual fingers.
  // After projection onto plane ⊥ grip_axis, this is mathematically equivalent to using individual
  // contacts (the component along grip_axis gets removed anyway).
  gp_Vec away_from_edge(edge_constraint.closest_point, tcp);

  if (away_from_edge.Magnitude() < 1e-9) {
    // Edge point coincides with contact, use arbitrary perpendicular
    if (std::abs(grip_axis.Z()) < 0.9) {
      away_from_edge = grip_axis.Crossed(gp_Vec(0, 0, 1));
    } else {
      away_from_edge = grip_axis.Crossed(gp_Vec(1, 0, 0));
    }
  }

  away_from_edge.Normalize();  // Need unit direction vector

  // Project to plane ⊥ grip axis (fingers must approach perpendicular to grip direction)
  gp_Vec projected = away_from_edge - (away_from_edge.Dot(grip_axis)) * grip_axis;

  if (projected.Magnitude() < 1e-6) {
    if (std::abs(grip_axis.Z()) < 0.9) {
      projected = grip_axis.Crossed(gp_Vec(0, 0, 1));
    } else {
      projected = grip_axis.Crossed(gp_Vec(1, 0, 0));
    }
  }

  projected.Normalize();  // Restore unit length after projection

  // Apply angle offset (rotation around grip axis at TCP)
  if (std::abs(angle_offset) > 1e-6) {
    gp_Ax1 rotation_axis(tcp, gp_Dir(grip_axis.X(), grip_axis.Y(), grip_axis.Z()));
    gp_Trsf rotation;
    rotation.SetRotation(rotation_axis, angle_offset * M_PI / 180.0);
    projected = projected.Transformed(rotation);
  }

  return projected;
}

gp_Trsf GraspOrientationFinder::compute_gripper_transform(
  const gp_Pnt & contact_1,
  const gp_Pnt & contact_2,
  const gp_Vec & approach) const
{
  // Calculate desired TCP position (midpoint between finger contact points)
  gp_Pnt tcp(
    (contact_1.X() + contact_2.X()) / 2.0,
    (contact_1.Y() + contact_2.Y()) / 2.0,
    (contact_1.Z() + contact_2.Z()) / 2.0
  );

  // TODO(@silanus23): delete these comments after finishing documentation.
  // URDF Convention:
  // Y axis = finger opening direction (grip axis: contact_1 -> contact_2)
  // Z axis = points from TCP toward gripper base (opposite of approach into object)
  // X axis = perpendicular (right-hand rule)

  // Y axis: grip direction (from contact_1 to contact_2)
  gp_Vec y_axis(contact_1, contact_2);
  y_axis.Normalize();  // Coordinate frame axes must be unit vectors

  // X axis: approach direction (perpendicular to grip axis)
  // This determines the "roll" of the gripper
  gp_Vec x_axis = approach;
  x_axis.Normalize();  // Coordinate frame axes must be unit vectors

  // Ensure X is perpendicular to Y (remove any component along Y due to numerical error)
  x_axis = x_axis - (x_axis.Dot(y_axis)) * y_axis;
  if (x_axis.Magnitude() < 1e-6) {
    // approach was parallel to grip axis, compute perpendicular
    if (std::abs(y_axis.Z()) < 0.9) {
      x_axis = y_axis.Crossed(gp_Vec(0, 0, 1));
    } else {
      x_axis = y_axis.Crossed(gp_Vec(1, 0, 0));
    }
  }
  x_axis.Normalize();  // Projection changes magnitude, restore unit length

  gp_Vec z_axis = x_axis.Crossed(y_axis);
  z_axis.Normalize();  // Complete orthonormal basis (unit vectors required)

  // CRITICAL: Account for TCP offset from gripper base
  // The gripper geometry has its base at origin and TCP at tcp_offset.
  // We want the TCP (not the base) to end up at the calculated tcp position.
  // So we need to calculate where the base should be positioned.
  //
  // In the gripper's local frame: tcp_local = base_local + tcp_offset
  // In world frame: tcp_world = base_world + R * tcp_offset
  // Therefore: base_world = tcp_world - R * tcp_offset
  //
  // We build the rotation matrix R from our axes, then transform the tcp_offset
  // and subtract it from the desired tcp position to get the base position.

  // Create rotation matrix from our computed axes (X, Y, Z)
  gp_Vec tcp_offset_local(
    gripper_.tcp_offset.x(),
    gripper_.tcp_offset.y(),
    gripper_.tcp_offset.z()
  );

  // Rotate tcp_offset into world frame using our gripper orientation
  // tcp_offset_world = R * tcp_offset_local
  // where R is defined by our x_axis, y_axis, z_axis
  gp_Vec tcp_offset_world(
    x_axis.X() * tcp_offset_local.X() + y_axis.X() * tcp_offset_local.Y() + z_axis.X() *
    tcp_offset_local.Z(),
    x_axis.Y() * tcp_offset_local.X() + y_axis.Y() * tcp_offset_local.Y() + z_axis.Y() *
    tcp_offset_local.Z(),
    x_axis.Z() * tcp_offset_local.X() + y_axis.Z() * tcp_offset_local.Y() + z_axis.Z() *
    tcp_offset_local.Z()
  );

  // Calculate where the base should be: base = tcp - R * tcp_offset
  gp_Pnt base(
    tcp.X() - tcp_offset_world.X(),
    tcp.Y() - tcp_offset_world.Y(),
    tcp.Z() - tcp_offset_world.Z()
  );

  // Create gripper frame at the BASE position (not TCP position)
  // This ensures that when the gripper geometry (which has TCP at an offset)
  // is transformed, the TCP ends up at the desired contact midpoint
  gp_Ax3 gripper_frame(
    base,
    gp_Dir(z_axis.X(), z_axis.Y(), z_axis.Z()),
    gp_Dir(x_axis.X(), x_axis.Y(), x_axis.Z())
  );

  gp_Trsf transform;
  transform.SetTransformation(gripper_frame, gp_Ax3());

  // Invert because SetTransformation gives transform from gripper to world
  // We want transform that positions gripper at this frame
  transform.Invert();

  return transform;
}

bool GraspOrientationFinder::collides_with_primary(
  const gp_Trsf & gripper_transform,
  double grip_distance) const
{
  // Use FCL if available (fast path)
  if (fcl_checker_ && fcl_checker_->is_valid()) {
    return fcl_checker_->collides_with_primary(
      gripper_transform, grip_distance, config_.collision_tolerance);
  }

  // TODO(@silanus23): These slow paths shall be elliminated in
  // time after making sure of the usage of FCL
  // Fallback to OCCT (slow path)
  TopoDS_Shape configured_gripper = configure_gripper(gripper_, grip_distance);

  TopoDS_Shape transformed_gripper =
    BRepBuilderAPI_Transform(configured_gripper, gripper_transform, Standard_True).Shape();

  // Check distance between gripper and primary shape
  BRepExtrema_DistShapeShape dist_check;
  dist_check.LoadS1(transformed_gripper);
  dist_check.LoadS2(primary_shape_);
  dist_check.Perform();

  if (dist_check.IsDone() && dist_check.NbSolution() > 0) {
    double min_distance = dist_check.Value();

    // Note: We allow some penetration tolerance for numerical stability
    if (min_distance < config_.collision_tolerance) {
      return true;
    }
  }

  return false;
}

// TODO(@silanus23): Make this configurable and pluginized
double GraspOrientationFinder::compute_quality_score(
  [[maybe_unused]] const gp_Pnt & contact_1,
  [[maybe_unused]] const gp_Pnt & contact_2,
  const std::vector<EdgeConstraint> & edges_1,
  const std::vector<EdgeConstraint> & edges_2) const
{
  // TODO(@silanus23): Use contact_1 and contact_2 for more sophisticated quality metrics
  // (e.g., distance to surface center, force closure analysis, etc.)
  double min_clearance_1 = config_.finger_length;
  double min_clearance_2 = config_.finger_length;

  for (const auto & edge : edges_1) {
    min_clearance_1 = std::min(min_clearance_1, edge.distance);
  }

  for (const auto & edge : edges_2) {
    min_clearance_2 = std::min(min_clearance_2, edge.distance);
  }

  double min_clearance = std::min(min_clearance_1, min_clearance_2);

  return min_clearance / config_.finger_length;
}

double GraspOrientationFinder::compute_closest_point_on_edge(
  const gp_Pnt & point,
  const TopoDS_Edge & edge,
  gp_Pnt & closest_point) const
{
  // Check for degenerate edges (poles/seams on surfaces of revolution)
  if (BRep_Tool::Degenerated(edge)) {
    RCLCPP_DEBUG(logger_, "compute_closest_point_on_edge: edge is degenerate");
    closest_point = point;
    return std::numeric_limits<double>::max();
  }

  try {
    BRepExtrema_DistShapeShape dist;
    dist.LoadS1(BRepBuilderAPI_MakeVertex(point).Vertex());
    dist.LoadS2(edge);
    dist.Perform();

    if (dist.IsDone() && dist.NbSolution() > 0) {
      closest_point = dist.PointOnShape2(1);
      return dist.Value();
    }

    closest_point = point;
    return std::numeric_limits<double>::max();
  } catch (Standard_Failure & f) {
    RCLCPP_DEBUG(logger_, "compute_closest_point_on_edge: OCCT exception - %s",
      f.GetMessageString());
    closest_point = point;
    return std::numeric_limits<double>::max();
  } catch (...) {
    RCLCPP_DEBUG(logger_, "compute_closest_point_on_edge: unknown exception");
    closest_point = point;
    return std::numeric_limits<double>::max();
  }
}

void GraspOrientationFinder::set_fcl_checker(
  std::shared_ptr<const geometry::FCLCollisionChecker> fcl_checker)
{
  fcl_checker_ = fcl_checker;
}

}  // namespace angle_finding
}  // namespace hold_and_weld_gripper_sampler
