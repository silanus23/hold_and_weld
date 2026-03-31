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
namespace angle_finding
{

Grasp to_grasp(const GraspCandidate & candidate)
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

  if (contact_pairs.empty()) {
    RCLCPP_WARN(logger_, "No contact pairs provided - returning empty grasp list");
    return valid_grasps;
  }

  size_t total_orientations_tested = 0;
  size_t rejected_by_primary = 0;
  size_t rejected_by_exclusion = 0;
  size_t rejected_by_secondary = 0;
  size_t pairs_with_no_edges = 0;

  RCLCPP_INFO(logger_, "Processing %zu contact pairs for orientation finding",
    contact_pairs.size());
  RCLCPP_DEBUG(logger_, "Config: finger_length=%.4f m, max_edge_candidates=%zu, "
    "dedup_tolerance=%.1f deg, collision_tolerance=%.4f m",
    config_.finger_length, config_.max_edge_candidates,
    config_.dual_seed_dedup_tolerance_deg, config_.collision_tolerance);

  // Fall back to a single zero offset if none configured
  std::vector<double> offsets_to_test = config_.angle_offsets;
  if (offsets_to_test.empty()) {
    offsets_to_test.push_back(0.0);
  }

  for (const auto & pair : contact_pairs) {
    gp_Vec grip_axis(pair.contact_1, pair.contact_2);

    // Guard against coincident contact points
    if (grip_axis.Magnitude() < 1e-9) {
      RCLCPP_WARN(logger_, "Contact points coincide for pair (surfaces %d-%d) - skipping",
        pair.surface_id_1, pair.surface_id_2);
      continue;
    }
    grip_axis.Normalize();

    auto edges_1 = find_edges_in_circle(pair.contact_1, pair.surface_id_1, topology);
    auto edges_2 = find_edges_in_circle(pair.contact_2, pair.surface_id_2, topology);

    RCLCPP_DEBUG(logger_, "Pair [%d-%d]: contact_1=(%.4f, %.4f, %.4f) "
      "contact_2=(%.4f, %.4f, %.4f) grip=%.4f m, edges=%zu/%zu",
      pair.surface_id_1, pair.surface_id_2,
      pair.contact_1.X(), pair.contact_1.Y(), pair.contact_1.Z(),
      pair.contact_2.X(), pair.contact_2.Y(), pair.contact_2.Z(),
      pair.grip_distance, edges_1.size(), edges_2.size());

    // One side having no edges is valid — the dual-seed strategy uses whichever
    // side has edges. Reject only when both sides have none.
    if (edges_1.empty() && edges_2.empty()) {
      pairs_with_no_edges++;
      RCLCPP_DEBUG(logger_, "  No edges within finger_length (%.4f m) - skipping",
        config_.finger_length);
      continue;
    }

    auto minima_1 = find_local_minima(edges_1);
    auto minima_2 = find_local_minima(edges_2);

    // Bearing angles must be computed before logging or deduplication
    for (auto & edge : minima_1) {
      edge.bearing_angle = compute_bearing_angle(pair.contact_1, edge.closest_point, grip_axis);
    }
    for (auto & edge : minima_2) {
      edge.bearing_angle = compute_bearing_angle(pair.contact_2, edge.closest_point, grip_axis);
    }

    for (size_t i = 0; i < edges_1.size() && i < 5; ++i) {
      RCLCPP_DEBUG(logger_, "    Edge_1[%zu]: dist=%.4f m, bearing=%.1f°",
        i, minima_1[i].distance, minima_1[i].bearing_angle * 180.0 / M_PI);
    }
    for (size_t i = 0; i < edges_2.size() && i < 5; ++i) {
      RCLCPP_DEBUG(logger_, "    Edge_2[%zu]: dist=%.4f m, bearing=%.1f°",
        i, minima_2[i].distance, minima_2[i].bearing_angle * 180.0 / M_PI);
    }

    // Merge seeds from both contacts. Finger symmetry means an edge at contact_2
    // with bearing θ is equivalent to one at contact_1 with bearing θ+π, so we
    // shift contact_2 angles by π before deduplication.
    std::vector<EdgeConstraint> master_seeds = minima_1;

    for (auto m2 : minima_2) {  // Intentional copy — bearing_angle is modified before insert
      m2.bearing_angle += M_PI;
      if (m2.bearing_angle > M_PI) {
        m2.bearing_angle -= 2.0 * M_PI;
      }

      const double dedup_rad = config_.dual_seed_dedup_tolerance_deg * M_PI / 180.0;
      bool is_duplicate = false;
      for (const auto & m1 : master_seeds) {
        double diff = std::abs(m1.bearing_angle - m2.bearing_angle);
        if (diff > M_PI) {diff = 2.0 * M_PI - diff;}
        if (diff < dedup_rad) {
          is_duplicate = true;
          break;
        }
      }
      if (!is_duplicate) {
        master_seeds.push_back(m2);
      }
    }

    RCLCPP_DEBUG(logger_, "  %zu master seed(s) after dual-seed merge", master_seeds.size());

    // Multiple approaches tried to see if angled approaches has cleaner sight
    std::vector<std::pair<gp_Vec, double>> approaches_to_test;
    for (const auto & seed : master_seeds) {
      for (double offset : offsets_to_test) {
        gp_Vec approach = compute_approach_direction(
          pair.contact_1, pair.contact_2, seed, offset, grip_axis);
        approaches_to_test.emplace_back(approach, seed.bearing_angle * 180.0 / M_PI + offset);
      }
    }

    // Silently drops lower-priority candidates (later seeds, later offsets) when capped
    if (config_.max_orientations_per_pair > 0 &&
      approaches_to_test.size() > config_.max_orientations_per_pair)
    {
      approaches_to_test.resize(config_.max_orientations_per_pair);
    }

    // TODO(@silanus23): Make constraints pluginizable for orientation checking
    // Constraint validation loop
    for (const auto & [approach, angle_for_log] : approaches_to_test) {
      total_orientations_tested++;

      gp_Trsf transform = compute_gripper_transform(
        pair.contact_1, pair.contact_2, approach);

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

      if (kissing_constraint_) {
        Eigen::Isometry3d grasp_pose = Eigen::Isometry3d::Identity();
        grasp_pose.translation() = geometry::extract_translation(transform);
        grasp_pose.linear() = geometry::extract_quaternion(transform).toRotationMatrix();

        if (kissing_constraint_->intersects_secondary(pair.grip_distance, grasp_pose)) {
          rejected_by_secondary++;
          continue;
        }
      }

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
      RCLCPP_DEBUG(logger_, "  angle=%.1f°: valid (quality=%.3f)",
        angle_for_log, candidate.quality_score);

      if (config_.stop_on_first_valid) {
        break;
      }
    }
  }

  RCLCPP_INFO(logger_, "Orientation finding complete: %zu valid grasps from %zu pairs "
    "(%zu no-edge rejections, %zu orientations tested, "
    "rejected: %zu primary / %zu exclusion / %zu secondary)",
    valid_grasps.size(), contact_pairs.size(), pairs_with_no_edges,
    total_orientations_tested,
    rejected_by_primary, rejected_by_exclusion, rejected_by_secondary);

  if (valid_grasps.empty()) {
    RCLCPP_WARN(logger_, "No valid grasps found");
  }

  return valid_grasps;
}

std::vector<EdgeConstraint> GraspOrientationFinder::find_edges_in_circle(
  const gp_Pnt & contact,
  int surface_id,
  const geometry::Topology & topology) const
{
  std::vector<EdgeConstraint> all_constraints;

  const auto & surface = topology.get_surface(surface_id);

  for (int edge_id : surface.edge_ids) {
    const auto & edge_data = topology.get_edge(edge_id);
    auto minima_on_edge = find_local_minima_on_edge(contact, edge_data.edge);
    all_constraints.insert(all_constraints.end(), minima_on_edge.begin(), minima_on_edge.end());
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
  // NOTE: For straight line edges, ExtPC finds endpoints not the perpendicular projection.

  std::vector<EdgeConstraint> minima;

  const auto make_fallback_constraint = [&]() -> std::vector<EdgeConstraint> {
    gp_Pnt closest_point;
    double distance = compute_closest_point_on_edge(contact, edge, closest_point);
    if (distance <= config_.finger_length && distance > 1e-6) {
      EdgeConstraint constraint;
      constraint.edge = edge;
      constraint.closest_point = closest_point;
      constraint.distance = distance;
      return {constraint};
    }
    return {};
  };

  try {
  BRepBuilderAPI_MakeVertex vertex_maker(contact);
  TopoDS_Vertex contact_vertex = vertex_maker.Vertex();
    BRepExtrema_ExtPC extrema(contact_vertex, edge);

    if (!extrema.IsDone()) {
      return make_fallback_constraint();
    }

    const Standard_Integer nb_extrema = extrema.NbExt();
    for (Standard_Integer i = 1; i <= nb_extrema; ++i) {
      if (!extrema.IsMin(i)) {
        continue;
      }

      double distance = std::sqrt(extrema.SquareDistance(i));
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
    if (minima.empty()) {
      return make_fallback_constraint();
    }
  } catch (Standard_Failure & f) {
    RCLCPP_DEBUG(logger_, "ExtPC exception on edge: %s - using fallback",
      f.GetMessageString());
    return make_fallback_constraint();
  }

  return minima;
}

std::vector<EdgeConstraint> GraspOrientationFinder::find_local_minima(
  const std::vector<EdgeConstraint> & edges) const
{
  if (edges.empty()) {
    return {};
  }

  std::vector<EdgeConstraint> result = edges;
  std::sort(result.begin(), result.end(),
    [](const EdgeConstraint & a, const EdgeConstraint & b) {
      return a.distance < b.distance;
    });

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
  gp_Vec to_edge(contact, edge_point);

  if (to_edge.Magnitude() < 1e-9) {
    return 0.0;
  }

  // Project onto plane perpendicular to grip axis
  gp_Vec projected = to_edge - (to_edge.Dot(grip_axis)) * grip_axis;

  if (projected.Magnitude() < 1e-9) {
    return 0.0;
  }

  projected.Normalize();

  // Arbitrary but consistent reference frame perpendicular to grip_axis
  gp_Vec ref_x;
  if (std::abs(grip_axis.Z()) < 0.9) {
    ref_x = grip_axis.Crossed(gp_Vec(0, 0, 1));
  } else {
    ref_x = grip_axis.Crossed(gp_Vec(1, 0, 0));
  }
  ref_x.Normalize();

  gp_Vec ref_y = grip_axis.Crossed(ref_x);
  ref_y.Normalize();

  return std::atan2(projected.Dot(ref_y), projected.Dot(ref_x));
}

gp_Vec GraspOrientationFinder::compute_approach_direction(
  const gp_Pnt & contact_1,
  const gp_Pnt & contact_2,
  const EdgeConstraint & edge_constraint,
  double angle_offset,
  const gp_Vec & grip_axis) const
{
  gp_Pnt tcp(
    (contact_1.X() + contact_2.X()) / 2.0,
    (contact_1.Y() + contact_2.Y()) / 2.0,
    (contact_1.Z() + contact_2.Z()) / 2.0
  );

  gp_Vec away_from_edge(edge_constraint.closest_point, tcp);

  if (away_from_edge.Magnitude() < 1e-9) {
    if (std::abs(grip_axis.Z()) < 0.9) {
      away_from_edge = grip_axis.Crossed(gp_Vec(0, 0, 1));
    } else {
      away_from_edge = grip_axis.Crossed(gp_Vec(1, 0, 0));
    }
  }

  away_from_edge.Normalize();

  // Project to plane perpendicular to grip axis
  gp_Vec projected = away_from_edge - (away_from_edge.Dot(grip_axis)) * grip_axis;

  if (projected.Magnitude() < 1e-9) {
    if (std::abs(grip_axis.Z()) < 0.9) {
      projected = grip_axis.Crossed(gp_Vec(0, 0, 1));
    } else {
      projected = grip_axis.Crossed(gp_Vec(1, 0, 0));
    }
  }

  projected.Normalize();

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
  gp_Pnt tcp(
    (contact_1.X() + contact_2.X()) / 2.0,
    (contact_1.Y() + contact_2.Y()) / 2.0,
    (contact_1.Z() + contact_2.Z()) / 2.0
  );

  // URDF convention: Y = grip axis, Z = toward base, X = approach (right-hand rule)
  gp_Vec y_axis(contact_1, contact_2);
  y_axis.Normalize();

  gp_Vec x_axis = approach;
  x_axis.Normalize();

  // Re-orthogonalize X against Y to remove any numerical drift
  x_axis = x_axis - (x_axis.Dot(y_axis)) * y_axis;
  if (x_axis.Magnitude() < 1e-6) {
    if (std::abs(y_axis.Z()) < 0.9) {
      x_axis = y_axis.Crossed(gp_Vec(0, 0, 1));
    } else {
      x_axis = y_axis.Crossed(gp_Vec(1, 0, 0));
    }
  }
  x_axis.Normalize();

  gp_Vec z_axis = x_axis.Crossed(y_axis);
  z_axis.Normalize();

  // Place the gripper base such that the TCP lands at the contact midpoint.
  // base_world = tcp_world - R * tcp_offset_local
  gp_Vec tcp_offset_local(
    gripper_.tcp_offset.x(),
    gripper_.tcp_offset.y(),
    gripper_.tcp_offset.z()
  );

  gp_Vec tcp_offset_world(
    x_axis.X() * tcp_offset_local.X() + y_axis.X() * tcp_offset_local.Y() +
    z_axis.X() * tcp_offset_local.Z(),
    x_axis.Y() * tcp_offset_local.X() + y_axis.Y() * tcp_offset_local.Y() +
    z_axis.Y() * tcp_offset_local.Z(),
    x_axis.Z() * tcp_offset_local.X() + y_axis.Z() * tcp_offset_local.Y() +
    z_axis.Z() * tcp_offset_local.Z()
  );

  gp_Pnt base(
    tcp.X() - tcp_offset_world.X(),
    tcp.Y() - tcp_offset_world.Y(),
    tcp.Z() - tcp_offset_world.Z()
  );

  gp_Ax3 gripper_frame(
    base,
    gp_Dir(z_axis.X(), z_axis.Y(), z_axis.Z()),
    gp_Dir(x_axis.X(), x_axis.Y(), x_axis.Z())
  );

  gp_Trsf transform;
  transform.SetTransformation(gripper_frame, gp_Ax3());
  transform.Invert();

  return transform;
}

bool GraspOrientationFinder::collides_with_primary(
  const gp_Trsf & gripper_transform,
  double grip_distance) const
{
  if (!fcl_checker_ || !fcl_checker_->is_valid()) {
    RCLCPP_WARN(logger_, "FCL checker not available - rejecting grasp conservatively");
    return true;
  }

  return fcl_checker_->collides_with_primary(
    gripper_transform, grip_distance, config_.collision_tolerance);
}

// TODO(@silanus23): Make this configurable and pluginized
double GraspOrientationFinder::compute_quality_score(
  [[maybe_unused]] const gp_Pnt & contact_1,
  [[maybe_unused]] const gp_Pnt & contact_2,
  const std::vector<EdgeConstraint> & edges_1,
  const std::vector<EdgeConstraint> & edges_2) const
{
  // TODO(@silanus23): Use contact_1 and contact_2 for more sophisticated quality metrics
  double min_clearance_1 = config_.finger_length;
  double min_clearance_2 = config_.finger_length;

  for (const auto & edge : edges_1) {
    min_clearance_1 = std::min(min_clearance_1, edge.distance);
  }
  for (const auto & edge : edges_2) {
    min_clearance_2 = std::min(min_clearance_2, edge.distance);
  }

  return std::min(min_clearance_1, min_clearance_2) / config_.finger_length;
}

double GraspOrientationFinder::compute_closest_point_on_edge(
  const gp_Pnt & point,
  const TopoDS_Edge & edge,
  gp_Pnt & closest_point) const
{
  if (BRep_Tool::Degenerated(edge)) {
    RCLCPP_DEBUG(logger_, "compute_closest_point_on_edge: degenerate edge");
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
