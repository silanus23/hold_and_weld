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

#include "hold_and_weld_gripper_sampler/core/grasp_finder.hpp"
#include "hold_and_weld_gripper_sampler/geometry/shape_refiner.hpp"

#include <algorithm>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <vector>

#include <rclcpp/rclcpp.hpp>

namespace hold_and_weld_gripper_sampler
{
namespace core
{

// Formats an Eigen vector as "(x, y, z)" for logging
static std::string vec_to_string(const Eigen::Vector3d & v)
{
  std::ostringstream oss;
  oss << "(" << std::fixed << std::setprecision(4)
      << v.x() << ", " << v.y() << ", " << v.z() << ")";
  return oss.str();
}

GraspFinder::GraspFinder(
  std::shared_ptr<const geometry::GeometryMapper> mapper,
  const TopoDS_Shape & primary_shape,
  const geometry::Topology & primary_topology,
  const ParsedGripper & gripper,
  const std::vector<TopoDS_Shape> & secondary_shapes,
  const std::optional<std::vector<constraints::exclusion_circle>> & exclusion_circles,
  const std::optional<std::vector<constraints::exclusion_polygon>> & exclusion_polygons,
  const std::optional<std::vector<constraints::exclusion_line>> & exclusion_lines,
  const GraspFinderConfig & config)
: primary_shape_(primary_shape),
  primary_topology_(primary_topology),
  gripper_(gripper),
  secondary_shapes_(secondary_shapes),
  config_(config),
  logger_(rclcpp::get_logger("gripper_sampler"))
{
  if (!mapper) {
    // No mapper provided — build one from the primary shape
    RCLCPP_WARN(logger_,"No mapper provided building one");
    auto m = std::make_shared<geometry::GeometryMapper>();
    if (!primary_shape_.IsNull()) {
      m->load_from_shape(primary_shape_, "workpiece");
    }
    mapper_ = std::move(m);
  } else {
    mapper_ = mapper;
  }

  if (exclusion_circles.has_value()) {exclusion_circles_ = exclusion_circles.value();}
  if (exclusion_polygons.has_value()) {exclusion_polygons_ = exclusion_polygons.value();}
  if (exclusion_lines.has_value()) {exclusion_lines_ = exclusion_lines.value();}

  RCLCPP_INFO(logger_, "GraspFinder: %zu surfaces, %zu secondaries, "
    "%zu circles, %zu polygons, %zu lines",
    primary_topology_.num_surfaces(), secondary_shapes_.size(),
    exclusion_circles_.size(), exclusion_polygons_.size(), exclusion_lines_.size());
  RCLCPP_DEBUG(logger_, "Config: density=%.4f m, opening=[%.4f, %.4f] m, "
    "kissing_threshold=%.1f%%",
    config_.sampling.sample_density,
    config_.sampling.min_gripper_opening, config_.sampling.max_gripper_opening,
    config_.kissing_contact_threshold * 100.0);
}

// TODO(@silanus23): For constraint pluginization constraints need to be able to take these
// valuse either from own reader or from a publisher current solution is temporary
std::string GraspFinder::initialize()
{
  if (initialized_) {
    return "";
  }

  try {
    RCLCPP_INFO(logger_, "Initializing GraspFinder");

    std::optional<std::vector<constraints::exclusion_circle>> circles_opt;
    std::optional<std::vector<constraints::exclusion_polygon>> polygons_opt;
    std::optional<std::vector<constraints::exclusion_line>> lines_opt;

    if (!exclusion_circles_.empty()) {circles_opt = exclusion_circles_;}
    if (!exclusion_polygons_.empty()) {polygons_opt = exclusion_polygons_;}
    if (!exclusion_lines_.empty()) {lines_opt = exclusion_lines_;}

    exclusion_constraint_ = std::make_shared<constraints::ExclusionZoneConstraint>(
      mapper_, gripper_, circles_opt, polygons_opt, lines_opt,
      config_.mesh_linear_deflection, config_.mesh_angular_deflection);

    kissing_constraint_ = std::make_shared<constraints::KissingSurfaceConstraint>(
      mapper_, gripper_, secondary_shapes_,
      config_.kissing_contact_threshold, config_.collision_tolerance);

    exclusion_constraint_->analyze_constraints(primary_shape_, primary_topology_);
    kissing_constraint_->analyze_constraints(primary_topology_);

    if (!config_.use_fcl) {
      RCLCPP_ERROR(logger_, "FCL is required - set use_fcl=true");
      return "FCL is required but use_fcl=false";
    }

    const bool use_ground = config_.enable_ground_plane_check && config_.use_fcl_for_ground_plane;

    fcl_checker_ = std::make_shared<geometry::FCLCollisionChecker>(
      gripper_,
      primary_shape_,
      exclusion_constraint_->get_collision_volumes(),
      kissing_constraint_->get_secondary_shapes(),
      use_ground,
      config_.ground_z,
      config_.triangulation_deflection);

    if (!fcl_checker_->is_valid()) {
      RCLCPP_ERROR(logger_, "FCL checker initialization failed");
      return "FCL checker initialization failed";
    }

    exclusion_constraint_->set_fcl_checker(fcl_checker_);
    kissing_constraint_->set_fcl_checker(fcl_checker_);

    RCLCPP_INFO(logger_, "Initialization complete: FCL ready, ground_plane=%s (z=%.3f m)",
      use_ground ? "enabled" : "disabled", config_.ground_z);

    initialized_ = true;
    return "";
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger_, "Initialization failed: %s", e.what());
    return std::string("Initialization failed: ") + e.what();
  } catch (...) {
    RCLCPP_ERROR(logger_, "Initialization failed: unknown exception");
    return "Initialization failed: unknown exception";
  }
}

GraspFinderResult GraspFinder::find()
{
  GraspFinderResult result;

  std::string init_error = initialize();
  if (!init_error.empty()) {
    result.success = false;
    result.error_message = init_error;
    return result;
  }

  try {
    auto banned_ids = kissing_constraint_->get_banned_surface_ids();
    auto valid_ids = compute_valid_surface_ids(banned_ids);
    auto exclusion_areas = merge_sample_areas();

    result.num_banned_surfaces = banned_ids.size();
    result.num_valid_surfaces = valid_ids.size();
    result.num_exclusion_areas = exclusion_areas.size();

    RCLCPP_INFO(logger_, "Phase 1: %zu/%zu surfaces valid, %zu banned, %zu exclusion wires",
      result.num_valid_surfaces, primary_topology_.num_surfaces(),
      result.num_banned_surfaces, result.num_exclusion_areas);

    if (!valid_ids.empty()) {
      // Build valid surface ID list for debug log
      std::string valid_str;
      for (size_t i = 0; i < valid_ids.size() && i < 20; i++) {
        if (i > 0) {valid_str += ", ";}
        valid_str += std::to_string(valid_ids[i]);
      }
      if (valid_ids.size() > 20) {valid_str += ", ...";}
      RCLCPP_DEBUG(logger_, "Valid surface IDs: [%s]", valid_str.c_str());
    }

    if (valid_ids.empty()) {
      result.success = true;
      result.error_message = "No valid surfaces for grasping (all banned or excluded)";
      RCLCPP_WARN(logger_, "%s", result.error_message.c_str());
      return result;
    }

    sampling::ContactPointSampler sampler(config_.sampling);
    auto contact_pairs = sampler.generate_contact_pairs(
      primary_topology_, valid_ids, exclusion_areas);

    result.num_contact_pairs = contact_pairs.size();

    RCLCPP_INFO(logger_, "Phase 2: %zu contact pair(s) found", result.num_contact_pairs);

    if (contact_pairs.empty()) {
      result.success = true;
      result.error_message = "No valid contact pairs found";
      RCLCPP_ERROR(logger_, "%s - check gripper opening range, exclusion zones, "
        "and surface normal requirements", result.error_message.c_str());
        return result;
    }

    angle_finding::GraspOrientationFinder finder(
      primary_shape_, gripper_,
      exclusion_constraint_, kissing_constraint_,
      config_.orientation);

    finder.set_fcl_checker(fcl_checker_);

    auto candidates = finder.find_valid_grasps(contact_pairs, primary_topology_);
    result.num_candidates = candidates.size();

    RCLCPP_INFO(logger_, "Phase 3: %zu valid candidate(s)", result.num_candidates);

    if (candidates.empty()) {
      result.success = true;
      result.error_message = "No collision-free grasp orientations found";
      RCLCPP_WARN(logger_, "%s - check collision tolerances and exclusion zones",
        result.error_message.c_str());
      return result;
    }

    result.grasps.reserve(candidates.size());
    for (const auto & candidate : candidates) {
      result.grasps.push_back(angle_finding::to_grasp(candidate));
    }

    // stop_on_first_valid produces a sparse candidate set — sort is less meaningful in that case
    sort_by_quality(result.grasps);

    result.success = true;

    RCLCPP_INFO(logger_, "GraspFinder complete: %zu grasp(s), "
      "best_quality=%.3f, worst_quality=%.3f",
      result.grasps.size(),
      result.grasps.front().quality_score,
      result.grasps.back().quality_score);

    // Log top 5 grasps
    for (size_t i = 0; i < std::min(result.grasps.size(), size_t(5)); i++) {
      const auto & g = result.grasps[i];
      RCLCPP_DEBUG(logger_, "  [%zu] quality=%.3f surfaces=[%d,%d] "
        "opening=%.4f m TCP=%s",
        i, g.quality_score, g.surface_id_1, g.surface_id_2,
        g.gripper_opening, vec_to_string(g.tcp_position).c_str());
    }

    // Collision stats summary
    auto stats = kissing_constraint_->get_collision_stats();
    RCLCPP_INFO(logger_, "Collision stats: %zu checks, %zu FCL rejections",
      stats.total_checks, stats.fcl_rejections);

    return result;
  } catch (const std::exception & e) {
    result.success = false;
    result.error_message = std::string("Find failed: ") + e.what();
    RCLCPP_ERROR(logger_, "%s", result.error_message.c_str());
    return result;
  } catch (...) {
    result.success = false;
    result.error_message = "Find failed: unknown exception";
    RCLCPP_ERROR(logger_, "%s", result.error_message.c_str());
    return result;
  }
}

std::vector<Grasp> GraspFinder::find_top(size_t n)
{
  auto result = find();

  if (!result.success || result.grasps.empty()) {
    return {};
  }

  if (result.grasps.size() <= n) {
    return result.grasps;
  }

  return std::vector<Grasp>(result.grasps.begin(), result.grasps.begin() + n);
}

std::optional<Grasp> GraspFinder::find_best()
{
  auto result = find();

  if (!result.success || result.grasps.empty()) {
    return std::nullopt;
  }

  return result.grasps.front();
}

std::vector<int> GraspFinder::compute_valid_surface_ids(
  const std::vector<int> & banned_ids) const
{
  // Use unordered_set for O(1) lookup instead of O(n) std::find
  const std::unordered_set<int> banned_set(banned_ids.begin(), banned_ids.end());

  const size_t n = primary_topology_.num_surfaces();
  std::vector<int> valid_ids;
  valid_ids.reserve(n);

  for (size_t i = 0; i < n; ++i) {
    int id = static_cast<int>(i);
    if (banned_set.find(id) == banned_set.end()) {
      valid_ids.push_back(id);
    }
  }

  return valid_ids;
}

std::vector<core::SampleArea> GraspFinder::merge_sample_areas() const
{
  auto exclusion_areas = exclusion_constraint_->get_sample_areas();
  auto kissing_areas = kissing_constraint_->get_sample_areas();

  std::vector<core::SampleArea> merged;
  merged.reserve(exclusion_areas.size() + kissing_areas.size());
  merged.insert(merged.end(), exclusion_areas.begin(), exclusion_areas.end());
  merged.insert(merged.end(), kissing_areas.begin(), kissing_areas.end());

  return merged;
}

}  // namespace core
}  // namespace hold_and_weld_gripper_sampler
