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
    RCLCPP_WARN(logger_, "No mapper provided; generating internal workpiece mapper.");
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

  RCLCPP_INFO(logger_,
        "GraspFinder: %zu surfaces, %zu secondaries, %zu circles, %zu polygons, %zu lines",
    primary_topology_.num_surfaces(), secondary_shapes_.size(),
    exclusion_circles_.size(), exclusion_polygons_.size(), exclusion_lines_.size());
}

// TODO(@silanus23): Constraint parameters should ideally be handled via parameter subscribers.
std::string GraspFinder::initialize()
{
  if (initialized_) {return "";}

  try {
    RCLCPP_INFO(logger_, "Initializing GraspFinder Constraints");

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

    // Analyze geometry
    exclusion_constraint_->analyze_constraints(primary_shape_, primary_topology_);
    kissing_constraint_->analyze_constraints(primary_topology_);

    if (!config_.use_fcl) {return "FCL is required for grasp sampling";}

    fcl_checker_ = std::make_shared<geometry::FCLCollisionChecker>(
      gripper_,
      primary_shape_,
      exclusion_constraint_->get_collision_volumes(),
      kissing_constraint_->get_secondary_shapes(),
      false, 0.0,
      config_.triangulation_deflection);

    if (config_.enable_ground_plane_check && config_.use_fcl_for_ground_plane) {
      fcl_checker_->add_ground_plane(
        config_.ground_bottom_z,
        std::max(config_.ground_size_x, config_.ground_size_y),
        0.1,
        config_.ground_center_x,
        config_.ground_center_y);
    }

    if (!fcl_checker_->is_valid()) {return "FCL checker initialization failed";}

    exclusion_constraint_->set_fcl_checker(fcl_checker_);
    kissing_constraint_->set_fcl_checker(fcl_checker_);

    initialized_ = true;
    return "";
  } catch (Standard_Failure & e) {
    RCLCPP_ERROR(logger_, "OCCT Failure during GraspFinder init: %s", e.GetMessageString());
    return std::string("OCCT Failure: ") + e.GetMessageString();
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger_, "Initialization failed: %s", e.what());
    return std::string("Initialization failed: ") + e.what();
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
    // Phase 1: Topology Filtering
    auto banned_ids = kissing_constraint_->get_banned_surface_ids();
    auto valid_ids = compute_valid_surface_ids(banned_ids);
    auto exclusion_areas = merge_sample_areas();

    result.num_banned_surfaces = banned_ids.size();
    result.num_valid_surfaces = valid_ids.size();
    result.num_exclusion_areas = exclusion_areas.size();

    RCLCPP_INFO(logger_, "Phase 1: %zu/%zu surfaces valid", result.num_valid_surfaces,
          primary_topology_.num_surfaces());

    if (valid_ids.empty()) {
      result.success = true;
      result.error_message = "No valid surfaces for grasping";
      return result;
    }

    // Phase 2: Contact Point Sampling
    sampling::ContactPointSampler sampler(config_.sampling);
    auto contact_pairs = sampler.generate_contact_pairs(primary_topology_, valid_ids,
          exclusion_areas);
    result.num_contact_pairs = contact_pairs.size();

    RCLCPP_INFO(logger_, "Phase 2: %zu contact pair(s) sampled", result.num_contact_pairs);

    if (contact_pairs.empty()) {
      result.success = true;
      result.error_message = "No valid contact pairs sampled";
      return result;
    }

    // Phase 3: Orientation & Collision Finding
    angle_finding::GraspOrientationFinder finder(
      primary_shape_, gripper_,
      exclusion_constraint_, kissing_constraint_,
      config_.orientation);

    finder.set_fcl_checker(fcl_checker_);

    auto candidates = finder.find_valid_grasps(contact_pairs, primary_topology_);
    result.num_candidates = candidates.size();

    RCLCPP_INFO(logger_, "Phase 3: %zu collision-free candidate(s)", result.num_candidates);

    if (candidates.empty()) {
      result.success = true;
      result.error_message = "All candidates rejected by collision/exclusion constraints";
      return result;
    }

    result.grasps.reserve(candidates.size());
    for (const auto & candidate : candidates) {
      result.grasps.push_back(angle_finding::to_grasp(candidate));
    }

    sort_by_quality(result.grasps);
    sort_by_diversity(result.grasps);

    result.success = true;

    auto stats = kissing_constraint_->get_collision_stats();
    RCLCPP_INFO(logger_, "GraspFinder Result: %zu grasps. FCL Stats: %zu checks, %zu rejections",
      result.grasps.size(), stats.total_checks, stats.fcl_rejections);

    return result;
  } catch (const std::exception & e) {
    result.success = false;
    result.error_message = std::string("Grasp Search Failed: ") + e.what();
    RCLCPP_ERROR(logger_, "%s", result.error_message.c_str());
    return result;
  }
}

std::vector<Grasp> GraspFinder::find_top(size_t n)
{
  auto result = find();
  if (!result.success || result.grasps.empty()) {return {};}
  if (result.grasps.size() <= n) {return result.grasps;}
  return std::vector<Grasp>(result.grasps.begin(), result.grasps.begin() + n);
}

std::optional<Grasp> GraspFinder::find_best()
{
  auto result = find();
  if (!result.success || result.grasps.empty()) {return std::nullopt;}
  return result.grasps.front();
}

std::vector<int> GraspFinder::compute_valid_surface_ids(const std::vector<int> & banned_ids) const
{
  const std::unordered_set<int> banned_set(banned_ids.begin(), banned_ids.end());
  std::vector<int> valid_ids;
  for (size_t i = 0; i < primary_topology_.num_surfaces(); ++i) {
    int id = static_cast<int>(i);
    if (banned_set.find(id) == banned_set.end()) {valid_ids.push_back(id);}
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

} // namespace core
} // namespace hold_and_weld_gripper_sampler
