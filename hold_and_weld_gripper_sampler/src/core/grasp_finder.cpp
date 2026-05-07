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

#include <fcl/fcl.h>

#include <algorithm>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "hold_and_weld_gripper_sampler/core/grasp_finder.hpp"
#include "hold_and_weld_gripper_sampler/geometry/shape_refiner.hpp"

namespace hold_and_weld_gripper_sampler
{
namespace core
{

GraspFinder::GraspFinder(
  std::shared_ptr<const geometry::GeometryMapper> mapper,
  const TopoDS_Shape & primary_shape,
  const geometry::Topology & primary_topology,
  const ParsedGripper & gripper,
  const std::vector<TopoDS_Shape> & secondary_shapes,
  const std::optional<std::vector<constraints::exclusion_circle>> & exclusion_circles,
  const std::optional<std::vector<constraints::exclusion_polygon>> & exclusion_polygons,
  const std::optional<std::vector<constraints::exclusion_line>> & exclusion_lines,
  const GraspFinderConfig & config,
  const TopoDS_Shape & fcl_primary_shape)
: primary_shape_(primary_shape),
  fcl_primary_shape_(fcl_primary_shape.IsNull() ? primary_shape : fcl_primary_shape),
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
  std::call_once(init_flag_, [this]() {
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

      // Merge ground shapes into the secondary list for kissing surface contact analysis.
      // Ground shapes must be included here so the bottom face of the workpiece is
      // correctly banned. They are kept separate for FCL (routed to ground_halfspace_).
        std::vector<TopoDS_Shape> all_contact_shapes = secondary_shapes_;
        for (const auto & gs : config_.ground_shapes) {
          all_contact_shapes.push_back(gs);
        }

        kissing_constraint_ = std::make_shared<constraints::KissingSurfaceConstraint>(
        mapper_, gripper_, all_contact_shapes,
        config_.kissing_contact_threshold, config_.collision_tolerance,
        config_.kissing_contact_distance_threshold, config_.mesh_linear_deflection);

        exclusion_constraint_->analyze_constraints(primary_shape_, primary_topology_);
        kissing_constraint_->analyze_constraints(primary_topology_);

        if (!config_.use_fcl) {
          RCLCPP_ERROR(logger_, "GraspFinder::initialize(): FCL is required but use_fcl=false. "
          "Set use_fcl: true in config. Subsequent find() calls will also fail.");
          init_error_ = "FCL is required for grasp sampling";
          return;
        }

      // Only pass the non-ground secondary shapes to add_secondary_shapes() so they
      // land in secondary_bvhs_ and show up correctly in FCL stats.
      // Ground shapes go through add_ground_plane_z() -> ground_halfspace_ separately.
        fcl_checker_ = std::make_shared<geometry::FCLCollisionChecker>(
        gripper_,
        fcl_primary_shape_,
        exclusion_constraint_->get_collision_volumes(),
        secondary_shapes_,
        false, 0.0,
        config_.triangulation_deflection);

        if (config_.use_fcl_for_ground_plane &&
        (!config_.ground_shapes.empty() || config_.enable_ground_plane_check))
        {
          fcl_checker_->add_ground_plane(Eigen::Vector3d(0.0, 0.0, 1.0), config_.ground_bottom_z);
          RCLCPP_DEBUG(logger_, "Ground halfspace added to FCL (z=%.4f)", config_.ground_bottom_z);
        }

        if (!fcl_checker_->is_valid()) {
          init_error_ = "FCL checker initialization failed";
          return;
        }

        exclusion_constraint_->set_fcl_checker(fcl_checker_);
        kissing_constraint_->set_fcl_checker(fcl_checker_);

        init_error_ = "";
      } catch (const Standard_Failure & e) {
        RCLCPP_ERROR(logger_, "OCCT Failure during GraspFinder init: %s", e.GetMessageString());
        init_error_ = std::string("OCCT Failure: ") + e.GetMessageString();
      } catch (const std::exception & e) {
        RCLCPP_ERROR(logger_, "Initialization failed: %s", e.what());
        init_error_ = std::string("Initialization failed: ") + e.what();
      }
  });
  return init_error_;
}

GraspFinderResult GraspFinder::find()
{
  // Return cached result if the pipeline already ran successfully.
  if (cached_result_.has_value()) {
    return *cached_result_;
  }
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

    RCLCPP_INFO(logger_, "Phase 1: %zu/%zu surfaces valid", result.num_valid_surfaces,
          primary_topology_.num_surfaces());

    if (valid_ids.empty()) {
      result.success = false;
      result.error_message = "No valid surfaces for grasping";
      return result;
    }

    sampling::ContactPointSampler sampler(config_.sampling);
    auto contact_pairs = sampler.generate_contact_pairs(primary_topology_, valid_ids,
          exclusion_areas);
    result.num_contact_pairs = contact_pairs.size();

    RCLCPP_INFO(logger_, "Phase 2: %zu contact pair(s) sampled", result.num_contact_pairs);

    if (contact_pairs.empty()) {
      result.success = false;
      result.error_message = "No valid contact pairs sampled";
      return result;
    }

    angle_finding::GraspOrientationFinder finder(
      primary_shape_, gripper_,
      exclusion_constraint_, kissing_constraint_,
      config_.orientation);

    finder.set_fcl_checker(fcl_checker_);
    if (fcl_checker_) {
      finder.set_embree_checker(fcl_checker_->get_embree_primary());
    }

    auto candidates = finder.find_valid_grasps(contact_pairs, primary_topology_);
    result.num_candidates = candidates.size();

    fcl_checker_->log_collision_stats();

    RCLCPP_INFO(logger_, "Phase 3: %zu collision-free candidate(s)", result.num_candidates);

    if (candidates.empty()) {
      result.success = false;
      result.error_message = "All candidates rejected by collision/exclusion constraints";
      return result;
    }

    result.grasps.reserve(candidates.size());
    for (const auto & candidate : candidates) {
      result.grasps.push_back(angle_finding::to_grasp(candidate));
    }

    std::stable_sort(
      result.grasps.begin(), result.grasps.end(),
      [](const Grasp & a, const Grasp & b) {
        return a.quality_score > b.quality_score;
      });

    result.success = true;

    // Cache so find_top() / find_best() don't re-run the pipeline.
    cached_result_ = result;

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
  if (!result.success) {
    RCLCPP_ERROR(logger_, "find_top() failed: %s", result.error_message.c_str());
    return {};
  }
  if (result.grasps.empty()) {return {};}
  if (result.grasps.size() <= n) {return result.grasps;}
  return std::vector<Grasp>(result.grasps.begin(), result.grasps.begin() + n);
}

std::optional<Grasp> GraspFinder::find_best()
{
  auto result = find();
  if (!result.success) {
    RCLCPP_ERROR(logger_, "find_best() failed: %s", result.error_message.c_str());
    return std::nullopt;
  }
  if (result.grasps.empty()) {return std::nullopt;}
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

}  // namespace core
}  // namespace hold_and_weld_gripper_sampler
