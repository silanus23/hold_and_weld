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
#include <Standard_Failure.hxx>

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

// TODO(silanus23): Constraint parameters should ideally be handled via parameter subscribers.
std::string GraspFinder::initialize()
{
  std::call_once(init_flag_, [this]() {
      try {
        RCLCPP_INFO(logger_, "Initializing GraspFinder Constraints");

        exclusion_constraint_ = std::make_shared<constraints::ExclusionZoneConstraint>(
        mapper_, gripper_, exclusion_circles_, exclusion_polygons_, exclusion_lines_,
        config_.mesh_linear_deflection, config_.mesh_angular_deflection,
        config_.exclusion_sample_density);

        kissing_constraint_ = std::make_shared<constraints::KissingSurfaceConstraint>(
        mapper_, gripper_, secondary_shapes_,
        config_.kissing_contact_threshold, config_.collision_tolerance,
        config_.kissing_contact_distance_threshold, config_.mesh_linear_deflection,
        config_.mesh_angular_deflection, config_.kissing_sample_density);

        constraints::GroundConfig ground_config;
        ground_config.surface_z = config_.ground_surface_z;
        ground_config.center_x = config_.ground_center_x;
        ground_config.center_y = config_.ground_center_y;
        ground_config.size_x = config_.ground_size_x;
        ground_config.size_y = config_.ground_size_y;
        ground_config.contact_band = config_.ground_safety_margin;
        ground_config.support_threshold = config_.kissing_contact_threshold;
        ground_config.collision_tolerance = config_.collision_tolerance;
        ground_config.sample_density = config_.kissing_sample_density;
        ground_constraint_ = std::make_shared<constraints::GroundConstraint>(ground_config);

        exclusion_constraint_->analyze_constraints(primary_shape_, primary_topology_);
        kissing_constraint_->analyze_constraints(primary_topology_);
        if (config_.ground_shapes.empty() && config_.enable_ground_plane_check) {
          RCLCPP_INFO(logger_, "No ground_plane secondary: using the implicit ground "
            "(%.1f x %.1f m at z=%.3f); set implicit_ground: false to disable",
            config_.ground_size_x, config_.ground_size_y, config_.ground_surface_z);
        }
        if (!config_.ground_shapes.empty() || config_.enable_ground_plane_check) {
          ground_constraint_->analyze_constraints(primary_topology_);
        }

        if (!config_.use_fcl) {
          RCLCPP_ERROR(logger_, "GraspFinder::initialize(): FCL is required but use_fcl=false. "
          "Set use_fcl: true in config. Subsequent find() calls will also fail.");
          init_error_ = "FCL is required for grasp sampling";
          return;
        }

        // Only the non-ground secondaries go to the constructor, so they land in
        // secondary_bvhs_ and are counted in FCL stats; the ground is added below.
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
          fcl_checker_->add_ground_plane(
            Eigen::Vector3d(0.0, 0.0, 1.0), config_.ground_surface_z,
            config_.ground_size_x, config_.ground_size_y,
            config_.ground_center_x, config_.ground_center_y);
          RCLCPP_INFO(logger_,
            "Ground added to FCL: %s, surface z=%.4f",
            fcl_checker_->has_finite_ground() ? "finite footprint" : "infinite halfspace",
            config_.ground_surface_z);
        }

        if (!fcl_checker_->is_valid()) {
          init_error_ = "FCL checker initialization failed";
          return;
        }

        exclusion_constraint_->set_fcl_checker(fcl_checker_);
        kissing_constraint_->set_fcl_checker(fcl_checker_);
        ground_constraint_->set_fcl_checker(fcl_checker_);

        if (config_.jaw_clearance.enabled) {
          jaw_clearance_check_ = std::make_shared<geometry::JawClearanceCheck>(
          config_.jaw_clearance, gripper_, config_.orientation.finger_length);
          jaw_clearance_check_->set_fcl_checker(fcl_checker_);
          RCLCPP_INFO(logger_,
          "Jaw clearance enabled: length=%.4fm, clearance margin=%.4fm",
          jaw_clearance_check_->get_length(), config_.jaw_clearance.clearance_margin);
        }

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

  result.skipped_constraints = collect_skipped_constraints();

  try {
    auto banned_ids = kissing_constraint_->get_banned_surface_ids();
    if (ground_constraint_) {
      const auto & ground_banned = ground_constraint_->get_banned_surface_ids();
      banned_ids.insert(banned_ids.end(), ground_banned.begin(), ground_banned.end());
    }
    auto valid_ids = compute_valid_surface_ids(banned_ids);
    auto exclusion_areas = merge_sample_areas();

    result.num_banned_surfaces = banned_ids.size();
    result.num_valid_surfaces = valid_ids.size();
    result.num_exclusion_areas = exclusion_areas.size();

    RCLCPP_INFO(logger_, "Surface filtering: %zu/%zu surfaces valid", result.num_valid_surfaces,
          primary_topology_.num_surfaces());

    if (valid_ids.empty()) {
      result.success = false;
      result.error_message = "No valid surfaces for grasping";
      return result;
    }

    // A contact span the fingers cannot reach is not a grasp, whatever sampling allows.
    // The tolerance keeps a part exactly max_opening wide: its sampled spans land a few
    // ulps either side of it.
    constexpr double kOpeningTolerance = 1e-6;
    sampling::SamplingConfig sampling_config = config_.sampling;
    const double reachable = gripper_.max_opening + kOpeningTolerance;
    if (gripper_.max_opening > 0.0 && sampling_config.max_gripper_opening > reachable) {
      RCLCPP_INFO(logger_, "sampling.max_gripper_opening %.4f m exceeds the gripper's "
        "max_opening %.4f m; sampling up to the gripper's",
        sampling_config.max_gripper_opening, gripper_.max_opening);
      sampling_config.max_gripper_opening = reachable;
    }
    sampling::ContactPointSampler sampler(sampling_config);
    auto contact_pairs = sampler.generate_contact_pairs(primary_topology_, valid_ids,
          exclusion_areas);
    result.num_contact_pairs = contact_pairs.size();

    RCLCPP_INFO(logger_, "Contact sampling: %zu contact pair(s) sampled", result.num_contact_pairs);

    if (contact_pairs.empty()) {
      result.success = false;
      result.error_message = "No valid contact pairs sampled";
      return result;
    }

    angle_finding::GraspOrientationFinder finder(
      primary_shape_, gripper_,
      exclusion_constraint_, kissing_constraint_,
      config_.orientation);

    finder.set_jaw_clearance_check(jaw_clearance_check_);
    finder.set_ground_constraint(ground_constraint_);
    finder.set_fcl_checker(fcl_checker_);
    if (fcl_checker_) {
      finder.set_embree_checker(fcl_checker_->get_embree_primary());
    }

    auto candidates = finder.find_valid_grasps(contact_pairs, primary_topology_);
    result.num_candidates = candidates.size();

    fcl_checker_->log_collision_stats();

    RCLCPP_INFO(logger_, "Orientation search: %zu collision-free candidate(s)",
      result.num_candidates);

    if (candidates.empty()) {
      result.success = false;
      result.error_message = "All candidates rejected by collision/exclusion constraints";
      return result;
    }

    result.grasps.reserve(candidates.size());
    for (const auto & candidate : candidates) {
      result.grasps.push_back(angle_finding::to_grasp(candidate));
    }

    sort_by_quality(result.grasps);

    result.success = true;

    cached_result_ = result;

    auto stats = kissing_constraint_->get_collision_stats();
    RCLCPP_INFO(logger_, "GraspFinder Result: %zu grasps. Secondary-obstacle checks: %zu, "
      "rejections: %zu", result.grasps.size(), stats.total_checks, stats.fcl_rejections);

    return result;
  } catch (const Standard_Failure & e) {
    result.success = false;
    result.error_message = std::string("Grasp Search Failed: OCCT error: ") +
      e.GetMessageString();
    RCLCPP_ERROR(logger_, "%s", result.error_message.c_str());
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

  if (ground_constraint_) {
    const auto & ground_areas = ground_constraint_->get_sample_areas();
    merged.insert(merged.end(), ground_areas.begin(), ground_areas.end());
  }

  return merged;
}

std::vector<std::string> GraspFinder::collect_skipped_constraints() const
{
  std::vector<std::string> skipped = exclusion_constraint_->get_skipped();

  const auto & labels = exclusion_constraint_->get_collision_volume_labels();
  for (size_t i : fcl_checker_->get_unchecked_exclusions()) {
    skipped.push_back(labels.at(i) + ": not collision-checked (BVH build failed)");
  }
  for (size_t i : fcl_checker_->get_unchecked_secondaries()) {
    skipped.push_back(
      "secondary obstacle " + std::to_string(i) + ": not collision-checked (BVH build failed)");
  }
  return skipped;
}

}  // namespace core
}  // namespace hold_and_weld_gripper_sampler
