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

#include <algorithm>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

namespace hold_and_weld_gripper_sampler
{
namespace core
{
static const rclcpp::Logger logger_ = rclcpp::get_logger("gripper_sampler");


// Helper to format Eigen vectors for logging
static std::string vec_to_string(const Eigen::Vector3d & v)
{
  std::ostringstream oss;
  oss << "(" << std::fixed << std::setprecision(4) << v.x() << ", " << v.y() << ", " << v.z() <<
    ")";
  return oss.str();
}

GraspFinder::GraspFinder(
  const TopoDS_Shape & primary_shape,
  const geometry::Topology & primary_topology,
  const io::ParsedGripper & gripper,
  const std::vector<TopoDS_Shape> & secondary_shapes,
  const std::optional<std::vector<constraints::exclusion_circle>> & exclusion_circles,
  const std::optional<std::vector<constraints::exclusion_polygon>> & exclusion_polygons,
  const std::optional<std::vector<constraints::exclusion_line>> & exclusion_lines,
  const GraspFinderConfig & config)
: mapper_(std::make_shared<geometry::GeometryMapper>()),
  primary_shape_(primary_shape),
  primary_topology_(primary_topology),
  gripper_(gripper),
  secondary_shapes_(secondary_shapes),
  config_(config),
  logger_(rclcpp::get_logger("gripper_sampler"))
{
  if (exclusion_circles.has_value()) {
    exclusion_circles_ = exclusion_circles.value();
  }
  if (exclusion_polygons.has_value()) {
    exclusion_polygons_ = exclusion_polygons.value();
  }
  if (exclusion_lines.has_value()) {
    exclusion_lines_ = exclusion_lines.value();
  }

  RCLCPP_INFO(logger_, "Input summary:");
  RCLCPP_INFO(logger_, "  Primary shape: %s", primary_shape_.IsNull() ? "NULL" : "loaded");
  RCLCPP_INFO(logger_, "  Topology surfaces: %zu", primary_topology_.num_surfaces());
  RCLCPP_INFO(logger_, "  Topology edges: %zu", primary_topology_.num_edges());
  RCLCPP_INFO(logger_, "  Topology corners: %zu", primary_topology_.num_corners());
  RCLCPP_INFO(logger_, "  Secondary shapes: %zu", secondary_shapes_.size());
  RCLCPP_INFO(logger_, "Exclusion zones:");
  RCLCPP_INFO(logger_, "  Circles: %zu", exclusion_circles_.size());
  RCLCPP_INFO(logger_, "  Polygons: %zu", exclusion_polygons_.size());
  RCLCPP_INFO(logger_, "  Lines: %zu", exclusion_lines_.size());
  RCLCPP_INFO(logger_, "Configuration:");
  RCLCPP_INFO(logger_, "  Gripper opening: [%.4f, %.4f] m",
    config_.sampling.min_gripper_opening, config_.sampling.max_gripper_opening);
  RCLCPP_INFO(logger_, "  Sample density: %.4f m", config_.sampling.sample_density);
  RCLCPP_INFO(logger_, "  Kissing threshold: %.1f%%", config_.kissing_contact_threshold * 100.0);
  RCLCPP_INFO(logger_, "  Use FCL: %s", config_.use_fcl ? "YES" : "NO (OCCT fallback)");
}

GraspFinder::GraspFinder(
  std::shared_ptr<const geometry::GeometryMapper> mapper,
  const TopoDS_Shape & primary_shape,
  const geometry::Topology & primary_topology,
  const io::ParsedGripper & gripper,
  const std::vector<TopoDS_Shape> & secondary_shapes,
  const std::optional<std::vector<constraints::exclusion_circle>> & exclusion_circles,
  const std::optional<std::vector<constraints::exclusion_polygon>> & exclusion_polygons,
  const std::optional<std::vector<constraints::exclusion_line>> & exclusion_lines,
  const GraspFinderConfig & config)
: mapper_(mapper),
  primary_shape_(primary_shape),
  primary_topology_(primary_topology),
  gripper_(gripper),
  secondary_shapes_(secondary_shapes),
  config_(config),
  logger_(rclcpp::get_logger("gripper_sampler"))
{
  if (!mapper_) {
    mapper_ = std::make_shared<geometry::GeometryMapper>();
  }

  // Copy exclusion zone definitions
  if (exclusion_circles.has_value()) {
    exclusion_circles_ = exclusion_circles.value();
  }
  if (exclusion_polygons.has_value()) {
    exclusion_polygons_ = exclusion_polygons.value();
  }
  if (exclusion_lines.has_value()) {
    exclusion_lines_ = exclusion_lines.value();
  }

  RCLCPP_DEBUG(logger_, "GraspFinder constructed with shared mapper");
}

std::string GraspFinder::initialize()
{
  if (initialized_) {
    RCLCPP_DEBUG(logger_, "GraspFinder already initialized, skipping");
    return "";
  }

  try {
    RCLCPP_INFO(logger_, "STEP 1: Creating constraint objects...");

    // Convert vectors to optionals for ExclusionZoneConstraint constructor
    std::optional<std::vector<constraints::exclusion_circle>> circles_opt;
    std::optional<std::vector<constraints::exclusion_polygon>> polygons_opt;
    std::optional<std::vector<constraints::exclusion_line>> lines_opt;

    if (!exclusion_circles_.empty()) {
      circles_opt = exclusion_circles_;
    }
    if (!exclusion_polygons_.empty()) {
      polygons_opt = exclusion_polygons_;
    }
    if (!exclusion_lines_.empty()) {
      lines_opt = exclusion_lines_;
    }

    exclusion_constraint_ = std::make_shared<constraints::ExclusionZoneConstraint>(
      mapper_,
      gripper_,
      circles_opt,
      polygons_opt,
      lines_opt,
      config_.mesh_linear_deflection,
      config_.mesh_angular_deflection
    );
    RCLCPP_INFO(logger_, "ExclusionZoneConstraint created");

    RCLCPP_INFO(logger_, "Creating KissingSurfaceConstraint");
    RCLCPP_INFO(logger_, "  Gripper min_opening: %.4f m", gripper_.min_opening);
    RCLCPP_INFO(logger_, "  Gripper max_opening: %.4f m", gripper_.max_opening);

    kissing_constraint_ = std::make_shared<constraints::KissingSurfaceConstraint>(
      mapper_,
      gripper_,
      secondary_shapes_,
      config_.kissing_contact_threshold,
      config_.collision_tolerance
    );
    RCLCPP_INFO(logger_, "KissingSurfaceConstraint created");

    RCLCPP_INFO(logger_, "");
    RCLCPP_INFO(logger_, "STEP 2: Analyzing constraints...");

    RCLCPP_INFO(logger_, "Analyzing exclusion zones...");
    exclusion_constraint_->analyze_constraints(primary_shape_, primary_topology_);
    RCLCPP_INFO(logger_, "Exclusion zone analysis complete");
    RCLCPP_INFO(logger_, "      Sample areas: %zu",
        exclusion_constraint_->get_sample_areas().size());
    RCLCPP_INFO(logger_, "      Collision volumes: %zu",
        exclusion_constraint_->get_collision_volumes().size());

    RCLCPP_INFO(logger_, "Analyzing kissing surfaces...");
    kissing_constraint_->analyze_constraints(primary_topology_);
    RCLCPP_INFO(logger_, "Kissing surface analysis complete");
    RCLCPP_INFO(logger_, "      Banned surfaces: %zu",
        kissing_constraint_->get_banned_surface_ids().size());
    RCLCPP_INFO(logger_, "      Partial exclusions: %zu",
        kissing_constraint_->get_sample_areas().size());

    RCLCPP_INFO(logger_, "");
    RCLCPP_INFO(logger_, "STEP 3: Building collision checker...");

    if (config_.use_fcl) {
      RCLCPP_INFO(logger_, "Initializing FCL collision checker...");

      fcl_checker_ = std::make_shared<geometry::FCLCollisionChecker>(
        gripper_,
        primary_shape_,
        config_.triangulation_deflection
      );

      RCLCPP_INFO(logger_, "FCL checker created");

      // Add exclusion volumes to FCL
      const auto & exclusion_volumes = exclusion_constraint_->get_collision_volumes();
      if (!exclusion_volumes.empty()) {
        RCLCPP_INFO(logger_, "  Adding %zu exclusion volumes to FCL...", exclusion_volumes.size());
        fcl_checker_->add_exclusion_volumes(exclusion_volumes);
        RCLCPP_INFO(logger_, "Exclusion volumes added");
      } else {
        RCLCPP_INFO(logger_, "  No exclusion volumes to add");
      }

      // Add secondary shapes to FCL
      const auto & secondaries = kissing_constraint_->get_secondary_shapes();
      if (!secondaries.empty()) {
        RCLCPP_INFO(logger_, "  Adding %zu secondary shapes to FCL...", secondaries.size());
        fcl_checker_->add_secondary_shapes(secondaries);
        RCLCPP_INFO(logger_, "Secondary shapes added");
      } else {
        RCLCPP_INFO(logger_, "  No secondary shapes to add");
      }

      // Add ground plane to FCL if enabled
      if (config_.enable_ground_plane_check && config_.use_fcl_for_ground_plane) {
        RCLCPP_INFO(logger_, "  Adding ground plane to FCL...");
        fcl_checker_->add_ground_plane(config_.ground_z, 100.0, 0.1, 0.0, 0.0);
        RCLCPP_INFO(logger_, "Ground plane added to FCL (z=%.3f m)", config_.ground_z);
      }

      if (!fcl_checker_->is_valid()) {
        RCLCPP_WARN(logger_, "   FCL checker initialization failed, falling back to OCCT");
        fcl_checker_.reset();
      } else {
        RCLCPP_INFO(logger_, "FCL checker validated and ready");
      }
    } else {
      RCLCPP_INFO(logger_, "FCL disabled - using OCCT for collision detection (slower)");
    }
    RCLCPP_INFO(logger_, "");
    RCLCPP_INFO(logger_, "STEP 4: Wiring components...");

    if (fcl_checker_) {
      RCLCPP_INFO(logger_, "Wiring FCL checker to constraints...");
      exclusion_constraint_->set_fcl_checker(fcl_checker_);
      kissing_constraint_->set_fcl_checker(fcl_checker_);
      RCLCPP_INFO(logger_, "FCL wired to ExclusionZoneConstraint");
      RCLCPP_INFO(logger_, "FCL wired to KissingSurfaceConstraint");
    } else {
      RCLCPP_INFO(logger_, "No FCL checker - constraints will use OCCT directly");
    }

    initialized_ = true;

    return "";
  } catch (const std::exception & e) {
    std::string error = std::string("Initialization failed: ") + e.what();
    RCLCPP_ERROR(logger_, "%s", error.c_str());
    return error;
  } catch (...) {
    std::string error = "Initialization failed: unknown exception";
    RCLCPP_ERROR(logger_, "%s", error.c_str());
    return error;
  }
}

GraspFinderResult GraspFinder::find()
{
  GraspFinderResult result;
  std::string init_error = initialize();
  if (!init_error.empty()) {
    result.success = false;
    result.error_message = init_error;
    RCLCPP_ERROR(logger_, "Initialization failed: %s", init_error.c_str());
    return result;
  }

  try {
    RCLCPP_INFO(logger_, "");
    RCLCPP_INFO(logger_, "PHASE 1: Computing valid surfaces...");

    auto banned_ids = kissing_constraint_->get_banned_surface_ids();
    auto valid_ids = compute_valid_surface_ids(banned_ids);
    auto exclusion_areas = merge_sample_areas();

    result.num_banned_surfaces = banned_ids.size();
    result.num_valid_surfaces = valid_ids.size();
    result.num_exclusion_areas = exclusion_areas.size();

    RCLCPP_INFO(logger_, "Surface analysis results:");
    RCLCPP_INFO(logger_, "  Total surfaces in topology: %zu", primary_topology_.num_surfaces());
    RCLCPP_INFO(logger_, "  Banned surfaces (kissing): %zu", result.num_banned_surfaces);
    RCLCPP_INFO(logger_, "  Valid surfaces for sampling: %zu", result.num_valid_surfaces);
    RCLCPP_INFO(logger_, "  Exclusion areas (wires): %zu", result.num_exclusion_areas);

    if (!banned_ids.empty()) {
      std::string banned_str = "  Banned IDs: [";
      for (size_t i = 0; i < banned_ids.size(); i++) {
        banned_str += std::to_string(banned_ids[i]);
        if (i < banned_ids.size() - 1) {banned_str += ", ";}
      }
      banned_str += "]";
      RCLCPP_INFO(logger_, "%s", banned_str.c_str());
    }

    if (!valid_ids.empty()) {
      std::string valid_str = "  Valid IDs: [";
      for (size_t i = 0; i < valid_ids.size() && i < 20; i++) {
        valid_str += std::to_string(valid_ids[i]);
        if (i < valid_ids.size() - 1 && i < 19) {valid_str += ", ";}
      }
      if (valid_ids.size() > 20) {valid_str += ", ...";}
      valid_str += "]";
      RCLCPP_INFO(logger_, "%s", valid_str.c_str());
    }

    if (valid_ids.empty()) {
      result.success = true;  // Not an error, just no valid surfaces
      result.error_message = "No valid surfaces for grasping (all banned or excluded)";
      RCLCPP_WARN(logger_, " %s", result.error_message.c_str());
      return result;
    }

    RCLCPP_INFO(logger_, "");
    RCLCPP_INFO(logger_, "PHASE 2: Sampling contact points...");
    RCLCPP_INFO(logger_, "Sampling config:");
    RCLCPP_INFO(logger_, "  Sample density: %.4f m", config_.sampling.sample_density);
    RCLCPP_INFO(logger_, "  Gripper opening: [%.4f, %.4f] m",
      config_.sampling.min_gripper_opening, config_.sampling.max_gripper_opening);
    RCLCPP_INFO(logger_, "  Normal angle range: [%.1f°, %.1f°]",
      config_.sampling.min_angle_deg, config_.sampling.max_angle_deg);

    sampling::ContactPointSampler sampler(config_.sampling);
    auto contact_pairs = sampler.generate_contact_pairs(
      primary_topology_,
      valid_ids,
      exclusion_areas
    );

    result.num_contact_pairs = contact_pairs.size();
    RCLCPP_INFO(logger_, "");
    RCLCPP_INFO(logger_, "Contact pair generation result: %zu pairs", result.num_contact_pairs);

    if (contact_pairs.empty()) {
      result.success = true;
      result.error_message = "No valid contact pairs found";
      RCLCPP_WARN(logger_, " %s", result.error_message.c_str());
      RCLCPP_WARN(logger_, "Possible causes:");
      RCLCPP_WARN(logger_, "  - No opposing surfaces within gripper opening range");
      RCLCPP_WARN(logger_, "  - All sample points in exclusion zones");
      RCLCPP_WARN(logger_, "  - Surface normals not antiparallel");
      return result;
    }


    RCLCPP_INFO(logger_, "PHASE 3: Finding valid grasp orientations...");
    RCLCPP_INFO(logger_, "Orientation config:");
    RCLCPP_INFO(logger_, "  Finger length: %.4f m", config_.orientation.finger_length);
    RCLCPP_INFO(logger_, "  Finger radius: %.4f m", config_.orientation.finger_radius);
    RCLCPP_INFO(logger_, "  Max edge candidates: %zu", config_.orientation.max_edge_candidates);
    RCLCPP_INFO(logger_, "  Dual seed dedup tolerance: %.1f°",
      config_.orientation.dual_seed_dedup_tolerance_deg);
    RCLCPP_INFO(logger_, "  Collision tolerance: %.4f m", config_.orientation.collision_tolerance);

    angle_finding::GraspOrientationFinder finder(
      primary_shape_,
      gripper_,
      exclusion_constraint_,
      kissing_constraint_,
      config_.orientation
    );

    // Wire FCL to orientation finder
    if (fcl_checker_) {
      finder.set_fcl_checker(fcl_checker_);
      RCLCPP_INFO(logger_, "FCL checker wired to orientation finder");
    }

    RCLCPP_INFO(logger_, "");
    auto candidates = finder.find_valid_grasps(contact_pairs, primary_topology_);

    result.num_candidates = candidates.size();
    RCLCPP_INFO(logger_, "");
    RCLCPP_INFO(logger_, "Orientation finding result: %zu valid candidates", result.num_candidates);

    if (candidates.empty()) {
      result.success = true;
      result.error_message = "No collision-free grasp orientations found";
      RCLCPP_WARN(logger_, " %s", result.error_message.c_str());
      RCLCPP_WARN(logger_, "Possible causes:");
      RCLCPP_WARN(logger_, "  - All orientations collide with primary shape");
      RCLCPP_WARN(logger_, "  - All orientations collide with exclusion zones");
      RCLCPP_WARN(logger_, "  - All orientations collide with secondary shapes (table/fixture)");
      return result;
    }

    RCLCPP_INFO(logger_, "PHASE 4: Converting and sorting results...");

    result.grasps.reserve(candidates.size());
    for (const auto & candidate : candidates) {
      result.grasps.push_back(to_grasp(candidate));
    }

    sort_by_quality(result.grasps);

    result.success = true;
    RCLCPP_INFO(logger_, "Results:");
    RCLCPP_INFO(logger_, "  Total grasps found: %zu", result.grasps.size());
    if (!result.grasps.empty()) {
      RCLCPP_INFO(logger_, "  Best quality: %.3f", result.grasps.front().quality_score);
      RCLCPP_INFO(logger_, "  Worst quality: %.3f", result.grasps.back().quality_score);

      // Log top 5 grasps
      RCLCPP_INFO(logger_, "Top grasps (up to 5):");
      for (size_t i = 0; i < std::min(result.grasps.size(), size_t(5)); i++) {
        const auto & g = result.grasps[i];
        RCLCPP_INFO(logger_, "  [%zu] quality=%.3f, surfaces=[%d,%d], opening=%.4fm, TCP=%s",
          i, g.quality_score, g.surface_id_1, g.surface_id_2, g.gripper_opening,
          vec_to_string(g.tcp_position).c_str());
      }
    }

    // Log collision rejection statistics
    if (kissing_constraint_) {
      auto collision_stats = kissing_constraint_->get_collision_stats();
      RCLCPP_INFO(logger_, "");
      RCLCPP_INFO(logger_, "Collision Check Statistics:");
      RCLCPP_INFO(logger_, "  Total collision checks: %zu", collision_stats.total_checks);
      if (collision_stats.total_checks > 0) {
        RCLCPP_INFO(logger_, "  Ground plane (simple check): %zu (%.1f%%)",
          collision_stats.ground_plane_simple_rejections,
          100.0 * collision_stats.ground_plane_simple_rejections / collision_stats.total_checks);
        RCLCPP_INFO(logger_, "  FCL secondary/ground: %zu (%.1f%%)",
          collision_stats.fcl_secondary_rejections,
          100.0 * collision_stats.fcl_secondary_rejections / collision_stats.total_checks);
        RCLCPP_INFO(logger_, "  OCCT secondary: %zu (%.1f%%)",
          collision_stats.occt_secondary_rejections,
          100.0 * collision_stats.occt_secondary_rejections / collision_stats.total_checks);
        if (collision_stats.mesh_failures > 0) {
          RCLCPP_WARN(logger_, "  Mesh failures: %zu", collision_stats.mesh_failures);
        }
        if (collision_stats.occt_failures > 0) {
          RCLCPP_WARN(logger_, "  OCCT failures: %zu", collision_stats.occt_failures);
        }
      }
    }

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

std::vector<int> GraspFinder::compute_valid_surface_ids(const std::vector<int> & banned_ids) const
{
  const size_t n = primary_topology_.num_surfaces();
  std::vector<int> valid_ids;
  valid_ids.reserve(n);

  for (size_t i = 0; i < n; ++i) {
    int id = static_cast<int>(i);
    bool is_banned = std::find(banned_ids.begin(), banned_ids.end(), id) != banned_ids.end();
    if (!is_banned) {
      valid_ids.push_back(id);
    }
  }

  return valid_ids;
}

std::vector<core::SampleArea> GraspFinder::merge_sample_areas() const
{
  std::vector<core::SampleArea> merged;

  // Get exclusion areas from both constraints
  auto exclusion_areas = exclusion_constraint_->get_sample_areas();
  auto kissing_areas = kissing_constraint_->get_sample_areas();

  merged.reserve(exclusion_areas.size() + kissing_areas.size());
  merged.insert(merged.end(), exclusion_areas.begin(), exclusion_areas.end());
  merged.insert(merged.end(), kissing_areas.begin(), kissing_areas.end());

  return merged;
}

void GraspFinder::sort_by_quality(std::vector<Grasp> & grasps)
{
  std::sort(grasps.begin(), grasps.end(),
    [](const Grasp & a, const Grasp & b) {
      return a.quality_score > b.quality_score;
    });
}

}  // namespace core
}  // namespace hold_and_weld_gripper_sampler
