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

/**
 * @file grasp_finder_node.cpp
 * @brief Simple executable to run GraspFinder from YAML config and output JSON
 *
 * Usage:
 *   ros2 run hold_and_weld_gripper_sampler grasp_finder_node --config <path_to_yaml>
 *   ros2 run hold_and_weld_gripper_sampler grasp_finder_node  # uses default config
 */

#include <chrono>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <BRepMesh_IncrementalMesh.hxx>

#include "hold_and_weld_gripper_sampler/core/grasp_finder.hpp"
#include "hold_and_weld_gripper_sampler/io/config_parser.hpp"
#include "hold_and_weld_gripper_sampler/io/result_writer.hpp"
#include "hold_and_weld_gripper_sampler/geometry/geometry_mapper.hpp"
#include "hold_and_weld_gripper_sampler/core/gripper.hpp"
#include "hold_and_weld_gripper_sampler/io/gripper_parser.hpp"
#include "hold_and_weld_gripper_sampler/io/shape_loader.hpp"

using namespace hold_and_weld_gripper_sampler;  // NOLINT
using hold_and_weld_gripper_sampler::core::GraspFinder;
using hold_and_weld_gripper_sampler::core::GraspFinderConfig;
using hold_and_weld_gripper_sampler::core::GraspFinderResult;

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("grasp_finder_node");
  auto logger = node->get_logger();

  // Get package directories
  std::string sampler_pkg_share;
  std::string app_pkg_share;

  try {
    sampler_pkg_share = ament_index_cpp::get_package_share_directory(
      "hold_and_weld_gripper_sampler");
    app_pkg_share = ament_index_cpp::get_package_share_directory(
      "hold_and_weld_application");
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger, "Could not find package share directories: %s", e.what());
    return 1;
  } catch (...) {
    RCLCPP_ERROR(logger, "Unknown error finding package share directories");
    return 1;
  }

  // Get config path from argument or use default
  std::string config_path;

  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    if ((arg == "--config" || arg == "-c") && i + 1 < argc) {
      config_path = argv[++i];
    }
  }

  if (config_path.empty()) {
    config_path = sampler_pkg_share + "/config/grasp_finder.yaml";
    RCLCPP_INFO(logger, "Using default config: %s", config_path.c_str());
  } else {
    RCLCPP_INFO(logger, "Using config: %s", config_path.c_str());
  }

  // Parse configuration
  io::ConfigParser parser;
  auto config_opt = parser.parse_file(config_path, sampler_pkg_share);

  if (!config_opt.has_value()) {
    RCLCPP_ERROR(logger, "Failed to parse config: %s", parser.get_last_error().c_str());
    return 1;
  }

  auto config = config_opt.value();
  RCLCPP_INFO(logger, "Configuration loaded successfully");
  RCLCPP_INFO(logger, "Frame ID: %s", config.frame_id.c_str());
  RCLCPP_INFO(logger, "Primary source: %s",
    config.primary.step_path.empty() ? config.primary.urdf_path.c_str() :
    config.primary.step_path.c_str());
  RCLCPP_INFO(logger, "Gripper: %s", config.gripper_urdf_path.c_str());
  RCLCPP_INFO(logger, "Secondaries: %zu", config.secondaries.size());
  RCLCPP_INFO(logger, "Exclusion circles: %zu", config.exclusion_circles.size());
  RCLCPP_INFO(logger, "Exclusion polygons: %zu", config.exclusion_polygons.size());
  RCLCPP_INFO(logger, "Exclusion lines: %zu", config.exclusion_lines.size());
  RCLCPP_INFO(logger, " ");

  auto mapper = std::make_shared<geometry::GeometryMapper>();
  geometry::Topology topology;
  TopoDS_Shape primary_shape;

  try {
    io::ShapeLoader loader;

    if (!config.primary.step_path.empty()) {
      topology = mapper->load_from_step(
        config.primary.step_path,
        config.primary.translation,
        config.primary.rotation);
      primary_shape = loader.load_from_step(
        config.primary.step_path,
        config.primary.translation,
        config.primary.rotation);
    } else if (!config.primary.urdf_path.empty()) {
      primary_shape = loader.load_from_urdf(config.primary.urdf_path);

      // Apply transform to primary shape (URDF is loaded in local frame)
      if (config.primary.translation.norm() > 1e-9 ||
        !config.primary.rotation.isApprox(Eigen::Quaterniond::Identity()))
      {
        primary_shape = loader.apply_transform(
          primary_shape,
          config.primary.translation,
          config.primary.rotation);
      }

      // Extract topology from the (possibly transformed) shape
      topology = mapper->load_from_shape(primary_shape, "workpiece");
    } else {
      RCLCPP_ERROR(logger, "No primary shape path specified");
      return 1;
    }

    // Triangulate primary shape for kissing surface detection
    RCLCPP_DEBUG(logger, "Triangulating primary shape for contact detection...");
    Standard_Real lin_deflection = 0.0001;
    Standard_Real ang_deflection = 0.5;
    BRepMesh_IncrementalMesh mesher(primary_shape, lin_deflection, Standard_False, ang_deflection);
    RCLCPP_DEBUG(logger, "Primary shape triangulation complete");
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger, "Failed to load primary shape: %s", e.what());
    return 1;
  } catch (...) {
    RCLCPP_ERROR(logger, "Unknown error loading primary shape");
    return 1;
  }

  RCLCPP_INFO(logger, "Primary shape loaded successfully:");
  RCLCPP_INFO(logger, "  Surfaces: %zu", topology.num_surfaces());
  RCLCPP_INFO(logger, "  Edges: %zu", topology.num_edges());
  RCLCPP_INFO(logger, "  Corners: %zu", topology.num_corners());
  RCLCPP_INFO(logger, "  Shape null: %s", primary_shape.IsNull() ? "YES (ERROR!)" : "NO");
  RCLCPP_INFO(logger, " ");

  // Load gripper
  RCLCPP_INFO(logger, "LOADING GRIPPER");
  io::GripperParser gripper_parser;
  ParsedGripper gripper;

  try {
    gripper = gripper_parser.parse_from_urdf_file(config.gripper_urdf_path);

    if (config.gripper_min_opening.has_value()) {
      gripper.min_opening = config.gripper_min_opening.value();
    }
    if (config.gripper_max_opening.has_value()) {
      gripper.max_opening = config.gripper_max_opening.value();
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger, "Failed to load gripper: %s", e.what());
    return 1;
  } catch (...) {
    RCLCPP_ERROR(logger, "Unknown error loading gripper");
    return 1;
  }

  RCLCPP_INFO(logger, "Gripper loaded successfully:");
  RCLCPP_INFO(logger, "  Type: %s", gripper.gripper_type.c_str());
  RCLCPP_INFO(logger, "  Opening range: [%.4f, %.4f] m", gripper.min_opening, gripper.max_opening);
  RCLCPP_INFO(logger, "  TCP offset: (%.4f, %.4f, %.4f)",
    gripper.tcp_offset.x(), gripper.tcp_offset.y(), gripper.tcp_offset.z());
  RCLCPP_INFO(logger, "  Finger 1 axis: (%.2f, %.2f, %.2f)",
    gripper.finger_1_axis.x(), gripper.finger_1_axis.y(), gripper.finger_1_axis.z());
  RCLCPP_INFO(logger, "  Finger 2 axis: (%.2f, %.2f, %.2f)",
    gripper.finger_2_axis.x(), gripper.finger_2_axis.y(), gripper.finger_2_axis.z());
  RCLCPP_INFO(logger, "  Base shape null: %s", gripper.base.IsNull() ? "YES (ERROR!)" : "NO");
  RCLCPP_INFO(logger, "  Finger 1 shape null: %s",
    gripper.finger_1.IsNull() ? "YES (ERROR!)" : "NO");
  RCLCPP_INFO(logger, "  Finger 2 shape null: %s",
    gripper.finger_2.IsNull() ? "YES (ERROR!)" : "NO");
  RCLCPP_INFO(logger, " ");

  RCLCPP_INFO(logger, "Configured secondaries: %zu", config.secondaries.size());
  std::vector<TopoDS_Shape> secondary_shapes;
  io::ShapeLoader shape_loader;

  for (const auto & sec_config : config.secondaries) {
    try {
      TopoDS_Shape shape;

      if (sec_config.type == "ground_plane") {
        shape = shape_loader.make_ground_plane(
          sec_config.size_x, sec_config.size_y, sec_config.z_position);
      } else if (sec_config.type == "box") {
        shape = shape_loader.make_box(
          sec_config.dimensions, sec_config.translation, sec_config.rotation);
      } else if (sec_config.type == "cylinder") {
        shape = shape_loader.make_cylinder(
          sec_config.radius, sec_config.height,
          sec_config.translation, sec_config.rotation);
      } else if (sec_config.type == "step") {
        shape = shape_loader.load_from_step(
          sec_config.file_path, sec_config.translation, sec_config.rotation);
      } else if (sec_config.type == "urdf") {
        shape = shape_loader.load_from_urdf(sec_config.file_path);
      } else {
        RCLCPP_WARN(logger, "Unknown secondary type '%s', skipping", sec_config.type.c_str());
        continue;
      }

      secondary_shapes.push_back(shape);
      RCLCPP_INFO(logger, "  [✓] %s (%s) - shape null: %s",
        sec_config.id.c_str(), sec_config.type.c_str(),
        shape.IsNull() ? "YES (ERROR!)" : "NO");
    } catch (const std::exception & e) {
      RCLCPP_WARN(logger, "Failed to load secondary '%s': %s",
        sec_config.id.c_str(), e.what());
      continue;
    } catch (...) {
      RCLCPP_WARN(logger, "Unknown error loading secondary '%s'", sec_config.id.c_str());
      continue;
    }
  }

  RCLCPP_INFO(logger, "Total secondary shapes loaded: %zu", secondary_shapes.size());
  RCLCPP_INFO(logger, " ");

  // Prepare exclusion zones
  RCLCPP_INFO(logger, "EXCLUSION ZONES");
  std::optional<std::vector<constraints::exclusion_circle>> circles_opt;
  std::optional<std::vector<constraints::exclusion_polygon>> polygons_opt;
  std::optional<std::vector<constraints::exclusion_line>> lines_opt;

  if (!config.exclusion_circles.empty()) {
    circles_opt = config.exclusion_circles;
    RCLCPP_INFO(logger, "Exclusion circles: %zu", config.exclusion_circles.size());
    for (size_t i = 0; i < config.exclusion_circles.size(); i++) {
      const auto & c = config.exclusion_circles[i];
      RCLCPP_INFO(logger, "  [%zu] center=(%.3f,%.3f,%.3f) radius=%.3f depth=%.3f",
        i, c.center.x(), c.center.y(), c.center.z(), c.radius, c.projection_depth);
    }
  } else {
    RCLCPP_INFO(logger, "Exclusion circles: 0 (none defined)");
  }

  if (!config.exclusion_polygons.empty()) {
    polygons_opt = config.exclusion_polygons;
    RCLCPP_INFO(logger, "Exclusion polygons: %zu", config.exclusion_polygons.size());
    for (size_t i = 0; i < config.exclusion_polygons.size(); i++) {
      const auto & p = config.exclusion_polygons[i];
      RCLCPP_INFO(logger, "  [%zu] corners=%zu depth=%.3f",
        i, p.exclusion_corners.size(), p.projection_depth);
    }
  } else {
    RCLCPP_INFO(logger, "Exclusion polygons: 0 (none defined)");
  }

  if (!config.exclusion_lines.empty()) {
    lines_opt = config.exclusion_lines;
    RCLCPP_INFO(logger, "Exclusion lines: %zu", config.exclusion_lines.size());
    for (size_t i = 0; i < config.exclusion_lines.size(); i++) {
      const auto & l = config.exclusion_lines[i];
      RCLCPP_INFO(logger, "  [%zu] start=(%.3f,%.3f,%.3f) end=(%.3f,%.3f,%.3f) radius=%.3f",
        i, l.start.x(), l.start.y(), l.start.z(), l.end.x(), l.end.y(), l.end.z(),
        l.exclusion_radius);
    }
  } else {
    RCLCPP_INFO(logger, "Exclusion lines: 0 (none defined)");
  }
  RCLCPP_INFO(logger, " ");

  // Create GraspFinder
  RCLCPP_INFO(logger, "CREATING GRASP FINDER");
  RCLCPP_INFO(logger, "Sampling config:");
  RCLCPP_INFO(logger, "  Min angle: %.1f°, Max angle: %.1f°",
    config.finder_config.sampling.min_angle_deg, config.finder_config.sampling.max_angle_deg);
  RCLCPP_INFO(logger, "  Gripper opening: [%.4f, %.4f] m",
    config.finder_config.sampling.min_gripper_opening,
    config.finder_config.sampling.max_gripper_opening);
  RCLCPP_INFO(logger, "  Sample density: %.4f m", config.finder_config.sampling.sample_density);
  RCLCPP_INFO(logger, "Orientation config:");
  RCLCPP_INFO(logger, "  Finger length: %.4f m, radius: %.4f m",
    config.finder_config.orientation.finger_length,
    config.finder_config.orientation.finger_radius);
  RCLCPP_INFO(logger, "  Max edge candidates: %zu",
    config.finder_config.orientation.max_edge_candidates);
  RCLCPP_INFO(logger, "  Dual seed dedup tolerance: %.1f°",
    config.finder_config.orientation.dual_seed_dedup_tolerance_deg);
  RCLCPP_INFO(logger, "  Angle offsets: %zu values",
    config.finder_config.orientation.angle_offsets.size());
  RCLCPP_INFO(logger, "FCL: %s, Kissing threshold: %.1f%%",
    config.finder_config.use_fcl ? "ENABLED" : "DISABLED",
    config.finder_config.kissing_contact_threshold * 100.0);
  RCLCPP_INFO(logger, " ");

  GraspFinder finder(
    mapper,
    primary_shape,
    topology,
    gripper,
    secondary_shapes,
    circles_opt,
    polygons_opt,
    lines_opt,
    config.finder_config
  );

  auto start_time = std::chrono::steady_clock::now();

  GraspFinderResult result = finder.find();

  auto end_time = std::chrono::steady_clock::now();
  double elapsed_seconds = std::chrono::duration<double>(end_time - start_time).count();

  RCLCPP_INFO(logger, " ");

  if (!result.success) {
    RCLCPP_ERROR(logger, "Grasp finding failed: %s", result.error_message.c_str());
    return 1;
  }

  RCLCPP_INFO(logger, "Time elapsed: %.2f seconds", elapsed_seconds);
  RCLCPP_INFO(logger, " ");
  RCLCPP_INFO(logger, "Pipeline statistics:");
  RCLCPP_INFO(logger, "  Valid surfaces: %zu", result.num_valid_surfaces);
  RCLCPP_INFO(logger, "  Banned surfaces: %zu", result.num_banned_surfaces);
  RCLCPP_INFO(logger, "  Exclusion areas: %zu", result.num_exclusion_areas);
  RCLCPP_INFO(logger, "  Contact pairs: %zu", result.num_contact_pairs);
  RCLCPP_INFO(logger, "  Grasp candidates: %zu", result.num_candidates);
  RCLCPP_INFO(logger, "  Final grasps: %zu", result.grasps.size());
  RCLCPP_INFO(logger, " ");

  // Write JSON output to hold_and_weld_application/grasps folder
  // Use source directory for development, not install
  std::string workspace_root = std::getenv("HOME") ?
    std::string(std::getenv("HOME")) + "/ros2_yaskawa" : ".";
  std::string output_dir = workspace_root +
    "/src/hold_and_weld/hold_and_weld_application/grasps";

  // Create directory if needed
  std::system(("mkdir -p " + output_dir).c_str());

  // Generate filename with timestamp
  std::string output_path = output_dir + "/grasps.json";

  io::ResultMetadata metadata;
  metadata.coordinate_frame = config.frame_id;
  metadata.primary_source = config.primary.step_path.empty() ?
    config.primary.urdf_path : config.primary.step_path;
  metadata.gripper_source = config.gripper_urdf_path;
  metadata.config_source = config_path;
  metadata.num_surfaces_total = topology.num_surfaces();
  metadata.total_time_seconds = elapsed_seconds;

  io::WriterOptions writer_options;
  writer_options.pretty_print = true;
  writer_options.include_debug = config.output.include_debug;
  writer_options.max_grasps = config.output.max_grasps;
  writer_options.min_quality = config.output.min_quality;

  io::ResultWriter writer;
  if (writer.write_to_file(result, output_path, metadata, writer_options)) {
    RCLCPP_INFO(logger, "Results written to: %s", output_path.c_str());
  } else {
    RCLCPP_ERROR(logger, "Failed to write results: %s", writer.get_last_error().c_str());
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
