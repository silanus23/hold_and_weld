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
 *   ros2 run hold_and_weld_gripper_sampler grasp_finder_node --output <path_to_json>
 *   ros2 run hold_and_weld_gripper_sampler grasp_finder_node  # uses default config
 *
 * -c/--config and -o/--output can be combined. Without --output, results go to
 * <hold_and_weld_application share>/grasps/grasps.json. Unknown arguments are rejected.
 */

#include <chrono>
#include <filesystem>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <BRepMesh_IncrementalMesh.hxx>
#include <rclcpp/rclcpp.hpp>
#include <Standard_Failure.hxx>

#include "hold_and_weld_gripper_sampler/core/grasp_finder.hpp"
#include "hold_and_weld_gripper_sampler/core/gripper.hpp"
#include "hold_and_weld_gripper_sampler/geometry/geometry_mapper.hpp"
#include "hold_and_weld_gripper_sampler/geometry/shape_refiner.hpp"
#include "hold_and_weld_gripper_sampler/io/config_parser.hpp"
#include "hold_and_weld_gripper_sampler/io/gripper_parser.hpp"
#include "hold_and_weld_gripper_sampler/io/result_writer.hpp"
#include "hold_and_weld_gripper_sampler/io/shape_loader.hpp"

using namespace hold_and_weld_gripper_sampler;  // NOLINT
using hold_and_weld_gripper_sampler::core::GraspFinder;
using hold_and_weld_gripper_sampler::core::GraspFinderConfig;
using hold_and_weld_gripper_sampler::core::GraspFinderResult;

namespace
{

// Returns the process exit code. Kept separate from main() so every exit path,
// including exceptions, goes through the single rclcpp::shutdown() in main().
int run(const std::vector<std::string> & args)
{
  try {
    auto node = std::make_shared<rclcpp::Node>("grasp_finder_node");
    auto logger = node->get_logger();

    std::string config_path;
    std::string output_path_arg;

    // args[0] is the program name; ROS arguments were already stripped.
    for (size_t i = 1; i < args.size(); ++i) {
      const std::string & arg = args[i];
      const bool is_config = (arg == "--config" || arg == "-c");
      const bool is_output = (arg == "--output" || arg == "-o");
      if (!is_config && !is_output) {
        RCLCPP_ERROR(logger, "Unknown argument '%s'. Usage: [--config <yaml>] [--output <json>]",
          arg.c_str());
        return 1;
      }
      if (i + 1 >= args.size()) {
        RCLCPP_ERROR(logger, "Argument '%s' needs a value", arg.c_str());
        return 1;
      }
      (is_config ? config_path : output_path_arg) = args[++i];
    }

    std::string sampler_pkg_share;
    try {
      sampler_pkg_share = ament_index_cpp::get_package_share_directory(
        "hold_and_weld_gripper_sampler");
    } catch (const std::exception & e) {
      RCLCPP_ERROR(logger, "Could not find package share directory: %s", e.what());
      return 1;
    }

    if (config_path.empty()) {
      config_path = sampler_pkg_share + "/config/grasp_finder.yaml";
      RCLCPP_INFO(logger, "Using default config: %s", config_path.c_str());
    } else {
      RCLCPP_INFO(logger, "Using config: %s", config_path.c_str());
    }

    io::ConfigParser parser;
    auto config_opt = parser.parse_file(config_path, sampler_pkg_share);

    if (!config_opt.has_value()) {
      RCLCPP_ERROR(logger, "Failed to parse config: %s", parser.get_last_error().c_str());
      return 1;
    }

    auto config = std::move(*config_opt);
    RCLCPP_INFO(logger, "Configuration loaded: frame=%s, gripper=%s, secondaries=%zu",
    config.frame_id.c_str(), config.gripper_urdf_path.c_str(), config.secondaries.size());

    auto mapper = std::make_shared<geometry::GeometryMapper>();
    geometry::Topology topology;
    TopoDS_Shape primary_shape;
    TopoDS_Shape fcl_primary_shape;  // pre-refiner copy for FCL collision checking

    try {
      io::ShapeLoader loader;

      if (!config.primary.step_path.empty()) {
        primary_shape = loader.load_from_step(
        config.primary.step_path,
        config.primary.translation,
        config.primary.rotation);
      } else if (!config.primary.urdf_path.empty()) {
        primary_shape = loader.load_from_urdf(config.primary.urdf_path);

        if (config.primary.translation.norm() > 1e-9 ||
          !config.primary.rotation.isApprox(Eigen::Quaterniond::Identity()))
        {
          primary_shape = loader.apply_transform(
          primary_shape,
          config.primary.translation,
          config.primary.rotation);
        }
      } else {
        RCLCPP_ERROR(logger, "No primary shape path specified");
        return 1;
      }

      // Save raw shape for FCL before any refinement — ShapeRefiner can introduce
      // face orientation artifacts that cause BVH holes in collision meshes.
      fcl_primary_shape = primary_shape;

      // Refine shape before mapping — ShapeRefiner must run on the raw shape
      // before the mapper builds its face index, so topology reflects refined geometry.
      if (config.finder_config.shape_refiner.enabled) {
        RCLCPP_INFO(logger, "Running ShapeRefiner on primary shape...");
        const auto & sr = config.finder_config.shape_refiner;
        geometry::ShapeRefiner refiner(
          sr.max_cylinder_radius,
          sr.max_arc_length,
          sr.enclave_area_ratio,
          sr.enclave_angle_threshold,
          sr.max_face_area_ratio,
          sr.planarity_tolerance_deg);
        TopoDS_Shape refined = refiner.refine(primary_shape);
        if (!refined.IsNull()) {
          primary_shape = refined;
          RCLCPP_INFO(logger, "Shape refinement complete");
        } else {
          RCLCPP_WARN(logger, "ShapeRefiner returned null shape - using original");
        }
      }

      topology = mapper->load_from_shape(primary_shape, "workpiece");

      // Triangulate primary shape for kissing surface detection (side-effect on shape)
      RCLCPP_DEBUG(logger, "Triangulating primary shape for contact detection...");
      BRepMesh_IncrementalMesh mesher(
        primary_shape, config.mesh_linear_deflection, Standard_False,
        config.mesh_angular_deflection);
      (void)mesher;
      RCLCPP_DEBUG(logger, "Primary shape triangulation complete");
    } catch (const std::exception & e) {
      RCLCPP_ERROR(logger, "Failed to load primary shape: %s", e.what());
      return 1;
    } catch (const Standard_Failure & e) {
      RCLCPP_ERROR(logger, "Failed to load primary shape: %s", e.GetMessageString());
      return 1;
    } catch (...) {
      RCLCPP_ERROR(logger, "Unknown error loading primary shape");
      return 1;
    }

    RCLCPP_INFO(logger, "Primary shape loaded: surfaces=%zu, edges=%zu, corners=%zu",
    topology.num_surfaces(), topology.num_edges(), topology.num_corners());

    io::GripperParser gripper_parser;
    ParsedGripper gripper;

    try {
      gripper = gripper_parser.parse_from_urdf_file(config.gripper_urdf_path);

      if (config.gripper_max_opening.has_value()) {
        gripper.max_opening = std::min(
            config.gripper_max_opening.value(),
            gripper.max_opening);
      }
    } catch (const std::exception & e) {
      RCLCPP_ERROR(logger, "Failed to load gripper: %s", e.what());
      return 1;
    } catch (const Standard_Failure & e) {
      RCLCPP_ERROR(logger, "Failed to load gripper: %s", e.GetMessageString());
      return 1;
    } catch (...) {
      RCLCPP_ERROR(logger, "Unknown error loading gripper");
      return 1;
    }

    RCLCPP_INFO(logger, "Gripper loaded: type=%s, max_opening=%.4f m",
    gripper.gripper_type.c_str(), gripper.max_opening);

    std::vector<TopoDS_Shape> secondary_shapes;
    io::ShapeLoader shape_loader;

    for (const auto & sec_config : config.secondaries) {
      try {
        TopoDS_Shape shape;

        if (sec_config.type == "ground_plane") {
          shape = shape_loader.make_ground_plane(
              sec_config.size_x,
              sec_config.size_y,
              sec_config.z_position,
              0.1,
              sec_config.translation.x(),
              sec_config.translation.y());

          // Override the FCL ground plane Z with the explicit YAML z_position.
          // ConfigParser allows only one ground_plane, so nothing is overwritten here.
          config.finder_config.ground_bottom_z = sec_config.z_position;
          config.finder_config.ground_center_x = sec_config.translation.x();
          config.finder_config.ground_center_y = sec_config.translation.y();
          config.finder_config.ground_size_x = sec_config.size_x;
          config.finder_config.ground_size_y = sec_config.size_y;
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
          if (sec_config.translation.norm() > 1e-9 ||
            !sec_config.rotation.isApprox(Eigen::Quaterniond::Identity()))
          {
            shape = shape_loader.apply_transform(
            shape, sec_config.translation, sec_config.rotation);
          }
        } else {
          RCLCPP_ERROR(logger, "Unknown secondary type '%s' for '%s'",
            sec_config.type.c_str(), sec_config.id.c_str());
          return 1;
        }

        if (sec_config.type == "ground_plane") {
          config.finder_config.ground_shapes.push_back(shape);
        } else {
          secondary_shapes.push_back(shape);
        }

        RCLCPP_DEBUG(logger, "Secondary loaded: %s (%s)",
        sec_config.id.c_str(), sec_config.type.c_str());
      } catch (const std::exception & e) {
        // Fail closed: grasps generated without this obstacle could collide with it.
        RCLCPP_ERROR(logger, "Failed to load secondary '%s': %s",
          sec_config.id.c_str(), e.what());
        return 1;
      } catch (const Standard_Failure & e) {
        RCLCPP_ERROR(logger, "Failed to load secondary '%s': %s",
          sec_config.id.c_str(), e.GetMessageString());
        return 1;
      } catch (...) {
        RCLCPP_ERROR(logger, "Unknown error loading secondary '%s'", sec_config.id.c_str());
        return 1;
      }
    }

    RCLCPP_INFO(logger, "Secondary shapes loaded: %zu / %zu",
      secondary_shapes.size() + config.finder_config.ground_shapes.size(),
      config.secondaries.size());

    RCLCPP_INFO(logger, "Shapes split: %zu secondary, %zu ground_plane",
    secondary_shapes.size(), config.finder_config.ground_shapes.size());

    std::optional<std::vector<constraints::exclusion_circle>> circles_opt;
    std::optional<std::vector<constraints::exclusion_polygon>> polygons_opt;
    std::optional<std::vector<constraints::exclusion_line>> lines_opt;

    if (!config.exclusion_circles.empty()) {
      circles_opt = config.exclusion_circles;
    }
    if (!config.exclusion_polygons.empty()) {
      polygons_opt = config.exclusion_polygons;
    }
    if (!config.exclusion_lines.empty()) {
      lines_opt = config.exclusion_lines;
    }

    RCLCPP_INFO(logger, "Exclusion zones: circles=%zu, polygons=%zu, lines=%zu",
    config.exclusion_circles.size(),
    config.exclusion_polygons.size(),
    config.exclusion_lines.size());

    RCLCPP_DEBUG(logger, "Sampling: angle=[%.1f°, %.1f°], opening=[%.4f, %.4f] m, density=%.4f m",
    config.finder_config.sampling.min_angle_deg, config.finder_config.sampling.max_angle_deg,
    config.finder_config.sampling.min_gripper_opening,
    config.finder_config.sampling.max_gripper_opening,
    config.finder_config.sampling.sample_density);

    GraspFinder finder(
      mapper,
      primary_shape,
      topology,
      gripper,
      secondary_shapes,
      circles_opt,
      polygons_opt,
      lines_opt,
      config.finder_config,
      fcl_primary_shape
    );

    auto start_time = std::chrono::steady_clock::now();

    GraspFinderResult result = finder.find();

    auto end_time = std::chrono::steady_clock::now();
    double elapsed_seconds = std::chrono::duration<double>(end_time - start_time).count();

    if (!result.success) {
      RCLCPP_ERROR(logger, "Grasp finding failed: %s", result.error_message.c_str());
      return 1;
    }

    RCLCPP_INFO(logger,
    "Grasp finding complete in %.2fs: valid_surfaces=%zu, contact_pairs=%zu, "
    "candidates=%zu, final_grasps=%zu",
    elapsed_seconds,
    result.num_valid_surfaces,
    result.num_contact_pairs,
    result.num_candidates,
    result.grasps.size());

    std::string output_path;
    if (!output_path_arg.empty()) {
      output_path = output_path_arg;
      RCLCPP_INFO(logger, "Output path: %s", output_path.c_str());
    } else {
      // Looked up only here so --output works without hold_and_weld_application installed.
      std::string app_pkg_share;
      try {
        app_pkg_share = ament_index_cpp::get_package_share_directory("hold_and_weld_application");
      } catch (const std::exception & e) {
        RCLCPP_ERROR(logger, "No --output given and default output package not found: %s",
          e.what());
        return 1;
      }
      std::string output_dir = app_pkg_share + "/grasps";
      try {
        std::filesystem::create_directories(output_dir);
      } catch (const std::filesystem::filesystem_error & e) {
        RCLCPP_ERROR(logger, "Failed to create output directory '%s': %s",
        output_dir.c_str(), e.what());
        return 1;
      }
      output_path = output_dir + "/grasps.json";
      RCLCPP_INFO(logger, "Output path: %s", output_path.c_str());
    }

    if (!output_path_arg.empty()) {
      std::filesystem::path parent = std::filesystem::path(output_path).parent_path();
      if (!parent.empty()) {
        try {
          std::filesystem::create_directories(parent);
        } catch (const std::filesystem::filesystem_error & e) {
          RCLCPP_ERROR(logger, "Failed to create output directory '%s': %s",
          parent.c_str(), e.what());
          return 1;
        }
      }
    }

    io::ResultMetadata metadata;
    metadata.coordinate_frame = config.frame_id;
    metadata.primary_source = config.primary.step_path.empty() ?
      config.primary.urdf_path : config.primary.step_path;
    metadata.gripper_source = config.gripper_urdf_path;
    metadata.config_source = config_path;
    metadata.num_surfaces_total = topology.num_surfaces();
    metadata.total_time_seconds = elapsed_seconds;
    metadata.finger_length = config.finder_config.orientation.finger_length;

    metadata.jaw_clearance_enabled = config.finder_config.jaw_clearance.enabled;
    metadata.jaw_clearance_margin = config.finder_config.jaw_clearance.clearance_margin;
    metadata.exclusion_circles = config.exclusion_circles;
    metadata.exclusion_lines = config.exclusion_lines;
    metadata.exclusion_polygons = config.exclusion_polygons;

    io::WriterOptions writer_options;
    writer_options.pretty_print = true;
    writer_options.max_grasps = config.output.max_grasps;
    writer_options.min_quality = config.output.min_quality;

    io::ResultWriter writer;
    if (writer.write_to_file(result, output_path, metadata, writer_options)) {
      RCLCPP_INFO(logger, "Results written to: %s", output_path.c_str());
    } else {
      RCLCPP_ERROR(logger, "Failed to write results: %s", writer.get_last_error().c_str());
      return 1;
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("grasp_finder_node"), "Fatal error: %s", e.what());
    return 1;
  } catch (const Standard_Failure & e) {
    RCLCPP_ERROR(rclcpp::get_logger("grasp_finder_node"), "Fatal OCCT error: %s",
      e.GetMessageString());
    return 1;
  } catch (...) {
    RCLCPP_ERROR(rclcpp::get_logger("grasp_finder_node"), "Fatal unknown error");
    return 1;
  }
  return 0;
}

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  const int exit_code = run(rclcpp::remove_ros_arguments(argc, argv));
  rclcpp::shutdown();
  return exit_code;
}
