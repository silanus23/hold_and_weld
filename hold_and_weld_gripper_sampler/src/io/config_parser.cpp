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

#include "hold_and_weld_gripper_sampler/io/config_parser.hpp"

#include <fstream>
#include <sstream>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>

namespace hold_and_weld_gripper_sampler
{
namespace io
{

static const rclcpp::Logger logger_ = rclcpp::get_logger("gripper_sampler");

std::optional<ParsedConfig> ConfigParser::parse_file(
  const std::string & yaml_path,
  const std::string & package_share_dir)
{
  // Set base directory for relative path resolution
  if (!package_share_dir.empty()) {
    base_dir_ = package_share_dir;
  } else {
    size_t last_slash = yaml_path.find_last_of('/');
    if (last_slash != std::string::npos) {
      base_dir_ = yaml_path.substr(0, last_slash);
    } else {
      base_dir_ = ".";
    }
  }

  YAML::Node root;
  try {
    root = YAML::LoadFile(yaml_path);
  } catch (const YAML::Exception & e) {
    set_error("Failed to load YAML file: " + std::string(e.what()));
    return std::nullopt;
  }

  return parse_node(root, base_dir_);
}

std::optional<ParsedConfig> ConfigParser::parse_string(
  const std::string & yaml_content,
  const std::string & base_dir)
{
  base_dir_ = base_dir.empty() ? "." : base_dir;

  YAML::Node root;
  try {
    root = YAML::Load(yaml_content);
  } catch (const YAML::Exception & e) {
    set_error("Failed to parse YAML string: " + std::string(e.what()));
    return std::nullopt;
  }

  return parse_node(root, base_dir_);
}

std::optional<ParsedConfig> ConfigParser::parse_node(
  const YAML::Node & root_node,
  const std::string & base_dir)
{
  base_dir_ = base_dir.empty() ? "." : base_dir;

  ParsedConfig config;

  try {
    YAML::Node params = get_parameters_node(root_node);

    if (!params || params.IsNull()) {
      set_error("No ros__parameters node found in YAML");
      return std::nullopt;
    }

    if (params["frame_id"]) {
      config.frame_id = params["frame_id"].as<std::string>();
    }

    if (params["primary"]) {
      if (!parse_primary(params["primary"], config.primary)) {
        return std::nullopt;
      }
    } else {
      set_error("Missing required 'primary' section");
      return std::nullopt;
    }

    if (params["gripper"]) {
      if (!parse_gripper(params["gripper"], config)) {
        return std::nullopt;
      }
    } else {
      set_error("Missing required 'gripper' section");
      return std::nullopt;
    }

    if (params["secondaries"]) {
      if (!parse_secondaries(params["secondaries"], config.secondaries)) {
        return std::nullopt;
      }
    }

    if (params["exclusion_zones"]) {
      if (!parse_exclusion_zones(params["exclusion_zones"], config)) {
        return std::nullopt;
      }
    }

    if (params["mesh_deflection"]) {
      if (params["mesh_deflection"]["linear"]) {
        config.mesh_linear_deflection = params["mesh_deflection"]["linear"].as<double>();
        config.finder_config.mesh_linear_deflection = config.mesh_linear_deflection;
      }
      if (params["mesh_deflection"]["angular"]) {
        config.mesh_angular_deflection = params["mesh_deflection"]["angular"].as<double>();
        config.finder_config.mesh_angular_deflection = config.mesh_angular_deflection;
      }
    }

    if (params["sampling"]) {
      if (!parse_sampling(params["sampling"], config.finder_config.sampling)) {
        return std::nullopt;
      }
    }

    if (params["orientation"]) {
      if (!parse_orientation(params["orientation"], config.finder_config.orientation)) {
        return std::nullopt;
      }
    }

    if (params["kissing"]) {
      if (params["kissing"]["contact_threshold"]) {
        config.finder_config.kissing_contact_threshold =
          params["kissing"]["contact_threshold"].as<double>();
      }
      if (params["kissing"]["collision_tolerance"]) {
        config.finder_config.collision_tolerance =
          params["kissing"]["collision_tolerance"].as<double>();
      }
    }

    if (params["fcl"]) {
      if (params["fcl"]["enabled"]) {
        config.finder_config.use_fcl = params["fcl"]["enabled"].as<bool>();
      }
      if (params["fcl"]["triangulation_deflection"]) {
        config.finder_config.triangulation_deflection =
          params["fcl"]["triangulation_deflection"].as<double>();
      }
    }

    if (params["output"]) {
      if (!parse_output(params["output"], config.output)) {
        return std::nullopt;
      }
    }

    RCLCPP_INFO(logger_, "Configuration parsed successfully");
    return config;
  } catch (const YAML::Exception & e) {
    set_error("YAML parsing error: " + std::string(e.what()));
    return std::nullopt;
  } catch (const std::exception & e) {
    set_error("Error parsing configuration: " + std::string(e.what()));
    return std::nullopt;
  } catch (...) {
    set_error("Unknown error parsing configuration");
    return std::nullopt;
  }
}

YAML::Node ConfigParser::get_parameters_node(const YAML::Node & root) const
{
  if (root["/**"]) {
    if (root["/**"]["ros__parameters"]) {
      return root["/**"]["ros__parameters"];
    }
  }

  if (root["ros__parameters"]) {
    return root["ros__parameters"];
  }

  return root;
}

bool ConfigParser::parse_primary(const YAML::Node & node, PrimaryConfig & config)
{
  if (node["step_path"]) {
    config.step_path = resolve_path(node["step_path"].as<std::string>());
  } else if (node["urdf_path"]) {
    config.urdf_path = resolve_path(node["urdf_path"].as<std::string>());
  } else {
    set_error("Primary must have either 'step_path' or 'urdf_path'");
    return false;
  }

  if (node["transform"]) {
    parse_transform(node["transform"], config.translation, config.rotation);
  }

  return true;
}

bool ConfigParser::parse_gripper(const YAML::Node & node, ParsedConfig & config)
{
  if (!node["urdf_path"]) {
    set_error("Gripper must have 'urdf_path'");
    return false;
  }

  config.gripper_urdf_path = resolve_path(node["urdf_path"].as<std::string>());

  if (node["max_opening"]) {
    config.gripper_max_opening = node["max_opening"].as<double>();
  }

  return true;
}

bool ConfigParser::parse_secondaries(
  const YAML::Node & node,
  std::vector<SecondaryConfig> & configs)
{
  if (!node.IsSequence()) {
    set_error("'secondaries' must be a sequence");
    return false;
  }

  for (const auto & item : node) {
    SecondaryConfig sec_config;
    if (!parse_secondary(item, sec_config)) {
      return false;
    }
    configs.push_back(sec_config);
  }

  return true;
}

bool ConfigParser::parse_secondary(const YAML::Node & node, SecondaryConfig & config)
{
  if (!node["type"]) {
    set_error("Secondary shape must have 'type'");
    return false;
  }

  config.type = node["type"].as<std::string>();

  if (node["id"]) {
    config.id = node["id"].as<std::string>();
  }

  if (config.type == "step" || config.type == "urdf") {
    if (node["step_path"]) {
      config.file_path = resolve_path(node["step_path"].as<std::string>());
    } else if (node["urdf_path"]) {
      config.file_path = resolve_path(node["urdf_path"].as<std::string>());
    } else {
      set_error("Secondary shape of type '" + config.type + "' must have file path");
      return false;
    }
  } else if (config.type == "box") {
    if (node["dimensions"]) {
      config.dimensions = parse_vector3(node["dimensions"]);
    } else {
      set_error("Box secondary must have 'dimensions'");
      return false;
    }
  } else if (config.type == "cylinder") {
    if (node["radius"] && node["height"]) {
      config.radius = node["radius"].as<double>();
      config.height = node["height"].as<double>();
    } else {
      set_error("Cylinder secondary must have 'radius' and 'height'");
      return false;
    }
  } else if (config.type == "ground_plane") {
    if (node["size_x"]) {
      config.size_x = node["size_x"].as<double>();
    }
    if (node["size_y"]) {
      config.size_y = node["size_y"].as<double>();
    }
    if (node["z_position"]) {
      config.z_position = node["z_position"].as<double>();
    }
  } else {
    set_error("Unknown secondary shape type: " + config.type);
    return false;
  }

  // Parse transform (optional for all types)
  if (node["transform"]) {
    parse_transform(node["transform"], config.translation, config.rotation);
  }

  return true;
}

bool ConfigParser::parse_exclusion_zones(const YAML::Node & node, ParsedConfig & config)
{
  if (node["circles"] && node["circles"].IsSequence()) {
    for (const auto & item : node["circles"]) {
      constraints::exclusion_circle circle;
      if (!parse_exclusion_circle(item, circle)) {
        return false;
      }
      config.exclusion_circles.push_back(circle);
    }
  }

  if (node["polygons"] && node["polygons"].IsSequence()) {
    for (const auto & item : node["polygons"]) {
      constraints::exclusion_polygon polygon;
      if (!parse_exclusion_polygon(item, polygon)) {
        return false;
      }
      config.exclusion_polygons.push_back(polygon);
    }
  }

  if (node["lines"] && node["lines"].IsSequence()) {
    for (const auto & item : node["lines"]) {
      constraints::exclusion_line line;
      if (!parse_exclusion_line(item, line)) {
        return false;
      }
      config.exclusion_lines.push_back(line);
    }
  }

  return true;
}

bool ConfigParser::parse_exclusion_circle(
  const YAML::Node & node,
  constraints::exclusion_circle & circle)
{
  if (!node["center"] || !node["normal"] || !node["radius"] || !node["projection_depth"]) {
    set_error("Exclusion circle must have 'center', 'normal', 'radius', and 'projection_depth'");
    return false;
  }

  circle.center = parse_vector3(node["center"]);
  circle.normal = parse_vector3(node["normal"]);
  circle.radius = node["radius"].as<double>();
  circle.projection_depth = node["projection_depth"].as<double>();

  if (node["clearance"]) {
    circle.clearance = node["clearance"].as<double>();
  }

  return true;
}

bool ConfigParser::parse_exclusion_polygon(
  const YAML::Node & node,
  constraints::exclusion_polygon & polygon)
{
  if (!node["corners"] || !node["corners"].IsSequence() || !node["projection_depth"]) {
    set_error("Exclusion polygon must have 'corners' (sequence) and 'projection_depth'");
    return false;
  }

  for (const auto & corner : node["corners"]) {
    polygon.exclusion_corners.push_back(parse_vector3(corner));
  }

  polygon.projection_depth = node["projection_depth"].as<double>();

  if (node["clearance"]) {
    polygon.clearance = node["clearance"].as<double>();
  }

  return true;
}

bool ConfigParser::parse_exclusion_line(
  const YAML::Node & node,
  constraints::exclusion_line & line)
{
  if (!node["start"] || !node["end"] || !node["exclusion_radius"]) {
    set_error(
      "Exclusion line must have 'start', 'end', and 'exclusion_radius'");
    return false;
  }

  line.start = parse_vector3(node["start"]);
  line.end = parse_vector3(node["end"]);
  line.exclusion_radius = node["exclusion_radius"].as<double>();

  if (node["clearance"]) {
    line.clearance = node["clearance"].as<double>();
  }

  return true;
}

bool ConfigParser::parse_sampling(const YAML::Node & node, sampling::SamplingConfig & config)
{
  if (node["min_angle_deg"]) {
    config.min_angle_deg = node["min_angle_deg"].as<double>();
  }
  if (node["max_angle_deg"]) {
    config.max_angle_deg = node["max_angle_deg"].as<double>();
  }
  if (node["min_gripper_opening"]) {
    config.min_gripper_opening = node["min_gripper_opening"].as<double>();
  }
  if (node["max_gripper_opening"]) {
    config.max_gripper_opening = node["max_gripper_opening"].as<double>();
  }
  if (node["sample_density"]) {
    config.sample_density = node["sample_density"].as<double>();
  }
  if (node["normal_sample_density"]) {
    config.normal_sample_density = node["normal_sample_density"].as<double>();
  }
  if (node["max_lateral_deviation"]) {
    config.max_lateral_deviation = node["max_lateral_deviation"].as<double>();
  }
  if (node["alignment_threshold"]) {
    config.alignment_threshold = node["alignment_threshold"].as<double>();
  }

  return true;
}

bool ConfigParser::parse_orientation(
  const YAML::Node & node,
  angle_finding::OrientationConfig & config)
{
  if (node["finger_length"]) {
    config.finger_length = node["finger_length"].as<double>();
  }
  if (node["finger_radius"]) {
    config.finger_radius = node["finger_radius"].as<double>();
  }
  if (node["max_edge_candidates"]) {
    config.max_edge_candidates = node["max_edge_candidates"].as<size_t>();
  }
  if (node["dual_seed_dedup_tolerance_deg"]) {
    config.dual_seed_dedup_tolerance_deg = node["dual_seed_dedup_tolerance_deg"].as<double>();
  }
  if (node["max_edges_per_contact"]) {
    config.max_edges_per_contact = node["max_edges_per_contact"].as<size_t>();
  }
  if (node["angle_offsets"] && node["angle_offsets"].IsSequence()) {
    config.angle_offsets.clear();
    for (const auto & offset : node["angle_offsets"]) {
      config.angle_offsets.push_back(offset.as<double>());
    }
  }
  if (node["stop_on_first_valid"]) {
    config.stop_on_first_valid = node["stop_on_first_valid"].as<bool>();
  }
  if (node["collision_tolerance"]) {
    config.collision_tolerance = node["collision_tolerance"].as<double>();
  }

  return true;
}

bool ConfigParser::parse_output(const YAML::Node & node, OutputConfig & config)
{
  if (node["json_path"]) {
    config.json_path = node["json_path"].as<std::string>();
  }
  if (node["max_grasps"]) {
    config.max_grasps = node["max_grasps"].as<size_t>();
  }
  if (node["min_quality"]) {
    config.min_quality = node["min_quality"].as<double>();
  }
  if (node["include_debug"]) {
    config.include_debug = node["include_debug"].as<bool>();
  }

  return true;
}

Eigen::Vector3d ConfigParser::parse_vector3(const YAML::Node & node) const
{
  Eigen::Vector3d vec = Eigen::Vector3d::Zero();

  if (node.IsSequence() && node.size() >= 3) {
    vec.x() = node[0].as<double>();
    vec.y() = node[1].as<double>();
    vec.z() = node[2].as<double>();
  } else if (node.IsMap()) {
    if (node["x"]) {
      vec.x() = node["x"].as<double>();
    }
    if (node["y"]) {
      vec.y() = node["y"].as<double>();
    }
    if (node["z"]) {
      vec.z() = node["z"].as<double>();
    }
  }

  return vec;
}

Eigen::Quaterniond ConfigParser::parse_quaternion(const YAML::Node & node) const
{
  Eigen::Quaterniond quat = Eigen::Quaterniond::Identity();

  if (node.IsSequence() && node.size() >= 4) {
    // Assume order: x, y, z, w
    quat.x() = node[0].as<double>();
    quat.y() = node[1].as<double>();
    quat.z() = node[2].as<double>();
    quat.w() = node[3].as<double>();
  } else if (node.IsMap()) {
    if (node["x"]) {
      quat.x() = node["x"].as<double>();
    }
    if (node["y"]) {
      quat.y() = node["y"].as<double>();
    }
    if (node["z"]) {
      quat.z() = node["z"].as<double>();
    }
    if (node["w"]) {
      quat.w() = node["w"].as<double>();
    }
  }

  return quat.normalized();
}

void ConfigParser::parse_transform(
  const YAML::Node & node,
  Eigen::Vector3d & translation,
  Eigen::Quaterniond & rotation) const
{
  if (node["translation"]) {
    translation = parse_vector3(node["translation"]);
  }
  if (node["rotation"]) {
    rotation = parse_quaternion(node["rotation"]);
  }
}

std::string ConfigParser::resolve_path(const std::string & path) const
{
  if (path.empty()) {
    return path;
  }

  if (path.substr(0, 10) == "package://") {
    size_t pkg_end = path.find('/', 10);
    if (pkg_end != std::string::npos) {
      std::string package_name = path.substr(10, pkg_end - 10);
      std::string relative_path = path.substr(pkg_end + 1);
      try {
        std::string pkg_share = ament_index_cpp::get_package_share_directory(package_name);
        return pkg_share + "/" + relative_path;
      } catch (const std::exception & e) {
        RCLCPP_WARN(logger_, "Could not resolve package '%s': %s", package_name.c_str(), e.what());
        return path;
      }
    }
  }

  if (path[0] == '/') {
    return path;
  }

  return base_dir_ + "/" + path;
}

void ConfigParser::set_error(const std::string & message)
{
  last_error_ = message;
  RCLCPP_ERROR(logger_, "%s", message.c_str());
}

}  // namespace io
}  // namespace hold_and_weld_gripper_sampler
