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

#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>

namespace hold_and_weld_gripper_sampler
{
namespace io
{

namespace
{

const rclcpp::Logger logger_ = rclcpp::get_logger("gripper_sampler");

constexpr double kMinDirectionNorm = 1e-6;
constexpr double kPlanarityTolerance = 1e-6;

// Every number in the config is read through this, so .nan / .inf never get in.
double finite_double(const YAML::Node & node)
{
  const double value = node.as<double>();
  if (!std::isfinite(value)) {
    throw std::runtime_error(
            "non-finite number '" + node.Scalar() + "' at line " +
            std::to_string(node.Mark().line + 1));
  }
  return value;
}

}  // namespace

std::optional<ParsedConfig> ConfigParser::parse_file(
  const std::string & yaml_path,
  const std::string & package_share_dir)
{
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

    if (params["implicit_ground"]) {
      config.finder_config.enable_ground_plane_check = params["implicit_ground"].as<bool>();
    }

    if (params["exclusion_zones"]) {
      if (!parse_exclusion_zones(params["exclusion_zones"], config)) {
        return std::nullopt;
      }
    }

    if (params["mesh_deflection"]) {
      if (params["mesh_deflection"]["linear"]) {
        config.mesh_linear_deflection = finite_double(params["mesh_deflection"]["linear"]);
        config.finder_config.mesh_linear_deflection = config.mesh_linear_deflection;
        if (!(config.mesh_linear_deflection > 0.0)) {
          set_error("mesh_deflection: 'linear' must be > 0");
          return std::nullopt;
        }
      }
      if (params["mesh_deflection"]["angular"]) {
        config.mesh_angular_deflection = finite_double(params["mesh_deflection"]["angular"]);
        config.finder_config.mesh_angular_deflection = config.mesh_angular_deflection;
        if (!(config.mesh_angular_deflection > 0.0)) {
          set_error("mesh_deflection: 'angular' must be > 0");
          return std::nullopt;
        }
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
          finite_double(params["kissing"]["contact_threshold"]);
      }
      if (params["kissing"]["collision_tolerance"]) {
        config.finder_config.collision_tolerance =
          finite_double(params["kissing"]["collision_tolerance"]);
      }
      if (params["kissing"]["contact_distance_threshold"]) {
        config.finder_config.kissing_contact_distance_threshold =
          finite_double(params["kissing"]["contact_distance_threshold"]);
      }
      if (params["kissing"]["sample_density"]) {
        config.finder_config.kissing_sample_density =
          finite_double(params["kissing"]["sample_density"]);
      }
      const auto & fc = config.finder_config;
      if (!(fc.kissing_contact_threshold >= 0.0 && fc.kissing_contact_threshold <= 1.0)) {
        set_error("kissing.contact_threshold must be in [0, 1]");
        return std::nullopt;
      }
      if (fc.collision_tolerance < 0.0 || fc.kissing_contact_distance_threshold < 0.0) {
        set_error("kissing.collision_tolerance and contact_distance_threshold must be >= 0");
        return std::nullopt;
      }
      if (!(fc.kissing_sample_density > 0.0)) {
        set_error("kissing.sample_density must be > 0");
        return std::nullopt;
      }
    }

    if (params["jaw_clearance"]) {
      auto & ac = config.finder_config.jaw_clearance;
      const auto & ac_node = params["jaw_clearance"];
      if (ac_node["enabled"]) {
        ac.enabled = ac_node["enabled"].as<bool>();
      }
      if (ac_node["clearance_margin"]) {
        ac.clearance_margin = finite_double(ac_node["clearance_margin"]);
        if (ac.clearance_margin < 0.0) {
          set_error("jaw_clearance.clearance_margin must be >= 0");
          return std::nullopt;
        }
      }
    }

    if (params["shape_refiner"]) {
      auto & sr = config.finder_config.shape_refiner;
      const auto & sr_node = params["shape_refiner"];
      if (sr_node["enabled"]) {
        sr.enabled = sr_node["enabled"].as<bool>();
      }
      if (sr_node["max_cylinder_radius"]) {
        sr.max_cylinder_radius = finite_double(sr_node["max_cylinder_radius"]);
      }
      if (sr_node["max_arc_length"]) {
        sr.max_arc_length = finite_double(sr_node["max_arc_length"]);
      }
      if (sr_node["enclave_area_ratio"]) {
        sr.enclave_area_ratio = finite_double(sr_node["enclave_area_ratio"]);
      }
      if (sr_node["enclave_angle_threshold"]) {
        sr.enclave_angle_threshold = finite_double(sr_node["enclave_angle_threshold"]);
      }
      if (sr_node["max_face_area_ratio"]) {
        sr.max_face_area_ratio = finite_double(sr_node["max_face_area_ratio"]);
      }
      if (sr_node["planarity_tolerance_deg"]) {
        sr.planarity_tolerance_deg = finite_double(sr_node["planarity_tolerance_deg"]);
      }
      if (sr_node["inflection_samples"]) {
        sr.inflection_samples = sr_node["inflection_samples"].as<int>();
      }
      // Negated comparisons so NaN is rejected too.
      if (!(sr.max_cylinder_radius > 0.0)) {
        set_error("shape_refiner.max_cylinder_radius must be > 0");
        return std::nullopt;
      }
      if (!(sr.max_arc_length > 0.0) || !std::isfinite(sr.max_arc_length)) {
        set_error("shape_refiner.max_arc_length must be finite and > 0");
        return std::nullopt;
      }
      if (!(sr.enclave_area_ratio >= 0.0 && sr.enclave_area_ratio <= 1.0)) {
        set_error("shape_refiner.enclave_area_ratio must be in [0, 1]");
        return std::nullopt;
      }
      if (!(sr.enclave_angle_threshold >= 0.0 && sr.enclave_angle_threshold <= 90.0)) {
        set_error("shape_refiner.enclave_angle_threshold must be in [0, 90] degrees");
        return std::nullopt;
      }
      if (!(sr.max_face_area_ratio > 0.0 && sr.max_face_area_ratio <= 1.0)) {
        set_error("shape_refiner.max_face_area_ratio must be in (0, 1]");
        return std::nullopt;
      }
      if (!(sr.planarity_tolerance_deg > 0.0 && sr.planarity_tolerance_deg < 90.0)) {
        set_error("shape_refiner.planarity_tolerance_deg must be in (0, 90) degrees");
        return std::nullopt;
      }
      if (sr.inflection_samples < 2) {
        set_error("shape_refiner.inflection_samples must be >= 2");
        return std::nullopt;
      }
    }

    if (params["fcl"]) {
      if (params["fcl"]["enabled"]) {
        config.finder_config.use_fcl = params["fcl"]["enabled"].as<bool>();
      }
      if (params["fcl"]["triangulation_deflection"]) {
        config.finder_config.triangulation_deflection =
          finite_double(params["fcl"]["triangulation_deflection"]);
      }
      if (!config.finder_config.use_fcl) {
        set_error("fcl.enabled: false is not supported; collision checking requires FCL");
        return std::nullopt;
      }
      if (config.finder_config.triangulation_deflection <= 0.0) {
        set_error("fcl.triangulation_deflection must be > 0");
        return std::nullopt;
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
  if (node["step_path"] && node["urdf_path"]) {
    set_error("Primary must have only one of 'step_path' and 'urdf_path'");
    return false;
  }
  if (node["step_path"]) {
    config.step_path = resolve_path(node["step_path"].as<std::string>(), base_dir_);
  } else if (node["urdf_path"]) {
    config.urdf_path = resolve_path(node["urdf_path"].as<std::string>(), base_dir_);
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

  config.gripper_urdf_path = resolve_path(node["urdf_path"].as<std::string>(), base_dir_);

  if (node["max_opening"]) {
    config.gripper_max_opening = finite_double(node["max_opening"]);
    if (!(*config.gripper_max_opening > 0.0)) {
      set_error("gripper.max_opening must be > 0");
      return false;
    }
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

  bool has_ground_plane = false;
  for (const auto & item : node) {
    SecondaryConfig sec_config;
    if (!parse_secondary(item, sec_config)) {
      return false;
    }
    // GroundConstraint holds a single plane, so a second one would silently replace the first.
    if (sec_config.type == "ground_plane") {
      if (has_ground_plane) {
        set_error("At most one secondary of type 'ground_plane' is allowed");
        return false;
      }
      has_ground_plane = true;
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
    const std::string path_key = config.type + "_path";
    const std::string other_key = config.type == "step" ? "urdf_path" : "step_path";
    if (!node[path_key] || node[other_key]) {
      set_error(
        "Secondary of type '" + config.type + "' must have '" + path_key + "' (and not '" +
        other_key + "')");
      return false;
    }
    config.file_path = resolve_path(node[path_key].as<std::string>(), base_dir_);
  } else if (config.type == "box") {
    if (node["dimensions"]) {
      config.dimensions = parse_vector3(node["dimensions"]);
    } else {
      set_error("Box secondary must have 'dimensions'");
      return false;
    }
    if (!(config.dimensions.minCoeff() > 0.0)) {
      set_error("Box secondary 'dimensions' must all be > 0");
      return false;
    }
  } else if (config.type == "cylinder") {
    if (node["radius"] && node["height"]) {
      config.radius = finite_double(node["radius"]);
      config.height = finite_double(node["height"]);
    } else {
      set_error("Cylinder secondary must have 'radius' and 'height'");
      return false;
    }
    if (!(config.radius > 0.0 && config.height > 0.0)) {
      set_error("Cylinder secondary 'radius' and 'height' must be > 0");
      return false;
    }
  } else if (config.type == "ground_plane") {
    if (node["size_x"]) {
      config.size_x = finite_double(node["size_x"]);
    }
    if (node["size_y"]) {
      config.size_y = finite_double(node["size_y"]);
    }
    if (config.size_x <= 0.0 || config.size_y <= 0.0) {
      set_error("Ground plane size_x and size_y must be > 0");
      return false;
    }
    if (node["z_position"]) {
      config.z_position = finite_double(node["z_position"]);
    }
  } else {
    set_error("Unknown secondary shape type: " + config.type);
    return false;
  }

  if (node["transform"]) {
    parse_transform(node["transform"], config.translation, config.rotation);
  }

  return true;
}

bool ConfigParser::parse_exclusion_zones(const YAML::Node & node, ParsedConfig & config)
{
  if (node["sample_density"]) {
    config.finder_config.exclusion_sample_density = finite_double(node["sample_density"]);
    if (config.finder_config.exclusion_sample_density <= 0.0) {
      set_error("exclusion_zones.sample_density must be > 0");
      return false;
    }
  }

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
  circle.radius = finite_double(node["radius"]);
  circle.projection_depth = finite_double(node["projection_depth"]);

  if (node["clearance"]) {
    circle.clearance = finite_double(node["clearance"]);
  }

  if (node["id"]) {
    circle.id = node["id"].as<std::string>();
  }

  const std::string name = "Exclusion circle" + constraints::id_suffix(circle.id);
  if (circle.radius <= 0.0 || circle.projection_depth <= 0.0) {
    set_error(name + ": 'radius' and 'projection_depth' must be > 0");
    return false;
  }
  if (circle.clearance < 0.0) {
    set_error(name + ": 'clearance' must be >= 0");
    return false;
  }
  if (circle.normal.norm() < kMinDirectionNorm) {
    set_error(name + ": 'normal' must not be zero");
    return false;
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

  polygon.projection_depth = finite_double(node["projection_depth"]);

  if (node["clearance"]) {
    polygon.clearance = finite_double(node["clearance"]);
  }

  if (node["id"]) {
    polygon.id = node["id"].as<std::string>();
  }

  const std::string name = "Exclusion polygon" + constraints::id_suffix(polygon.id);
  const auto & corners = polygon.exclusion_corners;
  if (corners.size() < 3) {
    set_error(name + ": needs at least 3 corners");
    return false;
  }
  // The polygon normal is taken from corners 0-2, so they must span a plane.
  if ((corners[1] - corners[0]).cross(corners[2] - corners[0]).norm() < 1e-6) {
    set_error(name + ": corners 0-2 are collinear; start the corner list at a real corner");
    return false;
  }
  const Eigen::Vector3d normal =
    (corners[1] - corners[0]).cross(corners[2] - corners[0]).normalized();
  for (size_t i = 3; i < corners.size(); ++i) {
    if (std::abs((corners[i] - corners[0]).dot(normal)) > kPlanarityTolerance) {
      set_error(name + ": corner " + std::to_string(i) + " is off the plane of corners 0-2");
      return false;
    }
  }
  if (polygon.projection_depth <= 0.0) {
    set_error(name + ": 'projection_depth' must be > 0");
    return false;
  }
  if (polygon.clearance < 0.0) {
    set_error(name + ": 'clearance' must be >= 0");
    return false;
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
  line.exclusion_radius = finite_double(node["exclusion_radius"]);

  if (node["clearance"]) {
    line.clearance = finite_double(node["clearance"]);
  }

  if (node["id"]) {
    line.id = node["id"].as<std::string>();
  }

  const std::string name = "Exclusion line" + constraints::id_suffix(line.id);
  if (line.exclusion_radius <= 0.0) {
    set_error(name + ": 'exclusion_radius' must be > 0");
    return false;
  }
  if (line.clearance < 0.0) {
    set_error(name + ": 'clearance' must be >= 0");
    return false;
  }
  if ((line.end - line.start).norm() < kMinDirectionNorm) {
    set_error(name + ": 'start' and 'end' must differ");
    return false;
  }

  return true;
}

bool ConfigParser::parse_sampling(const YAML::Node & node, sampling::SamplingConfig & config)
{
  if (node["min_angle_deg"]) {
    config.min_angle_deg = finite_double(node["min_angle_deg"]);
  }
  if (node["max_angle_deg"]) {
    config.max_angle_deg = finite_double(node["max_angle_deg"]);
  }
  if (node["min_gripper_opening"]) {
    config.min_gripper_opening = finite_double(node["min_gripper_opening"]);
  }
  if (node["max_gripper_opening"]) {
    config.max_gripper_opening = finite_double(node["max_gripper_opening"]);
  }
  if (node["sample_density"]) {
    config.sample_density = finite_double(node["sample_density"]);
  }
  if (node["normal_sample_density"]) {
    config.normal_sample_density = finite_double(node["normal_sample_density"]);
  }
  if (node["min_normal_samples"]) {
    config.min_normal_samples = node["min_normal_samples"].as<int>();
  }
  if (node["max_normal_samples"]) {
    config.max_normal_samples = node["max_normal_samples"].as<int>();
  }
  if (node["max_lateral_deviation"]) {
    config.max_lateral_deviation = finite_double(node["max_lateral_deviation"]);
  }
  if (node["alignment_threshold"]) {
    config.alignment_threshold = finite_double(node["alignment_threshold"]);
  }

  if (!(config.sample_density > 0.0)) {
    set_error("sampling: 'sample_density' must be > 0");
    return false;
  }
  if (!(config.normal_sample_density > 0.0)) {
    set_error("sampling: 'normal_sample_density' must be > 0");
    return false;
  }
  if (config.min_normal_samples < 1 || config.min_normal_samples > config.max_normal_samples) {
    set_error("sampling: need 1 <= 'min_normal_samples' <= 'max_normal_samples'");
    return false;
  }
  if (config.min_gripper_opening < 0.0 ||
    config.min_gripper_opening > config.max_gripper_opening)
  {
    set_error("sampling: need 0 <= 'min_gripper_opening' <= 'max_gripper_opening'");
    return false;
  }
  if (!(config.min_angle_deg >= 0.0 && config.min_angle_deg <= config.max_angle_deg &&
    config.max_angle_deg <= 180.0))
  {
    set_error("sampling: need 0 <= 'min_angle_deg' <= 'max_angle_deg' <= 180");
    return false;
  }
  if (!(config.alignment_threshold >= 0.0 && config.alignment_threshold <= 1.0)) {
    set_error("sampling: 'alignment_threshold' must be in [0, 1]");
    return false;
  }
  if (config.max_lateral_deviation < 0.0) {
    set_error("sampling: 'max_lateral_deviation' must be >= 0");
    return false;
  }

  return true;
}

bool ConfigParser::parse_orientation(
  const YAML::Node & node,
  angle_finding::OrientationConfig & config)
{
  if (node["finger_length"]) {
    config.finger_length = finite_double(node["finger_length"]);
  }
  if (node["finger_radius"]) {
    config.finger_radius = finite_double(node["finger_radius"]);
  }
  if (node["max_edge_candidates"]) {
    config.max_edge_candidates = node["max_edge_candidates"].as<size_t>();
  }
  if (node["max_orientations_per_pair"]) {
    config.max_orientations_per_pair = node["max_orientations_per_pair"].as<size_t>();
  }
  if (node["dual_seed_dedup_tolerance_deg"]) {
    config.dual_seed_dedup_tolerance_deg = finite_double(node["dual_seed_dedup_tolerance_deg"]);
  }
  if (node["max_edges_per_contact"]) {
    config.max_edges_per_contact = node["max_edges_per_contact"].as<size_t>();
  }
  if (node["angle_offsets"] && node["angle_offsets"].IsSequence()) {
    config.angle_offsets.clear();
    for (const auto & offset : node["angle_offsets"]) {
      config.angle_offsets.push_back(finite_double(offset));
    }
  }
  if (node["stop_on_first_valid"]) {
    config.stop_on_first_valid = node["stop_on_first_valid"].as<bool>();
  }
  if (node["collision_tolerance"]) {
    config.collision_tolerance = finite_double(node["collision_tolerance"]);
  }
  if (node["ring_step_size"]) {
    config.ring_step_size = finite_double(node["ring_step_size"]);
  }
  if (node["angular_step_deg"]) {
    config.angular_step_deg = finite_double(node["angular_step_deg"]);
  }
  if (node["flat_detection_tolerance_m"]) {
    config.flat_detection_tolerance_m = finite_double(node["flat_detection_tolerance_m"]);
  }
  if (node["cliff_merge_tolerance_deg"]) {
    config.cliff_merge_tolerance_deg = finite_double(node["cliff_merge_tolerance_deg"]);
  }
  if (node["min_cliff_width_deg"]) {
    config.min_cliff_width_deg = finite_double(node["min_cliff_width_deg"]);
  }
  if (node["randomize_seeds"]) {
    config.randomize_seeds = node["randomize_seeds"].as<bool>();
  }
  if (node["debug_full_sweep"]) {
    config.debug_full_sweep = node["debug_full_sweep"].as<bool>();
  }
  if (node["debug_sweep_step_deg"]) {
    config.debug_sweep_step_deg = finite_double(node["debug_sweep_step_deg"]);
  }
  if (node["ray_lift_offset"]) {
    config.ray_lift_offset = finite_double(node["ray_lift_offset"]);
  }
  if (node["seed_step_deg"]) {
    config.seed_step_deg = finite_double(node["seed_step_deg"]);
  }

  const std::pair<const char *, double> positive[] = {
    {"finger_length", config.finger_length},
    {"finger_radius", config.finger_radius},
    {"ring_step_size", config.ring_step_size},
    {"angular_step_deg", config.angular_step_deg},
    {"seed_step_deg", config.seed_step_deg},
    {"debug_sweep_step_deg", config.debug_sweep_step_deg},
  };
  for (const auto & [key, value] : positive) {
    if (!(value > 0.0)) {
      set_error(std::string("orientation: '") + key + "' must be > 0");
      return false;
    }
  }
  const std::pair<const char *, double> non_negative[] = {
    {"collision_tolerance", config.collision_tolerance},
    {"flat_detection_tolerance_m", config.flat_detection_tolerance_m},
    {"cliff_merge_tolerance_deg", config.cliff_merge_tolerance_deg},
    {"min_cliff_width_deg", config.min_cliff_width_deg},
    {"ray_lift_offset", config.ray_lift_offset},
  };
  for (const auto & [key, value] : non_negative) {
    if (!(value >= 0.0)) {
      set_error(std::string("orientation: '") + key + "' must be >= 0");
      return false;
    }
  }
  if (config.finger_radius > config.finger_length) {
    set_error("orientation: 'finger_radius' must be <= 'finger_length'");
    return false;
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
    config.min_quality = finite_double(node["min_quality"]);
  }
  if (node["fail_on_skipped_constraint"]) {
    config.fail_on_skipped_constraint = node["fail_on_skipped_constraint"].as<bool>();
  }
  if (!(config.min_quality >= 0.0 && config.min_quality <= 1.0)) {
    set_error("output.min_quality must be in [0, 1]");
    return false;
  }

  return true;
}

Eigen::Vector3d ConfigParser::parse_vector3(const YAML::Node & node) const
{
  Eigen::Vector3d vec = Eigen::Vector3d::Zero();

  if (node.IsSequence() && node.size() == 3) {
    vec.x() = finite_double(node[0]);
    vec.y() = finite_double(node[1]);
    vec.z() = finite_double(node[2]);
  } else if (node.IsMap()) {
    if (node["x"]) {
      vec.x() = finite_double(node["x"]);
    }
    if (node["y"]) {
      vec.y() = finite_double(node["y"]);
    }
    if (node["z"]) {
      vec.z() = finite_double(node["z"]);
    }
  } else {
    throw std::runtime_error(
            "expected a vector [x, y, z] or a map with x/y/z keys at line " +
            std::to_string(node.Mark().line + 1));
  }

  return vec;
}

Eigen::Quaterniond ConfigParser::parse_quaternion(const YAML::Node & node) const
{
  Eigen::Quaterniond quat = Eigen::Quaterniond::Identity();

  if (node.IsSequence() && node.size() == 4) {
    quat.x() = finite_double(node[0]);
    quat.y() = finite_double(node[1]);
    quat.z() = finite_double(node[2]);
    quat.w() = finite_double(node[3]);
  } else if (node.IsMap()) {
    if (node["x"]) {
      quat.x() = finite_double(node["x"]);
    }
    if (node["y"]) {
      quat.y() = finite_double(node["y"]);
    }
    if (node["z"]) {
      quat.z() = finite_double(node["z"]);
    }
    if (node["w"]) {
      quat.w() = finite_double(node["w"]);
    }
  } else {
    throw std::runtime_error(
            "expected a quaternion [x, y, z, w] or a map with x/y/z/w keys at line " +
            std::to_string(node.Mark().line + 1));
  }

  if (quat.norm() < 1e-6) {
    throw std::runtime_error(
            "quaternion at line " + std::to_string(node.Mark().line + 1) + " has zero norm");
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

std::string ConfigParser::resolve_path(
  const std::string & path,
  const std::string & base_dir) const
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
        throw std::runtime_error(
                "could not resolve package '" + package_name + "' in '" + path + "': " + e.what());
      }
    }
  }

  if (path[0] == '/') {
    return path;
  }

  return base_dir + "/" + path;
}

void ConfigParser::set_error(const std::string & message)
{
  last_error_ = message;
}

}  // namespace io
}  // namespace hold_and_weld_gripper_sampler
