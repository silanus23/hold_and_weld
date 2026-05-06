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

#ifndef HOLD_AND_WELD_GRIPPER_SAMPLER__IO__CONFIG_PARSER_HPP_
#define HOLD_AND_WELD_GRIPPER_SAMPLER__IO__CONFIG_PARSER_HPP_

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <yaml-cpp/yaml.h>

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "hold_and_weld_gripper_sampler/constraints/exclusion_zone_constraint.hpp"
#include "hold_and_weld_gripper_sampler/core/grasp_finder.hpp"

namespace hold_and_weld_gripper_sampler
{
namespace io
{

/**
 * @brief Parsed primary workpiece configuration
 */
struct PrimaryConfig
{
  std::string step_path;
  std::string urdf_path;
  Eigen::Vector3d translation = Eigen::Vector3d::Zero();
  Eigen::Quaterniond rotation = Eigen::Quaterniond::Identity();
};

/**
 * @brief Parsed secondary shape configuration
 */
struct SecondaryConfig
{
  std::string id;
  std::string type;  // "step", "urdf", "box", "cylinder", "ground_plane"

  std::string file_path;

  Eigen::Vector3d dimensions = Eigen::Vector3d::Zero();

  double radius = 0.0;
  double height = 0.0;

  double size_x = 2.0;
  double size_y = 2.0;
  double z_position = 0.0;

  Eigen::Vector3d translation = Eigen::Vector3d::Zero();
  Eigen::Quaterniond rotation = Eigen::Quaterniond::Identity();
};

/**
 * @brief Parsed output configuration
 */
struct OutputConfig
{
  std::string json_path = "grasps.json";
  size_t max_grasps = 0;  // 0 = all
  double min_quality = 0.0;
};

/**
 * @brief Complete parsed configuration for GraspFinder
 */
struct ParsedConfig
{
  std::string frame_id = "world";

  PrimaryConfig primary;

  std::string gripper_urdf_path;
  std::optional<double> gripper_max_opening;

  std::vector<SecondaryConfig> secondaries;

  std::vector<constraints::exclusion_circle> exclusion_circles;
  std::vector<constraints::exclusion_polygon> exclusion_polygons;
  std::vector<constraints::exclusion_line> exclusion_lines;

  double mesh_linear_deflection = 0.001;
  double mesh_angular_deflection = 0.1;

  core::GraspFinderConfig finder_config;

  OutputConfig output;
};

/**
 * @brief Parser for GraspFinder YAML configuration files
 *
 * Loads configuration from YAML files following the ROS2 parameter format.
 * Supports both full configuration files and partial overrides.
 */
class ConfigParser
{
public:
  ConfigParser() = default;
  ~ConfigParser() = default;

  /**
   * @brief Parse configuration from YAML file
   *
   * @param yaml_path Path to YAML configuration file
   * @param package_share_dir Optional package share directory for resolving relative paths
   * @return Parsed configuration, or std::nullopt on error
   */
  std::optional<ParsedConfig> parse_file(
    const std::string & yaml_path,
    const std::string & package_share_dir = "");

  /**
   * @brief Parse configuration from YAML string
   *
   * @param yaml_content YAML content as string
   * @param base_dir Base directory for resolving relative paths
   * @return Parsed configuration, or std::nullopt on error
   */
  std::optional<ParsedConfig> parse_string(
    const std::string & yaml_content,
    const std::string & base_dir = "");

  /**
   * @brief Parse configuration from YAML node
   *
   * @param root_node Root YAML node
   * @param base_dir Base directory for resolving relative paths
   * @return Parsed configuration, or std::nullopt on error
   */
  std::optional<ParsedConfig> parse_node(
    const YAML::Node & root_node,
    const std::string & base_dir = "");

  /**
   * @brief Get last error message
   *
   * @return Error message from last failed parse operation
   */
  std::string get_last_error() const {return last_error_;}

private:
  std::string last_error_;
  std::string base_dir_;

  /**
   * @brief Navigate to ros__parameters node (handles wildcard and ros__parameters: YAML formats)
   */
  YAML::Node get_parameters_node(const YAML::Node & root) const;

  /**
   * @brief Parse primary workpiece configuration
   */
  bool parse_primary(const YAML::Node & node, PrimaryConfig & config);

  /**
   * @brief Parse gripper configuration
   */
  bool parse_gripper(const YAML::Node & node, ParsedConfig & config);

  /**
   * @brief Parse secondary shapes configuration
   */
  bool parse_secondaries(const YAML::Node & node, std::vector<SecondaryConfig> & configs);

  /**
   * @brief Parse a single secondary shape
   */
  bool parse_secondary(const YAML::Node & node, SecondaryConfig & config);

  /**
   * @brief Parse exclusion zones configuration
   */
  bool parse_exclusion_zones(const YAML::Node & node, ParsedConfig & config);

  /**
   * @brief Parse exclusion circle
   */
  bool parse_exclusion_circle(
    const YAML::Node & node,
    constraints::exclusion_circle & circle);

  /**
   * @brief Parse exclusion polygon
   */
  bool parse_exclusion_polygon(
    const YAML::Node & node,
    constraints::exclusion_polygon & polygon);

  /**
   * @brief Parse exclusion line
   */
  bool parse_exclusion_line(
    const YAML::Node & node,
    constraints::exclusion_line & line);

  /**
   * @brief Parse sampling configuration
   */
  bool parse_sampling(const YAML::Node & node, sampling::SamplingConfig & config);

  /**
   * @brief Parse orientation configuration
   */
  bool parse_orientation(const YAML::Node & node, angle_finding::OrientationConfig & config);

  /**
   * @brief Parse output configuration
   */
  bool parse_output(const YAML::Node & node, OutputConfig & config);

  /**
   * @brief Parse 3D vector from YAML node
   */
  Eigen::Vector3d parse_vector3(const YAML::Node & node) const;

  /**
   * @brief Parse quaternion from YAML node
   */
  Eigen::Quaterniond parse_quaternion(const YAML::Node & node) const;

  /**
   * @brief Parse transform (translation + rotation) from YAML node
   */
  void parse_transform(
    const YAML::Node & node,
    Eigen::Vector3d & translation,
    Eigen::Quaterniond & rotation) const;

  /**
   * @brief Resolve file path (handle relative paths and package:// URLs)
   */
  std::string resolve_path(const std::string & path, const std::string & base_dir) const;

  /**
   * @brief Set error message
   */
  void set_error(const std::string & message);
};

}  // namespace io
}  // namespace hold_and_weld_gripper_sampler

#endif  // HOLD_AND_WELD_GRIPPER_SAMPLER__IO__CONFIG_PARSER_HPP_
