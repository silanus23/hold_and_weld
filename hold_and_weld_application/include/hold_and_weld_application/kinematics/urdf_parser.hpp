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


#ifndef HOLD_AND_WELD_APPLICATION__KINEMATICS__URDF_PARSER_HPP_
#define HOLD_AND_WELD_APPLICATION__KINEMATICS__URDF_PARSER_HPP_

#include <urdf_parser/urdf_parser.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <memory>
#include <string>
#include <vector>

namespace hold_and_weld
{
namespace kinematics
{

/**
 * @brief Information about a single robot joint
 *
 * Contains all kinematic parameters needed for forward kinematics
 * and Jacobian calculations.
 */
struct JointInfo
{
  std::string name;
  Eigen::Isometry3d origin_transform;
  Eigen::Vector3d axis;
  bool is_revolute;
  double q_min;
  double q_max;

  /**
   * @brief Default constructor. Leaves limits at [0, 0], which KinematicsSolver and
   * URDFParser::validate_chain reject, so an unfilled JointInfo cannot pass as valid.
   */
  JointInfo()
  : name(""),
    origin_transform(Eigen::Isometry3d::Identity()),
    axis(Eigen::Vector3d::UnitZ()),
    is_revolute(true),
    q_min(0.0),
    q_max(0.0)
  {}
};

/**
 * @brief Result of URDF parsing containing separated kinematic data
 *
 * Separates actuated joints (robot DOF) from fixed tool transforms (torch).
 * This separation is critical for Jacobian calculations where only actuated
 * joints contribute columns to the Jacobian matrix.
 */
struct ParsedChain
{
  std::vector<JointInfo> actuated_joints;
  Eigen::Isometry3d tool_transform;
  std::string base_link;
  std::string tip_link;

  /**
   * @brief Get degrees of freedom
   * @return Number of actuated joints (should be 6 for welders)
   */
  size_t dof() const {return actuated_joints.size();}
};

/**
 * @brief URDF parser for 6-DOF welder robots with fixed tool attachments.
 *
 * Expects exactly 6 revolute (or continuous) joints followed by a chain of fixed joints
 * forming the tool (e.g., torch). Accumulates consecutive fixed joints into one transform.
 * Continuous joints get limits [-pi, pi].
 */
class URDFParser {
public:
  /**
   * @brief Construct a new URDFParser object
   */
  URDFParser();

  /**
   * @brief Destructor
   */
  ~URDFParser();

  /**
   * @brief Extract the joint chain from a URDF file, running xacro first for *.xacro files.
   *
   * Yields 6 actuated (revolute) joints with their local transforms, plus the accumulated
   * tool transform from fixed joints after the last actuated joint.
   *
   * @param urdf_path Path to URDF or xacro file (supports package://)
   * @param base_link Starting link name (e.g., "robot2_base_link")
   * @param tip_link End link name/TCP (e.g., "robot2_wire_tip")
   * @return ParsedChain containing actuated joints and tool transform
   */
  ParsedChain extract_joint_chain(
    const std::string & urdf_path,
    const std::string & base_link,
    const std::string & tip_link);

  /**
   * @brief Extract the joint chain from a raw URDF XML string, bypassing the filesystem.
   *
   * Designed to consume the 'robot_description' parameter directly from the ROS 2 parameter
   * server.
   *
   * @param urdf_string The raw XML string containing the URDF robot description.
   * @param base_link The name of the root link of the desired kinematic chain.
   * @param tip_link The name of the end-effector link of the desired kinematic chain.
   * @return ParsedChain A structure containing the ordered actuated joints and tool transform.
   */
  ParsedChain extract_joint_chain_from_string(
    const std::string & urdf_string,
    const std::string & base_link,
    const std::string & tip_link);

private:
  /**
   * @brief Throw std::invalid_argument if either link name is empty.
   */
  static void validate_link_names(const std::string & base_link, const std::string & tip_link);

  /**
   * @brief Build, extract, and validate the base -> tip chain of a parsed model.
   */
  static ParsedChain chain_from_model(
    const urdf::ModelInterfaceSharedPtr & model,
    const std::string & base_link,
    const std::string & tip_link);

  /**
   * @brief Resolve a path that may be a package:// URI or absolute to an absolute path.
   */
  static std::string resolve_package_path(const std::string & path);

  /**
   * @brief Load and parse a URDF file.
   */
  static urdf::ModelInterfaceSharedPtr load_urdf(const std::string & urdf_path);

  /**
   * @brief Build the link chain, ordered base to tip.
   */
  static std::vector<urdf::LinkConstSharedPtr> build_link_chain(
    const urdf::ModelInterfaceSharedPtr & model,
    const std::string & base_link,
    const std::string & tip_link);

  /**
   * @brief Extract actuated joints from the link chain, accumulating fixed joint transforms
   * into the next actuated joint's origin transform.
   */
  static std::vector<JointInfo> extract_joints_from_chain(
    const std::vector<urdf::LinkConstSharedPtr> & link_chain);

  /**
   * @brief Accumulate the fixed joints after the last actuated joint (typically the
   * welding torch) into one tool transform.
   */
  static Eigen::Isometry3d extract_tool_transform(
    const std::vector<urdf::LinkConstSharedPtr> & link_chain,
    const std::vector<JointInfo> & actuated_joints);

  /**
   * @brief Convert a URDF pose to an Eigen transform.
   */
  static Eigen::Isometry3d urdf_pose_to_eigen(const urdf::Pose & pose);

  /**
   * @brief Validate the extracted joint chain: exactly 6 revolute joints, normalized axes,
   * and finite limits with q_min < q_max.
   */
  static void validate_chain(const std::vector<JointInfo> & joints);

  /**
   * @brief Parse a raw URDF XML string into an in-memory model.
   */
  static urdf::ModelInterfaceSharedPtr load_urdf_from_string(const std::string & urdf_string);
};

}  // namespace kinematics
}  // namespace hold_and_weld

#endif  // HOLD_AND_WELD_APPLICATION__KINEMATICS__URDF_PARSER_HPP_
