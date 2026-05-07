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
     * @brief Default constructor - initializes to safe values
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
 * @brief URDF parser for 6-DOF welder robots with fixed tool attachments
 *
 * Designed specifically for welder robots with:
 * - Exactly 6 revolute joints (base -> wrist)
 * - Chain of fixed joints forming tool attachment (e.g., torch)
 *
 * Handles consecutive fixed joints correctly by accumulating transforms.
 *
 * Example URDF structure:
 * @code
 *   base_link -> [joint1...joint6] -> tool0 -> [torch joints] -> wire_tip
 *      ↓            ↓ revolute         ↓         ↓ fixed        ↓
 *   6 DOF robot                    Tool transform
 * @endcode
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
  * @brief Extract joint chain from URDF file
  *
  * Parses URDF and extracts:
  * - 6 actuated (revolute) joints with their local transforms
  * - Tool transform (accumulated fixed joints after last actuated joint)
  *
  * @param urdf_path Path to URDF file (supports package://)
  * @param base_link Starting link name (e.g., "robot2_base_link")
  * @param tip_link End link name/TCP (e.g., "robot2_wire_tip")
  * @return ParsedChain containing actuated joints and tool transform
  */
  ParsedChain extract_joint_chain(
    const std::string & urdf_path,
    const std::string & base_link,
    const std::string & tip_link);

  /**
  * @brief Extracts a kinematic chain directly from a raw URDF XML string.
  *
  * This method bypasses the filesystem and parses a URDF string natively. It builds
  * the link tree, extracts the actuated joints between the specified base and tip
  * links, and computes the fixed tool transform. It is designed to consume the
  * 'robot_description' parameter directly from the ROS 2 parameter server.
  *
  * @param urdf_string The raw XML string containing the URDF robot description.
  * @param base_link The name of the root link of the desired kinematic chain.
  * @param tip_link The name of the end-effector link of the desired kinematic chain.
  * @return ParsedChain A structure containing the ordered actuated joints and tool transform.
  * @throws std::invalid_argument If the URDF string, base_link, or tip_link is empty.
  * @throws std::runtime_error If the URDF string fails to parse or the chain is invalid.
  */
  ParsedChain extract_joint_chain_from_string(
    const std::string & urdf_string,
    const std::string & base_link,
    const std::string & tip_link);

private:
    /**
     * @brief Resolve package:// URI to absolute filesystem path
     * @param path Path string (may be package:// URI or absolute path)
     * @return Resolved absolute path
     */
  static std::string resolve_package_path(const std::string & path);

    /**
     * @brief Load and parse URDF file
     * @param urdf_path Path to URDF file
     * @return Parsed URDF model
     */
  static urdf::ModelInterfaceSharedPtr load_urdf(const std::string & urdf_path);

    /**
     * @brief Build kinematic chain from base to tip link
     * @param model Parsed URDF model
     * @param base_link Base link name
     * @param tip_link Tip link name
     * @return Ordered vector of links from base to tip
     */
  static std::vector<urdf::LinkConstSharedPtr> build_link_chain(
    const urdf::ModelInterfaceSharedPtr & model,
    const std::string & base_link,
    const std::string & tip_link);

    /**
     * @brief Extract actuated joints from link chain
     *
     * Accumulates fixed joint transforms and attaches them to the next
     * actuated joint's origin transform.
     *
     * @param link_chain Ordered vector of links
     * @return Vector of actuated joint information
     */
  static std::vector<JointInfo> extract_joints_from_chain(
    const std::vector<urdf::LinkConstSharedPtr> & link_chain);

    /**
     * @brief Extract tool transform from fixed joints after last actuated joint
     *
     * Accumulates all fixed joint transforms that appear after the last
     * actuated joint in the chain (typically the welding torch).
     *
     * @param link_chain Ordered vector of links
     * @param actuated_joints Vector of actuated joints (to find last one)
     * @return Accumulated tool transform
     */
  static Eigen::Isometry3d extract_tool_transform(
    const std::vector<urdf::LinkConstSharedPtr> & link_chain,
    const std::vector<JointInfo> & actuated_joints);

    /**
     * @brief Convert URDF pose to Eigen transform
     * @param pose URDF pose (position + quaternion)
     * @return Eigen isometry (4x4 homogeneous transform)
     */
  static Eigen::Isometry3d urdf_pose_to_eigen(const urdf::Pose & pose);

    /**
     * @brief Validate extracted joint chain
     *
     * Checks:
     * - Exactly 6 joints
     * - All joints are revolute
     * - Joint axes are normalized
     * - Joint limits are valid
     *
     * @param joints Vector of joint information to validate
     * @throws std::runtime_error if validation fails
     */
  static void validate_chain(const std::vector<JointInfo> & joints);

  /**
  * @brief Parses a raw URDF XML string into a urdf::Model object in memory.
  *
  * @param urdf_string The raw XML string containing the URDF robot description.
  * @return urdf::ModelInterfaceSharedPtr A shared pointer to the parsed URDF model.
  * @throws std::runtime_error If the URDF string is invalid or fails to parse.
  */
  static urdf::ModelInterfaceSharedPtr load_urdf_from_string(const std::string & urdf_string);
};

}  // namespace kinematics
}  // namespace hold_and_weld

#endif  // HOLD_AND_WELD_APPLICATION__KINEMATICS__URDF_PARSER_HPP_
