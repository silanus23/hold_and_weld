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

#ifndef HOLD_AND_WELD_GRIPPER_SAMPLER__IO__GRIPPER_PARSER_HPP_
#define HOLD_AND_WELD_GRIPPER_SAMPLER__IO__GRIPPER_PARSER_HPP_

#include <tinyxml2.h>
#include <memory>
#include <optional>
#include <string>
#include <utility>

#include <gp_Trsf.hxx>
#include <TopoDS_Shape.hxx>

#include "hold_and_weld_gripper_sampler/core/gripper.hpp"

namespace hold_and_weld_gripper_sampler
{
namespace io
{

/**
 * @brief Parser for gripper URDFs following the hold_and_weld convention
 *
 * Parses gripper URDF files that contain a <gripper_metadata> section
 * with explicit definitions of base link, fingers, joints, and TCP.
 *
 * Expected URDF structure:
 * @code{.xml}
 * <gripper_metadata>
 *   <base_link name="gripper_base"/>
 *   <finger finger_id="1" link="left_finger" joint="left_finger_joint"/>
 *   <finger finger_id="2" link="right_finger" joint="right_finger_joint"/>
 *   <gripper_type>parallel</gripper_type>
 *   <tcp_offset xyz="0 0 0.22" rpy="0 0 0"/>
 * </gripper_metadata>
 * @endcode
 *
 * The parser extracts:
 * - Collision geometry shapes for each component
 * - Joint axes (opening directions) from prismatic joint definitions
 * - Joint limits for min/max opening calculation
 * - TCP offset for grasp frame definition
 *
 * Finger convention, enforced (parsing throws on a violation). Vendor grippers
 * are normalised to it in their wrapper macro under
 * hold_and_weld_description/urdf/end_effectors/, never here:
 * - Both finger joints are prismatic, parented directly to the base link, and
 *   have no <mimic>
 * - Limits are [0, travel] with 0 = closed and equal travel on both fingers
 * - Axes point in the opening direction and are opposite to each other
 * - Collision is box/cylinder/sphere only; the closed fingers do not overlap
 *   along the closing axis, so each finger's innermost face is its contact face
 */
class GripperParser
{
public:
  /**
   * @brief Default constructor
   */
  GripperParser() = default;

  /**
   * @brief Virtual destructor
   */
  ~GripperParser() = default;

  /**
   * @brief Parse gripper from URDF string
   *
   * @param urdf_string Complete URDF XML content as string
   * @return ParsedGripper with all extracted information
   */
  ParsedGripper parse_from_urdf_string(const std::string & urdf_string);

  /**
   * @brief Parse gripper from URDF file
   *
   * @param urdf_path Path to URDF file
   * @return ParsedGripper with all extracted information
   */
  ParsedGripper parse_from_urdf_file(const std::string & urdf_path);

  /**
   * @brief Parse gripper from xacro file (processes xacro first)
   *
   * Runs xacro processing to expand macros before parsing.
   * Requires xacro to be installed and available in PATH.
   *
   * @param xacro_path Path to xacro file
   * @param xacro_args Optional xacro arguments (e.g., "prefix:=left_")
   * @return ParsedGripper with all extracted information
   */
  ParsedGripper parse_from_xacro_file(
    const std::string & xacro_path,
    const std::string & xacro_args = "");

private:
  /**
   * @brief Extract shape from link's collision geometry (all <collision> elements)
   *
   * @param urdf_string Complete URDF string
   * @param link_name Name of the link to extract
   * @return TopoDS_Shape (compound if multiple collision elements) representing the collision geometry
   */
  TopoDS_Shape extract_link_shape(
    const std::string & urdf_string,
    const std::string & link_name);

  TopoDS_Shape extract_link_shape(
    tinyxml2::XMLElement * robot,
    const std::string & link_name);

  /**
   * @brief Extract joint axis direction
   *
   * @param urdf_string Complete URDF string
   * @param joint_name Name of the joint
   * @return Unit vector representing joint axis
   */
  Eigen::Vector3d extract_joint_axis(
    const std::string & urdf_string,
    const std::string & joint_name);

  Eigen::Vector3d extract_joint_axis(
    tinyxml2::XMLElement * robot,
    const std::string & joint_name);

  /**
   * @brief Extract joint position limits
   *
   * @param urdf_string Complete URDF string
   * @param joint_name Name of the joint
   * @return Pair of (lower_limit, upper_limit) in meters
   */
  std::pair<double, double> extract_joint_limits(
    const std::string & urdf_string,
    const std::string & joint_name);

  std::pair<double, double> extract_joint_limits(
    tinyxml2::XMLElement * robot,
    const std::string & joint_name);

  /**
   * @brief Extract joint origin transform
   *
   * @param urdf_string Complete URDF string
   * @param joint_name Name of the joint
   * @return Transform from parent link to child link origin
   */
  gp_Trsf extract_joint_origin(
    const std::string & urdf_string,
    const std::string & joint_name);

  gp_Trsf extract_joint_origin(
    tinyxml2::XMLElement * robot,
    const std::string & joint_name);

  /**
   * @brief Check a finger joint's topology against the finger convention
   *
   * Throws if the joint has a <mimic>, or is not base_link -> finger_link.
   *
   * @param robot Root <robot> element
   * @param joint_name Name of the finger joint
   * @param base_link Name of the gripper base link (the required parent)
   * @param finger_link Name of the finger link (the required child)
   */
  void validate_finger_joint(
    tinyxml2::XMLElement * robot,
    const std::string & joint_name,
    const std::string & base_link,
    const std::string & finger_link);

  /**
   * @brief Check the parsed finger pair against the finger convention
   *
   * Throws on limits other than equal [0, travel], non-opposing axes, axes
   * pointing toward each other, or fingers that overlap along the closing axis
   * at the closed pose.
   *
   * @param gripper Gripper with finger shapes (in base frame) and axes filled in
   * @param f1_limits (lower, upper) of finger 1's joint [m]
   * @param f2_limits (lower, upper) of finger 2's joint [m]
   */
  void validate_finger_pair(
    const ParsedGripper & gripper,
    const std::pair<double, double> & f1_limits,
    const std::pair<double, double> & f2_limits);

  /**
   * @brief Parse xyz string to Vector3d
   *
   * @param xyz_str String in format "x y z"
   * @return Parsed vector
   */
  Eigen::Vector3d parse_xyz(const std::string & xyz_str);

  /**
   * @brief Parse rpy string to Vector3d
   *
   * @param rpy_str String in format "roll pitch yaw"
   * @return Parsed vector (in radians)
   */
  Eigen::Vector3d parse_rpy(const std::string & rpy_str);
};

}  // namespace io
}  // namespace hold_and_weld_gripper_sampler

#endif  // HOLD_AND_WELD_GRIPPER_SAMPLER__IO__GRIPPER_PARSER_HPP_
