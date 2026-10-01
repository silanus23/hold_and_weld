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

#ifndef HOLD_AND_WELD_GRIPPER_SAMPLER__IO__URDF_COLLISION_LOADER_HPP_
#define HOLD_AND_WELD_GRIPPER_SAMPLER__IO__URDF_COLLISION_LOADER_HPP_

#include <tinyxml2.h>

#include <string>
#include <vector>

#include <gp_Trsf.hxx>
#include <TopoDS_Shape.hxx>

namespace hold_and_weld_gripper_sampler
{
namespace io
{

/**
 * @brief Collision geometry of one URDF link, placed in the URDF root frame.
 */
struct UrdfLinkCollision
{
  std::string link_name;
  /** All of the link's <collision> elements, already moved by link_to_root. */
  TopoDS_Shape shape;
  /** Chained joint <origin>s from the root link, i.e. every joint at position zero. */
  gp_Trsf link_to_root;
};

/**
 * @brief Collision geometry of a whole URDF.
 *
 * Box, cylinder and sphere are built; malformed ones throw. <mesh> and any other
 * geometry is not built and is listed in skipped instead, so each caller decides
 * whether a missing piece is fatal.
 */
struct UrdfCollisionModel
{
  /** Links with at least one built collision element. */
  std::vector<UrdfLinkCollision> links;
  /** One "<link>: <reason>" entry per collision element that was not built. */
  std::vector<std::string> skipped;
};

/**
 * @brief Build every link's collision geometry and place it by walking the joint tree.
 *
 * A link that no joint names as child is a root and sits at the identity, so several
 * unconnected links all share the root frame.
 *
 * @param urdf_string Complete URDF XML content
 * @return Placed collision geometry plus the elements that were skipped
 */
UrdfCollisionModel load_urdf_collision(const std::string & urdf_string);

/**
 * @brief Build one link's collision geometry in its own link frame.
 *
 * @param link The <link> element
 * @param skipped Receives one entry per collision element that was not built
 * @return Compound of the built elements, or a null shape if none was built
 */
TopoDS_Shape link_collision_shape(
  const tinyxml2::XMLElement * link,
  std::vector<std::string> & skipped);

/**
 * @brief Parse a URDF <origin> element; malformed xyz/rpy throws.
 *
 * @param origin The <origin> element, or nullptr for the identity
 * @return Translation then fixed-axis roll-pitch-yaw rotation, as URDF defines it
 */
gp_Trsf parse_origin(const tinyxml2::XMLElement * origin);

}  // namespace io
}  // namespace hold_and_weld_gripper_sampler

#endif  // HOLD_AND_WELD_GRIPPER_SAMPLER__IO__URDF_COLLISION_LOADER_HPP_
