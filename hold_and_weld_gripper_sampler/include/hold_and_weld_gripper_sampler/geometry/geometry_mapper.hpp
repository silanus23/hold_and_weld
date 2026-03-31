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

#ifndef HOLD_AND_WELD_GRIPPER_SAMPLER__GEOMETRY__GEOMETRY_MAPPER_HPP_
#define HOLD_AND_WELD_GRIPPER_SAMPLER__GEOMETRY__GEOMETRY_MAPPER_HPP_

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <string>
#include <vector>

#include <TopoDS.hxx>
#include <TopTools_IndexedMapOfShape.hxx>

#include "hold_and_weld_gripper_sampler/geometry/occt_utils.hpp"
#include "hold_and_weld_gripper_sampler/geometry/topology.hpp"

namespace hold_and_weld_gripper_sampler
{
namespace geometry
{

/**
 * @brief Loads CAD geometry and extracts topology for grasp sampling.
 *
 * Supports URDF strings, URDF files, STEP files, and raw OCCT shapes.
 * Maintains an internal face map for surface ID lookups after loading.
 */
class GeometryMapper
{
public:
  GeometryMapper();
  ~GeometryMapper();

  /**
   * @brief Load topology from a URDF XML string.
   *
   * @param urdf_string Complete URDF XML content
   * @return Extracted topology
   */
  Topology load_from_urdf_string(const std::string & urdf_string);

  /**
   * @brief Load topology from a URDF file.
   *
   * @param urdf_path Path to URDF file
   * @return Extracted topology
   */
  Topology load_from_urdf_file(const std::string & urdf_path);

  /**
   * @brief Load topology from a STEP file with transform.
   *
   * @param step_path Path to STEP file
   * @param translation Translation to apply in world frame [m]
   * @param rotation Rotation to apply as quaternion
   * @return Extracted topology
   */
  Topology load_from_step(
    const std::string & step_path,
    const Eigen::Vector3d & translation,
    const Eigen::Quaterniond & rotation
  );

  /**
   * @brief Load topology from an existing OCCT shape.
   *
   * @param shape OCCT shape to extract topology from
   * @param name Optional name tag for the shape
   * @return Extracted topology
   */
  Topology load_from_shape(
    const TopoDS_Shape & shape,
    const std::string & name = "shape"
  );

  /**
   * @brief Find the topology surface ID corresponding to an OCCT face.
   *
   * @param occt_face Face to look up
   * @return 0-indexed surface ID
   * @throws std::runtime_error if face not found in topology
   */
  int find_topology_surface_id(const TopoDS_Face & occt_face) const;

  /**
   * @brief Get the OCCT face corresponding to a topology surface ID.
   *
   * @param surface_id 0-indexed surface ID
   * @return Corresponding OCCT face
   * @throws std::out_of_range if surface_id is invalid
   */
  TopoDS_Face get_occt_face(int surface_id) const;

  /**
   * @brief Get the internal face index map.
   *
   * @return Const reference to the indexed face map
   */
  const TopTools_IndexedMapOfShape & get_face_map() const;

private:
  /**
   * @brief Parse URDF XML string and create an OCCT compound shape.
   *
   * @param urdf_string Complete URDF XML content
   * @return OCCT compound shape
   */
  TopoDS_Shape create_shape_from_urdf_string(const std::string & urdf_string);

  /**
   * @brief Load and parse a URDF file into an OCCT compound shape.
   *
   * @param urdf_path Path to URDF file
   * @return OCCT compound shape
   */
  TopoDS_Shape create_shape_from_urdf_file(const std::string & urdf_path);

  /**
   * @brief Load a STEP file and apply transform.
   *
   * @param step_path Path to STEP file
   * @param translation Translation to apply [m]
   * @param rotation Rotation to apply as quaternion
   * @return Transformed OCCT shape
   */
  TopoDS_Shape create_shape_from_step(
    const std::string & step_path,
    const Eigen::Vector3d & translation,
    const Eigen::Quaterniond & rotation
  );

  /**
   * @brief Extract topology from an OCCT shape and populate the face map.
   *
   * @param shape OCCT shape to process
   * @param link_name Name tag for the shape
   * @return Extracted topology
   */
  Topology create_topology_from_shape(
    const TopoDS_Shape & shape,
    const std::string & link_name
  );

  TopTools_IndexedMapOfShape face_map_;
};

}  // namespace geometry
}  // namespace hold_and_weld_gripper_sampler

#endif  // HOLD_AND_WELD_GRIPPER_SAMPLER__GEOMETRY__GEOMETRY_MAPPER_HPP_
