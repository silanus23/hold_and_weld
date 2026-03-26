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

class GeometryMapper {
public:
  GeometryMapper();
  ~GeometryMapper();

  Topology load_from_urdf_string(const std::string & urdf_string);

  Topology load_from_urdf_file(const std::string & urdf_path);

  Topology load_from_step(
    const std::string & step_path,
    const Eigen::Vector3d & translation,
    const Eigen::Quaterniond & rotation
  );

  Topology load_from_shape(
    const TopoDS_Shape & shape,
    const std::string & name = "shape"
  );

  int find_topology_surface_id(const TopoDS_Face & occt_face) const;
  TopoDS_Face get_occt_face(int surface_id) const;
  const TopTools_IndexedMapOfShape & get_face_map() const;

private:
  TopoDS_Shape create_shape_from_urdf_string(const std::string & urdf_string);

  TopoDS_Shape create_shape_from_urdf_file(const std::string & urdf_path);

  TopoDS_Shape create_shape_from_step(
    const std::string & step_path,
    const Eigen::Vector3d & translation,
    const Eigen::Quaterniond & rotation
  );

  Topology create_topology_from_shape(
    const TopoDS_Shape & shape,
    const std::string & link_name
  );
  TopTools_IndexedMapOfShape face_map_;
};

}  // namespace geometry
}  // namespace hold_and_weld_gripper_sampler

#endif  // HOLD_AND_WELD_GRIPPER_SAMPLER__GEOMETRY__GEOMETRY_MAPPER_HPP_
