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

#ifndef HOLD_AND_WELD_GRIPPER_SAMPLER__GEOMETRY__OCCT_UTILS_HPP_
#define HOLD_AND_WELD_GRIPPER_SAMPLER__GEOMETRY__OCCT_UTILS_HPP_

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <vector>

#include <BRepBuilderAPI_Transform.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>
#include <gp_Quaternion.hxx>
#include <gp_Trsf.hxx>
#include <gp_Vec.hxx>
#include <STEPControl_Reader.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Shape.hxx>
#include <TopoDS_Wire.hxx>

#include "hold_and_weld_gripper_sampler/geometry/topology.hpp"

namespace hold_and_weld_gripper_sampler
{
namespace geometry
{

/**
 * @brief Convert OCCT point to Eigen vector
 */
Eigen::Vector3d to_eigen(const gp_Pnt & pnt);

/**
 * @brief Convert Eigen vector to OCCT point
 */
gp_Pnt to_occt_point(const Eigen::Vector3d & vec);

/**
 * @brief Convert OCCT vector to Eigen vector
 */
Eigen::Vector3d to_eigen(const gp_Vec & vec);

/**
 * @brief Convert Eigen vector to OCCT vector
 */
gp_Vec to_occt_vec(const Eigen::Vector3d & vec);

/**
 * @brief Convert OCCT direction to Eigen unit vector
 */
Eigen::Vector3d to_eigen(const gp_Dir & dir);

/**
 * @brief Create OCCT transform from translation and quaternion
 */
gp_Trsf create_transform(
  const Eigen::Vector3d & translation,
  const Eigen::Quaterniond & quaternion);

/**
 * @brief Apply transform to OCCT shape (creates new transformed shape)
 */
TopoDS_Shape apply_transform(
  const TopoDS_Shape & shape,
  const gp_Trsf & transform);

/**
 * @brief Extract surface normal at face center with orientation correction
 *
 * CRITICAL: Handles TopAbs_REVERSED faces correctly!
 */
gp_Vec extract_surface_normal(const TopoDS_Face & face);

/**
 * @brief Extract surface center point
 */
gp_Pnt extract_surface_center(const TopoDS_Face & face);

/**
 * @brief Compute dot product between two OCCT vectors
 */
double dot_product(const gp_Vec & v1, const gp_Vec & v2);

/**
 * @brief Check if two normals are approximately parallel (same direction)
 */
bool are_normals_aligned(
  const gp_Vec & normal1,
  const gp_Vec & normal2,
  double tolerance);

/**
 * @brief Check if two normals are approximately anti-parallel (opposite)
 */
bool are_normals_opposite(
  const gp_Vec & normal1,
  const gp_Vec & normal2,
  double tolerance);

/**
 * @brief Compute distance between two OCCT points
 */
double distance(const gp_Pnt & p1, const gp_Pnt & p2);

/**
 * @brief Classify edge geometry type (LINE, CIRCLE, or SPLINE)
 *
 * Uses OCCT's GeomAdaptor_Curve to determine the underlying curve type.
 * - LINE: Straight line segment
 * - CIRCLE: Circular arc
 * - SPLINE: B-spline, Bezier, ellipse, hyperbola, parabola, or other curves
 *
 * @param edge The TopoDS_Edge to classify
 * @return EdgeType classification
 */
EdgeType classify_edge(const TopoDS_Edge & edge);

/**
 * @brief Check if a face has inner holes (inner wires)
 *
 * A face with inner holes has more than one wire:
 * - Outer wire: the boundary of the surface
 * - Inner wire(s): holes cut out from the surface
 *
 * Example: A washer (flat disk with center hole) has 1 outer + 1 inner wire.
 *
 * @param face The TopoDS_Face to check
 * @return true if face has inner holes (wire count > 1), false otherwise
 */
bool has_inner_holes(const TopoDS_Face & face);

/**
 * @brief Extract corner positions from a wire
 *
 * Extracts all vertices from the wire and converts them to Eigen vectors.
 * Uses indexed map to automatically filter out duplicate vertices.
 *
 * @param wire The TopoDS_Wire to extract corners from
 * @return Vector of corner positions in world frame
 */
std::vector<Eigen::Vector3d> extract_corners_from_wire(const TopoDS_Wire & wire);

/**
 * @brief Extract translation component from OCCT transform
 *
 * @param transform The gp_Trsf to extract from
 * @return Translation as Eigen::Vector3d
 */
Eigen::Vector3d extract_translation(const gp_Trsf & transform);

/**
 * @brief Extract rotation as quaternion from OCCT transform
 *
 * @param transform The gp_Trsf to extract from
 * @return Rotation as Eigen::Quaterniond (normalized)
 */
Eigen::Quaterniond extract_quaternion(const gp_Trsf & transform);

}  // namespace geometry
}  // namespace hold_and_weld_gripper_sampler

#endif  // HOLD_AND_WELD_GRIPPER_SAMPLER__GEOMETRY__OCCT_UTILS_HPP_
