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

#include <optional>
#include <string>
#include <vector>

#include <BRepBuilderAPI_Transform.hxx>
#include <IMeshTools_Parameters.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>
#include <gp_Quaternion.hxx>
#include <gp_Trsf.hxx>
#include <gp_Vec.hxx>
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
 *
 * @param pnt Point to convert
 * @return Same coordinates as an Eigen vector
 */
Eigen::Vector3d to_eigen(const gp_Pnt & pnt);

/**
 * @brief Convert Eigen vector to OCCT point
 *
 * @param vec Coordinates to convert
 * @return Same coordinates as a gp_Pnt
 */
gp_Pnt to_occt_point(const Eigen::Vector3d & vec);

/**
 * @brief Convert OCCT vector to Eigen vector
 *
 * @param vec Vector to convert
 * @return Same components as an Eigen vector
 */
Eigen::Vector3d to_eigen(const gp_Vec & vec);

/**
 * @brief Convert Eigen vector to OCCT vector
 *
 * @param vec Vector to convert
 * @return Same components as a gp_Vec
 */
gp_Vec to_occt_vec(const Eigen::Vector3d & vec);

/**
 * @brief Convert OCCT direction to Eigen unit vector
 *
 * @param dir Direction to convert
 * @return Unit vector along dir
 */
Eigen::Vector3d to_eigen(const gp_Dir & dir);

/**
 * @brief Convert ZYX Euler angles (roll, pitch, yaw) to a gp_Quaternion.
 *
 * Uses the aerospace (ZYX) convention: yaw applied first, then pitch, then roll.
 *
 * @param roll Rotation about X [rad]
 * @param pitch Rotation about Y [rad]
 * @param yaw Rotation about Z [rad]
 * @return The combined rotation
 */
gp_Quaternion rpy_to_quaternion(double roll, double pitch, double yaw);

/**
 * @brief Create OCCT transform from translation and quaternion
 *
 * @param translation Translation [m]
 * @param quaternion Rotation
 * @return Rotation followed by translation
 */
gp_Trsf create_transform(
  const Eigen::Vector3d & translation,
  const Eigen::Quaterniond & quaternion);

/**
 * @brief Apply transform to OCCT shape
 *
 * @param shape Shape to transform
 * @param transform Transform to apply
 * @return New transformed shape
 */
TopoDS_Shape apply_transform(
  const TopoDS_Shape & shape,
  const gp_Trsf & transform);

/**
 * @brief Meshing parameters shared by every collision mesh (FCL and Embree)
 *
 * Both checkers triangulate the same shapes in place, so they must agree on the
 * parameters or whichever runs second sees the other's mesh.
 *
 * @param linear_deflection Chord-height tolerance on face boundaries [m]
 * @return Parameters with the interior deflection at 10x linear_deflection
 */
IMeshTools_Parameters collision_mesh_parameters(double linear_deflection);

/**
 * @brief Outward unit normal at the middle of a face's UV bounding box.
 *
 * Retries 10% into the UV box when the middle lands on a pole (sphere apex,
 * cone tip). On a trimmed or holed face the middle can lie outside the face,
 * so this represents faces whose normal barely varies.
 *
 * @param face Face to evaluate
 * @return The normal, or std::nullopt if it is undefined at both points
 */
std::optional<gp_Vec> face_centre_normal(const TopoDS_Face & face);

/**
 * @brief Extract surface normal at face center.
 *
 * Handles TopAbs_REVERSED faces correctly.
 *
 * @param face Face to evaluate
 * @return Outward unit normal at the face's UV centre
 */
gp_Vec extract_surface_normal(const TopoDS_Face & face);

/**
 * @brief Extract surface centroid
 *
 * @param face Face to evaluate
 * @return Area centroid of the face
 */
gp_Pnt extract_surface_center(const TopoDS_Face & face);

/**
 * @brief Validate a shape's BRep topology, throwing with a defect summary if invalid.
 *
 * Every geometry entry point (STEP import today) should call this right after
 * the shape leaves the reader/transform and before it reaches topology
 * extraction or sampling: a malformed import (self-intersecting wire,
 * unorientable face) otherwise surfaces much later, as an opaque OCCT
 * exception or a silently wrong result, far from the file that caused it.
 *
 * @param shape Shape to validate
 * @param context Label identifying the source in the error message (e.g. the
 *   file path being loaded)
 */
void validate_shape_or_throw(const TopoDS_Shape & shape, const std::string & context);


/**
 * @brief Check if a face has inner holes (more than one wire)
 *
 * Example: a washer has one outer wire and one inner wire.
 *
 * @param face Face to check
 * @return true if face has inner holes
 */
bool has_inner_holes(const TopoDS_Face & face);

/**
 * @brief Extract corner positions from a wire
 *
 * Uses indexed map to filter duplicate vertices.
 *
 * @param wire Wire to extract corners from
 * @return Corner positions in world frame
 */
std::vector<Eigen::Vector3d> extract_corners_from_wire(const TopoDS_Wire & wire);

/**
 * @brief Extract translation component from OCCT transform
 *
 * @param transform Transform to extract from
 * @return Translation as Eigen::Vector3d
 */
Eigen::Vector3d extract_translation(const gp_Trsf & transform);

/**
 * @brief Extract rotation as quaternion from OCCT transform
 *
 * @param transform Transform to extract from
 * @return Normalized rotation as Eigen::Quaterniond
 */
Eigen::Quaterniond extract_quaternion(const gp_Trsf & transform);

/**
 * @brief Compute minimum distance between two faces
 *
 * Uses BRepExtrema_DistShapeShape for exact computation.
 * Returns std::numeric_limits<double>::max() on failure.
 *
 * @param face_1 First face
 * @param face_2 Second face
 * @return Minimum distance in meters
 */
double face_min_distance(const TopoDS_Face & face_1, const TopoDS_Face & face_2);

/**
 * @brief Compute surface normal at a point on a face
 *
 * Projects the point onto the surface to find UV parameters, then evaluates
 * the normal there. Handles TopAbs_REVERSED faces correctly.
 *
 * @param point Query point (should lie on or near the face)
 * @param face Face to evaluate normal on
 * @return Normal vector, or std::nullopt if the face is null, the projection
 *         fails, or the normal is undefined there (e.g. at a pole)
 */
std::optional<gp_Vec> surface_normal_at_point(const gp_Pnt & point, const TopoDS_Face & face);

}  // namespace geometry
}  // namespace hold_and_weld_gripper_sampler

#endif  // HOLD_AND_WELD_GRIPPER_SAMPLER__GEOMETRY__OCCT_UTILS_HPP_
