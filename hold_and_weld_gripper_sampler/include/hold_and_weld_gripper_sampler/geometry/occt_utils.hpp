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
#include <vector>

#include <BRepBuilderAPI_Transform.hxx>
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

/** @brief Convert OCCT point to Eigen vector */
Eigen::Vector3d to_eigen(const gp_Pnt & pnt);

/** @brief Convert Eigen vector to OCCT point */
gp_Pnt to_occt_point(const Eigen::Vector3d & vec);

/** @brief Convert OCCT vector to Eigen vector */
Eigen::Vector3d to_eigen(const gp_Vec & vec);

/** @brief Convert Eigen vector to OCCT vector */
gp_Vec to_occt_vec(const Eigen::Vector3d & vec);

/** @brief Convert OCCT direction to Eigen unit vector */
Eigen::Vector3d to_eigen(const gp_Dir & dir);

/**
 * @brief Convert ZYX Euler angles (roll, pitch, yaw) to a gp_Quaternion.
 *
 * Uses the aerospace (ZYX) convention: yaw applied first, then pitch, then roll.
 * Shared by geometry_mapper and gripper_parser to avoid duplication.
 */
gp_Quaternion rpy_to_quaternion(double roll, double pitch, double yaw);

/** @brief Create OCCT transform from translation and quaternion */
gp_Trsf create_transform(
  const Eigen::Vector3d & translation,
  const Eigen::Quaterniond & quaternion);

/** @brief Apply transform to OCCT shape, returns new transformed shape */
TopoDS_Shape apply_transform(
  const TopoDS_Shape & shape,
  const gp_Trsf & transform);

/**
 * @brief Extract surface normal at face center.
 *
 * Handles TopAbs_REVERSED faces correctly.
 */
gp_Vec extract_surface_normal(const TopoDS_Face & face);

/** @brief Extract surface centroid */
gp_Pnt extract_surface_center(const TopoDS_Face & face);


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
 * the normal there. Falls back to face center normal if projection fails.
 * Handles TopAbs_REVERSED faces correctly.
 *
 * @param point Query point (should lie on or near the face)
 * @param face Face to evaluate normal on
 * @return Normal vector, or std::nullopt if undefined
 */
std::optional<gp_Vec> surface_normal_at_point(const gp_Pnt & point, const TopoDS_Face & face);

}  // namespace geometry
}  // namespace hold_and_weld_gripper_sampler

#endif  // HOLD_AND_WELD_GRIPPER_SAMPLER__GEOMETRY__OCCT_UTILS_HPP_
