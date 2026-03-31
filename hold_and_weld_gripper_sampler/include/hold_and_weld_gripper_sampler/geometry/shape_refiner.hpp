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

#ifndef HOLD_AND_WELD_GRIPPER_SAMPLER__GEOMETRY__SHAPE_REFINER_HPP_
#define HOLD_AND_WELD_GRIPPER_SAMPLER__GEOMETRY__SHAPE_REFINER_HPP_

#include <vector>

#include <BRepAdaptor_Surface.hxx>
#include <gp_Dir.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Shape.hxx>
#include <TopoDS_Wire.hxx>
#include <TopTools_IndexedDataMapOfShapeListOfShape.hxx>
#include <TopTools_ListOfShape.hxx>

namespace hold_and_weld_gripper_sampler
{
namespace geometry
{

/**
 * @brief Refines CAD geometry for gripper sampling by removing small features
 *        and splitting large curved surfaces into manageable patches.
 *
 */
class ShapeRefiner
{
public:
  /**
   * @brief Constructor
   *
   * @param max_cylinder_radius Maximum cylinder radius before radial splitting [m]
   * @param max_arc_length Maximum edge arc length before surface splitting [m]
   * @param enclave_area_ratio Maximum enclave area as fraction of total area (e.g. 0.005 = 0.5%)
   * @param enclave_angle_threshold Maximum wall angle for enclave suppression [degrees].
   *        Enclaves with walls steeper than this are kept as real features.
   * @param max_face_area_ratio Maximum single-face area as fraction of total area before a
   *        warning is emitted and an additional edge-based split is attempted (e.g. 0.3 = 30%).
   */
  ShapeRefiner(
    double max_cylinder_radius,
    double max_arc_length,
    double enclave_area_ratio,
    double enclave_angle_threshold,
    double max_face_area_ratio = 0.3);

  /**
   * @brief Refine a shape by removing enclaves and splitting large surfaces.
   *
   * Large unsplit surfaces (>30% of total area) will trigger a WARN log.
   * This may indicate gentle BSplines, lofted surfaces, or offset surfaces
   * that cannot be automatically split.
   *
   * @param raw_shape Input shape (typically from STEP import)
   * @return Refined shape
   */
  TopoDS_Shape refine(const TopoDS_Shape & raw_shape) const;

private:
  double max_cylinder_radius_;
  double max_arc_length_;
  double enclave_area_ratio_;
  double enclave_angle_threshold_;
  double max_face_area_ratio_;

  /**
   * @brief Identify enclave features (small pockets/holes) to remove.
   *
   * @param shape Input shape
   * @param global_total_area Total surface area of the shape
   * @param kill_list Output list of faces to remove
   */
  void identify_enclave_features(
    const TopoDS_Shape & shape,
    double global_total_area,
    TopTools_ListOfShape & kill_list) const;

  /**
   * @brief Collect all faces forming an enclave via BFS from inner wire.
   *
   * @param footprint Inner wire defining the enclave boundary
   * @param parent_face Face containing the inner wire
   * @param edge_map Edge-to-face adjacency map
   * @param enclave_faces Output list of enclave faces
   */
  void collect_enclave_faces(
    const TopoDS_Wire & footprint,
    const TopoDS_Face & parent_face,
    const TopTools_IndexedDataMapOfShapeListOfShape & edge_map,
    TopTools_ListOfShape & enclave_faces) const;

  /**
   * @brief Determine if an enclave should be suppressed.
   *
   * Suppressed if:
   * 1. Total area < enclave_area_ratio of global area, AND
   * 2. ALL wall faces have shallow angles (< enclave_angle_threshold) to parent.
   *
   * If ANY wall is steep, the enclave is kept as a real feature.
   *
   * @param parent_face Face containing the enclave
   * @param enclave_faces Faces forming the enclave
   * @param global_total_area Total surface area
   * @return true if enclave should be removed
   */
  bool should_suppress_enclave(
    const TopoDS_Face & parent_face,
    const TopTools_ListOfShape & enclave_faces,
    double global_total_area) const;

  /**
   * @brief Find inflection points on BSpline/Bezier surfaces.
   *
   * Detects curvature sign changes by sampling Gaussian curvature
   * along the specified parameter direction.
   *
   * @param surface Surface to analyze
   * @param scan_u true to scan U direction, false for V
   * @param splits Output parameter values for splits
   */
  void find_inflections(
    const BRepAdaptor_Surface & surface,
    bool scan_u,
    std::vector<double> & splits) const;

  /**
   * @brief Get split parameters for perfect cylinders.
   *
   * Checks radius against max_cylinder_radius first, then falls back
   * to arc length check.
   *
   * @param face Cylindrical face
   * @param u_splits Output U parameter splits
   */
  void get_cylinder_splits(
    const TopoDS_Face & face,
    std::vector<double> & u_splits) const;

  /**
   * @brief Get split parameters for cones and other analytical surfaces.
   *
   * @param face Face to split
   * @param u_splits Output U parameter splits
   * @param v_splits Output V parameter splits
   */
  void get_analytical_splits(
    const TopoDS_Face & face,
    std::vector<double> & u_splits,
    std::vector<double> & v_splits) const;

  /**
   * @brief Check edge arc lengths and compute split parameters if limit exceeded.
   *
   * Handles both U and V directions to support partial spheres and bitten donuts.
   * Falls back to analytical estimation for surfaces without edges (closed spheres).
   *
   * @param face Face to analyze
   * @param u_splits Output U parameter splits
   * @param v_splits Output V parameter splits
   */
  void check_edge_arc_lengths(
    const TopoDS_Face & face,
    std::vector<double> & u_splits,
    std::vector<double> & v_splits) const;

  /**
   * @brief Check if a face is physically planar within 1° normal deviation.
   *
   * @param face Face to check
   * @return true if face is planar
   */
  bool is_physically_planar(const TopoDS_Face & face) const;

  /**
   * @brief Calculate a safe normal vector for a face at its center.
   *
   * Respects face orientation.
   *
   * @param face Face to compute normal for
   * @return Normal direction
   */
  gp_Dir calculate_safe_normal(const TopoDS_Face & face) const;
};

}  // namespace geometry
}  // namespace hold_and_weld_gripper_sampler

#endif  // HOLD_AND_WELD_GRIPPER_SAMPLER__GEOMETRY__SHAPE_REFINER_HPP_
