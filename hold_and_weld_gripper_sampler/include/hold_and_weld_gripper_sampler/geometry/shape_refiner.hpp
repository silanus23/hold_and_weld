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

#include <TopoDS_Shape.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Wire.hxx>
#include <TopTools_ListOfShape.hxx>
#include <TopTools_IndexedDataMapOfShapeListOfShape.hxx>

#include <BRepAdaptor_Surface.hxx>
#include <gp_Dir.hxx>

namespace hold_and_weld_gripper_sampler
{
namespace geometry
{

/**
 * @brief Refines CAD geometry for gripper sampling by removing small features
 *        and splitting large curved surfaces into manageable patches.
 *
 * The refiner processes geometry in multiple passes:
 * - PASS 1: Initial heal (fix broken geometry from STEP import)
 * - PASS 2: Calculate global surface area
 * - PASS 3: Identify enclave features (small pockets/holes)
 * - PASS 4: Remove enclaves via defeaturing
 * - PASS 5: Split large curved surfaces into patches
 * - PASS 5.5: Sanity check for large unsplit surfaces
 * - PASS 6: Final cleanup and unification
 */
class ShapeRefiner
{
public:
    /**
     * @brief Constructor
     * @param max_cylinder_radius Maximum cylinder radius before splitting (meters)
     *        Cylinders with radius exceeding this will be split radially
     * @param max_arc_length Maximum arc length for any edge before splitting (meters)
     *        Surfaces with edge arc length exceeding this will be split
     * @param enclave_area_ratio Maximum enclave area as fraction of total area (e.g., 0.005 = 0.5%)
     *        Enclaves larger than this ratio will be kept
     * @param enclave_angle_threshold Maximum angle difference for enclave suppression (degrees)
     *        Enclaves with walls at angles > threshold are kept (steep walls = real feature)
     *        Enclaves with walls at angles < threshold are removed (shallow pockets)
     */
  ShapeRefiner(
    double max_cylinder_radius,
    double max_arc_length,
    double enclave_area_ratio,
    double enclave_angle_threshold);

    /**
     * @brief Refine a shape by removing enclaves and splitting large surfaces
     * @param raw_shape Input shape (typically from STEP file)
     * @return Refined shape with enclaves removed and surfaces split into patches
     *
     * Note: If large unsplit surfaces are detected (>30% of total area), warnings
     * will be printed to stderr. This may indicate geometry that cannot be properly
     * split (gentle BSplines, lofted surfaces, offset surfaces with noise).
     */
  TopoDS_Shape refine(const TopoDS_Shape & raw_shape) const;

private:
    // Parameters
  double max_cylinder_radius_;        ///< Max cylinder radius before splitting
  double max_arc_length_;             ///< Max arc length before splitting
  double enclave_area_ratio_;         ///< Max enclave area ratio
  double enclave_angle_threshold_;    ///< Max angle for enclave suppression (degrees)

    /**
     * @brief Identify enclave features (small pockets/holes) to remove
     * @param shape Input shape
     * @param global_total_area Total surface area of the shape
     * @param kill_list Output list of faces to remove
     */
  void identify_enclave_features(
    const TopoDS_Shape & shape,
    double global_total_area,
    TopTools_ListOfShape & kill_list) const;

    /**
     * @brief Collect all faces forming an enclave (BFS from inner wire)
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
     * @brief Determine if an enclave should be suppressed
     *
     * An enclave is removed if:
     * 1. Its total area is less than enclave_area_ratio of global area, AND
     * 2. ALL its wall faces have shallow angles (< enclave_angle_threshold) to parent
     *
     * If ANY wall has steep angle (> threshold), the enclave is kept (real feature).
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
     * @brief Find inflection points on BSpline/Bezier surfaces
     *
     * Detects curvature sign changes (saddle points, transitions) by sampling
     * Gaussian curvature along parameter direction.
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
     * @brief Get split parameters for perfect cylinders
     *
     * First checks if radius exceeds max_cylinder_radius, then falls back
     * to arc length check if radius is within limits.
     *
     * @param face Cylindrical face
     * @param u_splits Output U parameter splits
     */
  void get_cylinder_splits(
    const TopoDS_Face & face,
    std::vector<double> & u_splits) const;

    /**
     * @brief Get split parameters for cones and other analytical surfaces
     *
     * Uses arc length measurement on edges to determine split locations.
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
     * @brief Check edge arc lengths and split if exceeding limit
     *
     * Measures all edges of the face, finds the longest in U and V directions,
     * and splits if they exceed max_arc_length. Handles both U and V directions
     * to support "bitten donut" and partial sphere cases.
     *
     * For surfaces without edges (closed spheres), uses analytical fallback.
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
     * @brief Check if a face is physically planar (flat)
     * @param face Face to check
     * @return true if face is planar within tolerance (1° normal deviation)
     */
  bool is_physically_planar(const TopoDS_Face & face) const;

    /**
     * @brief Calculate a safe normal vector for a face
     * @param face Face to compute normal for
     * @return Normal direction at face center (respects face orientation)
     */
  gp_Dir calculate_safe_normal(const TopoDS_Face & face) const;
};

}  // namespace geometry
}  // namespace hold_and_weld_gripper_sampler

#endif  // HOLD_AND_WELD_GRIPPER_SAMPLER__GEOMETRY__SHAPE_REFINER_HPP_
