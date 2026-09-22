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

#ifndef HOLD_AND_WELD_GRIPPER_SAMPLER__SAMPLING__FACE_SAMPLER_HPP_
#define HOLD_AND_WELD_GRIPPER_SAMPLER__SAMPLING__FACE_SAMPLER_HPP_

#include <functional>
#include <memory>
#include <utility>
#include <vector>

#include <BRepTopAdaptor_FClass2d.hxx>
#include <gp_Pnt.hxx>
#include <gp_Pnt2d.hxx>
#include <gp_Vec.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Wire.hxx>

namespace hold_and_weld_gripper_sampler
{
namespace sampling
{

/**
 * @brief One sample point inside the allowed region of a face.
 */
struct FaceSample
{
  gp_Pnt point;
  gp_Vec normal;
  gp_Pnt2d uv;
  double area_weight{0.0};
  // UV extent of the grid cell this sample stands for; cells vary across a
  // face, so a region's boundary has to be grown per sample.
  double cell_du{0.0};
  double cell_dv{0.0};
  bool has_normal{false};
};

/**
 * @brief Where samples sit relative to the UV grid cells.
 */
enum class GridLayout
{
  kNodes,
  kCellCentres
};

/**
 * @brief How densely to walk a face.
 */
struct FaceSamplingConfig
{
  double sample_density{0.01};
  int grid_steps{0};
  GridLayout layout{GridLayout::kNodes};
  bool compute_normals{false};
  double classifier_tolerance{1e-6};
  int max_cells_per_tile{16};
};

/**
 * @brief The UV grid a sampling run actually used.
 *
 * du and dv are the widest cell on each axis; per-tile subdivision makes cells
 * narrower elsewhere, so per-cell extents live on each FaceSample instead.
 */
struct FaceSamplingGrid
{
  double du{0.0};
  double dv{0.0};
  int u_steps{0};
  int v_steps{0};
};

/**
 * @brief One axis (U or V) of a face sampling grid, at whatever resolution
 * per-tile subdivision settled on.
 */
struct SampleAxis
{
  std::vector<double> coords;
  std::vector<double> widths;
  int cells{0};
  double max_width{0.0};
};

/**
 * @brief One region wire's classifier, built once per face and reused across
 * samples so its pcurve cache pays off. Held by pointer since
 * BRepTopAdaptor_FClass2d isn't vector-reallocation-safe.
 */
struct RegionClassifier
{
  std::unique_ptr<BRepTopAdaptor_FClass2d> classifier;
  bool is_exclusion_zone{false};
};

/**
 * @brief Walk the allowed region of a face on a UV grid.
 *
 * Single implementation behind every face-region walk in the package (contact
 * point sampling, normal sampling, ground classification, contact-ratio
 * measurement); callers differ only in config and in what they do with the
 * samples. Step size is computed per UV tile from the surface's local scale
 * rather than once for the whole face, since a single face-wide estimate
 * under-samples stretched regions (e.g. near a cone apex).
 *
 * @param face Face to sample
 * @param config Grid density and what to compute
 * @param wires_with_flags Optional region restrictions, each a wire plus an
 *   is_exclusion flag: true bans the wire's interior, false bans everything
 *   outside it. A sample must satisfy every entry to be kept.
 * @param grid_out Optional output: the UV grid used for this run
 * @return Samples inside the allowed region; empty if the face has no surface
 */
std::vector<FaceSample> sample_face_region(
  const TopoDS_Face & face,
  const FaceSamplingConfig & config,
  const std::vector<std::pair<TopoDS_Wire, bool>> & wires_with_flags = {},
  FaceSamplingGrid * grid_out = nullptr);

/**
 * @brief Area-weighted fraction of samples satisfying a predicate.
 *
 * Shared measurement behind ground classification and fixture contact, which
 * differ only in the predicate. Returns 0.0 when the samples carry no area.
 *
 * @param samples Samples from sample_face_region
 * @param predicate Test applied to each sample
 * @return Fraction in [0, 1] of total sampled area where predicate holds
 */
double area_fraction(
  const std::vector<FaceSample> & samples,
  const std::function<bool(const FaceSample &)> & predicate);

/**
 * @brief Face on the surface of @p face, bounded by @p wire, for UV classification.
 *
 * The one place a region wire becomes something BRepClass_FaceClassifier can
 * test. Built on the face's untransformed surface so wires from
 * bounding_wire_in_uv keep their pcurves when the face is located (STEP
 * assemblies, the loader's transform); a wire without pcurves only works on
 * planes, where OCCT projects its 3D curves.
 *
 * @param face Face whose surface and UV domain the wire lies in
 * @param wire Closed wire in world coordinates
 * @return The bounded face, or a null face if it fails to build
 */
TopoDS_Face face_bounded_by_wire(const TopoDS_Face & face, const TopoDS_Wire & wire);

/**
 * @brief Closed wire bounding a set of samples, built in the face's UV domain.
 *
 * Turns a measured region (ground contact, fixture contact) into a SampleArea
 * wire the samplers can exclude. Built from pcurve-based edges so it lies
 * exactly on the surface; conservative for non-rectangular regions, which are
 * covered by their UV bounding box instead. Each sample is grown by half its
 * own cell, so the box covers the cells the samples stand for.
 *
 * @param face Face the samples lie on
 * @param samples Samples from sample_face_region to bound; an empty set yields
 *   a null wire
 * @return Closed wire in world coordinates, or a null wire if the region is
 *   degenerate or fails to build
 */
TopoDS_Wire bounding_wire_in_uv(
  const TopoDS_Face & face,
  const std::vector<FaceSample> & samples);

/**
 * @brief Total surface area represented by a sample set, m^2.
 *
 * With GridLayout::kCellCentres this approximates the true area of the
 * allowed region, converging to it as sample_density shrinks. Trim-boundary
 * error is first order, bounded by h * P / sqrt(2) (h = cell size, P = trim
 * perimeter); see FaceSamplerTest.TrimBoundaryErrorStaysWithinFirstOrderBound.
 * The same bound applies to area_fraction.
 */
double sampled_area(const std::vector<FaceSample> & samples);

}  // namespace sampling
}  // namespace hold_and_weld_gripper_sampler

#endif  // HOLD_AND_WELD_GRIPPER_SAMPLER__SAMPLING__FACE_SAMPLER_HPP_
