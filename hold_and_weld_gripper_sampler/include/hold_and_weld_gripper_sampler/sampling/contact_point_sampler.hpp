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

#ifndef HOLD_AND_WELD_GRIPPER_SAMPLER__SAMPLING__CONTACT_POINT_SAMPLER_HPP_
#define HOLD_AND_WELD_GRIPPER_SAMPLER__SAMPLING__CONTACT_POINT_SAMPLER_HPP_

#include <optional>
#include <vector>

#include <Geom_Surface.hxx>
#include <GeomAPI_ProjectPointOnSurf.hxx>
#include <gp_Pnt.hxx>
#include <gp_Pnt2d.hxx>
#include <gp_Vec.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Wire.hxx>

#include "hold_and_weld_gripper_sampler/core/region_filter.hpp"
#include "hold_and_weld_gripper_sampler/geometry/topology.hpp"

namespace hold_and_weld_gripper_sampler
{
namespace sampling
{

/**
 * @brief Configuration for contact point sampling.
 */
struct SamplingConfig
{
  double sample_density = 0.01;          // Grid spacing on surfaces [m]
  double min_gripper_opening = 0.02;     // Minimum grip distance [m]
  double max_gripper_opening = 0.15;     // Maximum grip distance [m]
  double min_angle_deg = 160.0;          // Minimum angle between opposing normals [deg]
  double max_angle_deg = 180.0;          // Maximum angle between opposing normals [deg]
  double normal_sample_density = 1.0;    // Samples per cm² for normal validation
  double alignment_threshold = 0.95;     // Minimum dot product for grip axis / normal alignment
  double max_lateral_deviation = 0.02;   // Maximum allowed lateral offset between contact pair [m]
};

/**
 * @brief A pair of opposing surfaces that are candidates for grasping.
 */
struct SurfacePair
{
  int surface_id_1;
  int surface_id_2;
  TopoDS_Face face_1;
  TopoDS_Face face_2;
  gp_Vec normal_1;
  gp_Vec normal_2;
  double min_distance;  // Minimum distance between the two faces [m]
};

/**
 * @brief A pair of contact points on opposing surfaces with associated geometry.
 */
struct ContactPair
{
  gp_Pnt contact_1;
  gp_Pnt contact_2;
  int surface_id_1;
  int surface_id_2;
  TopoDS_Face face_1;
  TopoDS_Face face_2;
  gp_Vec normal_1;
  gp_Vec normal_2;
  double grip_distance;  // Distance between contact points [m]
};

/**
 * @brief Samples antipodal contact point pairs on opposing surfaces.
 *
 * For each valid surface pair, samples points on one surface and projects
 * them onto the opposing surface to find valid gripper contact locations.
 * Bidirectional sampling is used to maximize coverage. Spatial deduplication
 * is applied to the final set.
 */
class ContactPointSampler
{
public:
  /** @brief Constructor with optional configuration */
  explicit ContactPointSampler(const SamplingConfig & config = SamplingConfig{});

  /**
   * @brief Generate antipodal contact point pairs from valid surfaces.
   *
   * @param topology Primary shape topology
   * @param valid_surface_ids Surface IDs eligible for sampling
   * @param exclusion_areas Wires defining excluded regions per surface
   * @return Vector of valid contact pairs
   */
  std::vector<ContactPair> generate_contact_pairs(
    const geometry::Topology & topology,
    const std::vector<int> & valid_surface_ids,
    const std::vector<core::SampleArea> & exclusion_areas);

private:
  /**
   * @brief Find opposing surface pairs within gripper opening range.
   *
   * @param topology Primary shape topology
   * @param valid_surface_ids Surfaces to pair
   * @param exclusion_areas Exclusion wires for normal validation
   * @return Valid surface pairs
   */
  std::vector<SurfacePair> find_surface_pairs(
    const geometry::Topology & topology,
    const std::vector<int> & valid_surface_ids,
    const std::vector<core::SampleArea> & exclusion_areas) const;

  /**
   * @brief Sample points on a face, respecting exclusion areas.
   *
   * @param face Face to sample
   * @param surface_id Surface ID for exclusion lookup
   * @param exclusion_areas Exclusion wires
   * @return Sampled 3D points
   */
  std::vector<gp_Pnt> sample_surface(
    const TopoDS_Face & face,
    int surface_id,
    const std::vector<core::SampleArea> & exclusion_areas) const;

  /**
   * @brief Sample points on a face with no exclusions.
   *
   * @param face Face to sample
   * @return Sampled 3D points
   */
  std::vector<gp_Pnt> sample_full_face(const TopoDS_Face & face) const;

  /**
   * @brief Sample points on a face filtered by inclusion/exclusion wires.
   *
   * @param face Face to sample
   * @param wires Inclusion or exclusion wires
   * @return Sampled 3D points
   */
  std::vector<gp_Pnt> sample_with_exclusions(
    const TopoDS_Face & face,
    const std::vector<TopoDS_Wire> & wires) const;

  /**
   * @brief Check if a 2D UV point is inside a wire using ray casting.
   *
   * @param point_2d UV point to test
   * @param wire Wire to test against
   * @param face Face the wire belongs to
   * @return true if point is inside the wire
   */
  bool is_point_inside_wire(
    const gp_Pnt2d & point_2d,
    const TopoDS_Wire & wire,
    const TopoDS_Face & face) const;

  /**
   * @brief Check if a 3D point falls within any exclusion zone on a surface.
   *
   * @param point_3d Point to test
   * @param face Face the point belongs to
   * @param surface_id Surface ID for exclusion lookup
   * @param exclusion_areas Exclusion wires
   * @return true if point is in an exclusion zone
   */
  bool is_point_in_exclusion(
    const gp_Pnt & point_3d,
    const TopoDS_Face & face,
    int surface_id,
    const std::vector<core::SampleArea> & exclusion_areas) const;

  /**
   * @brief Check if a 3D point falls within the allowed sampling area on a surface.
   *
   * @param point_3d Point to test
   * @param face Face the point belongs to
   * @param surface_id Surface ID for area lookup
   * @param exclusion_areas Sample area wires
   * @return true if point is in an allowed area
   */
  bool is_point_in_allowed_area(
    const gp_Pnt & point_3d,
    const TopoDS_Face & face,
    int surface_id,
    const std::vector<core::SampleArea> & exclusion_areas) const;

  /**
   * @brief Project a contact point onto the opposing face to find the antipodal contact.
   *
   * Uses BRepExtrema_DistShapeShape to find the nearest point on face_2.
   *
   * @param contact_1 Contact point to project
   * @param face_2 Target face to project onto
   * @param opposing_contact Output contact point on face_2
   * @return true if a valid opposing contact was found
   */
  bool find_opposing_contact(
    const gp_Pnt & contact_1,
    const TopoDS_Face & face_2,
    gp_Pnt & opposing_contact) const;

  /**
   * @brief Validate that a contact pair is a direct (non-diagonal) grasp.
   *
   * Checks grip axis alignment with surface normals and lateral deviation.
   *
   * @param contact_1 First contact point
   * @param contact_2 Second contact point
   * @param face_1 First face
   * @param face_2 Second face
   * @return true if the pair is a valid direct grasp
   */
  bool is_valid_pairing(
    const gp_Pnt & contact_1,
    const gp_Pnt & contact_2,
    const TopoDS_Face & face_1,
    const TopoDS_Face & face_2) const;

  /**
   * @brief Check if two faces have antiparallel normals within the allowed region.
   *
   * Samples normals from the allowed area on each face and checks if any pair
   * satisfies the antiparallel angle constraint.
   *
   * @param face_1 First face
   * @param face_2 Second face
   * @param surface_id_1 Surface ID of first face
   * @param surface_id_2 Surface ID of second face
   * @param exclusion_areas Exclusion wires for allowed region computation
   * @param min_dot Minimum dot product (from max_angle_deg)
   * @param max_dot Maximum dot product (from min_angle_deg)
   * @return true if antiparallel normals exist in the allowed region
   */
  bool has_antiparallel_local_normals(
    const TopoDS_Face & face_1,
    const TopoDS_Face & face_2,
    int surface_id_1,
    int surface_id_2,
    const std::vector<core::SampleArea> & exclusion_areas,
    double min_dot,
    double max_dot) const;

  /**
   * @brief Sample surface normals from the allowed region on a face.
   *
   * @param face Face to sample
   * @param surf Geometric surface handle
   * @param wires Inclusion/exclusion wires defining allowed region
   * @param min_samples Minimum number of samples
   * @param max_samples Maximum number of samples
   * @param samples_per_cm2 Target sample density
   * @return Sampled normal vectors
   */
  std::vector<gp_Vec> sample_normals_from_allowed_region(
    const TopoDS_Face & face,
    const Handle(Geom_Surface) & surf,
    const std::vector<TopoDS_Wire> & wires,
    int min_samples,
    int max_samples,
    double samples_per_cm2) const;

  /**
   * @brief Remove spatially duplicate contact pairs using grid-based bucketing.
   *
   * @param pairs Input contact pairs
   * @param tolerance Grid cell size for deduplication [m]
   * @return Deduplicated contact pairs
   */
  std::vector<ContactPair> deduplicate_contact_pairs(
    const std::vector<ContactPair> & pairs,
    double tolerance) const;

  SamplingConfig config_;
};

}  // namespace sampling
}  // namespace hold_and_weld_gripper_sampler

#endif  // HOLD_AND_WELD_GRIPPER_SAMPLER__SAMPLING__CONTACT_POINT_SAMPLER_HPP_
