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

#include <cstddef>
#include <optional>
#include <utility>
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
  double sample_density = 0.01;
  double min_gripper_opening = 0.02;
  double max_gripper_opening = 0.15;
  double min_angle_deg = 160.0;
  double max_angle_deg = 180.0;
  double normal_sample_density = 1.0;
  double alignment_threshold = 0.95;
  double max_lateral_deviation = 0.02;
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
  double grip_distance;
};

/**
 * @brief Per-run counts of discarded contact point candidates, by reason.
 *
 * Populated by every call to generate_contact_pairs; reflects the most recent call.
 */
struct RejectionStats
{
  size_t total_samples = 0;
  size_t no_opposing = 0;
  size_t exclusion = 0;
  size_t not_in_allowed_area = 0;
  size_t diagonal = 0;
  size_t internal_grip = 0;
  size_t grip_distance = 0;
  size_t duplicate = 0;
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
    const std::vector<core::SampleArea> & exclusion_areas) const;

  /**
   * @brief Rejection counters from the most recent generate_contact_pairs call.
   *
   * All counters are zero before the first call.
   */
  const RejectionStats & last_rejection_stats() const {return last_stats_;}

private:
  /**
   * @brief Outcome of validating a candidate contact pair.
   */
  enum class PairingVerdict
  {
    Valid,
    Diagonal,
    InternalGrip
  };

  /**
   * @brief Find surface pairs whose normals are antiparallel within [min_angle_deg, max_angle_deg].
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
   * @brief Sample points on a face, optionally filtered by inclusion/exclusion wires.
   *
   * Thin wrapper over sampling::sample_face_region that keeps only the point of
   * each sample. Each wire entry is a (wire, is_exclusion) pair: when
   * is_exclusion is true, points inside the wire are rejected; when false,
   * points outside are rejected. Pass an empty vector to sample the whole face.
   *
   * @param face Face to sample
   * @param wires_with_flags Wires paired with their exclusion/inclusion flag
   * @return Sampled 3D points
   */
  std::vector<gp_Pnt> sample_with_exclusions(
    const TopoDS_Face & face,
    const std::vector<std::pair<TopoDS_Wire, bool>> & wires_with_flags) const;

  /**
   * @brief Check if a 2D UV point is inside a wire using BRepClass_FaceClassifier.
   *
   * @param point_2d UV point to test
   * @param wire Wire to test against
   * @param face Face the wire belongs to
   * @return true if point is inside or on the wire; std::nullopt if the wire
   *         could not be turned into a face to classify against
   */
  std::optional<bool> is_point_inside_wire(
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
   * @return true if point is in an exclusion zone, or could not be classified
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
   * Casts rays with IntCurvesFace_ShapeIntersector: along face_1's inward
   * normal first, then outward, then toward and away from face_2's centroid.
   * The first direction that hits face_2 gives the nearest hit along it.
   *
   * @param contact_1 Contact point to project
   * @param face_1 Face contact_1 lies on; supplies the primary ray direction
   * @param face_2 Target face to project onto
   * @param opposing_contact Output contact point on face_2
   * @return true if a valid opposing contact was found
   */
  bool find_opposing_contact(
    const gp_Pnt & contact_1,
    const TopoDS_Face & face_1,
    const TopoDS_Face & face_2,
    gp_Pnt & opposing_contact) const;

  /**
   * @brief Validate that a contact pair is a direct, external (non-diagonal) grasp.
   *
   * Checks grip axis alignment with surface normals, lateral deviation, and the
   * sidedness of the grip: contacts inside a pocket or channel are antiparallel
   * just like an external grip, but a closing parallel jaw moves away from them.
   *
   * @param contact_1 First contact point
   * @param contact_2 Second contact point
   * @param face_1 First face
   * @param face_2 Second face
   * @return Valid if the pair is a direct external grasp, otherwise the rejection reason
   */
  PairingVerdict is_valid_pairing(
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
   * Each entry is a (wire, is_exclusion) pair: when is_exclusion is true,
   * points inside the wire are excluded; when false, points outside are excluded.
   *
   * Normals are evaluated per sample, so curved faces report the full range of
   * directions rather than a single face-average normal.
   *
   * @param face Face to sample
   * @param wires_with_flags Wires paired with their exclusion/inclusion flag
   * @param target_samples Number of grid points to place (caller is responsible for sizing)
   * @return Sampled normal vectors
   */
  std::vector<gp_Vec> sample_normals_from_allowed_region(
    const TopoDS_Face & face,
    const std::vector<std::pair<TopoDS_Wire, bool>> & wires_with_flags,
    int target_samples) const;

  /**
   * @brief Remove spatially duplicate contact pairs using grid-based bucketing.
   *
   * Deduplication is symmetric: a pair (A->B) and its reverse (B->A) are treated
   * as the same pair. Whichever direction is encountered first is kept.
   *
   * @param pairs Input contact pairs
   * @param tolerance Grid cell size for deduplication [m]
   * @return Deduplicated contact pairs
   */
  std::vector<ContactPair> deduplicate_contact_pairs(
    const std::vector<ContactPair> & pairs,
    double tolerance) const;

  SamplingConfig config_;
  mutable RejectionStats last_stats_;
};

}  // namespace sampling
}  // namespace hold_and_weld_gripper_sampler

#endif  // HOLD_AND_WELD_GRIPPER_SAMPLER__SAMPLING__CONTACT_POINT_SAMPLER_HPP_
