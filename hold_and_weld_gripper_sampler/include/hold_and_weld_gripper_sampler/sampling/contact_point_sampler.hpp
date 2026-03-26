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

struct SurfacePair
{
  int surface_id_1;
  int surface_id_2;
  TopoDS_Face face_1;
  TopoDS_Face face_2;
  gp_Vec normal_1;
  gp_Vec normal_2;
  double min_distance;
};

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

class ContactPointSampler
{
public:
  explicit ContactPointSampler(const SamplingConfig & config = SamplingConfig{});
  std::vector<ContactPair> generate_contact_pairs(
    const geometry::Topology & topology,
    const std::vector<int> & valid_surface_ids,
    const std::vector<core::SampleArea> & exclusion_areas);

private:
  std::vector<SurfacePair> find_surface_pairs(
    const geometry::Topology & topology,
    const std::vector<int> & valid_surface_ids,
    const std::vector<core::SampleArea> & exclusion_areas) const;

  std::vector<gp_Pnt> sample_surface(
    const TopoDS_Face & face,
    int surface_id,
    const std::vector<core::SampleArea> & exclusion_areas) const;

  std::vector<gp_Pnt> sample_full_face(const TopoDS_Face & face) const;

  std::vector<gp_Pnt> sample_with_exclusions(
    const TopoDS_Face & face,
    const std::vector<TopoDS_Wire> & wires) const;

  bool is_point_inside_wire(
    const gp_Pnt2d & point_2d,
    const TopoDS_Wire & wire,
    const TopoDS_Face & face) const;

  bool is_point_in_exclusion(
    const gp_Pnt & point_3d,
    const TopoDS_Face & face,
    int surface_id,
    const std::vector<core::SampleArea> & exclusion_areas) const;

  bool is_point_in_allowed_area(
    const gp_Pnt & point_3d,
    const TopoDS_Face & face,
    int surface_id,
    const std::vector<core::SampleArea> & exclusion_areas) const;

  bool find_opposing_contact(
    const gp_Pnt & contact_1,
    const TopoDS_Face & face_1,
    const TopoDS_Face & face_2,
    gp_Pnt & opposing_contact) const;

  bool is_valid_pairing(
    const gp_Pnt & contact_1,
    const gp_Pnt & contact_2,
    const TopoDS_Face & face_1,
    const TopoDS_Face & face_2) const;

  double compute_min_distance(
    const TopoDS_Face & face_1,
    const TopoDS_Face & face_2) const;

  std::optional<gp_Vec> get_surface_normal_at_point(
    const gp_Pnt & point,
    const TopoDS_Face & face) const;

  bool has_antiparallel_local_normals(
    const TopoDS_Face & face_1,
    const TopoDS_Face & face_2,
    int surface_id_1,
    int surface_id_2,
    const std::vector<core::SampleArea> & exclusion_areas,
    double min_dot,
    double max_dot) const;

  std::vector<gp_Vec> sample_normals_from_allowed_region(
    const TopoDS_Face & face,
    const Handle(Geom_Surface) & surf,
    const std::vector<TopoDS_Wire> & wires,
    int min_samples,
    int max_samples,
    double samples_per_cm2) const;

  std::vector<ContactPair> deduplicate_contact_pairs(
    const std::vector<ContactPair> & pairs,
    double tolerance) const;

  SamplingConfig config_;
};

}  // namespace sampling
}  // namespace hold_and_weld_gripper_sampler

#endif  // HOLD_AND_WELD_GRIPPER_SAMPLER__SAMPLING__CONTACT_POINT_SAMPLER_HPP_
