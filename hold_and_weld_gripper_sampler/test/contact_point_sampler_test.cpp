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

#include <gtest/gtest.h>
#include <memory>
#include <vector>
#include <cmath>
#include <algorithm>

#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeWedge.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRep_Tool.hxx>
#include <Geom_Surface.hxx>
#include <GeomAPI_ProjectPointOnSurf.hxx>
#include <gp_Vec.hxx>
#include <gp_Pnt.hxx>
#include <TopoDS_Shape.hxx>

#include "hold_and_weld_gripper_sampler/sampling/contact_point_sampler.hpp"
#include "hold_and_weld_gripper_sampler/geometry/geometry_mapper.hpp"
#include "hold_and_weld_gripper_sampler/geometry/topology.hpp"
#include "hold_and_weld_gripper_sampler/core/region_filter.hpp"

using namespace hold_and_weld_gripper_sampler::sampling;  // NOLINT
using namespace hold_and_weld_gripper_sampler::geometry;  // NOLINT
using namespace hold_and_weld_gripper_sampler::core;  // NOLINT

namespace
{

std::vector<int> get_all_surface_ids(const Topology & topology)
{
  std::vector<int> ids;
  ids.reserve(topology.num_surfaces());
  for (size_t i = 0; i < topology.num_surfaces(); i++) {
    ids.push_back(static_cast<int>(i));
  }
  return ids;
}

std::vector<int> get_surfaces_by_normal(
  const Topology & topology,
  const gp_Vec & target_normal,
  double tolerance = 0.99)
{
  std::vector<int> result;
  gp_Vec norm = target_normal.Normalized();

  const auto & all_surfaces = topology.get_all_surfaces();
  for (size_t i = 0; i < all_surfaces.size(); i++) {
    gp_Vec surf_norm = all_surfaces[i].normal.Normalized();
    double dot = std::abs(norm.Dot(surf_norm));

    if (dot >= tolerance) {
      result.push_back(static_cast<int>(i));
    }
  }

  return result;
}

}  // namespace

class ContactPointSamplerTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    mapper_ = std::make_shared<GeometryMapper>();
  }

  std::shared_ptr<GeometryMapper> mapper_;
};

// Sampler constructs with default config without throwing.
TEST_F(ContactPointSamplerTest, DefaultConstructor)
{
  ContactPointSampler sampler;
  SUCCEED();
}

// Sampler accepts a fully populated custom config.
TEST_F(ContactPointSamplerTest, CustomConfiguration)
{
  SamplingConfig config;
  config.min_angle_deg = 170.0;
  config.max_angle_deg = 180.0;
  config.min_gripper_opening = 0.03;
  config.max_gripper_opening = 0.10;
  config.sample_density = 0.005;
  config.max_lateral_deviation = 0.01;
  config.alignment_threshold = 0.98;
  config.normal_sample_density = 2.0;

  ContactPointSampler sampler(config);
  SUCCEED();
}

// Opposite faces of a grippable box produce pairs with antiparallel normals and in-range distances.
TEST_F(ContactPointSamplerTest, BasicBoxPairing)
{
  TopoDS_Shape box = BRepPrimAPI_MakeBox(0.10, 0.10, 0.04).Shape();
  Topology topology = mapper_->load_from_shape(box, "box");

  std::vector<int> all_ids = get_all_surface_ids(topology);

  SamplingConfig config;
  config.min_gripper_opening = 0.03;
  config.max_gripper_opening = 0.05;
  config.sample_density = 0.02;

  ContactPointSampler sampler(config);

  std::vector<SampleArea> no_exclusions;
  auto pairs = sampler.generate_contact_pairs(topology, all_ids, no_exclusions);

  EXPECT_GT(pairs.size(), 0);

  for (const auto & pair : pairs) {
    EXPECT_GE(pair.grip_distance, config.min_gripper_opening * 0.95);
    EXPECT_LE(pair.grip_distance, config.max_gripper_opening * 1.05);

    double dot = pair.normal_1.Dot(pair.normal_2);
    EXPECT_LE(dot, -0.9);
  }
}

// Box faces wider than max_gripper_opening must produce no pairs.
TEST_F(ContactPointSamplerTest, DistanceFilteringTooLarge)
{
  TopoDS_Shape box = BRepPrimAPI_MakeBox(0.20, 0.20, 0.20).Shape();
  Topology topology = mapper_->load_from_shape(box, "large_box");

  std::vector<int> all_ids = get_all_surface_ids(topology);

  SamplingConfig config;
  config.min_gripper_opening = 0.05;
  config.max_gripper_opening = 0.10;

  ContactPointSampler sampler(config);

  std::vector<SampleArea> no_exclusions;
  auto pairs = sampler.generate_contact_pairs(topology, all_ids, no_exclusions);

  EXPECT_EQ(pairs.size(), 0);
}

// Box faces narrower than min_gripper_opening must produce no pairs.
TEST_F(ContactPointSamplerTest, DistanceFilteringTooSmall)
{
  TopoDS_Shape box = BRepPrimAPI_MakeBox(0.01, 0.01, 0.01).Shape();
  Topology topology = mapper_->load_from_shape(box, "tiny_box");

  std::vector<int> all_ids = get_all_surface_ids(topology);

  SamplingConfig config;
  config.min_gripper_opening = 0.03;
  config.max_gripper_opening = 0.10;

  ContactPointSampler sampler(config);

  std::vector<SampleArea> no_exclusions;
  auto pairs = sampler.generate_contact_pairs(topology, all_ids, no_exclusions);

  EXPECT_EQ(pairs.size(), 0);
}

// Tight angle window near 180° accepts only near-perfectly-opposing face pairs.
TEST_F(ContactPointSamplerTest, AngleFilteringAntipodal)
{
  TopoDS_Shape box = BRepPrimAPI_MakeBox(0.05, 0.05, 0.05).Shape();
  Topology topology = mapper_->load_from_shape(box, "box");

  std::vector<int> all_ids = get_all_surface_ids(topology);

  SamplingConfig config;
  config.min_angle_deg = 179.0;
  config.max_angle_deg = 180.0;
  config.min_gripper_opening = 0.04;
  config.max_gripper_opening = 0.06;
  config.sample_density = 0.02;

  ContactPointSampler sampler(config);

  std::vector<SampleArea> no_exclusions;
  auto pairs = sampler.generate_contact_pairs(topology, all_ids, no_exclusions);

  EXPECT_GT(pairs.size(), 0);

  for (const auto & pair : pairs) {
    double dot = pair.normal_1.Normalized().Dot(pair.normal_2.Normalized());
    EXPECT_LE(dot, -0.99);
  }
}

// Perpendicular faces (90° apart) must be rejected when min_angle_deg is 160°.
TEST_F(ContactPointSamplerTest, AngleFilteringPerpendicular)
{
  TopoDS_Shape box = BRepPrimAPI_MakeBox(0.10, 0.10, 0.05).Shape();
  Topology topology = mapper_->load_from_shape(box, "box");

  std::vector<int> x_surfaces = get_surfaces_by_normal(topology, gp_Vec(1, 0, 0));
  std::vector<int> y_surfaces = get_surfaces_by_normal(topology, gp_Vec(0, 1, 0));

  if (x_surfaces.empty() || y_surfaces.empty()) {
    GTEST_SKIP() << "Could not find perpendicular surfaces";
  }

  std::vector<int> perpendicular_surfaces = {x_surfaces[0], y_surfaces[0]};

  SamplingConfig config;
  config.min_angle_deg = 160.0;
  config.max_angle_deg = 180.0;
  config.min_gripper_opening = 0.01;
  config.max_gripper_opening = 0.20;

  ContactPointSampler sampler(config);

  std::vector<SampleArea> no_exclusions;
  auto pairs = sampler.generate_contact_pairs(topology, perpendicular_surfaces, no_exclusions);

  EXPECT_EQ(pairs.size(), 0);
}

// Excluding a central region of the top face reduces the total pair count.
TEST_F(ContactPointSamplerTest, ExclusionZoneReducesPairs)
{
  TopoDS_Shape box = BRepPrimAPI_MakeBox(0.10, 0.10, 0.05).Shape();
  Topology topology = mapper_->load_from_shape(box, "box");

  std::vector<int> all_ids = get_all_surface_ids(topology);

  SamplingConfig config;
  config.min_gripper_opening = 0.04;
  config.max_gripper_opening = 0.06;
  config.sample_density = 0.02;

  ContactPointSampler sampler(config);

  std::vector<SampleArea> no_exclusions;
  auto pairs_full = sampler.generate_contact_pairs(topology, all_ids, no_exclusions);

  std::vector<int> z_surfaces = get_surfaces_by_normal(topology, gp_Vec(0, 0, 1));

  if (!z_surfaces.empty() && pairs_full.size() > 0) {
    gp_Pnt p1(0.03, 0.03, 0);
    gp_Pnt p2(0.07, 0.03, 0);
    gp_Pnt p3(0.07, 0.07, 0);
    gp_Pnt p4(0.03, 0.07, 0);

    BRepBuilderAPI_MakeWire wire_builder;
    wire_builder.Add(BRepBuilderAPI_MakeEdge(p1, p2).Edge());
    wire_builder.Add(BRepBuilderAPI_MakeEdge(p2, p3).Edge());
    wire_builder.Add(BRepBuilderAPI_MakeEdge(p3, p4).Edge());
    wire_builder.Add(BRepBuilderAPI_MakeEdge(p4, p1).Edge());

    SampleArea exclusion;
    exclusion.surface_id = z_surfaces[0];
    exclusion.wire = wire_builder.Wire();
    exclusion.wire.Reverse();

    std::vector<SampleArea> exclusions = {exclusion};
    auto pairs_excluded = sampler.generate_contact_pairs(topology, all_ids, exclusions);

    EXPECT_LT(pairs_excluded.size(), pairs_full.size());
  }
}

// Inclusion zone wire on one face: any contact point on that surface must lie within bounds.
// Note: restricting one face of a pair does not reduce total pair count because the sampler
// also drives pairs from the opposing face and projects to find the contact — the bounds
// check below is the meaningful contract here.
TEST_F(ContactPointSamplerTest, InclusionZoneRestrictsSampling)
{
  TopoDS_Shape box = BRepPrimAPI_MakeBox(0.10, 0.10, 0.05).Shape();
  Topology topology = mapper_->load_from_shape(box, "box");

  std::vector<int> all_ids = get_all_surface_ids(topology);

  SamplingConfig config;
  config.min_gripper_opening = 0.04;
  config.max_gripper_opening = 0.06;
  config.sample_density = 0.01;

  ContactPointSampler sampler(config);

  std::vector<int> z_surfaces = get_surfaces_by_normal(topology, gp_Vec(0, 0, 1));
  ASSERT_FALSE(z_surfaces.empty()) << "Box must have at least one Z-normal face";

  // Inclusion zone: 40x40 mm sub-region of the 100x100 mm top face
  gp_Pnt p1(0.03, 0.03, 0);
  gp_Pnt p2(0.07, 0.03, 0);
  gp_Pnt p3(0.07, 0.07, 0);
  gp_Pnt p4(0.03, 0.07, 0);

  BRepBuilderAPI_MakeWire wire_builder;
  wire_builder.Add(BRepBuilderAPI_MakeEdge(p1, p2).Edge());
  wire_builder.Add(BRepBuilderAPI_MakeEdge(p2, p3).Edge());
  wire_builder.Add(BRepBuilderAPI_MakeEdge(p3, p4).Edge());
  wire_builder.Add(BRepBuilderAPI_MakeEdge(p4, p1).Edge());

  SampleArea inclusion;
  inclusion.surface_id = z_surfaces[0];
  inclusion.wire = wire_builder.Wire();

  std::vector<SampleArea> inclusions = {inclusion};
  auto pairs_restricted = sampler.generate_contact_pairs(topology, all_ids, inclusions);

  // Any contact point that landed on the restricted surface must be within the inclusion bounds
  for (const auto & pair : pairs_restricted) {
    if (pair.surface_id_1 == z_surfaces[0]) {
      EXPECT_GE(pair.contact_1.X(), 0.03 - 1e-6);
      EXPECT_LE(pair.contact_1.X(), 0.07 + 1e-6);
      EXPECT_GE(pair.contact_1.Y(), 0.03 - 1e-6);
      EXPECT_LE(pair.contact_1.Y(), 0.07 + 1e-6);
    }
    if (pair.surface_id_2 == z_surfaces[0]) {
      EXPECT_GE(pair.contact_2.X(), 0.03 - 1e-6);
      EXPECT_LE(pair.contact_2.X(), 0.07 + 1e-6);
      EXPECT_GE(pair.contact_2.Y(), 0.03 - 1e-6);
      EXPECT_LE(pair.contact_2.Y(), 0.07 + 1e-6);
    }
  }
}

// Each pair's contact points lie on their surfaces and the grip axis aligns with both normals.
TEST_F(ContactPointSamplerTest, PairContactGeometryIsValid)
{
  TopoDS_Shape box = BRepPrimAPI_MakeBox(0.10, 0.10, 0.05).Shape();
  Topology topology = mapper_->load_from_shape(box, "box");

  std::vector<int> all_ids = get_all_surface_ids(topology);

  SamplingConfig config;
  config.min_gripper_opening = 0.04;
  config.max_gripper_opening = 0.06;
  config.sample_density = 0.02;
  config.alignment_threshold = 0.999;
  config.max_lateral_deviation = 0.001;

  ContactPointSampler sampler(config);

  std::vector<SampleArea> no_exclusions;
  auto pairs = sampler.generate_contact_pairs(topology, all_ids, no_exclusions);

  ASSERT_GT(pairs.size(), 0);

  for (const auto & pair : pairs) {
    // Check projection distance: contact points lie on their respective surfaces
    Handle(Geom_Surface) surf1 = BRep_Tool::Surface(pair.face_1);
    GeomAPI_ProjectPointOnSurf proj1(pair.contact_1, surf1);
    if (proj1.NbPoints() > 0) {
      EXPECT_LT(proj1.LowerDistance(), 0.001);
    }

    Handle(Geom_Surface) surf2 = BRep_Tool::Surface(pair.face_2);
    GeomAPI_ProjectPointOnSurf proj2(pair.contact_2, surf2);
    if (proj2.NbPoints() > 0) {
      EXPECT_LT(proj2.LowerDistance(), 0.001);
    }

    // Check alignment: grip axis aligns with surface normals
    gp_Vec grip_axis(pair.contact_1, pair.contact_2);
    grip_axis.Normalize();

    double align1 = std::abs(grip_axis.Dot(pair.normal_1.Normalized()));
    double align2 = std::abs(grip_axis.Dot(pair.normal_2.Normalized()));

    EXPECT_GE(align1, config.alignment_threshold * 0.99);
    EXPECT_GE(align2, config.alignment_threshold * 0.99);

    // Check grip_distance matches computed point-to-point distance
    EXPECT_GE(pair.grip_distance, config.min_gripper_opening);
    EXPECT_LE(pair.grip_distance, config.max_gripper_opening);

    double computed_distance = pair.contact_1.Distance(pair.contact_2);
    EXPECT_NEAR(pair.grip_distance, computed_distance, 1e-9);
  }
}

// Non-box wedge geometry produces pairs and all satisfy the min_angle_deg antiparallelism bound.
TEST_F(ContactPointSamplerTest, WedgeAngledSurfaces)
{
  TopoDS_Shape wedge = BRepPrimAPI_MakeWedge(0.10, 0.08, 0.06, 0.05).Shape();

  Topology topology = mapper_->load_from_shape(wedge, "wedge");

  std::vector<int> all_ids = get_all_surface_ids(topology);

  SamplingConfig config;
  config.min_gripper_opening = 0.03;
  config.max_gripper_opening = 0.08;
  config.min_angle_deg = 160.0;
  config.sample_density = 0.02;

  ContactPointSampler sampler(config);

  std::vector<SampleArea> no_exclusions;
  auto pairs = sampler.generate_contact_pairs(topology, all_ids, no_exclusions);

  EXPECT_GT(pairs.size(), 0u);

  // Every pair must satisfy the antiparallelism constraint
  const double cos_min_angle = std::cos(config.min_angle_deg * M_PI / 180.0);
  for (const auto & pair : pairs) {
    gp_Vec n1 = pair.normal_1.Normalized();
    gp_Vec n2 = pair.normal_2.Normalized();
    double dot = n1.Dot(n2);
    EXPECT_LE(dot, cos_min_angle + 1e-6);
  }
}

// Empty surface list and single surface both yield zero pairs.
TEST_F(ContactPointSamplerTest, BoundaryConditions_ZeroPairs)
{
  TopoDS_Shape box = BRepPrimAPI_MakeBox(0.10, 0.10, 0.05).Shape();
  Topology topology = mapper_->load_from_shape(box, "box");

  ContactPointSampler sampler;
  std::vector<SampleArea> no_exclusions;

  // Empty id list must produce zero pairs
  std::vector<int> empty_ids;
  auto pairs_empty = sampler.generate_contact_pairs(topology, empty_ids, no_exclusions);
  EXPECT_EQ(pairs_empty.size(), 0);

  // A single surface id cannot form any pair
  std::vector<int> single_id = {0};
  auto pairs_single = sampler.generate_contact_pairs(topology, single_id, no_exclusions);
  EXPECT_EQ(pairs_single.size(), 0);
}

// Full-face exclusions covering every surface yield zero pairs.
TEST_F(ContactPointSamplerTest, AllSurfacesExcluded)
{
  TopoDS_Shape box = BRepPrimAPI_MakeBox(0.10, 0.10, 0.05).Shape();
  Topology topology = mapper_->load_from_shape(box, "box");

  std::vector<int> all_ids = get_all_surface_ids(topology);

  SamplingConfig config;
  config.min_gripper_opening = 0.04;
  config.max_gripper_opening = 0.06;
  config.sample_density = 0.01;

  ContactPointSampler sampler(config);

  std::vector<SampleArea> exclusions;

  for (int id : all_ids) {
    gp_Pnt p1(0, 0, 0);
    gp_Pnt p2(0.10, 0, 0);
    gp_Pnt p3(0.10, 0.10, 0);
    gp_Pnt p4(0, 0.10, 0);

    BRepBuilderAPI_MakeWire wire_builder;
    wire_builder.Add(BRepBuilderAPI_MakeEdge(p1, p2).Edge());
    wire_builder.Add(BRepBuilderAPI_MakeEdge(p2, p3).Edge());
    wire_builder.Add(BRepBuilderAPI_MakeEdge(p3, p4).Edge());
    wire_builder.Add(BRepBuilderAPI_MakeEdge(p4, p1).Edge());

    SampleArea exclusion;
    exclusion.surface_id = id;
    exclusion.wire = wire_builder.Wire();
    exclusion.wire.Reverse();

    exclusions.push_back(exclusion);
  }

  auto pairs = sampler.generate_contact_pairs(topology, all_ids, exclusions);

  EXPECT_EQ(pairs.size(), 0);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
