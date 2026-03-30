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
#include <set>

#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRepPrimAPI_MakeWedge.hxx>
#include <BRepPrimAPI_MakePrism.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakePolygon.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRep_Tool.hxx>
#include <Geom_Surface.hxx>
#include <GeomAPI_ProjectPointOnSurf.hxx>
#include <gp_Vec.hxx>
#include <gp_Pnt.hxx>
#include <gp_Ax2.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Shape.hxx>
#include <TopoDS_Wire.hxx>

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

TEST_F(ContactPointSamplerTest, DefaultConstructor)
{
  ContactPointSampler sampler;
  SUCCEED();
}

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

TEST_F(ContactPointSamplerTest, MultipleValidPairings)
{
  TopoDS_Shape box = BRepPrimAPI_MakeBox(0.08, 0.06, 0.04).Shape();
  Topology topology = mapper_->load_from_shape(box, "thin_box");

  std::vector<int> all_ids = get_all_surface_ids(topology);

  SamplingConfig config;
  config.min_gripper_opening = 0.03;
  config.max_gripper_opening = 0.09;
  config.sample_density = 0.02;
  config.min_angle_deg = 175.0;

  ContactPointSampler sampler(config);

  std::vector<SampleArea> no_exclusions;
  auto pairs = sampler.generate_contact_pairs(topology, all_ids, no_exclusions);

  EXPECT_GT(pairs.size(), 0);

  std::set<std::pair<int, int>> unique_pairs;
  for (const auto & pair : pairs) {
    int id1 = std::min(pair.surface_id_1, pair.surface_id_2);
    int id2 = std::max(pair.surface_id_1, pair.surface_id_2);
    unique_pairs.insert({id1, id2});
  }

  EXPECT_GE(unique_pairs.size(), 3);
}

TEST_F(ContactPointSamplerTest, BidirectionalSamplingGeneratesMorePairs)
{
  TopoDS_Shape box = BRepPrimAPI_MakeBox(0.10, 0.10, 0.05).Shape();
  Topology topology = mapper_->load_from_shape(box, "box");

  std::vector<int> all_ids = get_all_surface_ids(topology);

  SamplingConfig config;
  config.min_gripper_opening = 0.04;
  config.max_gripper_opening = 0.06;
  config.sample_density = 0.03;

  ContactPointSampler sampler(config);

  std::vector<SampleArea> no_exclusions;
  auto pairs = sampler.generate_contact_pairs(topology, all_ids, no_exclusions);

  EXPECT_GT(pairs.size(), 10);
}

TEST_F(ContactPointSamplerTest, FullFaceSampling)
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
  auto pairs = sampler.generate_contact_pairs(topology, all_ids, no_exclusions);

  EXPECT_GT(pairs.size(), 0);
}

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

  // Baseline: sample with no restrictions
  std::vector<SampleArea> no_restrictions;
  auto pairs_full = sampler.generate_contact_pairs(topology, all_ids, no_restrictions);

  std::vector<int> z_surfaces = get_surfaces_by_normal(topology, gp_Vec(0, 0, 1));

  // The box must have faces with a Z normal — fail fast if topology is broken
  ASSERT_FALSE(z_surfaces.empty()) << "Box must have at least one Z-normal face";

  // Inclusion zone: only a 40x40 mm sub-region of the 100x100 mm top face
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

  // The inclusion zone covers only (40/100)^2 = 16% of the face area, so the
  // restricted run must yield strictly fewer contact pairs than the full run.
  EXPECT_LT(pairs_restricted.size(), pairs_full.size())
    << "Inclusion zone covering 16% of the top face must reduce contact pair count";

  // Every contact_1 from the Z-surface must land inside the inclusion wire bounds
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

TEST_F(ContactPointSamplerTest, MultipleExclusionZones)
{
  TopoDS_Shape box = BRepPrimAPI_MakeBox(0.10, 0.10, 0.05).Shape();
  Topology topology = mapper_->load_from_shape(box, "box");

  std::vector<int> all_ids = get_all_surface_ids(topology);

  SamplingConfig config;
  config.min_gripper_opening = 0.04;
  config.max_gripper_opening = 0.06;
  config.sample_density = 0.01;

  ContactPointSampler sampler(config);

  std::vector<SampleArea> no_exclusions;
  auto pairs_full = sampler.generate_contact_pairs(topology, all_ids, no_exclusions);

  std::vector<int> z_surfaces = get_surfaces_by_normal(topology, gp_Vec(0, 0, 1));

  if (!z_surfaces.empty()) {
    gp_Pnt p1_a(0.01, 0.01, 0);
    gp_Pnt p2_a(0.03, 0.01, 0);
    gp_Pnt p3_a(0.03, 0.03, 0);
    gp_Pnt p4_a(0.01, 0.03, 0);

    BRepBuilderAPI_MakeWire wire_builder_1;
    wire_builder_1.Add(BRepBuilderAPI_MakeEdge(p1_a, p2_a).Edge());
    wire_builder_1.Add(BRepBuilderAPI_MakeEdge(p2_a, p3_a).Edge());
    wire_builder_1.Add(BRepBuilderAPI_MakeEdge(p3_a, p4_a).Edge());
    wire_builder_1.Add(BRepBuilderAPI_MakeEdge(p4_a, p1_a).Edge());

    SampleArea exclusion1;
    exclusion1.surface_id = z_surfaces[0];
    exclusion1.wire = wire_builder_1.Wire();
    exclusion1.wire.Reverse();

    gp_Pnt p1_b(0.06, 0.06, 0);
    gp_Pnt p2_b(0.09, 0.06, 0);
    gp_Pnt p3_b(0.09, 0.09, 0);
    gp_Pnt p4_b(0.06, 0.09, 0);

    BRepBuilderAPI_MakeWire wire_builder_2;
    wire_builder_2.Add(BRepBuilderAPI_MakeEdge(p1_b, p2_b).Edge());
    wire_builder_2.Add(BRepBuilderAPI_MakeEdge(p2_b, p3_b).Edge());
    wire_builder_2.Add(BRepBuilderAPI_MakeEdge(p3_b, p4_b).Edge());
    wire_builder_2.Add(BRepBuilderAPI_MakeEdge(p4_b, p1_b).Edge());

    SampleArea exclusion2;
    exclusion2.surface_id = z_surfaces[0];
    exclusion2.wire = wire_builder_2.Wire();
    exclusion2.wire.Reverse();

    std::vector<SampleArea> exclusions = {exclusion1, exclusion2};
    auto pairs_excluded = sampler.generate_contact_pairs(topology, all_ids, exclusions);

    EXPECT_LE(pairs_excluded.size(), pairs_full.size());
  }
}

TEST_F(ContactPointSamplerTest, OpposingContactProjection)
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
  auto pairs = sampler.generate_contact_pairs(topology, all_ids, no_exclusions);

  ASSERT_GT(pairs.size(), 0);

  for (const auto & pair : pairs) {
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
  }
}

TEST_F(ContactPointSamplerTest, AlignmentValidation)
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

  for (const auto & pair : pairs) {
    gp_Vec grip_axis(pair.contact_1, pair.contact_2);
    grip_axis.Normalize();

    double align1 = std::abs(grip_axis.Dot(pair.normal_1.Normalized()));
    double align2 = std::abs(grip_axis.Dot(pair.normal_2.Normalized()));

    EXPECT_GE(align1, config.alignment_threshold * 0.99);
    EXPECT_GE(align2, config.alignment_threshold * 0.99);
  }
}

TEST_F(ContactPointSamplerTest, GripDistanceRevalidation)
{
  TopoDS_Shape box = BRepPrimAPI_MakeBox(0.10, 0.10, 0.045).Shape();
  Topology topology = mapper_->load_from_shape(box, "box");

  std::vector<int> all_ids = get_all_surface_ids(topology);

  SamplingConfig config;
  config.min_gripper_opening = 0.044;
  config.max_gripper_opening = 0.046;
  config.sample_density = 0.02;

  ContactPointSampler sampler(config);

  std::vector<SampleArea> no_exclusions;
  auto pairs = sampler.generate_contact_pairs(topology, all_ids, no_exclusions);

  for (const auto & pair : pairs) {
    EXPECT_GE(pair.grip_distance, config.min_gripper_opening);
    EXPECT_LE(pair.grip_distance, config.max_gripper_opening);

    double computed_distance = pair.contact_1.Distance(pair.contact_2);
    EXPECT_NEAR(pair.grip_distance, computed_distance, 1e-9);
  }
}

TEST_F(ContactPointSamplerTest, CylinderCircularFaceUVBounds)
{
  gp_Ax2 axis(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1));
  TopoDS_Shape cylinder = BRepPrimAPI_MakeCylinder(axis, 0.05, 0.04).Shape();

  Topology topology = mapper_->load_from_shape(cylinder, "cylinder");

  EXPECT_EQ(topology.num_surfaces(), 3);

  std::vector<int> all_ids = get_all_surface_ids(topology);

  SamplingConfig config;
  config.min_gripper_opening = 0.03;
  config.max_gripper_opening = 0.05;
  config.sample_density = 0.005;

  ContactPointSampler sampler(config);

  std::vector<SampleArea> no_exclusions;
  auto pairs = sampler.generate_contact_pairs(topology, all_ids, no_exclusions);

  for (const auto & pair : pairs) {
    EXPECT_GE(pair.contact_1.Z(), -0.001);
    EXPECT_LE(pair.contact_1.Z(), 0.041);
    EXPECT_GE(pair.contact_2.Z(), -0.001);
    EXPECT_LE(pair.contact_2.Z(), 0.041);

    gp_Vec n1 = pair.normal_1.Normalized();
    gp_Vec n2 = pair.normal_2.Normalized();
    double z_align_1 = std::abs(n1.Dot(gp_Vec(0, 0, 1)));
    double z_align_2 = std::abs(n2.Dot(gp_Vec(0, 0, 1)));
    EXPECT_GT(z_align_1, 0.99);
    EXPECT_GT(z_align_2, 0.99);

    EXPECT_NEAR(pair.grip_distance, 0.04, 0.002);
  }
}

TEST_F(ContactPointSamplerTest, TriangularPrismNonRectangularUV)
{
  gp_Pnt p1(0, 0, 0);
  gp_Pnt p2(0.10, 0, 0);
  gp_Pnt p3(0.05, 0.08, 0);

  BRepBuilderAPI_MakePolygon polygon;
  polygon.Add(p1);
  polygon.Add(p2);
  polygon.Add(p3);
  polygon.Close();

  BRepBuilderAPI_MakeFace face_maker(polygon.Wire());
  TopoDS_Face triangle_face = face_maker.Face();

  gp_Vec extrusion_vec(0, 0, 0.05);
  TopoDS_Shape prism = BRepPrimAPI_MakePrism(triangle_face, extrusion_vec).Shape();

  Topology topology = mapper_->load_from_shape(prism, "triangular_prism");

  EXPECT_EQ(topology.num_surfaces(), 5);

  std::vector<int> all_ids = get_all_surface_ids(topology);

  SamplingConfig config;
  config.min_gripper_opening = 0.04;
  config.max_gripper_opening = 0.06;
  config.sample_density = 0.01;

  ContactPointSampler sampler(config);

  std::vector<SampleArea> no_exclusions;
  auto pairs = sampler.generate_contact_pairs(topology, all_ids, no_exclusions);

  EXPECT_GT(pairs.size(), 0);

  bool found_z_aligned_pair = false;
  for (const auto & pair : pairs) {
    gp_Vec n1 = pair.normal_1.Normalized();
    gp_Vec n2 = pair.normal_2.Normalized();
    double z_align_1 = std::abs(n1.Dot(gp_Vec(0, 0, 1)));
    double z_align_2 = std::abs(n2.Dot(gp_Vec(0, 0, 1)));

    if (z_align_1 > 0.99 && z_align_2 > 0.99) {
      found_z_aligned_pair = true;
      EXPECT_NEAR(pair.grip_distance, 0.05, 0.005);
    }
  }
  EXPECT_TRUE(found_z_aligned_pair);
}

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

  // The sampler must produce at least one contact pair on this wedge — the two
  // large planar faces are nearly parallel and separated by ~0.05 m which is
  // within [min_gripper_opening, max_gripper_opening].
  EXPECT_GT(pairs.size(), 0u) << "Expected at least one contact pair on the wedge";

  // Every returned pair must satisfy the antiparallelism constraint:
  // dot(n1, n2) <= cos(min_angle_deg) — for 160° that is cos(160°) ≈ -0.940.
  const double cos_min_angle = std::cos(config.min_angle_deg * M_PI / 180.0);
  for (const auto & pair : pairs) {
    gp_Vec n1 = pair.normal_1.Normalized();
    gp_Vec n2 = pair.normal_2.Normalized();
    double dot = n1.Dot(n2);
    EXPECT_LE(dot, cos_min_angle + 1e-6)
      << "Pair normals dot=" << dot
      << " violates min_angle_deg=" << config.min_angle_deg;
  }
}

TEST_F(ContactPointSamplerTest, SamplingDensityVariation)
{
  TopoDS_Shape box = BRepPrimAPI_MakeBox(0.10, 0.10, 0.05).Shape();
  Topology topology = mapper_->load_from_shape(box, "box");

  std::vector<int> all_ids = get_all_surface_ids(topology);

  SamplingConfig config_coarse;
  config_coarse.min_gripper_opening = 0.04;
  config_coarse.max_gripper_opening = 0.06;
  config_coarse.sample_density = 0.05;

  ContactPointSampler sampler_coarse(config_coarse);

  std::vector<SampleArea> no_exclusions;
  auto pairs_coarse = sampler_coarse.generate_contact_pairs(topology, all_ids, no_exclusions);

  SamplingConfig config_fine;
  config_fine.min_gripper_opening = 0.04;
  config_fine.max_gripper_opening = 0.06;
  config_fine.sample_density = 0.01;

  ContactPointSampler sampler_fine(config_fine);

  auto pairs_fine = sampler_fine.generate_contact_pairs(topology, all_ids, no_exclusions);

  EXPECT_GE(pairs_fine.size(), pairs_coarse.size());
}

TEST_F(ContactPointSamplerTest, DeduplicationRemovesSpatialDuplicates)
{
  TopoDS_Shape box = BRepPrimAPI_MakeBox(0.10, 0.10, 0.05).Shape();
  Topology topology = mapper_->load_from_shape(box, "box");

  std::vector<int> all_ids = get_all_surface_ids(topology);

  SamplingConfig config;
  config.min_gripper_opening = 0.04;
  config.max_gripper_opening = 0.06;
  config.sample_density = 0.005;

  ContactPointSampler sampler(config);

  std::vector<SampleArea> no_exclusions;
  auto pairs = sampler.generate_contact_pairs(topology, all_ids, no_exclusions);

  EXPECT_GT(pairs.size(), 0);

  for (size_t i = 0; i < pairs.size(); i++) {
    for (size_t j = i + 1; j < pairs.size(); j++) {
      double dist1 = pairs[i].contact_1.Distance(pairs[j].contact_1);
      double dist2 = pairs[i].contact_2.Distance(pairs[j].contact_2);
      double min_separation = config.sample_density / 2.0;

      bool close_enough = (dist1 < min_separation && dist2 < min_separation);
      EXPECT_FALSE(close_enough) << "Found spatial duplicate after deduplication";
    }
  }
}

TEST_F(ContactPointSamplerTest, EmptySurfaceList)
{
  TopoDS_Shape box = BRepPrimAPI_MakeBox(0.10, 0.10, 0.05).Shape();
  Topology topology = mapper_->load_from_shape(box, "box");

  ContactPointSampler sampler;

  std::vector<int> empty_ids;
  std::vector<SampleArea> no_exclusions;
  auto pairs = sampler.generate_contact_pairs(topology, empty_ids, no_exclusions);

  EXPECT_EQ(pairs.size(), 0);
}

TEST_F(ContactPointSamplerTest, SingleSurfaceNoPairing)
{
  TopoDS_Shape box = BRepPrimAPI_MakeBox(0.10, 0.10, 0.05).Shape();
  Topology topology = mapper_->load_from_shape(box, "box");

  ContactPointSampler sampler;

  std::vector<int> single_id = {0};
  std::vector<SampleArea> no_exclusions;
  auto pairs = sampler.generate_contact_pairs(topology, single_id, no_exclusions);

  EXPECT_EQ(pairs.size(), 0);
}

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
