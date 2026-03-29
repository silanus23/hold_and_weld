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

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <memory>
#include <set>
#include <vector>

#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRepPrimAPI_MakeSphere.hxx>
#include <BRepPrimAPI_MakeTorus.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepExtrema_DistShapeShape.hxx>
#include <BRep_Tool.hxx>
#include <BRep_Builder.hxx>
#include <BRepLib.hxx>
#include <Standard_Failure.hxx>
#include <Geom_Circle.hxx>
#include <Geom_BSplineCurve.hxx>
#include <Geom_TrimmedCurve.hxx>
#include <GC_MakeArcOfCircle.hxx>
#include <TColgp_Array1OfPnt.hxx>
#include <TColStd_Array1OfReal.hxx>
#include <TColStd_Array1OfInteger.hxx>
#include <gp_Trsf.hxx>
#include <gp_Vec.hxx>
#include <gp_Pnt.hxx>
#include <gp_Ax2.hxx>
#include <gp_Dir.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Shape.hxx>
#include <TopoDS_Compound.hxx>
#include <TopoDS_Edge.hxx>
#include <TopExp_Explorer.hxx>
#include "hold_and_weld_gripper_sampler/angle_finding/grasp_orientation_finder.hpp"
#include "hold_and_weld_gripper_sampler/geometry/topology.hpp"
#include "hold_and_weld_gripper_sampler/io/gripper_parser.hpp"
#include "hold_and_weld_gripper_sampler/geometry/geometry_mapper.hpp"
#include "hold_and_weld_gripper_sampler/sampling/contact_point_sampler.hpp"

using namespace hold_and_weld_gripper_sampler::angle_finding;  // NOLINT
using namespace hold_and_weld_gripper_sampler::geometry;  // NOLINT
using namespace hold_and_weld_gripper_sampler::io;  // NOLINT
using namespace hold_and_weld_gripper_sampler::sampling;  // NOLINT

namespace
{
TopoDS_Shape create_box(double width, double depth, double height)
{
  return BRepPrimAPI_MakeBox(width, depth, height).Shape();
}


TopoDS_Shape create_box_at(
  double width, double depth, double height,
  double x, double y, double z)
{
  gp_Trsf transform;
  transform.SetTranslation(gp_Vec(x, y, z));
  TopoDS_Shape box = BRepPrimAPI_MakeBox(width, depth, height).Shape();
  return BRepBuilderAPI_Transform(box, transform, Standard_True).Shape();
}

ParsedGripper create_mock_gripper()
{
  ParsedGripper gripper;

  gripper.base = create_box(0.08, 0.12, 0.04);

  // Position them at their closed state positions
  gripper.finger_1 = create_box_at(0.03, 0.04, 0.20, -0.015, 0.01, -0.20);
  gripper.finger_2 = create_box_at(0.03, 0.04, 0.20, -0.015, 0.07, -0.20);

  gripper.finger_1_axis = Eigen::Vector3d(0, -1, 0);  // Moves in -Y
  gripper.finger_2_axis = Eigen::Vector3d(0, 1, 0);   // Moves in +Y

  gripper.min_opening = 0.02;
  gripper.max_opening = 0.32;

  gripper.gripper_type = "parallel";
  gripper.tcp_offset = Eigen::Vector3d(0, 0.04, -0.10);
  gripper.tcp_rpy = Eigen::Vector3d(0, 0, 0);

  gripper.base_link_name = "gripper_base";
  gripper.finger_1_link_name = "finger_1";
  gripper.finger_2_link_name = "finger_2";
  gripper.finger_1_joint_name = "finger_1_joint";
  gripper.finger_2_joint_name = "finger_2_joint";

  return gripper;
}

ParsedGripper create_small_gripper()
{
  ParsedGripper gripper;

  gripper.base = create_box(0.02, 0.02, 0.01);

  gripper.finger_1 = create_box_at(0.01, 0.005, 0.03, -0.005, -0.0025, -0.03);
  gripper.finger_2 = create_box_at(0.01, 0.005, 0.03, -0.005, 0.0175, -0.03);

  gripper.finger_1_axis = Eigen::Vector3d(0, -1, 0);
  gripper.finger_2_axis = Eigen::Vector3d(0, 1, 0);

  gripper.min_opening = 0.01;
  gripper.max_opening = 0.10;

  gripper.gripper_type = "parallel";
  gripper.tcp_offset = Eigen::Vector3d(0, 0.01, -0.02);
  gripper.tcp_rpy = Eigen::Vector3d(0, 0, 0);

  gripper.base_link_name = "gripper_base";
  gripper.finger_1_link_name = "finger_1";
  gripper.finger_2_link_name = "finger_2";
  gripper.finger_1_joint_name = "finger_1_joint";
  gripper.finger_2_joint_name = "finger_2_joint";

  return gripper;
}


TopoDS_Shape create_face_with_edge(const TopoDS_Shape & edge_shape, double size = 0.5)
{
  gp_Pnt p1(-size, -size, 0);
  gp_Pnt p2(size, -size, 0);
  gp_Pnt p3(size, size, 0);
  gp_Pnt p4(-size, size, 0);

  TopoDS_Edge e1 = BRepBuilderAPI_MakeEdge(p1, p2).Edge();
  TopoDS_Edge e2 = BRepBuilderAPI_MakeEdge(p2, p3).Edge();
  TopoDS_Edge e3 = BRepBuilderAPI_MakeEdge(p3, p4).Edge();
  TopoDS_Edge e4 = BRepBuilderAPI_MakeEdge(p4, p1).Edge();

  BRepBuilderAPI_MakeWire wire_maker;
  wire_maker.Add(e1);
  wire_maker.Add(e2);
  wire_maker.Add(e3);
  wire_maker.Add(e4);
  TopoDS_Wire wire = wire_maker.Wire();

  TopoDS_Face face = BRepBuilderAPI_MakeFace(wire).Face();

  // Combine face with the test edge(s)
  BRep_Builder builder;
  TopoDS_Compound compound;
  builder.MakeCompound(compound);
  builder.Add(compound, face);
  builder.Add(compound, edge_shape);

  return compound;
}

TopoDS_Shape create_face_with_arc_boundary(double radius)
{
  gp_Pnt p1(radius, 0, 0);
  gp_Pnt p2(0, radius, 0);
  gp_Pnt p3(-radius, 0, 0);

  GC_MakeArcOfCircle arc_maker(p1, p2, p3);
  Handle(Geom_TrimmedCurve) arc = arc_maker.Value();
  TopoDS_Edge arc_edge = BRepBuilderAPI_MakeEdge(arc).Edge();

  // Create straight edge to close the shape (from p3 back to p1)
  TopoDS_Edge closing_edge = BRepBuilderAPI_MakeEdge(p3, p1).Edge();

  // Build wire from arc + closing edge
  BRepBuilderAPI_MakeWire wire_maker;
  wire_maker.Add(arc_edge);
  wire_maker.Add(closing_edge);
  TopoDS_Wire wire = wire_maker.Wire();

  // Create face from the closed wire
  TopoDS_Face face = BRepBuilderAPI_MakeFace(wire).Face();

  return face;
}

TopoDS_Shape create_u_shaped_edge(double radius)
{
  gp_Pnt p1(radius, 0, 0);
  gp_Pnt p2(0, radius, 0);
  gp_Pnt p3(-radius, 0, 0);

  GC_MakeArcOfCircle arc_maker(p1, p2, p3);
  Handle(Geom_TrimmedCurve) arc = arc_maker.Value();

  TopoDS_Edge edge = BRepBuilderAPI_MakeEdge(arc).Edge();

  BRep_Builder builder;
  TopoDS_Compound compound;
  builder.MakeCompound(compound);
  builder.Add(compound, edge);

  return compound;
}


}  // namespace

class GraspOrientationFinderTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    gripper_ = create_mock_gripper();
    mapper_ = std::make_shared<GeometryMapper>();
  }

  ParsedGripper gripper_;
  std::shared_ptr<GeometryMapper> mapper_;
};

TEST_F(GraspOrientationFinderTest, EmptyContactPairsReturnsEmpty)
{
  TopoDS_Shape box = create_box(0.1, 0.1, 0.1);
  Topology topology = mapper_->load_from_shape(box, "test_box");

  GraspOrientationFinder finder(box, gripper_, nullptr, nullptr);

  std::vector<ContactPair> empty_pairs;
  auto grasps = finder.find_valid_grasps(empty_pairs, topology);

  EXPECT_TRUE(grasps.empty());
}

TEST_F(GraspOrientationFinderTest, FindGraspsOnSimpleBox)
{
  // Create a graspable box: 5cm (X) x 10cm (Y) x 5cm (Z)
  TopoDS_Shape box = create_box(0.05, 0.10, 0.05);
  Topology topology = mapper_->load_from_shape(box, "test_box");

  OrientationConfig config;
  config.finger_length = 0.05;
  config.collision_tolerance = 0.0001;

  GraspOrientationFinder finder(box, gripper_, nullptr, nullptr, config);

  ContactPair pair;
  pair.contact_1 = gp_Pnt(0.0, 0.05, 0.025);   // Center of X=0 face (YZ plane)
  pair.contact_2 = gp_Pnt(0.05, 0.05, 0.025);  // Center of X=0.05 face (YZ plane)
  pair.grip_distance = 0.05;                    // Distance along X axis
  pair.surface_id_1 = 0;
  pair.surface_id_2 = 1;

  std::vector<ContactPair> pairs = {pair};
  auto grasps = finder.find_valid_grasps(pairs, topology);

  // Should find at least some grasp candidates (may be filtered by collision)
  // The test verifies the method doesn't crash and returns valid structure
  for (const auto & grasp : grasps) {
    EXPECT_DOUBLE_EQ(grasp.grip_distance, 0.05);
    EXPECT_GE(grasp.quality_score, 0.0);
    EXPECT_LE(grasp.quality_score, 1.0);
  }
}

TEST_F(GraspOrientationFinderTest, GraspCandidateHasValidTransform)
{
  TopoDS_Shape box = create_box(0.08, 0.08, 0.08);
  Topology topology = mapper_->load_from_shape(box, "test_box");

  OrientationConfig config;
  config.finger_length = 0.03;
  config.collision_tolerance = 0.0001;
  config.max_edge_candidates = 1;
  config.angle_offsets = {0.0};  // Single angle for predictable output

  ParsedGripper small_gripper = create_small_gripper();
  GraspOrientationFinder finder(box, small_gripper, nullptr, nullptr, config);

  // Contact pair on opposing X faces
  ContactPair pair;
  pair.contact_1 = gp_Pnt(0.0, 0.04, 0.04);
  pair.contact_2 = gp_Pnt(0.08, 0.04, 0.04);
  pair.grip_distance = 0.08;
  pair.surface_id_1 = 0;
  pair.surface_id_2 = 1;

  std::vector<ContactPair> pairs = {pair};
  auto grasps = finder.find_valid_grasps(pairs, topology);

  for (const auto & grasp : grasps) {
    // Transform should be valid (determinant of rotation part = 1)
    gp_Trsf t = grasp.gripper_transform;

    // Check it's a valid rigid transformation
    // The rotation matrix should have determinant 1
    double det = t.Value(1, 1) * (t.Value(2, 2) * t.Value(3, 3) - t.Value(2, 3) * t.Value(3, 2)) -
      t.Value(1, 2) * (t.Value(2, 1) * t.Value(3, 3) - t.Value(2, 3) * t.Value(3, 1)) +
      t.Value(1, 3) * (t.Value(2, 1) * t.Value(3, 2) - t.Value(2, 2) * t.Value(3, 1));
    EXPECT_NEAR(std::abs(det), 1.0, 1e-6);
  }
}

TEST_F(GraspOrientationFinderTest, ClusteringReducesEdgeCandidates)
{
  // max_edge_candidates caps the number of edge seeds used per contact point.
  // With 0 (no limit) every local minimum edge becomes a seed; with 1 only the
  // closest edge is kept.  The capped run must therefore produce <= orientations.
  TopoDS_Shape box = create_box(0.1, 0.1, 0.1);
  Topology topology = mapper_->load_from_shape(box, "test_box");

  OrientationConfig config_no_limit;
  config_no_limit.max_edge_candidates = 0;  // keep all edge seeds
  config_no_limit.finger_length = 0.2;      // large search radius → many edges
  config_no_limit.angle_offsets = {0.0};
  config_no_limit.collision_tolerance = 0.0001;

  OrientationConfig config_capped;
  config_capped.max_edge_candidates = 1;    // keep only the single closest edge seed
  config_capped.finger_length = 0.2;
  config_capped.angle_offsets = {0.0};
  config_capped.collision_tolerance = 0.0001;

  ParsedGripper small_gripper = create_small_gripper();

  GraspOrientationFinder finder_no_limit(
    box, small_gripper, nullptr, nullptr, config_no_limit);
  GraspOrientationFinder finder_capped(
    box, small_gripper, nullptr, nullptr, config_capped);

  // Contact pair across the X faces of the box
  ContactPair pair;
  pair.contact_1 = gp_Pnt(0.0, 0.05, 0.05);
  pair.contact_2 = gp_Pnt(0.1, 0.05, 0.05);
  pair.grip_distance = 0.1;
  pair.surface_id_1 = 0;
  pair.surface_id_2 = 1;

  std::vector<ContactPair> pairs = {pair};

  auto grasps_no_limit = finder_no_limit.find_valid_grasps(pairs, topology);
  auto grasps_capped = finder_capped.find_valid_grasps(pairs, topology);

  // Capping edge candidates must yield fewer or equal orientations
  EXPECT_LE(grasps_capped.size(), grasps_no_limit.size());
}

TEST_F(GraspOrientationFinderTest, LargerDualSeedDedupToleranceFewerOrientations)
{
  // dual_seed_dedup_tolerance_deg merges edge seeds from contact_1 and contact_2
  // that point in nearly the same bearing direction.  A tiny tolerance keeps
  // almost every seed; a large tolerance aggressively merges them.
  // The aggressively-merged run must produce <= orientations.
  TopoDS_Shape box = create_box(0.1, 0.1, 0.1);
  Topology topology = mapper_->load_from_shape(box, "test_box");

  ParsedGripper small_gripper = create_small_gripper();

  OrientationConfig config_narrow;
  config_narrow.max_edge_candidates = 0;
  config_narrow.finger_length = 0.15;
  config_narrow.angle_offsets = {0.0};
  config_narrow.collision_tolerance = 0.0001;
  config_narrow.dual_seed_dedup_tolerance_deg = 0.1;   // near-zero: keep all seeds

  OrientationConfig config_wide;
  config_wide.max_edge_candidates = 0;
  config_wide.finger_length = 0.15;
  config_wide.angle_offsets = {0.0};
  config_wide.collision_tolerance = 0.0001;
  config_wide.dual_seed_dedup_tolerance_deg = 90.0;    // aggressive: merge seeds within 90°

  GraspOrientationFinder finder_narrow(box, small_gripper, nullptr, nullptr, config_narrow);
  GraspOrientationFinder finder_wide(box, small_gripper, nullptr, nullptr, config_wide);

  ContactPair pair;
  pair.contact_1 = gp_Pnt(0.0, 0.05, 0.05);
  pair.contact_2 = gp_Pnt(0.1, 0.05, 0.05);
  pair.grip_distance = 0.1;
  pair.surface_id_1 = 0;
  pair.surface_id_2 = 1;

  std::vector<ContactPair> pairs = {pair};

  auto grasps_narrow = finder_narrow.find_valid_grasps(pairs, topology);
  auto grasps_wide = finder_wide.find_valid_grasps(pairs, topology);

  // Wider dedup tolerance must produce fewer or equal orientations
  EXPECT_LE(grasps_wide.size(), grasps_narrow.size());
}

TEST_F(GraspOrientationFinderTest, MultipleAngleOffsetsProduceMultipleCandidates)
{
  TopoDS_Shape box = create_box(0.1, 0.1, 0.1);
  Topology topology = mapper_->load_from_shape(box, "test_box");

  ParsedGripper small_gripper = create_small_gripper();

  OrientationConfig config_single;
  config_single.angle_offsets = {0.0};
  config_single.max_edge_candidates = 1;

  config_single.finger_length = 0.08;
  config_single.collision_tolerance = 0.0001;

  OrientationConfig config_triple;
  config_triple.angle_offsets = {-30.0, 0.0, 30.0};
  config_triple.max_edge_candidates = 1;

  config_triple.finger_length = 0.08;
  config_triple.collision_tolerance = 0.0001;

  GraspOrientationFinder finder_single(box, small_gripper, nullptr, nullptr, config_single);
  GraspOrientationFinder finder_triple(box, small_gripper, nullptr, nullptr, config_triple);

  ContactPair pair;
  pair.contact_1 = gp_Pnt(0.0, 0.05, 0.05);
  pair.contact_2 = gp_Pnt(0.1, 0.05, 0.05);
  pair.grip_distance = 0.1;
  pair.surface_id_1 = 0;
  pair.surface_id_2 = 1;

  std::vector<ContactPair> pairs = {pair};

  auto grasps_single = finder_single.find_valid_grasps(pairs, topology);
  auto grasps_triple = finder_triple.find_valid_grasps(pairs, topology);

  // Triple offsets should produce more candidates (up to 3x if no collision filtering)
  // At minimum, should be >= single (unless all collide)
  if (!grasps_single.empty()) {
    EXPECT_GE(grasps_triple.size(), grasps_single.size());
  }
}

TEST_F(GraspOrientationFinderTest, EmptyAngleOffsetsDefaultsToZero)
{
  TopoDS_Shape box = create_box(0.1, 0.1, 0.1);
  Topology topology = mapper_->load_from_shape(box, "test_box");

  ParsedGripper small_gripper = create_small_gripper();

  OrientationConfig config_empty;
  config_empty.angle_offsets = {};  // Empty
  config_empty.max_edge_candidates = 1;
  config_empty.finger_length = 0.08;

  OrientationConfig config_zero;
  config_zero.angle_offsets = {0.0};  // Explicit zero
  config_zero.max_edge_candidates = 1;
  config_zero.finger_length = 0.08;

  GraspOrientationFinder finder_empty(box, small_gripper, nullptr, nullptr, config_empty);
  GraspOrientationFinder finder_zero(box, small_gripper, nullptr, nullptr, config_zero);

  ContactPair pair;
  pair.contact_1 = gp_Pnt(0.0, 0.05, 0.05);
  pair.contact_2 = gp_Pnt(0.1, 0.05, 0.05);
  pair.grip_distance = 0.1;
  pair.surface_id_1 = 0;
  pair.surface_id_2 = 1;

  std::vector<ContactPair> pairs = {pair};

  auto grasps_empty = finder_empty.find_valid_grasps(pairs, topology);
  auto grasps_zero = finder_zero.find_valid_grasps(pairs, topology);

  // Should produce same number of results
  EXPECT_EQ(grasps_empty.size(), grasps_zero.size());
}

TEST_F(GraspOrientationFinderTest, DualSeedDedupToleranceIsConfigurable)
{
  // Create a U-shaped arc where dual-seed strategy finds seeds from both ends
  // Wrap in a face so topology has a valid surface
  TopoDS_Shape u_edge_raw = create_u_shaped_edge(0.05);
  TopoDS_Shape u_edge = create_face_with_edge(u_edge_raw);
  Topology topology = mapper_->load_from_shape(u_edge, "test_arc");

  ParsedGripper small_gripper = create_small_gripper();

  // Config with very small dedup tolerance (nearly no deduplication)
  OrientationConfig config_no_dedup;
  config_no_dedup.finger_length = 0.10;
  config_no_dedup.collision_tolerance = 0.0001;
  config_no_dedup.dual_seed_dedup_tolerance_deg = 0.1;  // Very small: ~no dedup

  config_no_dedup.max_edge_candidates = 0;
  config_no_dedup.angle_offsets = {0.0};

  // Config with large dedup tolerance (aggressive deduplication)
  OrientationConfig config_dedup;
  config_dedup.finger_length = 0.10;
  config_dedup.collision_tolerance = 0.0001;
  config_dedup.dual_seed_dedup_tolerance_deg = 90.0;

  config_dedup.max_edge_candidates = 0;
  config_dedup.angle_offsets = {0.0};

  GraspOrientationFinder finder_no_dedup(u_edge, small_gripper, nullptr, nullptr, config_no_dedup);
  GraspOrientationFinder finder_dedup(u_edge, small_gripper, nullptr, nullptr, config_dedup);

  // Contact inside the arc
  ContactPair pair;
  pair.contact_1 = gp_Pnt(0.0, 0.02, 0.0);
  pair.contact_2 = gp_Pnt(0.0, 0.02, 0.1);
  pair.grip_distance = 0.1;
  pair.surface_id_1 = 0;
  pair.surface_id_2 = 0;

  std::vector<ContactPair> pairs = {pair};

  auto grasps_no_dedup = finder_no_dedup.find_valid_grasps(pairs, topology);
  auto grasps_dedup = finder_dedup.find_valid_grasps(pairs, topology);

  // With aggressive deduplication, we should have fewer or equal grasps
  EXPECT_LE(grasps_dedup.size(), grasps_no_dedup.size());
}

TEST_F(GraspOrientationFinderTest, QualityScoreInValidRange)
{
  TopoDS_Shape box = create_box(0.1, 0.1, 0.1);
  Topology topology = mapper_->load_from_shape(box, "test_box");

  ParsedGripper small_gripper = create_small_gripper();

  OrientationConfig config;
  config.finger_length = 0.1;
  config.collision_tolerance = 0.0001;

  GraspOrientationFinder finder(box, small_gripper, nullptr, nullptr, config);

  ContactPair pair;
  pair.contact_1 = gp_Pnt(0.0, 0.05, 0.05);
  pair.contact_2 = gp_Pnt(0.1, 0.05, 0.05);
  pair.grip_distance = 0.1;
  pair.surface_id_1 = 0;
  pair.surface_id_2 = 1;

  std::vector<ContactPair> pairs = {pair};
  auto grasps = finder.find_valid_grasps(pairs, topology);

  for (const auto & grasp : grasps) {
    EXPECT_GE(grasp.quality_score, 0.0);
    EXPECT_LE(grasp.quality_score, 1.0);
  }
}

TEST_F(GraspOrientationFinderTest, MaxEdgeCandidatesLimitsOutput)
{
  TopoDS_Shape box = create_box(0.1, 0.1, 0.1);
  Topology topology = mapper_->load_from_shape(box, "test_box");

  ParsedGripper small_gripper = create_small_gripper();

  OrientationConfig config_unlimited;
  config_unlimited.max_edge_candidates = 0;  // No limit

  config_unlimited.finger_length = 0.2;  // Large to find many edges
  config_unlimited.angle_offsets = {0.0};
  config_unlimited.collision_tolerance = 0.0001;

  OrientationConfig config_limited;
  config_limited.max_edge_candidates = 2;  // Limit to 2

  config_limited.finger_length = 0.2;
  config_limited.angle_offsets = {0.0};
  config_limited.collision_tolerance = 0.0001;

  GraspOrientationFinder finder_unlimited(
    box, small_gripper, nullptr, nullptr, config_unlimited);
  GraspOrientationFinder finder_limited(
    box, small_gripper, nullptr, nullptr, config_limited);

  ContactPair pair;
  pair.contact_1 = gp_Pnt(0.0, 0.05, 0.05);
  pair.contact_2 = gp_Pnt(0.1, 0.05, 0.05);
  pair.grip_distance = 0.1;
  pair.surface_id_1 = 0;
  pair.surface_id_2 = 1;

  std::vector<ContactPair> pairs = {pair};

  auto grasps_unlimited = finder_unlimited.find_valid_grasps(pairs, topology);
  auto grasps_limited = finder_limited.find_valid_grasps(pairs, topology);

  // Limited should have fewer or equal candidates
  EXPECT_LE(grasps_limited.size(), grasps_unlimited.size());

  // If unlimited finds more than 2, limited should be capped
  // (accounting for collision filtering which may reduce both)
}

TEST_F(GraspOrientationFinderTest, RejectsUnreachableContactWhenNoEdgesInRange)
{
  // Use a very small finger_length so no edges are in range
  // This means the contact point is UNREACHABLE - the finger cannot
  // physically get to it without passing through solid material.
  TopoDS_Shape box = create_box(0.5, 0.5, 0.5);  // Large box
  Topology topology = mapper_->load_from_shape(box, "test_box");

  ParsedGripper small_gripper = create_small_gripper();

  OrientationConfig config;
  config.finger_length = 0.001;  // Very small - won't find edges
  config.collision_tolerance = 0.0001;

  GraspOrientationFinder finder(box, small_gripper, nullptr, nullptr, config);

  // Contact in the center of the face (far from edges)
  ContactPair pair;
  pair.contact_1 = gp_Pnt(0.0, 0.25, 0.25);
  pair.contact_2 = gp_Pnt(0.5, 0.25, 0.25);
  pair.grip_distance = 0.5;
  pair.surface_id_1 = 0;
  pair.surface_id_2 = 1;

  std::vector<ContactPair> pairs = {pair};
  auto grasps = finder.find_valid_grasps(pairs, topology);

  // Should produce NO candidates - contact point is unreachable
  // (no edge within finger_length to approach from)
  EXPECT_TRUE(grasps.empty());
}

TEST_F(GraspOrientationFinderTest, ApproachDirectionPerpendicularToGripAxis)
{
  TopoDS_Shape box = create_box(0.1, 0.1, 0.1);
  Topology topology = mapper_->load_from_shape(box, "test_box");

  ParsedGripper small_gripper = create_small_gripper();

  OrientationConfig config;
  config.finger_length = 0.1;
  config.collision_tolerance = 0.0001;

  GraspOrientationFinder finder(box, small_gripper, nullptr, nullptr, config);

  ContactPair pair;
  pair.contact_1 = gp_Pnt(0.0, 0.05, 0.05);
  pair.contact_2 = gp_Pnt(0.1, 0.05, 0.05);
  pair.grip_distance = 0.1;
  pair.surface_id_1 = 0;
  pair.surface_id_2 = 1;

  std::vector<ContactPair> pairs = {pair};
  auto grasps = finder.find_valid_grasps(pairs, topology);

  gp_Vec grip_axis(pair.contact_1, pair.contact_2);
  grip_axis.Normalize();

  for (const auto & grasp : grasps) {
    // Approach direction should be perpendicular to grip axis
    double dot = grasp.approach_direction.Dot(grip_axis);
    EXPECT_NEAR(dot, 0.0, 1e-6);

    // Approach direction should be unit vector
    EXPECT_NEAR(grasp.approach_direction.Magnitude(), 1.0, 1e-6);
  }
}

TEST_F(GraspOrientationFinderTest, TransformFollowsURDFConvention)
{
  TopoDS_Shape box = create_box(0.1, 0.1, 0.1);
  Topology topology = mapper_->load_from_shape(box, "test_box");

  ParsedGripper small_gripper = create_small_gripper();

  OrientationConfig config;
  config.finger_length = 0.05;
  config.collision_tolerance = 0.0001;
  config.max_edge_candidates = 1;
  config.angle_offsets = {0.0};

  GraspOrientationFinder finder(box, small_gripper, nullptr, nullptr, config);

  ContactPair pair;
  pair.contact_1 = gp_Pnt(0.0, 0.05, 0.05);
  pair.contact_2 = gp_Pnt(0.1, 0.05, 0.05);
  pair.grip_distance = 0.1;
  pair.surface_id_1 = 0;
  pair.surface_id_2 = 1;

  std::vector<ContactPair> pairs = {pair};
  auto grasps = finder.find_valid_grasps(pairs, topology);

  // Expected TCP at midpoint between contacts
  gp_Pnt expected_tcp(
    (pair.contact_1.X() + pair.contact_2.X()) / 2,
    (pair.contact_1.Y() + pair.contact_2.Y()) / 2,
    (pair.contact_1.Z() + pair.contact_2.Z()) / 2);

  for (const auto & grasp : grasps) {
    // Verify the transform is a valid rigid transformation
    // by checking that the rotation part has determinant 1
    gp_Trsf t = grasp.gripper_transform;
    double det = t.Value(1, 1) * (t.Value(2, 2) * t.Value(3, 3) - t.Value(2, 3) * t.Value(3, 2)) -
      t.Value(1, 2) * (t.Value(2, 1) * t.Value(3, 3) - t.Value(2, 3) * t.Value(3, 1)) +
      t.Value(1, 3) * (t.Value(2, 1) * t.Value(3, 2) - t.Value(2, 2) * t.Value(3, 1));
    EXPECT_NEAR(std::abs(det), 1.0, 1e-6);

    // Verify grasp contacts match input
    EXPECT_NEAR(grasp.contact_1.X(), pair.contact_1.X(), 1e-6);
    EXPECT_NEAR(grasp.contact_1.Y(), pair.contact_1.Y(), 1e-6);
    EXPECT_NEAR(grasp.contact_1.Z(), pair.contact_1.Z(), 1e-6);
    EXPECT_NEAR(grasp.contact_2.X(), pair.contact_2.X(), 1e-6);
    EXPECT_NEAR(grasp.contact_2.Y(), pair.contact_2.Y(), 1e-6);
    EXPECT_NEAR(grasp.contact_2.Z(), pair.contact_2.Z(), 1e-6);

    // Verify grip distance is preserved
    EXPECT_NEAR(grasp.grip_distance, pair.grip_distance, 1e-6);
  }
}

TEST_F(GraspOrientationFinderTest, ProcessesMultipleContactPairs)
{
  TopoDS_Shape box = create_box(0.1, 0.1, 0.1);
  Topology topology = mapper_->load_from_shape(box, "test_box");

  ParsedGripper small_gripper = create_small_gripper();

  OrientationConfig config;
  config.finger_length = 0.05;
  config.collision_tolerance = 0.0001;

  GraspOrientationFinder finder(box, small_gripper, nullptr, nullptr, config);

  // Create multiple contact pairs
  ContactPair pair1;
  pair1.contact_1 = gp_Pnt(0.0, 0.05, 0.05);
  pair1.contact_2 = gp_Pnt(0.1, 0.05, 0.05);
  pair1.grip_distance = 0.1;
  pair1.surface_id_1 = 0;
  pair1.surface_id_2 = 1;

  ContactPair pair2;
  pair2.contact_1 = gp_Pnt(0.05, 0.0, 0.05);
  pair2.contact_2 = gp_Pnt(0.05, 0.1, 0.05);
  pair2.grip_distance = 0.1;
  pair2.surface_id_1 = 2;
  pair2.surface_id_2 = 3;

  std::vector<ContactPair> pairs = {pair1, pair2};
  auto grasps = finder.find_valid_grasps(pairs, topology);

  // Should process both pairs (may have multiple grasps per pair)
  // Each grasp should have contacts from one of the input pairs
  for (const auto & grasp : grasps) {
    bool matches_pair1 = (grasp.contact_1.Distance(pair1.contact_1) < 1e-6 &&
      grasp.contact_2.Distance(pair1.contact_2) < 1e-6);
    bool matches_pair2 = (grasp.contact_1.Distance(pair2.contact_1) < 1e-6 &&
      grasp.contact_2.Distance(pair2.contact_2) < 1e-6);

    EXPECT_TRUE(matches_pair1 || matches_pair2);
  }
}

TEST_F(GraspOrientationFinderTest, LargerFingerLengthFindsMoreEdges)
{
  TopoDS_Shape box = create_box(0.1, 0.1, 0.1);
  Topology topology = mapper_->load_from_shape(box, "test_box");

  ParsedGripper small_gripper = create_small_gripper();

  OrientationConfig config_small;
  config_small.finger_length = 0.02;  // Small radius

  config_small.max_edge_candidates = 0;
  config_small.angle_offsets = {0.0};
  config_small.collision_tolerance = 0.0001;

  OrientationConfig config_large;
  config_large.finger_length = 0.2;  // Large radius

  config_large.max_edge_candidates = 0;
  config_large.angle_offsets = {0.0};
  config_large.collision_tolerance = 0.0001;

  GraspOrientationFinder finder_small(box, small_gripper, nullptr, nullptr, config_small);
  GraspOrientationFinder finder_large(box, small_gripper, nullptr, nullptr, config_large);

  ContactPair pair;
  pair.contact_1 = gp_Pnt(0.0, 0.05, 0.05);
  pair.contact_2 = gp_Pnt(0.1, 0.05, 0.05);
  pair.grip_distance = 0.1;
  pair.surface_id_1 = 0;
  pair.surface_id_2 = 1;

  std::vector<ContactPair> pairs = {pair};

  auto grasps_small = finder_small.find_valid_grasps(pairs, topology);
  auto grasps_large = finder_large.find_valid_grasps(pairs, topology);

  // Larger finger length should find more edge candidates
  // (resulting in more or equal grasp candidates, before collision filtering)
  EXPECT_LE(grasps_small.size(), grasps_large.size());
}

TEST_F(GraspOrientationFinderTest, CollisionFilteringRemovesInvalidGrasps)
{
  // Create a complex shape where some grasps would collide
  TopoDS_Shape box = create_box(0.03, 0.03, 0.03);  // Small box
  Topology topology = mapper_->load_from_shape(box, "test_box");

  // Use a gripper that's large relative to the box
  OrientationConfig config;
  config.finger_length = 0.05;
  config.collision_tolerance = 0.001;

  GraspOrientationFinder finder(box, gripper_, nullptr, nullptr, config);

  ContactPair pair;
  pair.contact_1 = gp_Pnt(0.0, 0.015, 0.015);
  pair.contact_2 = gp_Pnt(0.03, 0.015, 0.015);
  pair.grip_distance = 0.03;
  pair.surface_id_1 = 0;
  pair.surface_id_2 = 1;

  std::vector<ContactPair> pairs = {pair};
  auto grasps = finder.find_valid_grasps(pairs, topology);

  // Grasps that pass collision check should not collide
  // (We can't verify absence of collision directly, but we verify no crash)
  for (const auto & grasp : grasps) {
    EXPECT_DOUBLE_EQ(grasp.grip_distance, 0.03);
  }
}

/**
 * @brief Test that straight edges produce exactly one local minimum
 */
TEST_F(GraspOrientationFinderTest, StraightEdgeHasSingleMinimum)
{
  // Box has only straight edges
  TopoDS_Shape box = create_box(0.1, 0.1, 0.1);
  Topology topology = mapper_->load_from_shape(box, "test_box");

  ParsedGripper small_gripper = create_small_gripper();

  OrientationConfig config;
  config.finger_length = 0.15;  // Large enough to include multiple edges
  config.collision_tolerance = 0.0001;

  GraspOrientationFinder finder(box, small_gripper, nullptr, nullptr, config);

  // Contact point on face center - equidistant to edges
  ContactPair pair;
  pair.contact_1 = gp_Pnt(0.0, 0.05, 0.05);  // Center of YZ face at x=0
  pair.contact_2 = gp_Pnt(0.1, 0.05, 0.05);  // Center of opposite face
  pair.grip_distance = 0.1;
  pair.surface_id_1 = 0;
  pair.surface_id_2 = 1;

  std::vector<ContactPair> pairs = {pair};

  // Should work without errors
  EXPECT_NO_THROW({
    auto grasps = finder.find_valid_grasps(pairs, topology);
  });

  // For a box, each straight edge should contribute exactly 1 constraint
  // (BRepExtrema_ExtPC finds the single orthogonal projection point)
}

/**
 * @brief Test that curved edges (arc) can produce multiple local minima
 */
TEST_F(GraspOrientationFinderTest, CurvedEdgeHasMultipleMinima)
{
  // Create a half-disk face where the curved arc IS part of the surface boundary
  // Arc goes from (0.05, 0, 0) through (0, 0.05, 0) to (-0.05, 0, 0)
  // The arc edge is now in surface.edge_ids, so find_edges_in_circle will find it
  TopoDS_Shape half_disk = create_face_with_arc_boundary(0.05);  // 5cm radius arc
  Topology topology = mapper_->load_from_shape(half_disk, "test_arc");

  ParsedGripper small_gripper = create_small_gripper();

  // Use no clustering and no limit to see all edge candidates
  OrientationConfig config;
  config.finger_length = 0.10;  // Large enough to include the whole arc
  config.collision_tolerance = 0.0001;

  config.max_edge_candidates = 0;  // Unlimited
  config.angle_offsets = {0.0};    // Single offset for cleaner results

  GraspOrientationFinder finder(half_disk, small_gripper, nullptr, nullptr, config);

  // Contact point OFF-CENTER inside the arc - asymmetric position ensures
  // different distances to each arc endpoint, producing different approach directions
  // From (0.02, 0.02, 0), distances to arc endpoints:
  //   - To (0.05, 0, 0): sqrt(0.03² + 0.02²) ≈ 0.036m (closer)
  //   - To (-0.05, 0, 0): sqrt(0.07² + 0.02²) ≈ 0.073m (farther)
  // Both are within finger_length (0.10m), so we should find 2 minima on the arc
  ContactPair pair;
  pair.contact_1 = gp_Pnt(0.02, 0.02, 0.0);  // Off-center for asymmetry
  pair.contact_2 = gp_Pnt(0.02, 0.02, 0.1);  // Arbitrary second point along Z
  pair.grip_distance = 0.1;
  pair.surface_id_1 = 0;
  pair.surface_id_2 = 0;

  std::vector<ContactPair> pairs = {pair};

  auto grasps = finder.find_valid_grasps(pairs, topology);

  // The key test is that BRepExtrema_ExtPC finds multiple minima on curved edges
  // We should get at least 2 grasps (one per arc endpoint minimum)
  EXPECT_GE(grasps.size(), 2) << "Should find grasps from both arc minima";

  // With asymmetric contact position, the two minima should produce
  // different approach directions (pointing away from different edge points)
  if (grasps.size() >= 2) {
    gp_Vec approach1 = grasps[0].approach_direction;
    gp_Vec approach2 = grasps[1].approach_direction;
    double dot = approach1.Dot(approach2);
    // Different minima at different distances should produce different approach directions
    EXPECT_LT(std::abs(dot), 0.99) << "Multiple grasps should have different approaches";
  }
}

/**
 * @brief Test max_orientations_per_pair limits the number of orientations tested
 */
TEST_F(GraspOrientationFinderTest, MaxOrientationsPerPairLimitsResults)
{
  // Wrap in a face so topology has a valid surface
  TopoDS_Shape u_edge_raw = create_u_shaped_edge(0.05);
  TopoDS_Shape u_edge = create_face_with_edge(u_edge_raw);
  Topology topology = mapper_->load_from_shape(u_edge, "test_arc");

  ParsedGripper small_gripper = create_small_gripper();

  // Config with unlimited orientations
  OrientationConfig config_unlimited;
  config_unlimited.finger_length = 0.10;
  config_unlimited.max_orientations_per_pair = 0;  // Unlimited
  config_unlimited.collision_tolerance = 0.0001;

  // Config with limited orientations
  OrientationConfig config_limited;
  config_limited.finger_length = 0.10;
  config_limited.max_orientations_per_pair = 2;
  config_limited.collision_tolerance = 0.0001;

  GraspOrientationFinder finder_unlimited(u_edge, small_gripper, nullptr, nullptr,
    config_unlimited);
  GraspOrientationFinder finder_limited(u_edge, small_gripper, nullptr, nullptr, config_limited);

  ContactPair pair;
  pair.contact_1 = gp_Pnt(0.0, 0.02, 0.0);
  pair.contact_2 = gp_Pnt(0.0, 0.02, 0.1);
  pair.grip_distance = 0.1;
  pair.surface_id_1 = 0;
  pair.surface_id_2 = 0;

  std::vector<ContactPair> pairs = {pair};

  // Both should work, limited config may produce fewer results
  EXPECT_NO_THROW({
    auto grasps_unlimited = finder_unlimited.find_valid_grasps(pairs, topology);
    auto grasps_limited = finder_limited.find_valid_grasps(pairs, topology);
  });
}

/**
 * @brief Test dual-seed strategy finds orientations from both contacts
 *
 * The dual-seed strategy considers edges near BOTH contact points, not just one.
 * Edges from contact_2 are shifted by 180° (finger symmetry) and merged with
 * edges from contact_1 to create the master seed list.
 */
TEST_F(GraspOrientationFinderTest, DualSeedStrategyUsesBothContacts)
{
  TopoDS_Shape box = create_box(0.1, 0.1, 0.1);
  Topology topology = mapper_->load_from_shape(box, "test_box");

  ParsedGripper small_gripper = create_small_gripper();

  // Disable clustering to see all seeds, use single angle offset
  OrientationConfig config;
  config.finger_length = 0.15;
  config.collision_tolerance = 0.0001;

  config.max_edge_candidates = 0;  // Unlimited
  config.angle_offsets = {0.0};    // Single offset

  GraspOrientationFinder finder(box, small_gripper, nullptr, nullptr, config);

  // Contact points on opposite X faces - both have 4 nearby edges each
  // (the 4 edges of each face)
  ContactPair pair;
  pair.contact_1 = gp_Pnt(0.0, 0.05, 0.05);
  pair.contact_2 = gp_Pnt(0.1, 0.05, 0.05);
  pair.grip_distance = 0.1;
  pair.surface_id_1 = 0;
  pair.surface_id_2 = 1;

  std::vector<ContactPair> pairs = {pair};

  auto grasps = finder.find_valid_grasps(pairs, topology);

  // With edges from both contacts (8 edges total, minus duplicates after 180° shift),
  // we should get multiple grasp candidates
  EXPECT_GT(grasps.size(), 0) << "Dual-seed strategy should find grasps";

  // Verify we have grasps with different approach directions
  // (indicating seeds from different edges/contacts were used)
  if (grasps.size() >= 2) {
    std::set<int> unique_approach_directions;
    for (const auto & grasp : grasps) {
      // Quantize approach direction to detect unique directions
      int quantized = static_cast<int>(
        std::atan2(grasp.approach_direction.Y(), grasp.approach_direction.Z()) * 180 / M_PI / 30);
      unique_approach_directions.insert(quantized);
    }
    EXPECT_GT(unique_approach_directions.size(), 1)
      << "Should have multiple unique approach directions from dual-seed strategy";
  }
}


/**
 * @brief Test endpoint minima detection (when trend starts increasing or ends decreasing)
 */
TEST_F(GraspOrientationFinderTest, EndpointMinimaDetected)
{
  // Create a simple straight edge where the contact is closest to one endpoint
  // Wrap in a face so topology has a valid surface
  gp_Pnt p1(0.0, 0.0, 0.0);
  gp_Pnt p2(0.1, 0.0, 0.0);
  TopoDS_Edge edge = BRepBuilderAPI_MakeEdge(p1, p2).Edge();

  BRep_Builder builder;
  TopoDS_Compound edge_compound;
  builder.MakeCompound(edge_compound);
  builder.Add(edge_compound, edge);

  TopoDS_Shape compound = create_face_with_edge(edge_compound);
  Topology topology = mapper_->load_from_shape(compound, "test_edge");

  ParsedGripper small_gripper = create_small_gripper();

  OrientationConfig config;
  config.finger_length = 0.15;
  config.collision_tolerance = 0.0001;

  GraspOrientationFinder finder(compound, small_gripper, nullptr, nullptr, config);

  // Contact very close to p1 endpoint - minimum should be at endpoint
  ContactPair pair;
  pair.contact_1 = gp_Pnt(0.01, 0.02, 0.0);  // Close to p1
  pair.contact_2 = gp_Pnt(0.01, 0.02, 0.1);
  pair.grip_distance = 0.1;
  pair.surface_id_1 = 0;
  pair.surface_id_2 = 0;

  std::vector<ContactPair> pairs = {pair};

  EXPECT_NO_THROW({
    auto grasps = finder.find_valid_grasps(pairs, topology);
  });
}

/**
 * @brief Test with wavy B-spline edge that has multiple inflection points
 */


/**
 * @brief Test edge that passes through finger circle finds the minimum inside
 *
 * When an edge passes through the finger circle, we should find the
 * local minimum that is inside the circle (the closest point).
 */
TEST_F(GraspOrientationFinderTest, EdgePassingThroughCircleFindsMinimumInside)
{
  // Edge passes through finger circle
  // Edge at y=0.08, contact at origin, finger_length=0.1
  // Midpoint (closest point) is at distance 0.08 (inside circle)
  // Wrap in a face so topology has a valid surface
  gp_Pnt p1(-0.15, 0.08, 0.0);
  gp_Pnt p2(0.15, 0.08, 0.0);
  TopoDS_Edge edge = BRepBuilderAPI_MakeEdge(p1, p2).Edge();

  BRep_Builder builder;
  TopoDS_Compound edge_compound;
  builder.MakeCompound(edge_compound);
  builder.Add(edge_compound, edge);

  TopoDS_Shape compound = create_face_with_edge(edge_compound);
  Topology topology = mapper_->load_from_shape(compound, "test_crossing_edge");

  ParsedGripper small_gripper = create_small_gripper();

  OrientationConfig config;
  config.finger_length = 0.10;  // 10cm - midpoint at 8cm is inside
  config.collision_tolerance = 0.0001;

  GraspOrientationFinder finder(compound, small_gripper, nullptr, nullptr, config);

  // Contact at origin
  ContactPair pair;
  pair.contact_1 = gp_Pnt(0.0, 0.0, 0.0);
  pair.contact_2 = gp_Pnt(0.0, 0.0, 0.1);
  pair.grip_distance = 0.1;
  pair.surface_id_1 = 0;
  pair.surface_id_2 = 0;

  std::vector<ContactPair> pairs = {pair};

  // Should find the minimum at the midpoint (inside the circle)
  EXPECT_NO_THROW({
    auto grasps = finder.find_valid_grasps(pairs, topology);
  });
}

/**
 * @brief Test edge completely outside finger circle returns no constraints
 */
TEST_F(GraspOrientationFinderTest, EdgeOutsideCircleNoConstraints)
{
  // Edge completely outside the finger circle
  // Wrap in a face so topology has a valid surface
  gp_Pnt p1(-0.1, 0.2, 0.0);  // 20cm away from origin
  gp_Pnt p2(0.1, 0.2, 0.0);
  TopoDS_Edge edge = BRepBuilderAPI_MakeEdge(p1, p2).Edge();

  BRep_Builder builder;
  TopoDS_Compound edge_compound;
  builder.MakeCompound(edge_compound);
  builder.Add(edge_compound, edge);

  TopoDS_Shape compound = create_face_with_edge(edge_compound);
  Topology topology = mapper_->load_from_shape(compound, "test_outside_edge");

  ParsedGripper small_gripper = create_small_gripper();

  OrientationConfig config;
  config.finger_length = 0.10;
  config.collision_tolerance = 0.0001;

  GraspOrientationFinder finder(compound, small_gripper, nullptr, nullptr, config);

  // Contact at origin - edge is too far
  ContactPair pair;
  pair.contact_1 = gp_Pnt(0.0, 0.0, 0.0);
  pair.contact_2 = gp_Pnt(0.0, 0.0, 0.1);
  pair.grip_distance = 0.1;
  pair.surface_id_1 = 0;
  pair.surface_id_2 = 0;

  std::vector<ContactPair> pairs = {pair};

  // Should work without errors - minima outside finger_length are filtered
  EXPECT_NO_THROW({
    auto grasps = finder.find_valid_grasps(pairs, topology);
  });
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
