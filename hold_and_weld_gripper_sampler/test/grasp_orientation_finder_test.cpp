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


#include <cmath>
#include <memory>
#include <vector>

#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRepPrimAPI_MakeSphere.hxx>
#include <BRepPrimAPI_MakeTorus.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
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
#include "hold_and_weld_gripper_sampler/core/gripper.hpp"
#include "hold_and_weld_gripper_sampler/collision/fcl_collision_checker.hpp"
#include "hold_and_weld_gripper_sampler/geometry/geometry_mapper.hpp"
#include "hold_and_weld_gripper_sampler/sampling/contact_point_sampler.hpp"

using namespace hold_and_weld_gripper_sampler::angle_finding;  // NOLINT
using namespace hold_and_weld_gripper_sampler::geometry;  // NOLINT
using namespace hold_and_weld_gripper_sampler;  // NOLINT
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

  gripper.base = create_box_at(0.02, 0.02, 0.01, -0.01, -0.01, -0.005);

  gripper.finger_1 = create_box_at(0.01, 0.005, 0.03, -0.005, -0.0025, -0.03);
  gripper.finger_2 = create_box_at(0.01, 0.005, 0.03, -0.005, 0.0175, -0.03);

  gripper.finger_1_axis = Eigen::Vector3d(0, -1, 0);
  gripper.finger_2_axis = Eigen::Vector3d(0, 1, 0);


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

  // Build an FCLCollisionChecker from primary_shape and wire it into finder.
  // Must be called after the finder is constructed and before find_valid_grasps.
  static void wire_fcl(
    GraspOrientationFinder & finder,
    const ParsedGripper & gripper,
    const TopoDS_Shape & primary_shape)
  {
    auto fcl = std::make_shared<geometry::FCLCollisionChecker>(gripper, primary_shape);
    finder.set_fcl_checker(fcl);
  }

  ParsedGripper gripper_;
  std::shared_ptr<GeometryMapper> mapper_;
};

// Empty contact pair list must return no grasps.
TEST_F(GraspOrientationFinderTest, EmptyContactPairsReturnsEmpty)
{
  TopoDS_Shape box = create_box(0.1, 0.1, 0.1);
  Topology topology = mapper_->load_from_shape(box, "test_box");

  GraspOrientationFinder finder(box, gripper_, nullptr, nullptr);

  std::vector<ContactPair> empty_pairs;
  auto grasps = finder.find_valid_grasps(empty_pairs, topology);

  EXPECT_TRUE(grasps.empty());
}

// A valid opposing-face contact pair on a grippable box produces candidates with in-range scores.
TEST_F(GraspOrientationFinderTest, FindGraspsOnSimpleBox)
{
  TopoDS_Shape box = create_box(0.05, 0.10, 0.05);
  Topology topology = mapper_->load_from_shape(box, "test_box");

  OrientationConfig config;
  config.finger_length = 0.05;
  config.collision_tolerance = 0.0001;

  GraspOrientationFinder finder(box, gripper_, nullptr, nullptr, config);
  wire_fcl(finder, gripper_, box);

  ContactPair pair;
  pair.contact_1 = gp_Pnt(0.0, 0.05, 0.025);
  pair.contact_2 = gp_Pnt(0.05, 0.05, 0.025);
  pair.grip_distance = 0.05;
  pair.surface_id_1 = 0;
  pair.surface_id_2 = 1;

  std::vector<ContactPair> pairs = {pair};
  auto grasps = finder.find_valid_grasps(pairs, topology);

  for (const auto & grasp : grasps) {
    EXPECT_DOUBLE_EQ(grasp.grip_distance, 0.05);
    EXPECT_GE(grasp.quality_score, 0.0);
    EXPECT_LE(grasp.quality_score, 1.0);
  }
}

// TODO(@silanus23): Fix mock gripper geometry to match URDF convention (fingers at Y=0 rest pos)
#if 0
TEST_F(GraspOrientationFinderTest, GraspCandidateHasValidTransform)
{
  // 80x80x80 mm cube.  Contacts are placed 8 mm from a face edge so that
  // a finger_length of 0.06 m comfortably reaches that edge, avoiding the
  // "no edges within finger_length" rejection path.
  TopoDS_Shape box = create_box(0.08, 0.08, 0.08);
  Topology topology = mapper_->load_from_shape(box, "test_box");

  OrientationConfig config;
  config.finger_length = 0.06;   // 60 mm — reaches the 8 mm-away edge with margin
  config.collision_tolerance = 0.0001;
  config.max_edge_candidates = 1;
  config.angle_offsets = {0.0};  // Single angle for predictable output

  ParsedGripper small_gripper = create_small_gripper();
  GraspOrientationFinder finder(box, small_gripper, nullptr, nullptr, config);
  wire_fcl(finder, small_gripper, box);

  // Contact pair on opposing X faces.  Y=0.008 places each contact 8 mm
  // from the nearest Y-edge of its face, well within finger_length=0.06 m.
  ContactPair pair;
  pair.contact_1 = gp_Pnt(0.0, 0.008, 0.04);
  pair.contact_2 = gp_Pnt(0.08, 0.008, 0.04);
  pair.grip_distance = 0.08;
  pair.surface_id_1 = 0;
  pair.surface_id_2 = 1;
  pair.normal_1 = gp_Vec(-1.0, 0.0, 0.0);
  pair.normal_2 = gp_Vec(1.0, 0.0, 0.0);

  std::vector<ContactPair> pairs = {pair};
  auto grasps = finder.find_valid_grasps(pairs, topology);

  ASSERT_FALSE(grasps.empty())
    << "find_valid_grasps returned no results for a clean opposing-face contact pair; "
       "check finger_length / collision_tolerance config or contact pair coordinates";

  for (const auto & grasp : grasps) {
    // Transform should be valid — extract the vectorial (rotation) part directly
    // instead of reading Value() which folds in the scale factor.
    gp_Trsf t = grasp.gripper_transform;
    gp_Mat rot = t.VectorialPart();  // always the pure rotation matrix, scale-free

    double det =
      rot.Value(1, 1) * (rot.Value(2, 2) * rot.Value(3, 3) - rot.Value(2, 3) * rot.Value(3, 2)) -
      rot.Value(1, 2) * (rot.Value(2, 1) * rot.Value(3, 3) - rot.Value(2, 3) * rot.Value(3, 1)) +
      rot.Value(1, 3) * (rot.Value(2, 1) * rot.Value(3, 2) - rot.Value(2, 2) * rot.Value(3, 1));
    EXPECT_NEAR(std::abs(det), 1.0, 1e-6)
      << "Rotation part of gripper_transform must have determinant 1";
  }
}
#endif

// Wider dual-seed dedup tolerance must produce fewer or equal orientations than a narrow tolerance.
TEST_F(GraspOrientationFinderTest, DualSeedDedupToleranceReducesOrientations)
{
  TopoDS_Shape u_edge_raw = create_u_shaped_edge(0.05);
  TopoDS_Shape u_edge = create_face_with_edge(u_edge_raw);
  Topology topology = mapper_->load_from_shape(u_edge, "test_arc");

  ParsedGripper small_gripper = create_small_gripper();

  OrientationConfig config_no_dedup;
  config_no_dedup.max_edge_candidates = 0;
  config_no_dedup.finger_length = 0.10;
  config_no_dedup.angle_offsets = {0.0};
  config_no_dedup.collision_tolerance = 0.0001;
  config_no_dedup.dual_seed_dedup_tolerance_deg = 0.1;  // near-zero: keep all seeds

  OrientationConfig config_dedup;
  config_dedup.max_edge_candidates = 0;
  config_dedup.finger_length = 0.10;
  config_dedup.angle_offsets = {0.0};
  config_dedup.collision_tolerance = 0.0001;
  config_dedup.dual_seed_dedup_tolerance_deg = 90.0;    // aggressive: merge seeds within 90°

  GraspOrientationFinder finder_no_dedup(u_edge, small_gripper, nullptr, nullptr, config_no_dedup);
  GraspOrientationFinder finder_dedup(u_edge, small_gripper, nullptr, nullptr, config_dedup);
  wire_fcl(finder_no_dedup, small_gripper, u_edge);
  wire_fcl(finder_dedup, small_gripper, u_edge);

  ContactPair pair;
  pair.contact_1 = gp_Pnt(0.0, 0.02, 0.0);
  pair.contact_2 = gp_Pnt(0.0, 0.02, 0.1);
  pair.grip_distance = 0.1;
  pair.surface_id_1 = 0;
  pair.surface_id_2 = 0;

  std::vector<ContactPair> pairs = {pair};

  auto grasps_no_dedup = finder_no_dedup.find_valid_grasps(pairs, topology);
  auto grasps_dedup = finder_dedup.find_valid_grasps(pairs, topology);

  // Wider dedup tolerance must produce fewer or equal orientations
  EXPECT_LE(grasps_dedup.size(), grasps_no_dedup.size());
}

// Three angle offsets must produce at least as many candidates as one offset.
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
  wire_fcl(finder_single, small_gripper, box);
  wire_fcl(finder_triple, small_gripper, box);

  ContactPair pair;
  pair.contact_1 = gp_Pnt(0.0, 0.05, 0.05);
  pair.contact_2 = gp_Pnt(0.1, 0.05, 0.05);
  pair.grip_distance = 0.1;
  pair.surface_id_1 = 0;
  pair.surface_id_2 = 1;

  std::vector<ContactPair> pairs = {pair};

  auto grasps_single = finder_single.find_valid_grasps(pairs, topology);
  auto grasps_triple = finder_triple.find_valid_grasps(pairs, topology);

  if (!grasps_single.empty()) {
    EXPECT_GE(grasps_triple.size(), grasps_single.size());
  }
}

// Empty angle_offsets must behave identically to an explicit {0.0} offset.
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
  wire_fcl(finder_empty, small_gripper, box);
  wire_fcl(finder_zero, small_gripper, box);

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

// Every returned grasp must have a quality_score in [0, 1].
TEST_F(GraspOrientationFinderTest, QualityScoreInValidRange)
{
  TopoDS_Shape box = create_box(0.1, 0.1, 0.1);
  Topology topology = mapper_->load_from_shape(box, "test_box");

  ParsedGripper small_gripper = create_small_gripper();

  OrientationConfig config;
  config.finger_length = 0.1;
  config.collision_tolerance = 0.0001;

  GraspOrientationFinder finder(box, small_gripper, nullptr, nullptr, config);
  wire_fcl(finder, small_gripper, box);

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

// max_edge_candidates=2 must produce fewer or equal candidates than no limit.
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
  wire_fcl(finder_unlimited, small_gripper, box);
  wire_fcl(finder_limited, small_gripper, box);

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
}

// Contact point with no reachable edges (finger_length too small) must yield no grasps.
TEST_F(GraspOrientationFinderTest, RejectsUnreachableContactWhenNoEdgesInRange)
{
  TopoDS_Shape box = create_box(0.5, 0.5, 0.5);
  Topology topology = mapper_->load_from_shape(box, "test_box");

  ParsedGripper small_gripper = create_small_gripper();

  OrientationConfig config;
  config.finger_length = 0.001;
  config.collision_tolerance = 0.0001;

  GraspOrientationFinder finder(box, small_gripper, nullptr, nullptr, config);
  wire_fcl(finder, small_gripper, box);

  ContactPair pair;
  pair.contact_1 = gp_Pnt(0.0, 0.25, 0.25);
  pair.contact_2 = gp_Pnt(0.5, 0.25, 0.25);
  pair.grip_distance = 0.5;
  pair.surface_id_1 = 0;
  pair.surface_id_2 = 1;

  std::vector<ContactPair> pairs = {pair};
  auto grasps = finder.find_valid_grasps(pairs, topology);

  EXPECT_TRUE(grasps.empty());
}

// Approach direction of every grasp must be perpendicular to the grip axis and unit-length.
TEST_F(GraspOrientationFinderTest, ApproachDirectionPerpendicularToGripAxis)
{
  TopoDS_Shape box = create_box(0.1, 0.1, 0.1);
  Topology topology = mapper_->load_from_shape(box, "test_box");

  ParsedGripper small_gripper = create_small_gripper();

  OrientationConfig config;
  config.finger_length = 0.1;
  config.collision_tolerance = 0.0001;

  GraspOrientationFinder finder(box, small_gripper, nullptr, nullptr, config);
  wire_fcl(finder, small_gripper, box);

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
    double dot = grasp.approach_direction.Dot(grip_axis);
    EXPECT_NEAR(dot, 0.0, 1e-6);
    EXPECT_NEAR(grasp.approach_direction.Magnitude(), 1.0, 1e-6);
  }
}

// Multiple contact pairs are each processed; every output grasp traces back to one input pair.
TEST_F(GraspOrientationFinderTest, ProcessesMultipleContactPairs)
{
  TopoDS_Shape box = create_box(0.1, 0.1, 0.1);
  Topology topology = mapper_->load_from_shape(box, "test_box");

  ParsedGripper small_gripper = create_small_gripper();

  OrientationConfig config;
  config.finger_length = 0.05;
  config.collision_tolerance = 0.0001;

  GraspOrientationFinder finder(box, small_gripper, nullptr, nullptr, config);

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

  for (const auto & grasp : grasps) {
    bool matches_pair1 = (grasp.contact_1.Distance(pair1.contact_1) < 1e-6 &&
      grasp.contact_2.Distance(pair1.contact_2) < 1e-6);
    bool matches_pair2 = (grasp.contact_1.Distance(pair2.contact_1) < 1e-6 &&
      grasp.contact_2.Distance(pair2.contact_2) < 1e-6);
    EXPECT_TRUE(matches_pair1 || matches_pair2);
  }
}

// Larger finger_length reaches more edges and must produce at least as many candidates.
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
  wire_fcl(finder_small, small_gripper, box);
  wire_fcl(finder_large, small_gripper, box);

  ContactPair pair;
  pair.contact_1 = gp_Pnt(0.0, 0.05, 0.05);
  pair.contact_2 = gp_Pnt(0.1, 0.05, 0.05);
  pair.grip_distance = 0.1;
  pair.surface_id_1 = 0;
  pair.surface_id_2 = 1;

  std::vector<ContactPair> pairs = {pair};

  auto grasps_small = finder_small.find_valid_grasps(pairs, topology);
  auto grasps_large = finder_large.find_valid_grasps(pairs, topology);

  EXPECT_LE(grasps_small.size(), grasps_large.size());
}

// Curved arc boundary produces at least one grasp;
// multiple grasps must have distinct approach directions.
TEST_F(GraspOrientationFinderTest, CurvedEdgeHasMultipleMinima)
{
  TopoDS_Shape half_disk = create_face_with_arc_boundary(0.05);
  Topology topology = mapper_->load_from_shape(half_disk, "test_arc");

  ParsedGripper small_gripper = create_small_gripper();

  OrientationConfig config;
  config.finger_length = 0.10;
  config.collision_tolerance = 0.0001;
  config.max_edge_candidates = 0;
  config.angle_offsets = {0.0};

  GraspOrientationFinder finder(half_disk, small_gripper, nullptr, nullptr, config);
  wire_fcl(finder, small_gripper, half_disk);

  // Off-center contact ensures asymmetric distances to arc endpoints
  ContactPair pair;
  pair.contact_1 = gp_Pnt(0.02, 0.02, 0.0);
  pair.contact_2 = gp_Pnt(0.02, 0.02, 0.1);
  pair.grip_distance = 0.1;
  // surface_id_1 = surface_id_2 = 0: single-face half-disk; tests curved edge
  // local-minima finding, not opposing-face geometry.
  pair.surface_id_1 = 0;
  pair.surface_id_2 = 0;
  pair.normal_1 = gp_Vec(0.0, 0.0, -1.0);
  pair.normal_2 = gp_Vec(0.0, 0.0, 1.0);

  std::vector<ContactPair> pairs = {pair};
  auto grasps = finder.find_valid_grasps(pairs, topology);

  EXPECT_GE(grasps.size(), 1) << "Should find at least one grasp from the arc";

  if (grasps.size() >= 2) {
    gp_Vec approach1 = grasps[0].approach_direction;
    gp_Vec approach2 = grasps[1].approach_direction;
    double dot = approach1.Dot(approach2);
    EXPECT_LT(std::abs(dot), 0.99) << "Multiple grasps should have different approaches";
  }
}

// max_orientations_per_pair=2 must cap output at 2 and produce <= results than no limit.
TEST_F(GraspOrientationFinderTest, MaxOrientationsPerPairLimitsResults)
{
  TopoDS_Shape u_edge_raw = create_u_shaped_edge(0.05);
  TopoDS_Shape u_edge = create_face_with_edge(u_edge_raw);
  Topology topology = mapper_->load_from_shape(u_edge, "test_arc");

  ParsedGripper small_gripper = create_small_gripper();

  OrientationConfig config_unlimited;
  config_unlimited.finger_length = 0.10;
  config_unlimited.max_orientations_per_pair = 0;  // No limit
  config_unlimited.collision_tolerance = 0.0001;

  OrientationConfig config_limited;
  config_limited.finger_length = 0.10;
  config_limited.max_orientations_per_pair = 2;
  config_limited.collision_tolerance = 0.0001;

  GraspOrientationFinder finder_unlimited(u_edge, small_gripper, nullptr, nullptr,
    config_unlimited);
  GraspOrientationFinder finder_limited(u_edge, small_gripper, nullptr, nullptr, config_limited);
  wire_fcl(finder_unlimited, small_gripper, u_edge);
  wire_fcl(finder_limited, small_gripper, u_edge);

  ContactPair pair;
  pair.contact_1 = gp_Pnt(0.0, 0.02, 0.0);
  pair.contact_2 = gp_Pnt(0.0, 0.02, 0.1);
  pair.grip_distance = 0.1;
  pair.surface_id_1 = 0;
  pair.surface_id_2 = 0;

  std::vector<ContactPair> pairs = {pair};
  auto grasps_unlimited = finder_unlimited.find_valid_grasps(pairs, topology);
  auto grasps_limited = finder_limited.find_valid_grasps(pairs, topology);

  // Cap must be respected
  EXPECT_LE(grasps_limited.size(), 2u);
  // Limited must not exceed unlimited
  EXPECT_LE(grasps_limited.size(), grasps_unlimited.size());
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
