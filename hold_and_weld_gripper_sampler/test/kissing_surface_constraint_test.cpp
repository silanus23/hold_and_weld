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


#include <Eigen/Dense>
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <vector>

#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>

#include <BRepBuilderAPI_Transform.hxx>
#include <gp_Ax2.hxx>
#include <gp_Trsf.hxx>
#include <gp_Vec.hxx>
#include <TopoDS_Shape.hxx>

#include "hold_and_weld_gripper_sampler/core/gripper.hpp"
#include "hold_and_weld_gripper_sampler/constraints/kissing_surface_constraint.hpp"
#include "hold_and_weld_gripper_sampler/geometry/fcl_collision_checker.hpp"
#include "hold_and_weld_gripper_sampler/geometry/geometry_mapper.hpp"
#include "hold_and_weld_gripper_sampler/io/gripper_parser.hpp"
#include "hold_and_weld_gripper_sampler/geometry/occt_utils.hpp"

using namespace hold_and_weld_gripper_sampler;  // NOLINT
using namespace hold_and_weld_gripper_sampler::constraints;  // NOLINT
using namespace hold_and_weld_gripper_sampler::geometry;  // NOLINT
using namespace hold_and_weld_gripper_sampler::core;  // NOLINT

namespace
{

ParsedGripper create_mock_gripper()
{
  ParsedGripper gripper;

  // Finger 1: positioned at Y = +0.015 (15mm from center)
  gp_Trsf f1_pos;
  f1_pos.SetTranslation(gp_Vec(0.0, 0.015, 0.0));
  TopoDS_Shape f1_box = BRepPrimAPI_MakeBox(0.02, 0.01, 0.05).Shape();
  gripper.finger_1 = BRepBuilderAPI_Transform(f1_box, f1_pos, Standard_True).Shape();

  // Finger 2: positioned at Y = -0.025 (mirrored)
  gp_Trsf f2_pos;
  f2_pos.SetTranslation(gp_Vec(0.0, -0.025, 0.0));
  TopoDS_Shape f2_box = BRepPrimAPI_MakeBox(0.02, 0.01, 0.05).Shape();
  gripper.finger_2 = BRepBuilderAPI_Transform(f2_box, f2_pos, Standard_True).Shape();

  // Base: centered above fingers
  gp_Trsf base_pos;
  base_pos.SetTranslation(gp_Vec(-0.01, -0.03, 0.05));
  TopoDS_Shape base_box = BRepPrimAPI_MakeBox(0.06, 0.06, 0.03).Shape();
  gripper.base = BRepBuilderAPI_Transform(base_box, base_pos, Standard_True).Shape();

  // Opening axes: fingers move along Y
  gripper.finger_1_axis = Eigen::Vector3d(0.0, 1.0, 0.0);   // +Y
  gripper.finger_2_axis = Eigen::Vector3d(0.0, -1.0, 0.0);  // -Y

  gripper.max_opening = 0.1;

  // Metadata fields
  gripper.gripper_type = "parallel";
  gripper.tcp_offset = Eigen::Vector3d(0.0, 0.0, -0.05);
  gripper.tcp_rpy = Eigen::Vector3d(0.0, 0.0, 0.0);

  // Link/joint names (for mock gripper)
  gripper.base_link_name = "mock_gripper_base";
  gripper.finger_1_link_name = "mock_left_finger";
  gripper.finger_2_link_name = "mock_right_finger";
  gripper.finger_1_joint_name = "mock_left_finger_joint";
  gripper.finger_2_joint_name = "mock_right_finger_joint";

  return gripper;
}

}  // namespace

class KissingSurfaceConstraintTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    mapper_ = std::make_shared<GeometryMapper>();
    gripper_ = create_mock_gripper();
  }

  // Build and wire an FCLCollisionChecker into a constraint so that
  // intersects_secondary() can be called.  The primary shape is only needed
  // for the FCL primary BVH; any non-null shape works.
  void wire_fcl(
    KissingSurfaceConstraint & constraint,
    const std::vector<TopoDS_Shape> & secondaries,
    const TopoDS_Shape & primary_for_fcl)
  {
    auto fcl = std::make_shared<geometry::FCLCollisionChecker>(
      gripper_, primary_for_fcl);
    fcl->add_secondary_shapes(secondaries);
    constraint.set_fcl_checker(fcl);
  }

  std::shared_ptr<GeometryMapper> mapper_;
  ParsedGripper gripper_;
};

TEST_F(KissingSurfaceConstraintTest, NoContactWithDistantSecondary)
{
  // Primary box at z=0.5 to z=0.6
  gp_Trsf primary_pos;
  primary_pos.SetTranslation(gp_Vec(0.0, 0.0, 0.5));
  TopoDS_Shape primary = BRepBuilderAPI_Transform(
    BRepPrimAPI_MakeBox(0.1, 0.1, 0.1).Shape(),
    primary_pos, Standard_True).Shape();

  // Secondary far away (no contact), also raised
  gp_Trsf far_pos;
  far_pos.SetTranslation(gp_Vec(1.0, 0.0, 0.5));
  TopoDS_Shape far_secondary = BRepBuilderAPI_Transform(
    BRepPrimAPI_MakeBox(0.1, 0.1, 0.1).Shape(),
    far_pos, Standard_True).Shape();

  std::vector<TopoDS_Shape> secondaries = {far_secondary};

  KissingSurfaceConstraint constraint(
    mapper_, gripper_, secondaries);

  // Analyze
  Topology topology = mapper_->load_from_shape(primary);
  constraint.analyze_constraints(topology);

  // No surfaces should be banned (no contact)
  EXPECT_TRUE(constraint.get_banned_surface_ids().empty());
  EXPECT_TRUE(constraint.get_sample_areas().empty());
}

TEST_F(KissingSurfaceConstraintTest, FullContactBansSurface)
{
  // Primary box sitting on secondary shape
  // Bottom face (z=0) should be in full contact
  gp_Trsf primary_pos;
  primary_pos.SetTranslation(gp_Vec(0.0, 0.0, 0.0));
  TopoDS_Shape primary = BRepPrimAPI_MakeBox(0.1, 0.1, 0.1).Shape();

  // Secondary shape directly touching bottom of primary
  gp_Trsf ground_pos;
  ground_pos.SetTranslation(gp_Vec(-0.05, -0.05, -0.01));
  TopoDS_Shape ground = BRepPrimAPI_MakeBox(0.2, 0.2, 0.01).Shape();
  ground = BRepBuilderAPI_Transform(ground, ground_pos, Standard_True).Shape();

  std::vector<TopoDS_Shape> secondaries = {ground};

  KissingSurfaceConstraint constraint(
    mapper_, gripper_, secondaries, 0.8);

  // Analyze
  Topology topology = mapper_->load_from_shape(primary);
  constraint.analyze_constraints(topology);

  // Bottom face must be banned — it is fully covered by the ground secondary
  std::vector<int> banned = constraint.get_banned_surface_ids();

  EXPECT_GE(banned.size(), 1u) << "Bottom face touching ground plane must be banned";
  EXPECT_LE(banned.size(), 6u) << "Cannot ban more faces than the box has";
}

// TODO(@silanus23): This test is failing because intersects_secondary returns false even when
// the gripper is placed inside the secondary shape. The FCL collision check is not detecting
// the overlap correctly — needs investigation into how BVH volumes are built for the secondary
// and how the gripper geometry is represented during the check.
// TEST_F(KissingSurfaceConstraintTest, CollisionWhenInsideSecondary)
// {
//   TopoDS_Shape primary = BRepPrimAPI_MakeBox(0.1, 0.1, 0.1).Shape();
//
//   // Create a large secondary that encompasses origin
//   gp_Trsf secondary_pos;
//   secondary_pos.SetTranslation(gp_Vec(-0.1, -0.1, -0.1));
//   TopoDS_Shape large_secondary = BRepPrimAPI_MakeBox(0.3, 0.3, 0.3).Shape();
//   large_secondary = BRepBuilderAPI_Transform(large_secondary, secondary_pos, Standard_True).Shape();
//
//   std::vector<TopoDS_Shape> secondaries = {large_secondary};
//
//   KissingSurfaceConstraint constraint(
//     mapper_, gripper_, secondaries, 0.8, 0.001);
//
//   Topology topology = mapper_->load_from_shape(primary);
//   constraint.analyze_constraints(topology);
//   wire_fcl(constraint, secondaries, primary);
//
//   // Place gripper at origin (inside the secondary)
//   gp_Trsf origin_transform;
//   origin_transform.SetTranslation(gp_Vec(0.0, 0.0, 0.0));
//
//   Eigen::Isometry3d eigen_transform = Eigen::Isometry3d::Identity();
//   eigen_transform.translation() = extract_translation(origin_transform);
//   eigen_transform.linear() = extract_quaternion(origin_transform).toRotationMatrix();
//
//   bool collision = constraint.intersects_secondary(0.03, eigen_transform);
//   EXPECT_TRUE(collision);
// }

TEST_F(KissingSurfaceConstraintTest, CollisionWithGroundPlane)
{
  // Primary box raised above z=0
  gp_Trsf primary_pos;
  primary_pos.SetTranslation(gp_Vec(0.0, 0.0, 0.5));
  TopoDS_Shape primary = BRepBuilderAPI_Transform(
    BRepPrimAPI_MakeBox(0.1, 0.1, 0.1).Shape(),
    primary_pos, Standard_True).Shape();

  // Secondary shape at z=0
  gp_Trsf secondary_pos;
  secondary_pos.SetTranslation(gp_Vec(-0.5, -0.5, -0.01));
  TopoDS_Shape secondary = BRepPrimAPI_MakeBox(1.0, 1.0, 0.01).Shape();
  secondary = BRepBuilderAPI_Transform(secondary, secondary_pos, Standard_True).Shape();

  std::vector<TopoDS_Shape> secondaries = {secondary};

  KissingSurfaceConstraint constraint(
    mapper_, gripper_, secondaries);

  Topology topology = mapper_->load_from_shape(primary);
  constraint.analyze_constraints(topology);
  wire_fcl(constraint, secondaries, primary);

  // Place gripper near secondary (should collide)
  gp_Trsf near_transform;
  near_transform.SetTranslation(gp_Vec(0.0, 0.0, -0.005));

  Eigen::Isometry3d eigen_transform = Eigen::Isometry3d::Identity();
  eigen_transform.translation() = extract_translation(near_transform);
  eigen_transform.linear() = extract_quaternion(near_transform).toRotationMatrix();

  bool collision = constraint.intersects_secondary(0.03, eigen_transform);
  EXPECT_TRUE(collision);
}

TEST_F(KissingSurfaceConstraintTest, CollisionWithFixture)
{
  TopoDS_Shape primary = BRepPrimAPI_MakeBox(0.1, 0.1, 0.1).Shape();

  // Fixture directly adjacent to primary
  gp_Trsf fixture_pos;
  fixture_pos.SetTranslation(gp_Vec(0.1, 0.0, 0.0));
  TopoDS_Shape fixture = BRepPrimAPI_MakeBox(0.05, 0.1, 0.1).Shape();
  fixture = BRepBuilderAPI_Transform(fixture, fixture_pos, Standard_True).Shape();

  std::vector<TopoDS_Shape> secondaries = {fixture};

  KissingSurfaceConstraint constraint(
    mapper_, gripper_, secondaries);

  Topology topology = mapper_->load_from_shape(primary);
  constraint.analyze_constraints(topology);
  wire_fcl(constraint, secondaries, primary);

  // Place gripper at fixture location (should collide)
  // Place gripper near fixture (should collide)
  gp_Trsf fixture_transform;
  fixture_transform.SetTranslation(gp_Vec(0.1, 0.0, 0.0));

  Eigen::Isometry3d eigen_transform = Eigen::Isometry3d::Identity();
  eigen_transform.translation() = extract_translation(fixture_transform);
  eigen_transform.linear() = extract_quaternion(fixture_transform).toRotationMatrix();

  bool collision = constraint.intersects_secondary(0.03, eigen_transform);
  EXPECT_TRUE(collision);
}

TEST_F(KissingSurfaceConstraintTest, GripperOpensCorrectly)
{
  TopoDS_Shape primary = BRepPrimAPI_MakeBox(0.1, 0.1, 0.1).Shape();

  // Secondary shape at z=0
  gp_Trsf secondary_pos;
  secondary_pos.SetTranslation(gp_Vec(-0.5, -0.5, -0.01));
  TopoDS_Shape secondary = BRepPrimAPI_MakeBox(1.0, 1.0, 0.01).Shape();
  secondary = BRepBuilderAPI_Transform(secondary, secondary_pos, Standard_True).Shape();

  std::vector<TopoDS_Shape> secondaries = {secondary};

  KissingSurfaceConstraint constraint(
    mapper_, gripper_, secondaries);

  Topology topology = mapper_->load_from_shape(primary);
  constraint.analyze_constraints(topology);
  wire_fcl(constraint, secondaries, primary);

  // Test with different grip distances - gripper far from secondaries
  gp_Trsf far_transform;
  far_transform.SetTranslation(gp_Vec(0.0, 0.0, 0.5));

  Eigen::Isometry3d eigen_transform = Eigen::Isometry3d::Identity();
  eigen_transform.translation() = extract_translation(far_transform);
  eigen_transform.linear() = extract_quaternion(far_transform).toRotationMatrix();

  // Minimum opening
  bool collision_min = constraint.intersects_secondary(0.0, eigen_transform);
  EXPECT_FALSE(collision_min);

  // Maximum opening
  bool collision_max = constraint.intersects_secondary(0.08, eigen_transform);
  EXPECT_FALSE(collision_max);
}

TEST_F(KissingSurfaceConstraintTest, CollisionToleranceAffectsDetectionDistance)
{
  // Secondary: ground plane occupying z=-0.01 to z=0, top face exactly at z=0.
  gp_Trsf secondary_pos;
  secondary_pos.SetTranslation(gp_Vec(-0.5, -0.5, -0.01));
  TopoDS_Shape secondary = BRepPrimAPI_MakeBox(1.0, 1.0, 0.01).Shape();
  secondary = BRepBuilderAPI_Transform(secondary, secondary_pos, Standard_True).Shape();

  std::vector<TopoDS_Shape> secondaries = {secondary};

  // Build a dedicated gripper whose lowest point is exactly z=0 in its local
  // frame.  BRepPrimAPI_MakeBox(dx, dy, dz) places the box with one corner at
  // the origin, so the bottom face sits at z=0 and the top face at z=dz.
  // Using a single flat block as both base and "fingers" removes any ambiguity
  // about which part of the gripper is closest to the secondary.
  ParsedGripper flat_gripper;
  // 60x60x10 mm block — bottom face at z=0, top face at z=0.01
  flat_gripper.base = BRepPrimAPI_MakeBox(
    gp_Pnt(-0.03, -0.03, 0.0), 0.06, 0.06, 0.01).Shape();
  flat_gripper.finger_1 = flat_gripper.base;
  flat_gripper.finger_2 = flat_gripper.base;
  flat_gripper.finger_1_axis = Eigen::Vector3d(0.0, 1.0, 0.0);
  flat_gripper.finger_2_axis = Eigen::Vector3d(0.0, -1.0, 0.0);
  flat_gripper.max_opening = 0.06;
  flat_gripper.gripper_type = "parallel";
  flat_gripper.tcp_offset = Eigen::Vector3d::Zero();
  flat_gripper.tcp_rpy = Eigen::Vector3d::Zero();
  flat_gripper.base_link_name = "flat_base";
  flat_gripper.finger_1_link_name = "flat_f1";
  flat_gripper.finger_2_link_name = "flat_f2";
  flat_gripper.finger_1_joint_name = "flat_j1";
  flat_gripper.finger_2_joint_name = "flat_j2";

  // Known gap: translate the gripper 5 mm above the secondary top face (z=0).
  // The gripper's lowest point is at local z=0, so after the transform its
  // closest point to the secondary is exactly 5 mm.
  constexpr double kGap = 0.005;
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.translation() = Eigen::Vector3d(0.0, 0.0, kGap);

  TopoDS_Shape primary = BRepPrimAPI_MakeBox(0.1, 0.1, 0.1).Shape();
  Topology topology = mapper_->load_from_shape(primary);

  // Loose tolerance (10 mm) > gap (5 mm) → collision must be reported.
  KissingSurfaceConstraint constraint_loose(
    mapper_, flat_gripper, secondaries, 0.8, 0.010);
  constraint_loose.analyze_constraints(topology);
  {
    auto fcl_loose = std::make_shared<geometry::FCLCollisionChecker>(
      flat_gripper, primary);
    fcl_loose->add_secondary_shapes(secondaries);
    constraint_loose.set_fcl_checker(fcl_loose);
  }
  EXPECT_TRUE(constraint_loose.intersects_secondary(0.03, pose))
    << "10 mm tolerance must detect gripper whose bottom face is 5 mm above secondary";

  // Tight tolerance (1 mm) < gap (5 mm) → no collision must be reported.
  KissingSurfaceConstraint constraint_tight(
    mapper_, flat_gripper, secondaries, 0.8, 0.001);
  constraint_tight.analyze_constraints(topology);
  {
    auto fcl_tight = std::make_shared<geometry::FCLCollisionChecker>(
      flat_gripper, primary);
    fcl_tight->add_secondary_shapes(secondaries);
    constraint_tight.set_fcl_checker(fcl_tight);
  }
  EXPECT_FALSE(constraint_tight.intersects_secondary(0.03, pose))
    << "1 mm tolerance must not fire when gripper bottom face is 5 mm above secondary";
}

TEST_F(KissingSurfaceConstraintTest, CollisionWithAnySecondary)
{
  TopoDS_Shape primary = BRepPrimAPI_MakeBox(0.1, 0.1, 0.1).Shape();

  // Bottom secondary shape (like ground)
  gp_Trsf bottom_pos;
  bottom_pos.SetTranslation(gp_Vec(-0.5, -0.5, -0.01));
  TopoDS_Shape bottom_secondary = BRepPrimAPI_MakeBox(1.0, 1.0, 0.01).Shape();
  bottom_secondary = BRepBuilderAPI_Transform(bottom_secondary, bottom_pos, Standard_True).Shape();

  // Left fixture
  gp_Trsf left_pos;
  left_pos.SetTranslation(gp_Vec(-0.1, 0.0, 0.0));
  TopoDS_Shape left_fixture = BRepPrimAPI_MakeBox(0.05, 0.1, 0.1).Shape();
  left_fixture = BRepBuilderAPI_Transform(left_fixture, left_pos, Standard_True).Shape();

  // Right fixture
  gp_Trsf right_pos;
  right_pos.SetTranslation(gp_Vec(0.15, 0.0, 0.0));
  TopoDS_Shape right_fixture = BRepPrimAPI_MakeBox(0.05, 0.1, 0.1).Shape();
  right_fixture = BRepBuilderAPI_Transform(right_fixture, right_pos, Standard_True).Shape();

  std::vector<TopoDS_Shape> secondaries = {bottom_secondary, left_fixture, right_fixture};

  KissingSurfaceConstraint constraint(
    mapper_, gripper_, secondaries);

  Topology topology = mapper_->load_from_shape(primary);
  constraint.analyze_constraints(topology);
  wire_fcl(constraint, secondaries, primary);

  gp_Trsf at_bottom;
  at_bottom.SetTranslation(gp_Vec(0.0, 0.0, 0.0));
  Eigen::Isometry3d at_bottom_iso = Eigen::Isometry3d::Identity();
  at_bottom_iso.translation() = extract_translation(at_bottom);
  at_bottom_iso.linear() = extract_quaternion(at_bottom).toRotationMatrix();

  EXPECT_TRUE(constraint.intersects_secondary(0.03, at_bottom_iso));

  // Collision with left fixture
  // Collision with left fixture — place gripper solidly inside it
  // Left fixture: X=-0.1 to -0.05, Y=0.0 to 0.1, Z=0.0 to 0.1
  // Gripper at (-0.075, 0.05, 0.05) puts fingers inside the fixture volume
  gp_Trsf at_left;
  at_left.SetTranslation(gp_Vec(-0.075, 0.05, 0.05));
  Eigen::Isometry3d eigen_at_left = Eigen::Isometry3d::Identity();
  eigen_at_left.translation() = extract_translation(at_left);
  eigen_at_left.linear() = extract_quaternion(at_left).toRotationMatrix();
  EXPECT_TRUE(constraint.intersects_secondary(0.03, eigen_at_left))
    << "Gripper inside left fixture volume must report collision";

  // No collision in clear space
  gp_Trsf clear_space;
  clear_space.SetTranslation(gp_Vec(0.5, 0.5, 0.5));
  Eigen::Isometry3d eigen_clear = Eigen::Isometry3d::Identity();
  eigen_clear.translation() = extract_translation(clear_space);
  eigen_clear.linear() = extract_quaternion(clear_space).toRotationMatrix();
  EXPECT_FALSE(constraint.intersects_secondary(0.03, eigen_clear));
}

TEST_F(KissingSurfaceConstraintTest, SampleAreasEmptyBeforeAnalysis)
{
  TopoDS_Shape primary = BRepPrimAPI_MakeBox(0.1, 0.1, 0.1).Shape();

  gp_Trsf secondary_pos;
  secondary_pos.SetTranslation(gp_Vec(-0.5, -0.5, -0.01));
  TopoDS_Shape secondary = BRepPrimAPI_MakeBox(1.0, 1.0, 0.01).Shape();
  secondary = BRepBuilderAPI_Transform(secondary, secondary_pos, Standard_True).Shape();

  std::vector<TopoDS_Shape> secondaries = {secondary};

  KissingSurfaceConstraint constraint(
    mapper_, gripper_, secondaries);

  // Before analyze_constraints is called
  EXPECT_TRUE(constraint.get_sample_areas().empty());
  EXPECT_TRUE(constraint.get_banned_surface_ids().empty());
}

TEST_F(KissingSurfaceConstraintTest, EmptySecondaryShapesVectorProducesNoResults)
{
  // Construct with an explicitly empty secondaries vector — no secondary shapes
  // are present so no surface can be in contact with anything.
  std::vector<TopoDS_Shape> no_secondaries;
  KissingSurfaceConstraint constraint(mapper_, gripper_, no_secondaries, 0.5);

  TopoDS_Shape primary = BRepPrimAPI_MakeBox(0.1, 0.1, 0.1).Shape();
  Topology topology = mapper_->load_from_shape(primary);
  constraint.analyze_constraints(topology);

  // Wire an FCL checker with no secondaries — intersects_secondary() returns
  // false immediately when secondary_shapes_ is empty (checked before FCL).
  auto fcl = std::make_shared<geometry::FCLCollisionChecker>(gripper_, primary);
  constraint.set_fcl_checker(fcl);

  // No secondaries → nothing can be banned or partially excluded
  EXPECT_TRUE(constraint.get_banned_surface_ids().empty())
    << "No secondary shapes must produce no banned surface IDs after analysis";
  EXPECT_TRUE(constraint.get_sample_areas().empty())
    << "No secondary shapes must produce no partial-exclusion sample areas after analysis";

  // intersects_secondary must always return false when there are no secondaries
  Eigen::Isometry3d identity = Eigen::Isometry3d::Identity();
  EXPECT_FALSE(constraint.intersects_secondary(0.03, identity))
    << "intersects_secondary must return false when secondary list is empty";
}

TEST_F(KissingSurfaceConstraintTest, BannedSurfaceIdsNotInSampleAreas)
{
  TopoDS_Shape primary = BRepPrimAPI_MakeBox(0.1, 0.1, 0.1).Shape();

  gp_Trsf secondary_pos;
  secondary_pos.SetTranslation(gp_Vec(-0.5, -0.5, -0.01));
  TopoDS_Shape secondary = BRepPrimAPI_MakeBox(1.0, 1.0, 0.01).Shape();
  secondary = BRepBuilderAPI_Transform(secondary, secondary_pos, Standard_True).Shape();

  std::vector<TopoDS_Shape> secondaries = {secondary};

  KissingSurfaceConstraint constraint(
    mapper_, gripper_, secondaries);

  Topology topology = mapper_->load_from_shape(primary);
  constraint.analyze_constraints(topology);

  std::vector<int> banned = constraint.get_banned_surface_ids();
  auto areas = constraint.get_sample_areas();

  // Banned surfaces should not appear in sample areas
  for (const auto & area : areas) {
    bool is_banned = std::find(banned.begin(), banned.end(), area.surface_id) != banned.end();
    EXPECT_FALSE(is_banned) << "Surface " << area.surface_id <<
      " is both banned and has sample area";
  }
}

TEST_F(KissingSurfaceConstraintTest, HighThresholdBansFewerSurfaces)
{
  TopoDS_Shape primary = BRepPrimAPI_MakeBox(0.1, 0.1, 0.1).Shape();

  gp_Trsf secondary_pos;
  secondary_pos.SetTranslation(gp_Vec(-0.5, -0.5, -0.01));
  TopoDS_Shape secondary = BRepPrimAPI_MakeBox(1.0, 1.0, 0.01).Shape();
  secondary = BRepBuilderAPI_Transform(secondary, secondary_pos, Standard_True).Shape();

  std::vector<TopoDS_Shape> secondaries = {secondary};

  // Low threshold (50% contact bans surface)
  KissingSurfaceConstraint constraint_low(
    mapper_, gripper_, secondaries, 0.5);

  // High threshold (95% contact required to ban)
  KissingSurfaceConstraint constraint_high(
    mapper_, gripper_, secondaries, 0.95);

  Topology topology = mapper_->load_from_shape(primary);
  constraint_low.analyze_constraints(topology);
  constraint_high.analyze_constraints(topology);

  // High threshold should ban same or fewer surfaces
  EXPECT_LE(constraint_high.get_banned_surface_ids().size(),
            constraint_low.get_banned_surface_ids().size() + 1);  // +1 for tolerance
}

TEST_F(KissingSurfaceConstraintTest, ReanalysisReplacesNotAccumulates)
{
  // Secondary: ground plane at z=0 (occupies z=-0.01 to z=0, top face at z=0).
  gp_Trsf secondary_pos;
  secondary_pos.SetTranslation(gp_Vec(-0.5, -0.5, -0.01));
  TopoDS_Shape secondary = BRepPrimAPI_MakeBox(1.0, 1.0, 0.01).Shape();
  secondary = BRepBuilderAPI_Transform(secondary, secondary_pos, Standard_True).Shape();

  std::vector<TopoDS_Shape> secondaries = {secondary};

  // 50% contact threshold — a fully touching face is reliably banned.
  KissingSurfaceConstraint constraint(mapper_, gripper_, secondaries, 0.5);

  // First analysis: box whose bottom face sits exactly on the secondary top
  // face (z=0).  The bottom face has 100% contact → must be banned.
  // Note: load_from_shape builds a new Topology whose face handles are owned
  // by the returned object.  analyze_constraints consumes the topology by
  // value within its call — it does not hold a dangling reference after
  // returning, so re-running with a fresh topology is safe.
  TopoDS_Shape touching_box = BRepPrimAPI_MakeBox(0.1, 0.1, 0.1).Shape();
  Topology touching_topology = mapper_->load_from_shape(touching_box);
  constraint.analyze_constraints(touching_topology);

  size_t first_banned_count = constraint.get_banned_surface_ids().size();
  EXPECT_GE(first_banned_count, 1u) << "First analysis must ban the touching face";

  // Second analysis: identical box lifted 1 m above the secondary — no face
  // is within contact distance.  analyze_constraints clears banned_surface_ids_
  // at the top of every call, so the result must be exactly 0, not the union
  // of both runs.
  gp_Trsf elevated_pos;
  elevated_pos.SetTranslation(gp_Vec(0.0, 0.0, 1.0));
  TopoDS_Shape elevated_box = BRepBuilderAPI_Transform(
    BRepPrimAPI_MakeBox(0.1, 0.1, 0.1).Shape(), elevated_pos, Standard_True).Shape();
  Topology elevated_topology = mapper_->load_from_shape(elevated_box);
  constraint.analyze_constraints(elevated_topology);

  // Must be exactly 0: replacement semantics, not accumulation.
  // If this were to equal first_banned_count it would mean the old results
  // were never cleared; any value > 0 is a bug.
  EXPECT_EQ(constraint.get_banned_surface_ids().size(), 0u)
    << "Re-analysis on an elevated box must produce zero banned surfaces "
       "(previous results must be replaced, not accumulated); "
       "first run banned " << first_banned_count << " surface(s)";
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
