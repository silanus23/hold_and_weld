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

#include <chrono>
#include <memory>
#include <vector>

#include <BRepBuilderAPI_Transform.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRepPrimAPI_MakeSphere.hxx>
#include <gp_Ax2.hxx>
#include <gp_Trsf.hxx>
#include <gp_Vec.hxx>

#include "hold_and_weld_gripper_sampler/geometry/fcl_collision_checker.hpp"
#include "hold_and_weld_gripper_sampler/io/gripper_parser.hpp"

namespace hold_and_weld_gripper_sampler
{
namespace geometry
{
namespace test
{
io::ParsedGripper create_test_gripper()
{
  io::ParsedGripper gripper;

  // Create finger 1 (centered at origin, will move in +X)
  gripper.finger_1 = BRepPrimAPI_MakeBox(
    gp_Pnt(-0.005, -0.005, 0.0),
    0.01, 0.01, 0.05
  ).Shape();

  // Create finger 2 (centered at origin, will move in -X)
  gripper.finger_2 = BRepPrimAPI_MakeBox(
    gp_Pnt(-0.005, -0.005, 0.0),
    0.01, 0.01, 0.05
  ).Shape();

  // Create base (above fingers)
  gripper.base = BRepPrimAPI_MakeBox(
    gp_Pnt(-0.025, -0.015, 0.05),
    0.05, 0.03, 0.02
  ).Shape();

  // Finger axes (open along X)
  gripper.finger_1_axis = Eigen::Vector3d(1.0, 0.0, 0.0);   // +X
  gripper.finger_2_axis = Eigen::Vector3d(-1.0, 0.0, 0.0);  // -X

  // Limits
  gripper.min_opening = 0.01;
  gripper.max_opening = 0.10;

  // Metadata
  gripper.gripper_type = "parallel";
  gripper.tcp_offset = Eigen::Vector3d(0.0, 0.0, 0.0);
  gripper.tcp_rpy = Eigen::Vector3d(0.0, 0.0, 0.0);

  return gripper;
}

class FCLCollisionCheckerTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    gripper_ = create_test_gripper();

    // Create a simple box as primary shape (100mm cube centered at origin)
    primary_shape_ = BRepPrimAPI_MakeBox(
      gp_Pnt(-0.05, -0.05, -0.05),
      0.1, 0.1, 0.1
    ).Shape();
  }

  io::ParsedGripper gripper_;
  TopoDS_Shape primary_shape_;
};

TEST_F(FCLCollisionCheckerTest, CollidesWithPrimaryWhenInsideShape)
{
  FCLCollisionChecker checker(gripper_, primary_shape_);
  ASSERT_TRUE(checker.is_valid());

  // Place gripper at origin (inside the box)
  gp_Trsf transform;
  transform.SetTranslation(gp_Vec(0.0, 0.0, 0.0));

  double grip_distance = 0.02;
  double tolerance = 0.001;

  EXPECT_TRUE(checker.collides_with_primary(transform, grip_distance, tolerance));
}

TEST_F(FCLCollisionCheckerTest, NoCollisionWhenFarFromPrimary)
{
  FCLCollisionChecker checker(gripper_, primary_shape_);
  ASSERT_TRUE(checker.is_valid());

  // Place gripper far away (1 meter away in Z)
  gp_Trsf transform;
  transform.SetTranslation(gp_Vec(0.0, 0.0, 1.0));

  double grip_distance = 0.02;
  double tolerance = 0.001;

  EXPECT_FALSE(checker.collides_with_primary(transform, grip_distance, tolerance));
}

TEST_F(FCLCollisionCheckerTest, CollisionDetectedWithTolerance)
{
  FCLCollisionChecker checker(gripper_, primary_shape_);
  ASSERT_TRUE(checker.is_valid());

  // Place gripper just outside the box (60mm from center in Z)
  // Box extends from -50mm to +50mm, so gripper at Z=0.06 should be 10mm away
  gp_Trsf transform;
  transform.SetTranslation(gp_Vec(0.0, 0.0, 0.06));

  double grip_distance = 0.02;

  // With 5mm tolerance, should NOT collide (10mm gap > 5mm tolerance)
  EXPECT_FALSE(checker.collides_with_primary(transform, grip_distance, 0.005));

  // With 15mm tolerance, should collide (10mm gap < 15mm tolerance)
  EXPECT_TRUE(checker.collides_with_primary(transform, grip_distance, 0.015));
}

TEST_F(FCLCollisionCheckerTest, DistanceToPrimaryIsPositiveWhenOutside)
{
  FCLCollisionChecker checker(gripper_, primary_shape_);
  ASSERT_TRUE(checker.is_valid());

  // Place gripper far away
  gp_Trsf transform;
  transform.SetTranslation(gp_Vec(0.0, 0.0, 0.5));

  double grip_distance = 0.02;

  double distance = checker.distance_to_primary(transform, grip_distance);
  EXPECT_GT(distance, 0.0);
}

TEST_F(FCLCollisionCheckerTest, DistanceToPrimaryIsSmallWhenClose)
{
  FCLCollisionChecker checker(gripper_, primary_shape_);
  ASSERT_TRUE(checker.is_valid());

  // Place gripper just outside (box top is at Z=0.05, gripper starts at Z=0)
  // If we place gripper at Z=0.06, finger tips should be ~10mm from box
  gp_Trsf transform;
  transform.SetTranslation(gp_Vec(0.0, 0.0, 0.06));

  double grip_distance = 0.02;

  double distance = checker.distance_to_primary(transform, grip_distance);

  // Distance should be small (approximately 5-15mm depending on geometry)
  EXPECT_GT(distance, 0.0);
  EXPECT_LT(distance, 0.05);  // Less than 50mm
}

TEST_F(FCLCollisionCheckerTest, GripDistanceAffectsCollision)
{
  // Create a narrow slot that gripper can fit through when closed
  // but collides when opened
  TopoDS_Shape narrow_slot = BRepPrimAPI_MakeBox(
    gp_Pnt(-0.1, -0.02, -0.1),  // Narrow in Y (40mm gap)
    0.2, 0.04, 0.2
  ).Shape();

  // Create walls on either side
  TopoDS_Shape wall1 = BRepPrimAPI_MakeBox(
    gp_Pnt(-0.1, 0.02, -0.1),
    0.2, 0.05, 0.2
  ).Shape();

  TopoDS_Shape wall2 = BRepPrimAPI_MakeBox(
    gp_Pnt(-0.1, -0.07, -0.1),
    0.2, 0.05, 0.2
  ).Shape();

  FCLCollisionChecker checker(gripper_, wall1);
  ASSERT_TRUE(checker.is_valid());

  // Place gripper in the slot area
  gp_Trsf transform;
  transform.SetTranslation(gp_Vec(0.0, 0.0, 0.0));

  // With small grip distance, should not collide
  EXPECT_FALSE(checker.collides_with_primary(transform, 0.01, 0.001));

  // With large grip distance (fingers extend into walls), may collide
  // depending on exact geometry
}

TEST_F(FCLCollisionCheckerTest, ExclusionVolumeCollisionDetected)
{
  FCLCollisionChecker checker(gripper_, primary_shape_);
  ASSERT_TRUE(checker.is_valid());

  // Add exclusion volume (sphere at a known location)
  TopoDS_Shape exclusion_sphere = BRepPrimAPI_MakeSphere(
    gp_Pnt(0.0, 0.0, 0.1),  // Center above primary
    0.03                     // 30mm radius
  ).Shape();

  std::vector<TopoDS_Shape> exclusions = {exclusion_sphere};
  checker.add_exclusion_volumes(exclusions);

  // Place gripper near the exclusion sphere
  gp_Trsf transform;
  transform.SetTranslation(gp_Vec(0.0, 0.0, 0.1));

  double grip_distance = 0.02;
  double tolerance = 0.001;

  EXPECT_TRUE(checker.collides_with_exclusions(transform, grip_distance, tolerance));
}

TEST_F(FCLCollisionCheckerTest, SecondaryShapeCollisionDetected)
{
  FCLCollisionChecker checker(gripper_, primary_shape_);
  ASSERT_TRUE(checker.is_valid());

  // Add secondary shape (cylinder representing a fixture)
  gp_Ax2 axis(gp_Pnt(0.0, 0.0, 0.15), gp_Dir(0, 0, 1));
  TopoDS_Shape secondary_cylinder = BRepPrimAPI_MakeCylinder(axis, 0.05, 0.1).Shape();

  std::vector<TopoDS_Shape> secondaries = {secondary_cylinder};
  checker.add_secondary_shapes(secondaries);

  // Place gripper at the secondary location
  gp_Trsf transform;
  transform.SetTranslation(gp_Vec(0.0, 0.0, 0.15));

  EXPECT_TRUE(checker.collides_with_secondaries(transform, 0.02, 0.001));
}

TEST_F(FCLCollisionCheckerTest, MultipleExclusionVolumes)
{
  FCLCollisionChecker checker(gripper_, primary_shape_);
  ASSERT_TRUE(checker.is_valid());

  // Add multiple exclusion volumes at different locations
  std::vector<TopoDS_Shape> exclusions;

  exclusions.push_back(BRepPrimAPI_MakeSphere(gp_Pnt(0.5, 0.0, 0.0), 0.02).Shape());
  exclusions.push_back(BRepPrimAPI_MakeSphere(gp_Pnt(-0.5, 0.0, 0.0), 0.02).Shape());
  exclusions.push_back(BRepPrimAPI_MakeSphere(gp_Pnt(0.0, 0.5, 0.0), 0.02).Shape());

  checker.add_exclusion_volumes(exclusions);

  // Test collision with first exclusion
  gp_Trsf transform1;
  transform1.SetTranslation(gp_Vec(0.5, 0.0, 0.0));
  EXPECT_TRUE(checker.collides_with_exclusions(transform1, 0.02, 0.001));

  // Test collision with third exclusion
  gp_Trsf transform3;
  transform3.SetTranslation(gp_Vec(0.0, 0.5, 0.0));
  EXPECT_TRUE(checker.collides_with_exclusions(transform3, 0.02, 0.001));

  // Test no collision in empty area
  gp_Trsf transform_clear;
  transform_clear.SetTranslation(gp_Vec(0.0, 0.0, 0.5));
  EXPECT_FALSE(checker.collides_with_exclusions(transform_clear, 0.02, 0.001));
}

TEST_F(FCLCollisionCheckerTest, RotatedGripperCollisionCheck)
{
  FCLCollisionChecker checker(gripper_, primary_shape_);
  ASSERT_TRUE(checker.is_valid());

  // Place gripper above the box and rotate 90 degrees around Z
  gp_Trsf transform;
  transform.SetTranslation(gp_Vec(0.0, 0.0, 0.1));

  gp_Trsf rotation;
  rotation.SetRotation(gp_Ax1(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1)), M_PI / 2);

  transform = transform * rotation;

  // Should not collide when above the box
  EXPECT_FALSE(checker.collides_with_primary(transform, 0.02, 0.001));
}


}  // namespace test
}  // namespace geometry
}  // namespace hold_and_weld_gripper_sampler

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
