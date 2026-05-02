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

#include <cmath>
#include <memory>
#include <vector>

#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRepPrimAPI_MakeSphere.hxx>
#include <gp_Ax2.hxx>
#include <gp_Pnt.hxx>
#include <gp_Trsf.hxx>
#include <gp_Vec.hxx>

#include "hold_and_weld_gripper_sampler/collision/embree_mesh_query.hpp"
#include "hold_and_weld_gripper_sampler/collision/fcl_collision_checker.hpp"
#include "hold_and_weld_gripper_sampler/core/gripper.hpp"

using namespace hold_and_weld_gripper_sampler;        // NOLINT
using namespace hold_and_weld_gripper_sampler::geometry;  // NOLINT

namespace
{

// Box: 160 × 120 × 200 mm centred at origin
constexpr double kHalfX = 0.08, kHalfY = 0.06, kHalfZ = 0.10;

TopoDS_Shape make_prism()
{
  return BRepPrimAPI_MakeBox(
    gp_Pnt(-kHalfX, -kHalfY, -kHalfZ),
    2.0 * kHalfX, 2.0 * kHalfY, 2.0 * kHalfZ).Shape();
}

// Minimal parallel-jaw gripper:
//   finger_1 / finger_2 : 10×10×50 mm boxes at the joint origin (closed position)
//   base                : 50×30×20 mm box above the fingers
//   opening axes        : fingers spread along ±X
ParsedGripper make_gripper()
{
  ParsedGripper g;
  g.finger_1 = BRepPrimAPI_MakeBox(gp_Pnt(-0.005, -0.005, 0.0), 0.01, 0.01, 0.05).Shape();
  g.finger_2 = BRepPrimAPI_MakeBox(gp_Pnt(-0.005, -0.005, 0.0), 0.01, 0.01, 0.05).Shape();
  g.base = BRepPrimAPI_MakeBox(gp_Pnt(-0.025, -0.015, 0.05), 0.05, 0.03, 0.02).Shape();
  g.finger_1_axis = Eigen::Vector3d(1.0, 0.0, 0.0);
  g.finger_2_axis = Eigen::Vector3d(-1.0, 0.0, 0.0);
  g.max_opening = 0.10;
  g.gripper_type = "parallel";
  g.tcp_offset = Eigen::Vector3d::Zero();
  g.tcp_rpy = Eigen::Vector3d::Zero();
  return g;
}

// Build a pure-translation gp_Trsf
gp_Trsf translation(double x, double y, double z)
{
  gp_Trsf tf;
  tf.SetTranslation(gp_Vec(x, y, z));
  return tf;
}

// Build a gp_Trsf that translates to base_pos and rotates so local Z = z_dir.
// Y axis of the gripper frame is kept as world Y when possible.
gp_Trsf pose(const gp_Pnt & base_pos, const gp_Vec & z_dir)
{
  gp_Vec z = z_dir.Normalized();
  gp_Vec y(0.0, 1.0, 0.0);
  gp_Vec x = y.Crossed(z);
  if (x.Magnitude() < 1e-6) {x = gp_Vec(1.0, 0.0, 0.0);}
  x.Normalize();
  y = z.Crossed(x);
  y.Normalize();

  gp_Trsf tf;
  tf.SetValues(
    x.X(), y.X(), z.X(), base_pos.X(),
    x.Y(), y.Y(), z.Y(), base_pos.Y(),
    x.Z(), y.Z(), z.Z(), base_pos.Z());
  return tf;
}

}  // namespace

class FCLTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    gripper_ = make_gripper();
    // 100×100×100 mm box centred at origin
    primary_ = BRepPrimAPI_MakeBox(gp_Pnt(-0.05, -0.05, -0.05), 0.1, 0.1, 0.1).Shape();
  }

  ParsedGripper gripper_;
  TopoDS_Shape primary_;
};

// Checker must be usable immediately after construction.
TEST_F(FCLTest, IsValidAfterConstruction)
{
  FCLCollisionChecker checker(gripper_, primary_);
  EXPECT_TRUE(checker.is_valid());
}

// Gripper placed at origin (inside the primary box) must register as collision.
TEST_F(FCLTest, CollidesWhenInsidePrimary)
{
  FCLCollisionChecker checker(gripper_, primary_);
  EXPECT_TRUE(checker.collides_with_primary(translation(0.0, 0.0, 0.0), 0.02, 0.001));
}

// Gripper 1 m above the primary must be clear.
TEST_F(FCLTest, NoCollisionWhenFarFromPrimary)
{
  FCLCollisionChecker checker(gripper_, primary_);
  EXPECT_FALSE(checker.collides_with_primary(translation(0.0, 0.0, 1.0), 0.02, 0.001));
}

// Same position, opposite orientation: fingers toward primary collide, fingers away do not.
TEST_F(FCLTest, GripperOrientation_AffectsCollisionResult)
{
  FCLCollisionChecker checker(gripper_, primary_);

  // Base at z=0.08 m: fingers pointing -Z reach z=0.03 (inside primary), +Z stays clear.
  gp_Pnt base_pos(0.0, 0.0, 0.08);
  gp_Trsf toward = pose(base_pos, gp_Vec(0.0, 0.0, -1.0));
  gp_Trsf away = pose(base_pos, gp_Vec(0.0, 0.0, 1.0));

  EXPECT_TRUE(checker.collides_with_primary(toward, 0.02, 0.001));
  EXPECT_FALSE(checker.collides_with_primary(away, 0.02, 0.001));
}

// Exclusion sphere placed 100 mm above origin — gripper at that position must collide.
TEST_F(FCLTest, ExclusionVolumeDetected)
{
  FCLCollisionChecker checker(gripper_, primary_);
  TopoDS_Shape sphere = BRepPrimAPI_MakeSphere(gp_Pnt(0.0, 0.0, 0.1), 0.03).Shape();
  checker.add_exclusion_volumes({sphere});

  EXPECT_TRUE(checker.collides_with_exclusions(translation(0.0, 0.0, 0.1), 0.02, 0.001));
}

// Secondary cylinder placed 150 mm above origin — gripper at that position must collide.
TEST_F(FCLTest, SecondaryShapeDetected)
{
  FCLCollisionChecker checker(gripper_, primary_);
  gp_Ax2 axis(gp_Pnt(0.0, 0.0, 0.15), gp_Dir(0, 0, 1));
  TopoDS_Shape cyl = BRepPrimAPI_MakeCylinder(axis, 0.05, 0.1).Shape();
  checker.add_secondary_shapes({cyl});

  EXPECT_TRUE(checker.collides_with_secondaries(translation(0.0, 0.0, 0.15), 0.02, 0.001));
}

// Ground plane at z=0: gripper below the plane collides, gripper above does not.
TEST_F(FCLTest, GroundPlane_CollidesBelow_ClearAbove)
{
  FCLCollisionChecker checker(gripper_, primary_);
  checker.add_ground_plane(Eigen::Vector3d(0.0, 0.0, 1.0), 0.0);
  ASSERT_TRUE(checker.has_ground_plane());

  EXPECT_TRUE(checker.collides_with_ground(translation(0.0, 0.0, -0.2), 0.02, 0.001));
  EXPECT_FALSE(checker.collides_with_ground(translation(0.0, 0.0, 0.5), 0.02, 0.001));
}

class EmbreeTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    prism_ = make_prism();
    query_ = std::make_shared<EmbreeMeshQuery>(prism_);
  }

  TopoDS_Shape prism_;
  std::shared_ptr<EmbreeMeshQuery> query_;
};

// Scene must be ready to query immediately after construction.
TEST_F(EmbreeTest, IsValidAfterConstruction)
{
  EXPECT_TRUE(query_->is_valid());
  EXPECT_GT(query_->num_triangles(), 0u);
  EXPECT_GT(query_->num_vertices(), 0u);
}

// Ray fired from outside the +X face straight toward it must hit near the face boundary.
TEST_F(EmbreeTest, RayHitsFromOutside)
{
  gp_Pnt origin(kHalfX + 0.05, 0.0, 0.0);
  gp_Dir dir(-1.0, 0.0, 0.0);
  auto hit = query_->ray_intersect(origin, dir, 0.5);
  ASSERT_TRUE(hit.has_value());
  EXPECT_NEAR(hit->X(), kHalfX, 0.002);
}

// Ray offset to 70% of the face extent proves the intersector works across the full face.
TEST_F(EmbreeTest, RayHitsOffCenter)
{
  gp_Pnt origin(kHalfX + 0.05, kHalfY * 0.7, kHalfZ * 0.7);
  gp_Dir dir(-1.0, 0.0, 0.0);
  auto hit = query_->ray_intersect(origin, dir, 0.5);
  ASSERT_TRUE(hit.has_value());
  EXPECT_NEAR(hit->X(), kHalfX, 0.002);
}

// Ray fired upward from well above the prism must miss.
TEST_F(EmbreeTest, RayMissesInClearDirection)
{
  gp_Pnt origin(0.0, 0.0, kHalfZ + 1.0);
  gp_Dir dir(0.0, 0.0, 1.0);
  EXPECT_FALSE(query_->ray_intersect(origin, dir, 0.5).has_value());
}

// Ray just outside the +X face travelling parallel to it must never enter the prism.
TEST_F(EmbreeTest, RayParallelToFace_Misses)
{
  // Origin is offset by a small epsilon outside the face to avoid the numerically
  // degenerate case of a ray origin lying exactly on a mesh triangle boundary,
  // which can produce spurious Embree hits due to floating-point rounding.
  gp_Pnt origin(kHalfX + 1e-4, 0.0, 0.0);
  gp_Dir dir(0.0, 1.0, 0.0);
  EXPECT_FALSE(query_->ray_intersect(origin, dir, 1.0).has_value());
}

// Point at the prism centre must be detected as inside.
TEST_F(EmbreeTest, PointInsideDetected)
{
  EXPECT_TRUE(query_->point_inside(gp_Pnt(0.0, 0.0, 0.0)));
}

// Point 1 m away from the prism must not be detected as inside.
TEST_F(EmbreeTest, PointOutsideNotDetected)
{
  EXPECT_FALSE(query_->point_inside(gp_Pnt(1.0, 0.0, 0.0)));
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
