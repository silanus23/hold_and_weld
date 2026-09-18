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

#include <gp_Ax1.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>
#include <gp_Trsf.hxx>
#include <gp_Vec.hxx>

#include "hold_and_weld_gripper_sampler/collision/jaw_clearance_check.hpp"
#include "test_helpers.hpp"

using hold_and_weld_gripper_sampler::geometry::JawClearanceConfig;
using hold_and_weld_gripper_sampler::geometry::JawClearanceCheck;

namespace
{

// Jaw-clearance volume for the small gripper at an identity pose:
// TCP is at z=0.11, the cylinder runs back to z=0.01, radius = 0.04/2 + margin.
constexpr double kFingerLength = 0.10;
constexpr double kGripDistance = 0.04;
constexpr double kMargin = 0.005;

}  // namespace

class JawClearanceCheckTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    gripper_ = create_small_gripper();
    // A plate well clear of the jaws: the primary must never drive a rejection.
    primary_ = create_box_at_helper(0.20, 0.20, 0.02, -0.10, -0.10, 0.11);
  }

  std::shared_ptr<JawClearanceCheck> make_check(
    const std::shared_ptr<FCLCollisionChecker> & checker,
    double margin = kMargin,
    bool enabled = true)
  {
    JawClearanceConfig config;
    config.enabled = enabled;
    config.clearance_margin = margin;

    auto check = std::make_shared<JawClearanceCheck>(
      config, gripper_, kFingerLength);
    check->set_fcl_checker(checker);
    return check;
  }

  ParsedGripper gripper_;
  TopoDS_Shape primary_;
  gp_Trsf identity_;
};

TEST_F(JawClearanceCheckTest, ClearMouthIsKept)
{
  auto checker = make_fcl_checker(gripper_, primary_);
  auto check = make_check(checker);

  EXPECT_FALSE(check->intrudes(identity_, kGripDistance));
}

TEST_F(JawClearanceCheckTest, SecondaryInMouthIsRejected)
{
  auto checker = make_fcl_checker(gripper_, primary_);
  // Nearest face 10mm off the jaw axis, mid-mouth — inside the 25mm radius.
  checker->add_secondary_shapes({create_box_at_helper(0.02, 0.02, 0.02, 0.01, 0.0, 0.05)});
  auto check = make_check(checker);

  EXPECT_TRUE(check->intrudes(identity_, kGripDistance));
}

TEST_F(JawClearanceCheckTest, SecondaryOutsideRadiusIsKept)
{
  auto checker = make_fcl_checker(gripper_, primary_);
  checker->add_secondary_shapes({create_box_at_helper(0.02, 0.02, 0.02, 0.10, 0.0, 0.05)});
  auto check = make_check(checker);

  EXPECT_FALSE(check->intrudes(identity_, kGripDistance));
}

TEST_F(JawClearanceCheckTest, MarginDecidesBorderlineObstacle)
{
  auto checker = make_fcl_checker(gripper_, primary_);
  // Nearest face at 30mm: outside a 25mm radius, inside a 35mm one.
  checker->add_secondary_shapes({create_box_at_helper(0.02, 0.02, 0.02, 0.03, 0.0, 0.05)});

  EXPECT_FALSE(make_check(checker, 0.005)->intrudes(identity_, kGripDistance));
  EXPECT_TRUE(make_check(checker, 0.015)->intrudes(identity_, kGripDistance));
}

TEST_F(JawClearanceCheckTest, ObstacleBeyondCylinderLengthIsKept)
{
  auto checker = make_fcl_checker(gripper_, primary_);
  // On the axis, but behind the cylinder (it spans z=0.01..0.11).
  checker->add_secondary_shapes({create_box_at_helper(0.02, 0.02, 0.02, 0.0, 0.0, -0.05)});
  auto check = make_check(checker);

  EXPECT_FALSE(check->intrudes(identity_, kGripDistance));
}

TEST_F(JawClearanceCheckTest, ExclusionVolumeInMouthIsRejected)
{
  auto checker = make_fcl_checker(gripper_, primary_);
  checker->add_exclusion_volumes({create_box_at_helper(0.02, 0.02, 0.02, 0.01, 0.0, 0.05)});
  auto check = make_check(checker);

  EXPECT_TRUE(check->intrudes(identity_, kGripDistance));
}

TEST_F(JawClearanceCheckTest, GroundIsNotTested)
{
  auto checker = make_fcl_checker(gripper_, primary_);
  // Solid below z=0.05, slicing the cylinder in half. The gripper is checked against
  // the ground with its real geometry elsewhere; this test must stay out of it,
  // or every side grasp near the floor is lost to the round envelope.
  checker->add_ground_plane(Eigen::Vector3d(0.0, 0.0, 1.0), 0.05);
  auto check = make_check(checker);

  EXPECT_FALSE(check->intrudes(identity_, kGripDistance));
}

TEST_F(JawClearanceCheckTest, VolumeFollowsTheGripperPose)
{
  auto checker = make_fcl_checker(gripper_, primary_);
  // Obstacle on the +X axis, where the jaw axis points after a -90 deg Y rotation.
  checker->add_secondary_shapes({create_box_at_helper(0.02, 0.02, 0.02, 0.05, 0.0, 0.0)});
  auto check = make_check(checker);

  // Identity pose: the jaw axis runs along +Z and misses the obstacle.
  EXPECT_FALSE(check->intrudes(identity_, kGripDistance));

  gp_Trsf rotated;
  rotated.SetRotation(gp_Ax1(gp_Pnt(0, 0, 0), gp_Dir(0, 1, 0)), M_PI / 2.0);
  EXPECT_TRUE(check->intrudes(rotated, kGripDistance));
}

TEST_F(JawClearanceCheckTest, DisabledCheckKeepsEverything)
{
  auto checker = make_fcl_checker(gripper_, primary_);
  checker->add_secondary_shapes({create_box_at_helper(0.02, 0.02, 0.02, 0.01, 0.0, 0.05)});
  auto check = make_check(checker, kMargin, false);

  EXPECT_FALSE(check->intrudes(identity_, kGripDistance));
}

TEST_F(JawClearanceCheckTest, MissingCheckerRejectsConservatively)
{
  JawClearanceConfig config;
  config.enabled = true;
  config.clearance_margin = kMargin;
  JawClearanceCheck check(config, gripper_, kFingerLength);

  EXPECT_TRUE(check.intrudes(identity_, kGripDistance));
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
