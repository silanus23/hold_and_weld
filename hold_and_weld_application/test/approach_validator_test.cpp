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

#include <limits>
#include <memory>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <rclcpp/rclcpp.hpp>

#include "hold_and_weld_application/kinematics/approach_validator.hpp"
#include "hold_and_weld_application/kinematics/ceres_ik_solver.hpp"
#include "hold_and_weld_application/kinematics/kinematics_solver.hpp"
#include "hold_and_weld_application/kinematics/urdf_parser.hpp"

using hold_and_weld::kinematics::ApproachValidator;
using hold_and_weld::kinematics::ApproachValidatorParams;
using hold_and_weld::kinematics::CeresIKSolver;
using hold_and_weld::kinematics::KinematicsSolver;
using hold_and_weld::kinematics::URDFParser;
using Vector6d = ApproachValidator::Vector6d;

class ApproachValidatorTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
    URDFParser parser;
    auto chain = parser.extract_joint_chain(
      "package://hold_and_weld_description/urdf/dual_robot.xacro",
      "robot2_base_link", "robot2_wire_tip");
    kin_ = std::make_shared<KinematicsSolver>(chain);
    ik_ = std::make_shared<CeresIKSolver>(kin_);

    // Short straight seam of reachable, well-conditioned poses.
    q_first_ << 0.1, 0.4, 0.3, 0.2, -1.0, 0.5;
    for (int i = 0; i < 4; ++i) {
      Vector6d q = q_first_;
      q(0) += 0.02 * i;
      seam_.poses.push_back(to_msg(q));
    }
    // Seed off the exact solution so IK has work to do.
    q_approach_ = q_first_;
    q_approach_(1) += 0.05;
    q_approach_(4) -= 0.05;
  }

  void TearDown() override
  {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }

  geometry_msgs::msg::Pose to_msg(const Vector6d & q) const
  {
    const Eigen::Isometry3d T =
      kin_->compute_fk(std::vector<double>(q.data(), q.data() + 6));
    const Eigen::Quaterniond r(T.rotation());
    geometry_msgs::msg::Pose p;
    p.position.x = T.translation().x();
    p.position.y = T.translation().y();
    p.position.z = T.translation().z();
    p.orientation.w = r.w();
    p.orientation.x = r.x();
    p.orientation.y = r.y();
    p.orientation.z = r.z();
    return p;
  }

  bool validate(const ApproachValidatorParams & params, const hold_and_weld::WeldSeam & seam)
  {
    ApproachValidator validator(kin_, ik_, params);
    validator.set_weld_seam(seam);
    return validator.is_approach_valid(q_approach_);
  }

  std::shared_ptr<KinematicsSolver> kin_;
  std::shared_ptr<CeresIKSolver> ik_;
  hold_and_weld::WeldSeam seam_;
  Vector6d q_first_;
  Vector6d q_approach_;
};

TEST_F(ApproachValidatorTest, AcceptsReachableSeam)
{
  EXPECT_TRUE(validate(ApproachValidatorParams{}, seam_));
}

TEST_F(ApproachValidatorTest, ConfiguredTolerancesAreApplied)
{
  // No IK solution lands within 1e-15 m of a target it was not seeded exactly on.
  ApproachValidatorParams first_tight;
  first_tight.first_point_tol_pos = 1e-15;
  EXPECT_FALSE(validate(first_tight, seam_));

  ApproachValidatorParams seam_tight;
  seam_tight.seam_tol_pos = 1e-15;
  EXPECT_FALSE(validate(seam_tight, seam_));
}

TEST_F(ApproachValidatorTest, NormalisesSeamQuaternions)
{
  hold_and_weld::WeldSeam scaled = seam_;
  for (auto & p : scaled.poses) {
    p.orientation.w *= 3.0;
    p.orientation.x *= 3.0;
    p.orientation.y *= 3.0;
    p.orientation.z *= 3.0;
  }
  EXPECT_TRUE(validate(ApproachValidatorParams{}, scaled));
}

TEST_F(ApproachValidatorTest, RejectsDegenerateSeamPoses)
{
  hold_and_weld::WeldSeam zero_quat = seam_;
  zero_quat.poses[2].orientation.w = 0.0;
  zero_quat.poses[2].orientation.x = 0.0;
  zero_quat.poses[2].orientation.y = 0.0;
  zero_quat.poses[2].orientation.z = 0.0;
  EXPECT_FALSE(validate(ApproachValidatorParams{}, zero_quat));

  hold_and_weld::WeldSeam nan_pos = seam_;
  nan_pos.poses[0].position.y = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(validate(ApproachValidatorParams{}, nan_pos));
}

TEST_F(ApproachValidatorTest, ConstructorRejectsInvalidArguments)
{
  EXPECT_THROW(ApproachValidator(nullptr, ik_, ApproachValidatorParams{}), std::invalid_argument);
  EXPECT_THROW(ApproachValidator(kin_, nullptr, ApproachValidatorParams{}), std::invalid_argument);

  ApproachValidatorParams p;
  p.seam_tol_rot = 0.0;
  EXPECT_THROW(ApproachValidator(kin_, ik_, p), std::invalid_argument);

  p = ApproachValidatorParams{};
  p.manipulability_threshold = -1.0;
  EXPECT_THROW(ApproachValidator(kin_, ik_, p), std::invalid_argument);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
