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
#include <rclcpp/rclcpp.hpp>
#include "hold_and_weld_application/kinematics/urdf_parser.hpp"

using hold_and_weld::kinematics::URDFParser;

namespace
{
constexpr double WIRE_TIP_OFFSET_MIN = 0.05;  // meters
constexpr double WIRE_TIP_OFFSET_MAX = 0.5;   // meters
}  // namespace

class URDFParserTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  void TearDown() override
  {
    rclcpp::shutdown();
  }
};

TEST_F(URDFParserTest, ParsesRobotChain) {
  URDFParser parser;

  auto chain = parser.extract_joint_chain(
    "package://hold_and_weld_description/urdf/dual_robot.xacro",
    "robot2_base_link",
    "robot2_wire_tip"
  );

  EXPECT_EQ(chain.dof(), 6);
  EXPECT_EQ(chain.actuated_joints.size(), 6);
  EXPECT_EQ(chain.base_link, "robot2_base_link");
  EXPECT_EQ(chain.tip_link, "robot2_wire_tip");

  EXPECT_EQ(chain.actuated_joints[0].name, "robot2_joint_1");
  EXPECT_EQ(chain.actuated_joints[1].name, "robot2_joint_2");
  EXPECT_EQ(chain.actuated_joints[2].name, "robot2_joint_3");
  EXPECT_EQ(chain.actuated_joints[3].name, "robot2_joint_4");
  EXPECT_EQ(chain.actuated_joints[4].name, "robot2_joint_5");
  EXPECT_EQ(chain.actuated_joints[5].name, "robot2_joint_6");

  for (const auto & joint : chain.actuated_joints) {
    EXPECT_TRUE(joint.is_revolute);
    EXPECT_LT(joint.q_min, joint.q_max);
    EXPECT_NEAR(joint.axis.norm(), 1.0, 1e-6);
  }
}

TEST_F(URDFParserTest, ExtractsToolTransform) {
  URDFParser parser;

  auto chain = parser.extract_joint_chain(
    "package://hold_and_weld_description/urdf/dual_robot.xacro",
    "robot2_base_link",
    "robot2_wire_tip"
  );

  Eigen::Isometry3d identity = Eigen::Isometry3d::Identity();
  EXPECT_FALSE(chain.tool_transform.isApprox(identity, 1e-6));

  double tool_length = chain.tool_transform.translation().norm();
  EXPECT_GT(tool_length, WIRE_TIP_OFFSET_MIN);
  EXPECT_LT(tool_length, WIRE_TIP_OFFSET_MAX);
}

TEST_F(URDFParserTest, RejectsEmptyInputs) {
  URDFParser parser;

  EXPECT_THROW(
    parser.extract_joint_chain("", "robot2_base_link", "robot2_wire_tip"),
    std::invalid_argument
  );

  EXPECT_THROW(
    parser.extract_joint_chain(
      "package://hold_and_weld_description/urdf/dual_robot.xacro", "", "robot2_wire_tip"
    ),
    std::invalid_argument
  );

  EXPECT_THROW(
    parser.extract_joint_chain(
      "package://hold_and_weld_description/urdf/dual_robot.xacro", "robot2_base_link", ""
    ),
    std::invalid_argument
  );
}

TEST_F(URDFParserTest, RejectsInvalidLinks) {
  URDFParser parser;

  EXPECT_THROW(
    parser.extract_joint_chain(
      "package://hold_and_weld_description/urdf/dual_robot.xacro",
      "nonexistent_base", "robot2_wire_tip"
    ),
    std::runtime_error
  );

  EXPECT_THROW(
    parser.extract_joint_chain(
      "package://hold_and_weld_description/urdf/dual_robot.xacro",
      "robot2_base_link", "nonexistent_tip"
    ),
    std::runtime_error
  );
}

TEST_F(URDFParserTest, RejectsInvalidPaths) {
  URDFParser parser;

  EXPECT_THROW(
    parser.extract_joint_chain(
      "package://nonexistent_package/urdf/robot.urdf", "base", "tip"
    ),
    std::runtime_error
  );
}

TEST_F(URDFParserTest, Rejects7DOF) {
  URDFParser parser;

  EXPECT_THROW(
    parser.extract_joint_chain(
      "package://hold_and_weld_description/urdf/dual_robot.xacro",
      "robot1_rail_base", "robot1_tool0"
    ),
    std::runtime_error
  );
}

TEST_F(URDFParserTest, HandlesFixedJoints)
{
  URDFParser parser;

  // Extract chain that includes fixed joints (e.g., flange to tool mount)
  auto chain = parser.extract_joint_chain(
    "package://hold_and_weld_description/urdf/dual_robot.xacro",
    "robot2_base_link",
    "robot2_wire_tip"
  );

  // Fixed joints should be collapsed into tool transform, not counted as actuated
  EXPECT_EQ(chain.dof(), 6) << "Fixed joints should not increase DOF";

  // Tool transform should accumulate all fixed joint transforms
  EXPECT_GT(chain.tool_transform.translation().norm(), 0.0)
    << "Tool transform should include fixed joint offsets";
}

TEST_F(URDFParserTest, ParsesDeterministically)
{
  URDFParser parser;

  auto chain1 = parser.extract_joint_chain(
    "package://hold_and_weld_description/urdf/dual_robot.xacro",
    "robot2_base_link", "robot2_wire_tip"
  );

  auto chain2 = parser.extract_joint_chain(
    "package://hold_and_weld_description/urdf/dual_robot.xacro",
    "robot2_base_link", "robot2_wire_tip"
  );

  EXPECT_EQ(chain1.dof(), chain2.dof());
  EXPECT_EQ(chain1.actuated_joints.size(), chain2.actuated_joints.size());

  for (size_t i = 0; i < chain1.actuated_joints.size(); ++i) {
    EXPECT_EQ(chain1.actuated_joints[i].name, chain2.actuated_joints[i].name);
    EXPECT_NEAR(chain1.actuated_joints[i].q_min, chain2.actuated_joints[i].q_min, 1e-9);
    EXPECT_NEAR(chain1.actuated_joints[i].q_max, chain2.actuated_joints[i].q_max, 1e-9);
    EXPECT_TRUE(chain1.actuated_joints[i].axis.isApprox(chain2.actuated_joints[i].axis, 1e-9));
    EXPECT_TRUE(chain1.actuated_joints[i].origin_transform.isApprox(
      chain2.actuated_joints[i].origin_transform, 1e-9));
  }

  EXPECT_TRUE(chain1.tool_transform.isApprox(chain2.tool_transform, 1e-9));
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
