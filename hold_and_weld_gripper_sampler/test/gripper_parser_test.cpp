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

#include "hold_and_weld_gripper_sampler/core/gripper.hpp"
#include "hold_and_weld_gripper_sampler/io/gripper_parser.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <ostream>
#include <string>
#include <vector>

#include <Bnd_Box.hxx>
#include <BRepBndLib.hxx>

using hold_and_weld_gripper_sampler::ParsedGripper;
using hold_and_weld_gripper_sampler::io::GripperParser;

namespace test_constants
{
constexpr double kPositionTolerance = 1e-6;
constexpr double kUnitMagnitude = 1.0;

constexpr double kValidGripperMaxOpening = 0.30;
constexpr double kValidGripperTcpOffsetZ = -0.12;

constexpr double kCylinderGripperMaxOpening = 0.10;

const char * const kDefaultGripperType = "parallel";
const char * const kValidBaseLinkName = "gripper_base";
const char * const kValidFinger1LinkName = "left_finger";
const char * const kValidFinger2LinkName = "right_finger";
const char * const kValidFinger1JointName = "left_finger_joint";
const char * const kValidFinger2JointName = "right_finger_joint";
}  // namespace test_constants

namespace
{

/**
 * @brief Valid gripper URDF following the hold_and_weld convention
 */
const char VALID_GRIPPER_URDF[] =
  R"(
<?xml version="1.0"?>
<robot name="test_gripper">
  <gripper_metadata>
    <base_link name="gripper_base"/>
    <finger finger_id="1" link="left_finger" joint="left_finger_joint"/>
    <finger finger_id="2" link="right_finger" joint="right_finger_joint"/>
    <gripper_type>parallel</gripper_type>
    <tcp_offset xyz="0 0 -0.12" rpy="0 0 0"/>
  </gripper_metadata>

  <link name="gripper_base">
    <collision>
      <geometry>
        <box size="0.08 0.12 0.04"/>
      </geometry>
    </collision>
  </link>

  <link name="left_finger">
    <collision>
      <origin xyz="0 0 -0.1" rpy="0 0 0"/>
      <geometry>
        <box size="0.03 0.04 0.20"/>
      </geometry>
    </collision>
  </link>

  <link name="right_finger">
    <collision>
      <origin xyz="0 0 -0.1" rpy="0 0 0"/>
      <geometry>
        <box size="0.03 0.04 0.20"/>
      </geometry>
    </collision>
  </link>

  <joint name="left_finger_joint" type="prismatic">
    <parent link="gripper_base"/>
    <child link="left_finger"/>
    <origin xyz="0 0.03 -0.02" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="0.0" upper="0.15" effort="100" velocity="0.1"/>
  </joint>

  <joint name="right_finger_joint" type="prismatic">
    <parent link="gripper_base"/>
    <child link="right_finger"/>
    <origin xyz="0 -0.03 -0.02" rpy="0 0 0"/>
    <axis xyz="0 -1 0"/>
    <limit lower="0.0" upper="0.15" effort="100" velocity="0.1"/>
  </joint>
</robot>
)";

/**
 * @brief Gripper URDF with cylinder fingers
 */
const char CYLINDER_GRIPPER_URDF[] =
  R"(
<?xml version="1.0"?>
<robot name="cylinder_gripper">
  <gripper_metadata>
    <base_link name="base"/>
    <finger finger_id="1" link="finger_1" joint="finger_1_joint"/>
    <finger finger_id="2" link="finger_2" joint="finger_2_joint"/>
    <gripper_type>parallel</gripper_type>
    <tcp_offset xyz="0 0 -0.08" rpy="0 0 0"/>
  </gripper_metadata>

  <link name="base">
    <collision>
      <geometry>
        <cylinder radius="0.04" length="0.06"/>
      </geometry>
    </collision>
  </link>

  <link name="finger_1">
    <collision>
      <geometry>
        <cylinder radius="0.01" length="0.10"/>
      </geometry>
    </collision>
  </link>

  <link name="finger_2">
    <collision>
      <geometry>
        <cylinder radius="0.01" length="0.10"/>
      </geometry>
    </collision>
  </link>

  <joint name="finger_1_joint" type="prismatic">
    <parent link="base"/>
    <child link="finger_1"/>
    <origin xyz="0.03 0 0" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="0.0" upper="0.05" effort="50" velocity="0.05"/>
  </joint>

  <joint name="finger_2_joint" type="prismatic">
    <parent link="base"/>
    <child link="finger_2"/>
    <origin xyz="-0.03 0 0" rpy="0 0 0"/>
    <axis xyz="-1 0 0"/>
    <limit lower="0.0" upper="0.05" effort="50" velocity="0.05"/>
  </joint>
</robot>
)";

/**
 * @brief Gripper URDF missing metadata
 */
const char NO_METADATA_URDF[] =
  R"(
<?xml version="1.0"?>
<robot name="no_metadata_gripper">
  <link name="base">
    <collision>
      <geometry>
        <box size="0.1 0.1 0.1"/>
      </geometry>
    </collision>
  </link>
</robot>
)";

/**
 * @brief Gripper URDF with missing finger definition
 */
const char MISSING_FINGER_URDF[] =
  R"(
<?xml version="1.0"?>
<robot name="missing_finger_gripper">
  <gripper_metadata>
    <base_link name="base"/>
    <finger finger_id="1" link="finger_1" joint="finger_1_joint"/>
    <!-- Missing finger_id="2" -->
    <gripper_type>parallel</gripper_type>
  </gripper_metadata>

  <link name="base">
    <collision>
      <geometry>
        <box size="0.1 0.1 0.1"/>
      </geometry>
    </collision>
  </link>

  <link name="finger_1">
    <collision>
      <geometry>
        <box size="0.02 0.02 0.1"/>
      </geometry>
    </collision>
  </link>

  <joint name="finger_1_joint" type="prismatic">
    <parent link="base"/>
    <child link="finger_1"/>
    <axis xyz="0 1 0"/>
    <limit lower="0.0" upper="0.05" effort="50" velocity="0.1"/>
  </joint>
</robot>
)";

/**
 * @brief Gripper with non-prismatic joint
 */
const char REVOLUTE_JOINT_URDF[] =
  R"(
<?xml version="1.0"?>
<robot name="revolute_gripper">
  <gripper_metadata>
    <base_link name="base"/>
    <finger finger_id="1" link="finger_1" joint="finger_1_joint"/>
    <finger finger_id="2" link="finger_2" joint="finger_2_joint"/>
  </gripper_metadata>

  <link name="base">
    <collision>
      <geometry>
        <box size="0.1 0.1 0.1"/>
      </geometry>
    </collision>
  </link>

  <link name="finger_1">
    <collision>
      <geometry>
        <box size="0.02 0.02 0.1"/>
      </geometry>
    </collision>
  </link>

  <link name="finger_2">
    <collision>
      <geometry>
        <box size="0.02 0.02 0.1"/>
      </geometry>
    </collision>
  </link>

  <joint name="finger_1_joint" type="revolute">
    <parent link="base"/>
    <child link="finger_1"/>
    <axis xyz="0 0 1"/>
    <limit lower="-0.5" upper="0.5" effort="50" velocity="0.1"/>
  </joint>

  <joint name="finger_2_joint" type="prismatic">
    <parent link="base"/>
    <child link="finger_2"/>
    <axis xyz="0 -1 0"/>
    <limit lower="0.0" upper="0.05" effort="50" velocity="0.1"/>
  </joint>
</robot>
)";

/**
 * @brief Rethink electric gripper as its wrapper macro emits it: vendor right
 * finger flipped to an outward axis with [0, travel], mimic dropped, carriage
 * box left out and tip-link collision merged into the finger links
 */
const char RETHINK_WRAPPER_URDF[] =
  R"(
<?xml version="1.0"?>
<robot name="rethink_electric_gripper">
  <gripper_metadata>
    <base_link name="gripper_base"/>
    <finger finger_id="1" link="left_finger" joint="left_finger_joint"/>
    <finger finger_id="2" link="right_finger" joint="right_finger_joint"/>
    <gripper_type>parallel</gripper_type>
    <tcp_offset xyz="0 0 -0.1177" rpy="0 0 0"/>
  </gripper_metadata>

  <link name="gripper_base">
    <collision>
      <origin xyz="0 0 0" rpy="1.5707963 0 0"/>
      <geometry>
        <cylinder radius="0.029" length="0.1"/>
      </geometry>
    </collision>
  </link>

  <link name="left_finger">
    <collision>
      <origin xyz="0 0.01725 -0.0615" rpy="0 0 0"/>
      <geometry>
        <box size="0.01 0.0135 0.1127"/>
      </geometry>
    </collision>
    <collision>
      <origin xyz="0 0.01275 -0.0977" rpy="0 0 0"/>
      <geometry>
        <box size="0.042 0.0065 0.037"/>
      </geometry>
    </collision>
  </link>

  <link name="right_finger">
    <collision>
      <origin xyz="0 -0.01725 -0.0615" rpy="0 0 0"/>
      <geometry>
        <box size="0.01 0.0135 0.1127"/>
      </geometry>
    </collision>
    <collision>
      <origin xyz="0 -0.01275 -0.0977" rpy="0 0 0"/>
      <geometry>
        <box size="0.042 0.0065 0.037"/>
      </geometry>
    </collision>
  </link>

  <joint name="left_finger_joint" type="prismatic">
    <parent link="gripper_base"/>
    <child link="left_finger"/>
    <origin xyz="0 -0.0015 -0.02" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="0.0" upper="0.020833" effort="20" velocity="0.1"/>
  </joint>

  <joint name="right_finger_joint" type="prismatic">
    <parent link="gripper_base"/>
    <child link="right_finger"/>
    <origin xyz="0 0.0015 -0.02" rpy="0 0 0"/>
    <axis xyz="0 -1 0"/>
    <limit lower="0.0" upper="0.020833" effort="20" velocity="0.1"/>
  </joint>
</robot>
)";

/**
 * @brief One finger joint of a convention-test gripper; defaults are canonical
 */
struct FingerJointSpec
{
  std::string origin_xyz;
  std::string axis_xyz;
  std::string parent = "gripper_base";
  std::string lower = "0.0";
  std::string upper = "0.02";
  std::string extra;
};

/**
 * @brief Two-finger gripper URDF built from per-joint specs, 0.02 m cube fingers
 */
std::string make_gripper_urdf(
  const FingerJointSpec & left,
  const FingerJointSpec & right,
  const std::string & finger_geometry = "<box size=\"0.02 0.02 0.02\"/>")
{
  auto finger_link = [&](const std::string & name) {
      return "  <link name=\"" + name + "\">\n"
             "    <collision><geometry>" + finger_geometry + "</geometry></collision>\n"
             "  </link>\n";
    };
  auto finger_joint = [](const std::string & name, const std::string & child,
    const FingerJointSpec & spec) {
      return "  <joint name=\"" + name + "\" type=\"prismatic\">\n"
             "    <parent link=\"" + spec.parent + "\"/>\n"
             "    <child link=\"" + child + "\"/>\n"
             "    <origin xyz=\"" + spec.origin_xyz + "\" rpy=\"0 0 0\"/>\n"
             "    <axis xyz=\"" + spec.axis_xyz + "\"/>\n"
             "    <limit lower=\"" + spec.lower + "\" upper=\"" + spec.upper +
             "\" effort=\"50\" velocity=\"0.1\"/>\n"
             "    " + spec.extra + "\n"
             "  </joint>\n";
    };
  return
    "<?xml version=\"1.0\"?>\n"
    "<robot name=\"convention_gripper\">\n"
    "  <gripper_metadata>\n"
    "    <base_link name=\"gripper_base\"/>\n"
    "    <finger finger_id=\"1\" link=\"left_finger\" joint=\"left_finger_joint\"/>\n"
    "    <finger finger_id=\"2\" link=\"right_finger\" joint=\"right_finger_joint\"/>\n"
    "  </gripper_metadata>\n"
    "  <link name=\"gripper_base\">\n"
    "    <collision><geometry><box size=\"0.1 0.1 0.02\"/></geometry></collision>\n"
    "  </link>\n" +
    finger_link("left_finger") + finger_link("right_finger") +
    finger_joint("left_finger_joint", "left_finger", left) +
    finger_joint("right_finger_joint", "right_finger", right) +
    "</robot>\n";
}

FingerJointSpec canonical_left()
{
  FingerJointSpec spec;
  spec.origin_xyz = "0 0.02 -0.02";
  spec.axis_xyz = "0 1 0";
  return spec;
}

FingerJointSpec canonical_right()
{
  FingerJointSpec spec;
  spec.origin_xyz = "0 -0.02 -0.02";
  spec.axis_xyz = "0 -1 0";
  return spec;
}

}  // namespace

class GripperParserTest : public ::testing::Test
{
protected:
  GripperParser parser_;
};

class GripperParserThrowTest : public ::testing::TestWithParam<const char *>
{
protected:
  GripperParser parser_;
};

TEST_P(GripperParserThrowTest, ParseFromUrdfString_WithInvalidInput_ThrowsRuntimeError)
{
  EXPECT_THROW(
    parser_.parse_from_urdf_string(GetParam()),
    std::runtime_error
  );
}

INSTANTIATE_TEST_SUITE_P(
  InvalidInputs,
  GripperParserThrowTest,
  ::testing::Values(
    NO_METADATA_URDF,
    MISSING_FINGER_URDF,
    REVOLUTE_JOINT_URDF,
    "not valid xml at all <><>",
    "<?xml version=\"1.0\"?><something></something>",
    R"(
<?xml version="1.0"?>
<robot name="test">
  <gripper_metadata>
    <base_link name="nonexistent_base"/>
    <finger finger_id="1" link="finger_1" joint="joint_1"/>
    <finger finger_id="2" link="finger_2" joint="joint_2"/>
  </gripper_metadata>
</robot>
)"
  )
);

TEST_F(GripperParserTest, ParseFromUrdfString_WithValidGripper_ExtractsAllFieldsCorrectly)
{
  ParsedGripper gripper = parser_.parse_from_urdf_string(VALID_GRIPPER_URDF);

  EXPECT_EQ(gripper.base_link_name, test_constants::kValidBaseLinkName);
  EXPECT_EQ(gripper.finger_1_link_name, test_constants::kValidFinger1LinkName);
  EXPECT_EQ(gripper.finger_2_link_name, test_constants::kValidFinger2LinkName);

  EXPECT_EQ(gripper.finger_1_joint_name, test_constants::kValidFinger1JointName);
  EXPECT_EQ(gripper.finger_2_joint_name, test_constants::kValidFinger2JointName);

  EXPECT_EQ(gripper.gripper_type, test_constants::kDefaultGripperType);

  // left finger moves along +Y, right finger moves along -Y
  EXPECT_NEAR(gripper.finger_1_axis.x(), 0.0, test_constants::kPositionTolerance);
  EXPECT_NEAR(gripper.finger_1_axis.y(), 1.0, test_constants::kPositionTolerance);
  EXPECT_NEAR(gripper.finger_1_axis.z(), 0.0, test_constants::kPositionTolerance);

  EXPECT_NEAR(gripper.finger_2_axis.x(), 0.0, test_constants::kPositionTolerance);
  EXPECT_NEAR(gripper.finger_2_axis.y(), -1.0, test_constants::kPositionTolerance);
  EXPECT_NEAR(gripper.finger_2_axis.z(), 0.0, test_constants::kPositionTolerance);

  // max = 2 * 0.15 = 0.30 (min is always 0, not stored)
  EXPECT_NEAR(gripper.max_opening, test_constants::kValidGripperMaxOpening,
    test_constants::kPositionTolerance);

  EXPECT_NEAR(gripper.tcp_offset.x(), 0.0, test_constants::kPositionTolerance);
  EXPECT_NEAR(gripper.tcp_offset.y(), 0.0, test_constants::kPositionTolerance);
  EXPECT_NEAR(gripper.tcp_offset.z(), test_constants::kValidGripperTcpOffsetZ,
    test_constants::kPositionTolerance);

  EXPECT_NEAR(gripper.tcp_rpy.x(), 0.0, test_constants::kPositionTolerance);
  EXPECT_NEAR(gripper.tcp_rpy.y(), 0.0, test_constants::kPositionTolerance);
  EXPECT_NEAR(gripper.tcp_rpy.z(), 0.0, test_constants::kPositionTolerance);

  EXPECT_FALSE(gripper.base.IsNull());
  EXPECT_FALSE(gripper.finger_1.IsNull());
  EXPECT_FALSE(gripper.finger_2.IsNull());
}

TEST_F(GripperParserTest, ParseFromUrdfString_WithValidGripper_CreatesCorrectGeometry)
{
  ParsedGripper gripper = parser_.parse_from_urdf_string(VALID_GRIPPER_URDF);

  // Base is a box with size="0.08 0.12 0.04"
  Bnd_Box base_bbox;
  BRepBndLib::Add(gripper.base, base_bbox);

  double xmin, ymin, zmin, xmax, ymax, zmax;
  base_bbox.Get(xmin, ymin, zmin, xmax, ymax, zmax);

  double base_x_size = xmax - xmin;
  double base_y_size = ymax - ymin;
  double base_z_size = zmax - zmin;

  EXPECT_NEAR(base_x_size, 0.08, test_constants::kPositionTolerance);
  EXPECT_NEAR(base_y_size, 0.12, test_constants::kPositionTolerance);
  EXPECT_NEAR(base_z_size, 0.04, test_constants::kPositionTolerance);

  // Each finger is a box with size="0.03 0.04 0.20", origin xyz="0 0 -0.1" (offset)
  Bnd_Box finger1_bbox;
  BRepBndLib::Add(gripper.finger_1, finger1_bbox);

  finger1_bbox.Get(xmin, ymin, zmin, xmax, ymax, zmax);

  double finger1_x_size = xmax - xmin;
  double finger1_y_size = ymax - ymin;
  double finger1_z_size = zmax - zmin;

  EXPECT_NEAR(finger1_x_size, 0.03, test_constants::kPositionTolerance);
  EXPECT_NEAR(finger1_y_size, 0.04, test_constants::kPositionTolerance);
  EXPECT_NEAR(finger1_z_size, 0.20, test_constants::kPositionTolerance);
}

TEST_F(GripperParserTest, ConfigureGripper_WithValidGripper_CreatesCompoundShape)
{
  ParsedGripper gripper = parser_.parse_from_urdf_string(VALID_GRIPPER_URDF);

  // min opening (closed) is always 0
  TopoDS_Shape closed_gripper = gripper.configure(0.0);
  EXPECT_FALSE(closed_gripper.IsNull());

  double mid_opening = gripper.max_opening / 2.0;
  TopoDS_Shape mid_gripper = gripper.configure(mid_opening);
  EXPECT_FALSE(mid_gripper.IsNull());

  TopoDS_Shape open_gripper = gripper.configure(gripper.max_opening);
  EXPECT_FALSE(open_gripper.IsNull());

  // Gripper opens along Y axis, so Y span must increase as it opens
  Bnd_Box closed_bbox, mid_bbox, open_bbox;
  BRepBndLib::Add(closed_gripper, closed_bbox);
  BRepBndLib::Add(mid_gripper, mid_bbox);
  BRepBndLib::Add(open_gripper, open_bbox);

  double xmin, ymin, zmin, xmax, ymax, zmax;

  closed_bbox.Get(xmin, ymin, zmin, xmax, ymax, zmax);
  double closed_y_span = ymax - ymin;

  mid_bbox.Get(xmin, ymin, zmin, xmax, ymax, zmax);
  double mid_y_span = ymax - ymin;

  open_bbox.Get(xmin, ymin, zmin, xmax, ymax, zmax);
  double open_y_span = ymax - ymin;

  EXPECT_LT(closed_y_span, mid_y_span);
  EXPECT_LT(mid_y_span, open_y_span);
}

TEST_F(GripperParserTest, ConfigureGripper_WithExcessiveOpening_ClampsToMaxOpening)
{
  ParsedGripper gripper = parser_.parse_from_urdf_string(VALID_GRIPPER_URDF);

  double excessive_opening = gripper.max_opening + 0.5;
  TopoDS_Shape clamped_gripper = gripper.configure(excessive_opening);
  TopoDS_Shape max_gripper = gripper.configure(gripper.max_opening);

  Bnd_Box clamped_bbox, max_bbox;
  BRepBndLib::Add(clamped_gripper, clamped_bbox);
  BRepBndLib::Add(max_gripper, max_bbox);

  double xmin, ymin, zmin, xmax, ymax, zmax;

  clamped_bbox.Get(xmin, ymin, zmin, xmax, ymax, zmax);
  double clamped_y_span = ymax - ymin;

  max_bbox.Get(xmin, ymin, zmin, xmax, ymax, zmax);
  double max_y_span = ymax - ymin;

  EXPECT_NEAR(clamped_y_span, max_y_span, test_constants::kPositionTolerance);
}

TEST_F(GripperParserTest, ParseFromUrdfString_WithCylinderGripper_ParsesCorrectly)
{
  ParsedGripper gripper = parser_.parse_from_urdf_string(CYLINDER_GRIPPER_URDF);

  EXPECT_FALSE(gripper.base.IsNull());
  EXPECT_FALSE(gripper.finger_1.IsNull());
  EXPECT_FALSE(gripper.finger_2.IsNull());

  // finger 1: +X, finger 2: -X
  EXPECT_NEAR(gripper.finger_1_axis.x(), 1.0, test_constants::kPositionTolerance);
  EXPECT_NEAR(gripper.finger_1_axis.y(), 0.0, test_constants::kPositionTolerance);
  EXPECT_NEAR(gripper.finger_1_axis.z(), 0.0, test_constants::kPositionTolerance);

  EXPECT_NEAR(gripper.finger_2_axis.x(), -1.0, test_constants::kPositionTolerance);
  EXPECT_NEAR(gripper.finger_2_axis.y(), 0.0, test_constants::kPositionTolerance);
  EXPECT_NEAR(gripper.finger_2_axis.z(), 0.0, test_constants::kPositionTolerance);

  // max = 2 * 0.05 = 0.10
  EXPECT_NEAR(gripper.max_opening, test_constants::kCylinderGripperMaxOpening,
    test_constants::kPositionTolerance);
}

TEST_F(GripperParserTest, ParseFromUrdfString_WithMissingOptionalFields_UsesDefaults)
{
  const char minimal_urdf[] =
    R"(
<?xml version="1.0"?>
<robot name="test_gripper">
  <gripper_metadata>
    <base_link name="base"/>
    <finger finger_id="1" link="f1" joint="j1"/>
    <finger finger_id="2" link="f2" joint="j2"/>
    <!-- No tcp_offset or gripper_type specified -->
  </gripper_metadata>

  <link name="base">
    <collision>
      <geometry>
        <box size="0.1 0.1 0.1"/>
      </geometry>
    </collision>
  </link>

  <link name="f1">
    <collision>
      <geometry>
        <box size="0.02 0.02 0.1"/>
      </geometry>
    </collision>
  </link>

  <link name="f2">
    <collision>
      <geometry>
        <box size="0.02 0.02 0.1"/>
      </geometry>
    </collision>
  </link>

  <joint name="j1" type="prismatic">
    <parent link="base"/>
    <child link="f1"/>
    <origin xyz="0 0.03 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="0.0" upper="0.05" effort="50" velocity="0.1"/>
  </joint>

  <joint name="j2" type="prismatic">
    <parent link="base"/>
    <child link="f2"/>
    <origin xyz="0 -0.03 0" rpy="0 0 0"/>
    <axis xyz="0 -1 0"/>
    <limit lower="0.0" upper="0.05" effort="50" velocity="0.1"/>
  </joint>
</robot>
)";

  ParsedGripper gripper = parser_.parse_from_urdf_string(minimal_urdf);

  // default TCP offset is zero
  EXPECT_NEAR(gripper.tcp_offset.x(), 0.0, test_constants::kPositionTolerance);
  EXPECT_NEAR(gripper.tcp_offset.y(), 0.0, test_constants::kPositionTolerance);
  EXPECT_NEAR(gripper.tcp_offset.z(), 0.0, test_constants::kPositionTolerance);

  EXPECT_NEAR(gripper.tcp_rpy.x(), 0.0, test_constants::kPositionTolerance);
  EXPECT_NEAR(gripper.tcp_rpy.y(), 0.0, test_constants::kPositionTolerance);
  EXPECT_NEAR(gripper.tcp_rpy.z(), 0.0, test_constants::kPositionTolerance);

  // default gripper type is "parallel"
  EXPECT_EQ(gripper.gripper_type, test_constants::kDefaultGripperType);
}

TEST_F(GripperParserTest, ParseFromUrdfString_WithNonNormalizedAxis_NormalizesToUnitVector)
{
  const char non_normalized_urdf[] =
    R"(
<?xml version="1.0"?>
<robot name="test_gripper">
  <gripper_metadata>
    <base_link name="base"/>
    <finger finger_id="1" link="f1" joint="j1"/>
    <finger finger_id="2" link="f2" joint="j2"/>
  </gripper_metadata>

  <link name="base">
    <collision>
      <geometry>
        <box size="0.1 0.1 0.1"/>
      </geometry>
    </collision>
  </link>

  <link name="f1">
    <collision>
      <geometry>
        <box size="0.02 0.02 0.1"/>
      </geometry>
    </collision>
  </link>

  <link name="f2">
    <collision>
      <geometry>
        <box size="0.02 0.02 0.1"/>
      </geometry>
    </collision>
  </link>

  <joint name="j1" type="prismatic">
    <parent link="base"/>
    <child link="f1"/>
    <origin xyz="0.03 0.04 0" rpy="0 0 0"/>
    <axis xyz="3 4 0"/>
    <limit lower="0.0" upper="0.05" effort="50" velocity="0.1"/>
  </joint>

  <joint name="j2" type="prismatic">
    <parent link="base"/>
    <child link="f2"/>
    <origin xyz="-0.03 -0.04 0" rpy="0 0 0"/>
    <axis xyz="-6 -8 0"/>
    <limit lower="0.0" upper="0.05" effort="50" velocity="0.1"/>
  </joint>
</robot>
)";

  ParsedGripper gripper = parser_.parse_from_urdf_string(non_normalized_urdf);

  // both axes normalized to unit vectors
  double finger_1_magnitude = gripper.finger_1_axis.norm();
  double finger_2_magnitude = gripper.finger_2_axis.norm();

  EXPECT_NEAR(finger_1_magnitude, test_constants::kUnitMagnitude,
    test_constants::kPositionTolerance);
  EXPECT_NEAR(finger_2_magnitude, test_constants::kUnitMagnitude,
    test_constants::kPositionTolerance);

  EXPECT_NEAR(gripper.finger_1_axis.x(), 0.6, test_constants::kPositionTolerance);
  EXPECT_NEAR(gripper.finger_1_axis.y(), 0.8, test_constants::kPositionTolerance);
  EXPECT_NEAR(gripper.finger_1_axis.z(), 0.0, test_constants::kPositionTolerance);

  EXPECT_NEAR(gripper.finger_2_axis.x(), -0.6, test_constants::kPositionTolerance);
  EXPECT_NEAR(gripper.finger_2_axis.y(), -0.8, test_constants::kPositionTolerance);
  EXPECT_NEAR(gripper.finger_2_axis.z(), 0.0, test_constants::kPositionTolerance);
}

TEST_F(GripperParserTest, ParseFromUrdfString_WithConventionBuilderDefaults_Parses)
{
  // Guards the builder itself: the malformed cases below differ from this by one field.
  ParsedGripper gripper = parser_.parse_from_urdf_string(
    make_gripper_urdf(canonical_left(), canonical_right()));

  EXPECT_NEAR(gripper.max_opening, 0.04, test_constants::kPositionTolerance);
}

TEST_F(GripperParserTest, ParseFromUrdfString_WithRethinkWrapper_ExtractsStrokeAndAxes)
{
  ParsedGripper gripper = parser_.parse_from_urdf_string(RETHINK_WRAPPER_URDF);

  EXPECT_NEAR(gripper.max_opening, 2.0 * 0.020833, test_constants::kPositionTolerance);

  EXPECT_NEAR(gripper.finger_1_axis.y(), 1.0, test_constants::kPositionTolerance);
  EXPECT_NEAR(gripper.finger_2_axis.y(), -1.0, test_constants::kPositionTolerance);

  EXPECT_NEAR(gripper.tcp_offset.z(), -0.1177, test_constants::kPositionTolerance);

  // Both collision elements of each finger (bar + merged tip) are kept.
  Bnd_Box finger1_bbox;
  BRepBndLib::Add(gripper.finger_1, finger1_bbox);
  double xmin, ymin, zmin, xmax, ymax, zmax;
  finger1_bbox.Get(xmin, ymin, zmin, xmax, ymax, zmax);
  EXPECT_NEAR(xmax - xmin, 0.042, test_constants::kPositionTolerance);
}

TEST_F(GripperParserTest, ParseFromUrdfString_WithMeshCollision_ReportsMeshAsTheCause)
{
  const std::string urdf = make_gripper_urdf(canonical_left(), canonical_right(),
      "<mesh filename=\"package://vendor/meshes/finger.stl\"/>");
  try {
    parser_.parse_from_urdf_string(urdf);
    FAIL() << "mesh collision was accepted";
  } catch (const std::runtime_error & e) {
    EXPECT_NE(std::string(e.what()).find("Mesh geometry not supported"), std::string::npos)
      << e.what();
  }
}

TEST_F(GripperParserTest, ParseFromUrdfString_WithClosingAxes_ReportsAxisDirectionAsTheCause)
{
  // Opposite axes, but each pointing toward the other finger
  FingerJointSpec left = canonical_left();
  FingerJointSpec right = canonical_right();
  left.axis_xyz = "0 -1 0";
  right.axis_xyz = "0 1 0";
  try {
    parser_.parse_from_urdf_string(make_gripper_urdf(left, right));
    FAIL() << "closing axes were accepted";
  } catch (const std::runtime_error & e) {
    EXPECT_NE(std::string(e.what()).find("opening direction"), std::string::npos) << e.what();
  }
}

struct ConventionViolation
{
  const char * name;
  std::string urdf;
};

void PrintTo(const ConventionViolation & violation, std::ostream * os)
{
  *os << violation.name;
}

class GripperParserConventionTest : public ::testing::TestWithParam<ConventionViolation>
{
protected:
  GripperParser parser_;
};

TEST_P(GripperParserConventionTest, ParseFromUrdfString_WithConventionViolation_Throws)
{
  EXPECT_THROW(
    parser_.parse_from_urdf_string(GetParam().urdf),
    std::runtime_error
  );
}

std::vector<ConventionViolation> convention_violations()
{
  std::vector<ConventionViolation> cases;

  FingerJointSpec mimic_right = canonical_right();
  mimic_right.extra = "<mimic joint=\"left_finger_joint\" multiplier=\"1\"/>";
  cases.push_back({"MimicOnFingerJoint", make_gripper_urdf(canonical_left(), mimic_right)});

  // Schunk PG70 as shipped: both fingers [-0.001, 0.0301]
  FingerJointSpec pg70_left = canonical_left();
  FingerJointSpec pg70_right = canonical_right();
  pg70_left.lower = pg70_right.lower = "-0.001";
  pg70_left.upper = pg70_right.upper = "0.0301";
  cases.push_back({"NonZeroLowerLimit", make_gripper_urdf(pg70_left, pg70_right)});

  // Rethink as shipped: right finger on the left finger's axis with [-travel, 0]
  FingerJointSpec rethink_right = canonical_right();
  rethink_right.axis_xyz = "0 1 0";
  rethink_right.lower = "-0.02";
  rethink_right.upper = "0.0";
  cases.push_back({"NegativeRangeFinger", make_gripper_urdf(canonical_left(), rethink_right)});

  FingerJointSpec longer_right = canonical_right();
  longer_right.upper = "0.03";
  cases.push_back({"UnequalUpperLimits", make_gripper_urdf(canonical_left(), longer_right)});

  FingerJointSpec same_axis_right = canonical_right();
  same_axis_right.axis_xyz = "0 1 0";
  cases.push_back({"NonOpposingAxes", make_gripper_urdf(canonical_left(), same_axis_right)});

  FingerJointSpec perpendicular_right = canonical_right();
  perpendicular_right.axis_xyz = "1 0 0";
  cases.push_back(
    {"PerpendicularAxes", make_gripper_urdf(canonical_left(), perpendicular_right)});

  FingerJointSpec chained_right = canonical_right();
  chained_right.parent = "left_finger";
  cases.push_back(
    {"FingerNotParentedToBase", make_gripper_urdf(canonical_left(), chained_right)});

  // Fingers 0.02 m wide, joint origins only 0.01 m apart: they interpenetrate when closed
  FingerJointSpec overlapping_left = canonical_left();
  FingerJointSpec overlapping_right = canonical_right();
  overlapping_left.origin_xyz = "0 0.005 -0.02";
  overlapping_right.origin_xyz = "0 -0.005 -0.02";
  cases.push_back(
    {"FingersOverlapWhenClosed", make_gripper_urdf(overlapping_left, overlapping_right)});

  cases.push_back(
    {"MeshFingerCollision", make_gripper_urdf(canonical_left(), canonical_right(),
      "<mesh filename=\"package://vendor/meshes/finger.stl\"/>")});

  return cases;
}

INSTANTIATE_TEST_SUITE_P(
  ConventionViolations,
  GripperParserConventionTest,
  ::testing::ValuesIn(convention_violations()),
  [](const ::testing::TestParamInfo<ConventionViolation> & info) {return info.param.name;}
);

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
