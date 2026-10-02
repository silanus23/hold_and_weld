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

#include <string>

#include "hold_and_weld_gripper_sampler/io/config_parser.hpp"

using hold_and_weld_gripper_sampler::io::ConfigParser;

namespace
{

// Smallest config the parser accepts; each test appends one section to it.
const char kBase[] =
  "primary:\n"
  "  urdf_path: /tmp/part.urdf\n"
  "gripper:\n"
  "  urdf_path: /tmp/gripper.urdf\n";

bool parses(const std::string & extra)
{
  ConfigParser parser;
  return parser.parse_string(kBase + extra).has_value();
}

}  // namespace

TEST(ConfigParserTest, ValidConfigParses)
{
  EXPECT_TRUE(parses(
      "secondaries:\n"
      "  - type: ground_plane\n"
      "    size_x: 1.0\n"
      "    size_y: 1.0\n"
      "exclusion_zones:\n"
      "  circles:\n"
      "    - center: [0.0, 0.0, 0.1]\n"
      "      normal: [0.0, 0.0, 1.0]\n"
      "      radius: 0.02\n"
      "      projection_depth: 0.02\n"
      "  polygons:\n"
      "    - corners: [[0.0, 0.0, 0.1], [0.1, 0.0, 0.1], [0.1, 0.1, 0.1]]\n"
      "      projection_depth: 0.02\n"
      "      clearance: 0.0\n"
      "  lines:\n"
      "    - start: [0.0, 0.0, 0.1]\n"
      "      end: [0.1, 0.0, 0.1]\n"
      "      exclusion_radius: 0.005\n"));
}

TEST(ConfigParserTest, GroundPlaneSizeMustBePositive)
{
  EXPECT_FALSE(parses(
      "secondaries:\n"
      "  - type: ground_plane\n"
      "    size_x: 0.0\n"
      "    size_y: 1.0\n"));
  EXPECT_FALSE(parses(
      "secondaries:\n"
      "  - type: ground_plane\n"
      "    size_x: 1.0\n"
      "    size_y: -1.0\n"));
}

TEST(ConfigParserTest, CircleRadiusDepthAndClearanceAreChecked)
{
  const std::string head =
    "exclusion_zones:\n"
    "  circles:\n"
    "    - center: [0.0, 0.0, 0.1]\n"
    "      normal: [0.0, 0.0, 1.0]\n";
  EXPECT_FALSE(parses(head + "      radius: 0.0\n      projection_depth: 0.02\n"));
  EXPECT_FALSE(parses(head + "      radius: 0.02\n      projection_depth: -0.01\n"));
  EXPECT_FALSE(parses(
      head + "      radius: 0.02\n      projection_depth: 0.02\n      clearance: -0.001\n"));
}

TEST(ConfigParserTest, PolygonNeedsThreeCornersAndPositiveDepth)
{
  EXPECT_FALSE(parses(
      "exclusion_zones:\n"
      "  polygons:\n"
      "    - corners: [[0.0, 0.0, 0.1], [0.1, 0.0, 0.1]]\n"
      "      projection_depth: 0.02\n"));
  EXPECT_FALSE(parses(
      "exclusion_zones:\n"
      "  polygons:\n"
      "    - corners: [[0.0, 0.0, 0.1], [0.1, 0.0, 0.1], [0.1, 0.1, 0.1]]\n"
      "      projection_depth: 0.0\n"));
}

// The polygon normal is taken from corners 0-2, so they must not be collinear.
TEST(ConfigParserTest, PolygonLeadingCornersMustNotBeCollinear)
{
  EXPECT_FALSE(parses(
      "exclusion_zones:\n"
      "  polygons:\n"
      "    - corners: [[0.03, 0.03, 0.1], [0.05, 0.03, 0.1], [0.07, 0.03, 0.1],\n"
      "                [0.07, 0.07, 0.1], [0.03, 0.07, 0.1]]\n"
      "      projection_depth: 0.02\n"));
}

TEST(ConfigParserTest, LineRadiusAndClearanceAreChecked)
{
  const std::string head =
    "exclusion_zones:\n"
    "  lines:\n"
    "    - start: [0.0, 0.0, 0.1]\n"
    "      end: [0.1, 0.0, 0.1]\n";
  EXPECT_FALSE(parses(head + "      exclusion_radius: 0.0\n"));
  EXPECT_FALSE(parses(head + "      exclusion_radius: 0.005\n      clearance: -0.01\n"));
}

// Each of these must be rejected at parse time, not fail later or silently become
// something else.
TEST(ConfigParserTest, NonFiniteDegenerateOrAmbiguousInputIsRejected)
{
  const std::string circle =
    "exclusion_zones:\n"
    "  circles:\n"
    "    - center: [0.0, 0.0, 0.1]\n"
    "      radius: 0.02\n"
    "      projection_depth: 0.02\n";
  EXPECT_FALSE(parses(circle + "      normal: [0.0, 0.0, .nan]\n")) << "NaN";
  EXPECT_FALSE(parses(circle + "      normal: [0.0, 0.0, 0.0]\n")) << "zero normal";
  EXPECT_FALSE(parses(circle + "      normal: [0.0, 1.0]\n")) << "2-element vector";
  EXPECT_FALSE(parses(
      "exclusion_zones:\n"
      "  lines:\n"
      "    - start: [0.1, 0.0, 0.1]\n"
      "      end: [0.1, 0.0, 0.1]\n"
      "      exclusion_radius: 0.005\n")) << "zero-length line";
  EXPECT_FALSE(parses(
      "exclusion_zones:\n"
      "  polygons:\n"
      "    - corners: [[0.0, 0.0, 0.1], [0.1, 0.0, 0.1], [0.1, 0.1, 0.1], [0.0, 0.1, 0.3]]\n"
      "      projection_depth: 0.02\n")) << "non-planar polygon";
  EXPECT_FALSE(parses("sampling:\n  min_angle_deg: 170.0\n  max_angle_deg: 190.0\n"))
    << "angle above 180";
  EXPECT_FALSE(parses("kissing:\n  contact_threshold: 1.5\n")) << "ratio above 1";
  EXPECT_FALSE(parses("fcl:\n  enabled: false\n")) << "FCL is required";
  EXPECT_FALSE(parses(
      "secondaries:\n"
      "  - type: box\n"
      "    dimensions: [0.1, 0.0, 0.1]\n")) << "zero box dimension";
  EXPECT_FALSE(parses(
      "secondaries:\n"
      "  - type: box\n"
      "    dimensions: [0.1, 0.1, 0.1]\n"
      "    transform:\n"
      "      rotation: [0.0, 0.0, 0.0, 0.0]\n")) << "zero quaternion";
  ConfigParser parser;
  EXPECT_FALSE(parser.parse_string(
      "primary:\n"
      "  urdf_path: /tmp/part.urdf\n"
      "  step_path: /tmp/part.step\n"
      "gripper:\n"
      "  urdf_path: /tmp/gripper.urdf\n").has_value()) << "both primary paths";
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
