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
#include "hold_and_weld_gripper_sampler/geometry/geometry_mapper.hpp"

using namespace hold_and_weld_gripper_sampler::geometry;  // NOLINT

namespace test_constants
{
// Expected topology counts for primitives
constexpr int kBoxCorners = 8;
constexpr int kBoxEdges = 12;
constexpr int kBoxSurfaces = 6;
constexpr int kCylinderSurfaces = 3;  // 1 curved + 2 caps

// Unit vector magnitude
constexpr double kUnitMagnitude = 1.0;
constexpr double kTolerance = 1e-6;
}  // namespace test_constants

class TopologyExtractionTest : public ::testing::Test
{
protected:
  GeometryMapper mapper_;
};

TEST_F(TopologyExtractionTest, LoadFromUrdfString_WithBoxGeometry_ExtractsCorrectTopology)
{
  std::string urdf =
    R"(
    <robot name="test_robot">
      <link name="test_box">
        <collision>
          <geometry>
            <box size="0.1 0.1 0.1"/>
          </geometry>
          <origin xyz="0 0 0" rpy="0 0 0"/>
        </collision>
      </link>
    </robot>
    )";
         // NOLINT

  Topology topology = mapper_.load_from_urdf_string(urdf);

  // Box should have: 8 corners, 12 edges, 6 surfaces
  EXPECT_EQ(topology.num_corners(), test_constants::kBoxCorners);
  EXPECT_EQ(topology.num_edges(), test_constants::kBoxEdges);
  EXPECT_EQ(topology.num_surfaces(), test_constants::kBoxSurfaces);

  // Check that all surfaces have valid normals
  EXPECT_EQ(topology.num_surfaces(), test_constants::kBoxSurfaces);

  const auto & all_surfaces = topology.get_all_surfaces();
  for (size_t i = 0; i < all_surfaces.size(); i++) {
    const auto & surface = all_surfaces[i];

    // Normal should be unit vector
    double magnitude = surface.normal.Magnitude();
    EXPECT_NEAR(magnitude, test_constants::kUnitMagnitude, test_constants::kTolerance);

    // Surface should be graspable by default
    EXPECT_TRUE(surface.is_graspable);
  }
}

TEST_F(TopologyExtractionTest, LoadFromUrdfString_WithCylinderGeometry_ExtractsThreeSurfaces)
{
  std::string urdf =
    R"(
    <robot name="test_robot">
      <link name="test_cylinder">
        <collision>
          <geometry>
            <cylinder radius="0.05" length="0.2"/>
          </geometry>
          <origin xyz="0 0 0" rpy="0 0 0"/>
        </collision>
      </link>
    </robot>
    )";
         // NOLINT

  Topology topology = mapper_.load_from_urdf_string(urdf);

  // Cylinder should have: 3 surfaces (1 curved + 2 circular caps)
  EXPECT_EQ(topology.num_surfaces(), test_constants::kCylinderSurfaces);

  // All surfaces should have valid unit normals
  const auto & all_surfaces = topology.get_all_surfaces();
  for (size_t i = 0; i < all_surfaces.size(); i++) {
    const auto & surface = all_surfaces[i];
    double magnitude = surface.normal.Magnitude();
    EXPECT_NEAR(magnitude, test_constants::kUnitMagnitude, test_constants::kTolerance);
  }
}

TEST_F(TopologyExtractionTest, LoadFromUrdfString_WithMultipleBoxLinks_CombinesTotalSurfaces)
{
  std::string urdf =
    R"(
    <robot name="test_robot">
      <link name="box1">
        <collision>
          <geometry>
            <box size="0.1 0.1 0.1"/>
          </geometry>
        </collision>
      </link>
      <link name="box2">
        <collision>
          <geometry>
            <box size="0.05 0.05 0.05"/>
          </geometry>
        </collision>
      </link>
    </robot>
    )";
         // NOLINT

  Topology topology = mapper_.load_from_urdf_string(urdf);

  // Two boxes: 2 * 6 = 12 surfaces total
  EXPECT_EQ(topology.num_surfaces(), 2 * test_constants::kBoxSurfaces);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
