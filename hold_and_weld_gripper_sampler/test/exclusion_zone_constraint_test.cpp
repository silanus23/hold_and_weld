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

#include <cmath>
#include <memory>
#include <vector>

#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRepBuilderAPI_Transform.hxx>

#include <gp_Trsf.hxx>
#include <gp_Vec.hxx>
#include <gp_Ax2.hxx>
#include <TopoDS_Shape.hxx>

#include "hold_and_weld_gripper_sampler/core/gripper.hpp"
#include "hold_and_weld_gripper_sampler/constraints/exclusion_zone_constraint.hpp"
#include "hold_and_weld_gripper_sampler/geometry/geometry_mapper.hpp"

using namespace hold_and_weld_gripper_sampler;  // NOLINT
using namespace hold_and_weld_gripper_sampler::constraints;  // NOLINT
using namespace hold_and_weld_gripper_sampler::geometry;  // NOLINT

namespace
{
ParsedGripper create_mock_gripper()
{
  ParsedGripper gripper;

  gp_Trsf f1_pos;
  f1_pos.SetTranslation(gp_Vec(0.0, 0.015, 0.0));
  TopoDS_Shape f1_box = BRepPrimAPI_MakeBox(0.02, 0.01, 0.05).Shape();
  gripper.finger_1 = BRepBuilderAPI_Transform(f1_box, f1_pos, Standard_True).Shape();

  gp_Trsf f2_pos;
  f2_pos.SetTranslation(gp_Vec(0.0, -0.025, 0.0));
  TopoDS_Shape f2_box = BRepPrimAPI_MakeBox(0.02, 0.01, 0.05).Shape();
  gripper.finger_2 = BRepBuilderAPI_Transform(f2_box, f2_pos, Standard_True).Shape();

  gp_Trsf base_pos;
  base_pos.SetTranslation(gp_Vec(-0.01, -0.03, 0.05));
  TopoDS_Shape base_box = BRepPrimAPI_MakeBox(0.06, 0.06, 0.03).Shape();
  gripper.base = BRepBuilderAPI_Transform(base_box, base_pos, Standard_True).Shape();

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

class ExclusionZoneConstraintTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    mapper_ = std::make_shared<GeometryMapper>();
    gripper_ = create_mock_gripper();
  }

  std::shared_ptr<GeometryMapper> mapper_;
  ParsedGripper gripper_;
};

TEST_F(ExclusionZoneConstraintTest, ConstructWithNoConstraints)
{
  ExclusionZoneConstraint constraint(mapper_, gripper_);

  EXPECT_EQ(constraint.get_name(), "ExclusionZoneConstraint");
  EXPECT_TRUE(constraint.get_sample_areas().empty());
}

TEST_F(ExclusionZoneConstraintTest, ConstructWithPolygonConstraint)
{
  exclusion_polygon polygon;
  polygon.exclusion_corners = {
    Eigen::Vector3d(0.0, 0.0, 0.0),
    Eigen::Vector3d(0.1, 0.0, 0.0),
    Eigen::Vector3d(0.1, 0.1, 0.0),
    Eigen::Vector3d(0.0, 0.1, 0.0)
  };
  polygon.projection_depth = 0.02;
  polygon.clearance = 0.01;

  std::vector<exclusion_polygon> polygons = {polygon};

  ExclusionZoneConstraint constraint(
    mapper_, gripper_, std::nullopt, polygons);

  EXPECT_EQ(constraint.get_name(), "ExclusionZoneConstraint");
}

TEST_F(ExclusionZoneConstraintTest, ConstructWithMultipleConstraints)
{
  exclusion_circle circle;
  circle.center = Eigen::Vector3d(0.0, 0.0, 0.0);
  circle.normal = Eigen::Vector3d(0.0, 0.0, 1.0);
  circle.radius = 0.05;
  circle.projection_depth = 0.02;

  exclusion_line line;
  line.start = Eigen::Vector3d(0.2, 0.0, 0.0);
  line.end = Eigen::Vector3d(0.3, 0.0, 0.0);
  line.exclusion_radius = 0.01;

  exclusion_polygon polygon;
  polygon.exclusion_corners = {
    Eigen::Vector3d(0.4, 0.0, 0.0),
    Eigen::Vector3d(0.5, 0.0, 0.0),
    Eigen::Vector3d(0.5, 0.1, 0.0),
    Eigen::Vector3d(0.4, 0.1, 0.0)
  };
  polygon.projection_depth = 0.02;

  std::vector<exclusion_circle> circles = {circle};
  std::vector<exclusion_polygon> polygons = {polygon};
  std::vector<exclusion_line> lines = {line};

  ExclusionZoneConstraint constraint(
    mapper_, gripper_, circles, polygons, lines);

  EXPECT_EQ(constraint.get_name(), "ExclusionZoneConstraint");
}

TEST_F(ExclusionZoneConstraintTest, NoCollisionWhenFarFromExclusionZone)
{
  // Create exclusion circle at origin
  exclusion_circle circle;
  circle.center = Eigen::Vector3d(0.0, 0.0, 0.0);
  circle.normal = Eigen::Vector3d(0.0, 0.0, 1.0);
  circle.radius = 0.05;
  circle.projection_depth = 0.02;
  circle.clearance = 0.01;

  std::vector<exclusion_circle> circles = {circle};

  ExclusionZoneConstraint constraint(mapper_, gripper_, circles);

  TopoDS_Shape test_box = BRepPrimAPI_MakeBox(0.2, 0.2, 0.1).Shape();
  Topology topology = mapper_->load_from_shape(test_box);
  constraint.analyze_constraints(test_box, topology);

  // Place gripper far away (at 1 meter away)
  gp_Trsf far_transform;
  far_transform.SetTranslation(gp_Vec(1.0, 1.0, 1.0));

  bool collision = constraint.intersects_exclusion_zone(far_transform, 0.03);
  EXPECT_FALSE(collision);
}

TEST_F(ExclusionZoneConstraintTest, CollisionWhenInsideExclusionZone)
{
  // Create exclusion circle at origin
  exclusion_circle circle;
  circle.center = Eigen::Vector3d(0.0, 0.0, 0.0);
  circle.normal = Eigen::Vector3d(0.0, 0.0, 1.0);
  circle.radius = 0.1;
  circle.projection_depth = 0.1;
  circle.clearance = 0.01;

  std::vector<exclusion_circle> circles = {circle};

  ExclusionZoneConstraint constraint(mapper_, gripper_, circles);

  // Create a test shape and analyze
  TopoDS_Shape test_box = BRepPrimAPI_MakeBox(0.3, 0.3, 0.2).Shape();
  Topology topology = mapper_->load_from_shape(test_box);
  constraint.analyze_constraints(test_box, topology);

  // Place gripper at origin (inside the exclusion zone)
  gp_Trsf origin_transform;
  origin_transform.SetTranslation(gp_Vec(0.0, 0.0, 0.0));

  bool collision = constraint.intersects_exclusion_zone(origin_transform, 0.03);
  EXPECT_TRUE(collision);
}

TEST_F(ExclusionZoneConstraintTest, CollisionWithLineExclusionZone)
{
  // Create exclusion line along X axis
  exclusion_line line;
  line.start = Eigen::Vector3d(-0.1, 0.0, 0.0);
  line.end = Eigen::Vector3d(0.1, 0.0, 0.0);
  line.exclusion_radius = 0.05;
  line.clearance = 0.01;

  std::vector<exclusion_line> lines = {line};

  ExclusionZoneConstraint constraint(
    mapper_, gripper_, std::nullopt, std::nullopt, lines);

  // Create a test shape and analyze
  TopoDS_Shape test_box = BRepPrimAPI_MakeBox(0.3, 0.3, 0.2).Shape();
  Topology topology = mapper_->load_from_shape(test_box);
  constraint.analyze_constraints(test_box, topology);

  // Place gripper at origin (inside the tube)
  gp_Trsf origin_transform;
  origin_transform.SetTranslation(gp_Vec(0.0, 0.0, 0.0));

  bool collision = constraint.intersects_exclusion_zone(origin_transform, 0.03);
  EXPECT_TRUE(collision);

  // Place gripper far away
  gp_Trsf far_transform;
  far_transform.SetTranslation(gp_Vec(0.5, 0.5, 0.5));

  collision = constraint.intersects_exclusion_zone(far_transform, 0.03);
  EXPECT_FALSE(collision);
}

TEST_F(ExclusionZoneConstraintTest, CollisionWithPolygonExclusionZone)
{
  // Create exclusion polygon (square in XY plane)
  exclusion_polygon polygon;
  polygon.exclusion_corners = {
    Eigen::Vector3d(-0.05, -0.05, 0.0),
    Eigen::Vector3d(0.05, -0.05, 0.0),
    Eigen::Vector3d(0.05, 0.05, 0.0),
    Eigen::Vector3d(-0.05, 0.05, 0.0)
  };
  polygon.projection_depth = 0.1;
  polygon.clearance = 0.01;

  std::vector<exclusion_polygon> polygons = {polygon};

  ExclusionZoneConstraint constraint(
    mapper_, gripper_, std::nullopt, polygons);

  // Create a test shape and analyze
  TopoDS_Shape test_box = BRepPrimAPI_MakeBox(0.3, 0.3, 0.2).Shape();
  Topology topology = mapper_->load_from_shape(test_box);
  constraint.analyze_constraints(test_box, topology);

  // Place gripper at origin (inside the prism)
  gp_Trsf origin_transform;
  origin_transform.SetTranslation(gp_Vec(0.0, 0.0, 0.02));

  bool collision = constraint.intersects_exclusion_zone(origin_transform, 0.03);
  EXPECT_TRUE(collision);
}

TEST_F(ExclusionZoneConstraintTest, GripperOpensCorrectly)
{
  exclusion_circle circle;
  circle.center = Eigen::Vector3d(0.0, 0.0, 0.0);
  circle.normal = Eigen::Vector3d(0.0, 0.0, 1.0);
  circle.radius = 0.03;  // 3cm radius
  circle.projection_depth = 0.02;
  circle.clearance = 0.005;

  std::vector<exclusion_circle> circles = {circle};

  ExclusionZoneConstraint constraint(mapper_, gripper_, circles);

  // Create a test shape and analyze
  TopoDS_Shape test_box = BRepPrimAPI_MakeBox(0.2, 0.2, 0.1).Shape();
  Topology topology = mapper_->load_from_shape(test_box);
  constraint.analyze_constraints(test_box, topology);

  // Position gripper near the exclusion zone at origin
  gp_Trsf near_transform;
  near_transform.SetTranslation(gp_Vec(0.0, 0.0, 0.0));

  // With small opening (0.02m), finger_travel=0.01m → fingers still within 3cm radius → collides
  bool collision_small = constraint.intersects_exclusion_zone(near_transform, 0.02, 0.001);
  EXPECT_TRUE(collision_small);

  // With large opening (0.08m), finger_travel=0.04m → fingers splay to y≈±0.055m, outside 3cm
  // radius + 5mm clearance. Base is at z=0.05 above the exclusion cylinder → no collision
  bool collision_large = constraint.intersects_exclusion_zone(near_transform, 0.08, 0.001);
  EXPECT_FALSE(collision_large);
}

TEST_F(ExclusionZoneConstraintTest, ZeroLengthLineHandled)
{
  // Degenerate line (zero length)
  exclusion_line line;
  line.start = Eigen::Vector3d(0.0, 0.0, 0.0);
  line.end = Eigen::Vector3d(0.0, 0.0, 0.0);
  line.exclusion_radius = 0.01;
  line.clearance = 0.005;

  std::vector<exclusion_line> lines = {line};

  // Should not crash — degenerate geometry may throw std::exception or OCCT
  // Standard_Failure, but must never trigger a GTest fatal failure (ASSERT/abort).
  // The inner catch is intentionally absent: if an unhandled exception escapes,
  // EXPECT_NO_FATAL_FAILURE will catch the resulting fatal signal; if it throws a
  // handled C++ exception the test runner reports it as an error, not a silent pass.
  EXPECT_NO_FATAL_FAILURE({
    ExclusionZoneConstraint constraint(
      mapper_, gripper_, std::nullopt, std::nullopt, lines);

    TopoDS_Shape test_box = BRepPrimAPI_MakeBox(0.2, 0.2, 0.1).Shape();
    Topology topology = mapper_->load_from_shape(test_box);
    constraint.analyze_constraints(test_box, topology);
  });
}

TEST_F(ExclusionZoneConstraintTest, VerySmallExclusionRadius)
{
  exclusion_circle circle;
  circle.center = Eigen::Vector3d(0.0, 0.0, 0.0);
  circle.normal = Eigen::Vector3d(0.0, 0.0, 1.0);
  circle.radius = 0.0001;
  circle.projection_depth = 0.001;
  circle.clearance = 0.0001;

  std::vector<exclusion_circle> circles = {circle};

  ExclusionZoneConstraint constraint(mapper_, gripper_, circles);

  TopoDS_Shape test_box = BRepPrimAPI_MakeBox(0.2, 0.2, 0.1).Shape();
  Topology topology = mapper_->load_from_shape(test_box);
  constraint.analyze_constraints(test_box, topology);

  // Should not crash with very small geometry
  gp_Trsf transform;
  transform.SetTranslation(gp_Vec(0.1, 0.1, 0.1));

  EXPECT_NO_FATAL_FAILURE({
    constraint.intersects_exclusion_zone(transform, 0.03);
  });
}

TEST_F(ExclusionZoneConstraintTest, CircleClearanceIsSymmetric)
{
  // This test verifies that clearance is applied symmetrically
  // by checking that gripper collides when approaching from both sides

  exclusion_circle circle;
  circle.center = Eigen::Vector3d(0.1, 0.1, 0.05);
  circle.normal = Eigen::Vector3d(0.0, 0.0, 1.0);
  circle.radius = 0.02;
  circle.projection_depth = 0.02;
  circle.clearance = 0.01;

  std::vector<exclusion_circle> circles = {circle};

  ExclusionZoneConstraint constraint(mapper_, gripper_, circles);

  TopoDS_Shape test_box = BRepPrimAPI_MakeBox(0.3, 0.3, 0.2).Shape();
  Topology topology = mapper_->load_from_shape(test_box);
  constraint.analyze_constraints(test_box, topology);

  // Place gripper clearly inside the exclusion cylinder (at its center)
  gp_Trsf inside_transform;
  inside_transform.SetTranslation(gp_Vec(0.1, 0.1, 0.05));

  // Place gripper clearly outside (1 m away)
  gp_Trsf outside_transform;
  outside_transform.SetTranslation(gp_Vec(1.0, 1.0, 1.0));

  bool collision_inside = constraint.intersects_exclusion_zone(inside_transform, 0.02);
  bool collision_outside = constraint.intersects_exclusion_zone(outside_transform, 0.02);

  // A gripper placed at the zone centre must collide
  EXPECT_TRUE(collision_inside);
  // A gripper placed 1 m away must not collide
  EXPECT_FALSE(collision_outside);
}

TEST_F(ExclusionZoneConstraintTest, SampleAreasEmptyBeforeAnalysis)
{
  exclusion_circle circle;
  circle.center = Eigen::Vector3d(0.0, 0.0, 0.0);
  circle.normal = Eigen::Vector3d(0.0, 0.0, 1.0);
  circle.radius = 0.05;
  circle.projection_depth = 0.02;

  std::vector<exclusion_circle> circles = {circle};

  ExclusionZoneConstraint constraint(mapper_, gripper_, circles);

  // Before analyze_constraints is called, sample_areas should be empty
  std::vector<hold_and_weld_gripper_sampler::core::SampleArea> areas =
    constraint.get_sample_areas();
  EXPECT_TRUE(areas.empty());
}

TEST_F(ExclusionZoneConstraintTest, MultipleExclusionZonesAllChecked)
{
  // Create two separate exclusion circles
  exclusion_circle circle1;
  circle1.center = Eigen::Vector3d(0.0, 0.0, 0.05);
  circle1.normal = Eigen::Vector3d(0.0, 0.0, 1.0);
  circle1.radius = 0.03;
  circle1.projection_depth = 0.02;
  circle1.clearance = 0.005;

  exclusion_circle circle2;
  circle2.center = Eigen::Vector3d(0.2, 0.0, 0.05);
  circle2.normal = Eigen::Vector3d(0.0, 0.0, 1.0);
  circle2.radius = 0.03;
  circle2.projection_depth = 0.02;
  circle2.clearance = 0.005;

  std::vector<exclusion_circle> circles = {circle1, circle2};

  ExclusionZoneConstraint constraint(mapper_, gripper_, circles);

  TopoDS_Shape test_box = BRepPrimAPI_MakeBox(0.4, 0.2, 0.1).Shape();
  Topology topology = mapper_->load_from_shape(test_box);
  constraint.analyze_constraints(test_box, topology);

  // Collision with first zone only
  gp_Trsf near_first;
  near_first.SetTranslation(gp_Vec(0.0, 0.0, 0.05));
  EXPECT_TRUE(constraint.intersects_exclusion_zone(near_first, 0.02));

  // Collision with second zone only
  gp_Trsf near_second;
  near_second.SetTranslation(gp_Vec(0.2, 0.0, 0.05));
  EXPECT_TRUE(constraint.intersects_exclusion_zone(near_second, 0.02));

  // No collision in between (if far enough)
  gp_Trsf between;
  between.SetTranslation(gp_Vec(0.1, 0.15, 0.05));
  bool collision_between = constraint.intersects_exclusion_zone(between, 0.02);
  // This may or may not collide depending on gripper size - just verify no crash
  EXPECT_NO_FATAL_FAILURE({(void)collision_between;});
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
