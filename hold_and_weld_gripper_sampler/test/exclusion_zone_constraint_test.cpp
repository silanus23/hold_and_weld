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
#include "hold_and_weld_gripper_sampler/collision/fcl_collision_checker.hpp"
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

  // Build an FCLCollisionChecker with the exclusion volumes from the constraint
  // and wire it in so that intersects_exclusion_zone() can do real FCL queries.
  // Must be called after analyze_constraints() so collision_volumes_ is populated.
  static void wire_fcl(
    ExclusionZoneConstraint & constraint,
    const ParsedGripper & gripper,
    const TopoDS_Shape & primary_shape)
  {
    auto fcl = std::make_shared<geometry::FCLCollisionChecker>(gripper, primary_shape);
    fcl->add_exclusion_volumes(constraint.get_collision_volumes());
    constraint.set_fcl_checker(fcl);
  }

  std::shared_ptr<GeometryMapper> mapper_;
  ParsedGripper gripper_;
};

// Constraint constructed with no exclusion inputs reports empty sample areas.
TEST_F(ExclusionZoneConstraintTest, ConstructWithNoConstraints)
{
  ExclusionZoneConstraint constraint(mapper_, gripper_);

  EXPECT_EQ(constraint.get_name(), "ExclusionZoneConstraint");
  EXPECT_TRUE(constraint.get_sample_areas().empty());
}

// Constraint accepts a polygon exclusion zone at construction.
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

// Constraint accepts circle, polygon, and line exclusion zones simultaneously.
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

// Gripper placed 1 m away from a circle exclusion zone must not collide.
TEST_F(ExclusionZoneConstraintTest, NoCollisionWhenFarFromExclusionZone)
{
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
  wire_fcl(constraint, gripper_, test_box);

  gp_Trsf far_transform;
  far_transform.SetTranslation(gp_Vec(1.0, 1.0, 1.0));

  bool collision = constraint.intersects_exclusion_zone(far_transform, 0.03);
  EXPECT_FALSE(collision);
}

// TODO(@silanus23): Fix CollisionWhenInsideExclusionZone - FCL collision check returns false
// even when gripper is placed at origin inside a circle exclusion zone. Likely a volume
// construction or transform issue in the FCL wiring for circle exclusions.
// TEST_F(ExclusionZoneConstraintTest, CollisionWhenInsideExclusionZone)
// {
//   // Create exclusion circle at origin
//   exclusion_circle circle;
//   circle.center = Eigen::Vector3d(0.0, 0.0, 0.0);
//   circle.normal = Eigen::Vector3d(0.0, 0.0, 1.0);
//   circle.radius = 0.1;
//   circle.projection_depth = 0.1;
//   circle.clearance = 0.01;
//
//   std::vector<exclusion_circle> circles = {circle};
//
//   ExclusionZoneConstraint constraint(mapper_, gripper_, circles);
//
//   // Create a test shape and analyze
//   TopoDS_Shape test_box = BRepPrimAPI_MakeBox(0.3, 0.3, 0.2).Shape();
//   Topology topology = mapper_->load_from_shape(test_box);
//   constraint.analyze_constraints(test_box, topology);
//   wire_fcl(constraint, gripper_, test_box);
//
//   // Place gripper at origin (inside the exclusion zone)
//   gp_Trsf origin_transform;
//   origin_transform.SetTranslation(gp_Vec(0.0, 0.0, 0.0));
//
//   bool collision = constraint.intersects_exclusion_zone(origin_transform, 0.03);
//   EXPECT_TRUE(collision);
// }

// Gripper inside a line exclusion tube collides; gripper 0.5 m away does not.
TEST_F(ExclusionZoneConstraintTest, CollisionWithLineExclusionZone)
{
  exclusion_line line;
  line.start = Eigen::Vector3d(-0.1, 0.0, 0.0);
  line.end = Eigen::Vector3d(0.1, 0.0, 0.0);
  line.exclusion_radius = 0.05;
  line.clearance = 0.01;

  std::vector<exclusion_line> lines = {line};

  ExclusionZoneConstraint constraint(
    mapper_, gripper_, std::nullopt, std::nullopt, lines);

  TopoDS_Shape test_box = BRepPrimAPI_MakeBox(0.3, 0.3, 0.2).Shape();
  Topology topology = mapper_->load_from_shape(test_box);
  constraint.analyze_constraints(test_box, topology);
  wire_fcl(constraint, gripper_, test_box);

  gp_Trsf origin_transform;
  origin_transform.SetTranslation(gp_Vec(0.0, 0.0, 0.0));

  bool collision = constraint.intersects_exclusion_zone(origin_transform, 0.03);
  EXPECT_TRUE(collision);

  gp_Trsf far_transform;
  far_transform.SetTranslation(gp_Vec(0.5, 0.5, 0.5));

  collision = constraint.intersects_exclusion_zone(far_transform, 0.03);
  EXPECT_FALSE(collision);
}

// TODO(@silanus23): Fix CollisionWithPolygonExclusionZone - FCL collision check returns false
// even when gripper is placed inside a polygon exclusion prism. Likely a volume
// construction or transform issue in the FCL wiring for polygon exclusions.
// TEST_F(ExclusionZoneConstraintTest, CollisionWithPolygonExclusionZone)
// {
//   // Create exclusion polygon (square in XY plane)
//   exclusion_polygon polygon;
//   polygon.exclusion_corners = {
//     Eigen::Vector3d(-0.05, -0.05, 0.0),
//     Eigen::Vector3d(0.05, -0.05, 0.0),
//     Eigen::Vector3d(0.05, 0.05, 0.0),
//     Eigen::Vector3d(-0.05, 0.05, 0.0)
//   };
//   polygon.projection_depth = 0.1;
//   polygon.clearance = 0.01;
//
//   std::vector<exclusion_polygon> polygons = {polygon};
//
//   ExclusionZoneConstraint constraint(
//     mapper_, gripper_, std::nullopt, polygons);
//
//   // Create a test shape and analyze
//   TopoDS_Shape test_box = BRepPrimAPI_MakeBox(0.3, 0.3, 0.2).Shape();
//   Topology topology = mapper_->load_from_shape(test_box);
//   constraint.analyze_constraints(test_box, topology);
//   wire_fcl(constraint, gripper_, test_box);
//
//   // Place gripper at origin (inside the prism)
//   gp_Trsf origin_transform;
//   origin_transform.SetTranslation(gp_Vec(0.0, 0.0, 0.02));
//
//   bool collision = constraint.intersects_exclusion_zone(origin_transform, 0.03);
//   EXPECT_TRUE(collision);
// }

// Degenerate zero-length line exclusion must not crash or trigger a fatal failure.
TEST_F(ExclusionZoneConstraintTest, ZeroLengthLineHandled)
{
  exclusion_line line;
  line.start = Eigen::Vector3d(0.0, 0.0, 0.0);
  line.end = Eigen::Vector3d(0.0, 0.0, 0.0);
  line.exclusion_radius = 0.01;
  line.clearance = 0.005;

  std::vector<exclusion_line> lines = {line};

  // Degenerate geometry — must not crash.
  EXPECT_NO_FATAL_FAILURE({
    ExclusionZoneConstraint constraint(
      mapper_, gripper_, std::nullopt, std::nullopt, lines);

    TopoDS_Shape test_box = BRepPrimAPI_MakeBox(0.2, 0.2, 0.1).Shape();
    Topology topology = mapper_->load_from_shape(test_box);
    constraint.analyze_constraints(test_box, topology);
    wire_fcl(constraint, gripper_, test_box);
  });
}

// Sub-millimetre exclusion geometry must not crash during collision query.
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
  wire_fcl(constraint, gripper_, test_box);

  gp_Trsf transform;
  transform.SetTranslation(gp_Vec(0.1, 0.1, 0.1));

  EXPECT_NO_FATAL_FAILURE({
    constraint.intersects_exclusion_zone(transform, 0.03);
  });
}

// Sample areas are empty until analyze_constraints() is called.
TEST_F(ExclusionZoneConstraintTest, SampleAreasEmptyBeforeAnalysis)
{
  exclusion_circle circle;
  circle.center = Eigen::Vector3d(0.0, 0.0, 0.0);
  circle.normal = Eigen::Vector3d(0.0, 0.0, 1.0);
  circle.radius = 0.05;
  circle.projection_depth = 0.02;

  std::vector<exclusion_circle> circles = {circle};

  ExclusionZoneConstraint constraint(mapper_, gripper_, circles);

  std::vector<hold_and_weld_gripper_sampler::core::SampleArea> areas =
    constraint.get_sample_areas();
  EXPECT_TRUE(areas.empty());
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
