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

#include <algorithm>
#include <cmath>
#include <memory>
#include <vector>

#include <BRepBuilderAPI_Transform.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <TopoDS_Wire.hxx>
#include <gp_Ax1.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>
#include <gp_Trsf.hxx>
#include <gp_Vec.hxx>

#include "hold_and_weld_gripper_sampler/constraints/ground_constraint.hpp"
#include "hold_and_weld_gripper_sampler/geometry/geometry_mapper.hpp"
#include "test_helpers.hpp"

using hold_and_weld_gripper_sampler::constraints::GroundConfig;
using hold_and_weld_gripper_sampler::constraints::GroundConstraint;
using hold_and_weld_gripper_sampler::geometry::GeometryMapper;
using hold_and_weld_gripper_sampler::geometry::Topology;

namespace
{

TopoDS_Shape translated(const TopoDS_Shape & shape, double x, double y, double z)
{
  gp_Trsf move;
  move.SetTranslation(gp_Vec(x, y, z));
  return BRepBuilderAPI_Transform(shape, move, Standard_True).Shape();
}

}  // namespace

class GroundConstraintTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    mapper_ = std::make_shared<GeometryMapper>();
  }

  Topology load(const TopoDS_Shape & shape) {return mapper_->load_from_shape(shape);}

  std::shared_ptr<GeometryMapper> mapper_;
};

// A box sitting flat on the ground: the bottom face is fully supported and is
// banned outright. The four side faces are not supported, but their lowest strip
// is inside the contact band — a contact point a couple of millimetres off the
// floor would put a finger through it — so each gets an exclusion wire. That
// near-ground strip was invisible to the old triangle-centroid measurement.
TEST_F(GroundConstraintTest, FlatRestingBoxBansBottomAndExcludesSideStrips)
{
  const TopoDS_Shape box = BRepPrimAPI_MakeBox(gp_Pnt(-0.05, -0.05, 0.0), 0.1, 0.1, 0.1).Shape();

  GroundConstraint constraint{GroundConfig{}};
  constraint.analyze_constraints(load(box));

  const auto banned = constraint.get_banned_surface_ids();
  const auto areas = constraint.get_sample_areas();

  ASSERT_EQ(banned.size(), 1u) << "exactly the bottom face rests on the ground";
  EXPECT_EQ(areas.size(), 4u) << "the four side faces graze the contact band; the top does not";

  for (const auto & area : areas) {
    EXPECT_EQ(std::count(banned.begin(), banned.end(), area.surface_id), 0)
      << "a banned face should not also carry an exclusion wire";
    ASSERT_FALSE(area.wire.IsNull());
    EXPECT_TRUE(is_wire_closed(area.wire));
  }
}

// A box lifted clear of the ground touches nothing.
TEST_F(GroundConstraintTest, FloatingBoxIsUntouched)
{
  const TopoDS_Shape box =
    translated(BRepPrimAPI_MakeBox(gp_Pnt(-0.05, -0.05, 0.0), 0.1, 0.1, 0.1).Shape(), 0, 0, 0.5);

  GroundConstraint constraint{GroundConfig{}};
  constraint.analyze_constraints(load(box));

  EXPECT_TRUE(constraint.get_banned_surface_ids().empty());
  EXPECT_TRUE(constraint.get_sample_areas().empty());
}

// The case the triangle-centroid sampler could not see at all: a box rolled 45
// degrees balances on one edge, and the two faces meeting at that edge graze the
// ground over a 7.07 mm strip. None is supported enough to ban, but contact
// points must not be placed down in that strip.
TEST_F(GroundConstraintTest, EdgeRestingBoxProducesClosedExclusionWires)
{
  gp_Trsf roll;
  roll.SetRotation(gp_Ax1(gp_Pnt(0, 0, 0), gp_Dir(1, 0, 0)), M_PI / 4.0);
  gp_Trsf lift;
  lift.SetTranslation(gp_Vec(0, 0, 0.05 * M_SQRT1_2));

  const TopoDS_Shape box = BRepPrimAPI_MakeBox(gp_Pnt(-0.05, -0.05, 0.0), 0.1, 0.1, 0.1).Shape();
  const TopoDS_Shape rolled = BRepBuilderAPI_Transform(
    BRepBuilderAPI_Transform(box, roll, Standard_True).Shape(), lift, Standard_True).Shape();

  GroundConstraint constraint{GroundConfig{}};
  constraint.analyze_constraints(load(rolled));

  EXPECT_TRUE(constraint.get_banned_surface_ids().empty())
    << "no face of an edge-resting box is supported over most of its area";

  const auto areas = constraint.get_sample_areas();
  ASSERT_FALSE(areas.empty())
    << "the grazing faces must be excluded near the resting edge";

  for (const auto & area : areas) {
    ASSERT_FALSE(area.wire.IsNull());
    EXPECT_TRUE(area.is_exclusion);
    EXPECT_TRUE(is_wire_closed(area.wire))
      << "surface " << area.surface_id << ": open wires break point-in-wire ray casting";
  }
}

// The footprint is finite. The same box, moved beyond the edge of the ground,
// is over open floor and rests on nothing.
TEST_F(GroundConstraintTest, PartOutsideFootprintIsNotSupported)
{
  const TopoDS_Shape box = BRepPrimAPI_MakeBox(gp_Pnt(-0.05, -0.05, 0.0), 0.1, 0.1, 0.1).Shape();

  GroundConfig config;
  config.size_x = 1.0;
  config.size_y = 1.0;

  GroundConstraint inside{config};
  inside.analyze_constraints(load(box));
  ASSERT_EQ(inside.get_banned_surface_ids().size(), 1u) << "sanity: supported when over the ground";

  GroundConstraint outside{config};
  outside.analyze_constraints(load(translated(box, 5.0, 0.0, 0.0)));
  EXPECT_TRUE(outside.get_banned_surface_ids().empty())
    << "a part 5 m out is past the 1 m footprint and rests on nothing";
  EXPECT_TRUE(outside.get_sample_areas().empty());
}

// Moving the footprint centre moves where the ground is.
TEST_F(GroundConstraintTest, FootprintCentreIsHonoured)
{
  const TopoDS_Shape box = translated(
    BRepPrimAPI_MakeBox(gp_Pnt(-0.05, -0.05, 0.0), 0.1, 0.1, 0.1).Shape(), 5.0, 0.0, 0.0);

  GroundConfig config;
  config.size_x = 1.0;
  config.size_y = 1.0;
  config.center_x = 5.0;

  GroundConstraint constraint{config};
  constraint.analyze_constraints(load(box));

  EXPECT_EQ(constraint.get_banned_surface_ids().size(), 1u)
    << "the part is over the footprint once the centre is moved to meet it";
}

TEST_F(GroundConstraintTest, FootprintBoundaryIsInclusive)
{
  GroundConfig config;
  config.size_x = 10.0;
  config.size_y = 10.0;

  GroundConstraint constraint{config};
  EXPECT_TRUE(constraint.is_within_footprint(0.0, 0.0));
  EXPECT_TRUE(constraint.is_within_footprint(5.0, 5.0));
  EXPECT_TRUE(constraint.is_within_footprint(-5.0, -5.0));
  EXPECT_FALSE(constraint.is_within_footprint(5.001, 0.0));
  EXPECT_FALSE(constraint.is_within_footprint(0.0, -5.001));
}

// A non-planar face resting on the ground: the cylinder's lateral surface
// touches along a line, which is a vanishing fraction of its area, so it must be
// excluded rather than banned. This is the case face normals cannot classify.
TEST_F(GroundConstraintTest, CylinderOnItsSideIsExcludedNotBanned)
{
  gp_Trsf lay_down;
  lay_down.SetRotation(gp_Ax1(gp_Pnt(0, 0, 0), gp_Dir(1, 0, 0)), M_PI / 2.0);
  gp_Trsf lift;
  lift.SetTranslation(gp_Vec(0, 0, 0.05));

  const TopoDS_Shape cyl = BRepPrimAPI_MakeCylinder(0.05, 0.3).Shape();
  const TopoDS_Shape lying = BRepBuilderAPI_Transform(
    BRepBuilderAPI_Transform(cyl, lay_down, Standard_True).Shape(), lift, Standard_True).Shape();

  GroundConstraint constraint{GroundConfig{}};
  constraint.analyze_constraints(load(lying));

  EXPECT_TRUE(constraint.get_banned_surface_ids().empty())
    << "line contact is a negligible area fraction and must not ban the face";
  EXPECT_FALSE(constraint.get_sample_areas().empty())
    << "the strip along the contact line must still be excluded";
}

// Re-running replaces the previous verdict instead of accumulating it.
TEST_F(GroundConstraintTest, ReanalysisReplacesNotAccumulates)
{
  const TopoDS_Shape box = BRepPrimAPI_MakeBox(gp_Pnt(-0.05, -0.05, 0.0), 0.1, 0.1, 0.1).Shape();

  GroundConstraint constraint{GroundConfig{}};
  constraint.analyze_constraints(load(box));
  const size_t first = constraint.get_banned_surface_ids().size();

  constraint.analyze_constraints(load(box));
  EXPECT_EQ(constraint.get_banned_surface_ids().size(), first);
}

class GroundPoseTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    gripper_ = create_mock_gripper();
    primary_ = create_box_at_helper(0.1, 0.1, 0.1, 0.0, 0.0, 1.0);
  }

  static gp_Trsf at_height(double z)
  {
    gp_Trsf pose;
    pose.SetTranslation(gp_Vec(0.0, 0.0, z));
    return pose;
  }

  ParsedGripper gripper_;
  TopoDS_Shape primary_;
};

TEST_F(GroundPoseTest, GripperThroughTheGroundIsRejected)
{
  auto fcl = make_fcl_checker(gripper_, primary_);
  fcl->add_ground_plane(Eigen::Vector3d(0.0, 0.0, 1.0), 0.0);

  GroundConstraint constraint{GroundConfig{}};
  constraint.set_fcl_checker(fcl);

  EXPECT_TRUE(constraint.intersects_ground(at_height(-0.2), 0.03));
  EXPECT_FALSE(constraint.intersects_ground(at_height(0.5), 0.03));
}

// Ground is only added to FCL when use_fcl_for_ground_plane is set; without it
// there is nothing to hit and the pose check must pass.
TEST_F(GroundPoseTest, CheckerWithoutGroundRejectsNothing)
{
  auto fcl = make_fcl_checker(gripper_, primary_);

  GroundConstraint constraint{GroundConfig{}};
  constraint.set_fcl_checker(fcl);

  EXPECT_FALSE(constraint.intersects_ground(at_height(-0.2), 0.03));
}

// Same policy as the other constraints: no checker means the pose cannot be
// cleared, so it is rejected rather than silently accepted.
TEST_F(GroundPoseTest, MissingCheckerRejectsConservatively)
{
  GroundConstraint constraint{GroundConfig{}};
  EXPECT_TRUE(constraint.intersects_ground(at_height(0.5), 0.03));
}

// The tolerance comes from GroundConfig: a gripper hovering 5 mm above the
// floor is a hit at 10 mm tolerance and clear at 1 mm.
TEST_F(GroundPoseTest, ToleranceComesFromConfig)
{
  ParsedGripper flat = gripper_;
  flat.base = BRepPrimAPI_MakeBox(gp_Pnt(-0.03, -0.03, 0.0), 0.06, 0.06, 0.01).Shape();
  flat.finger_1 = flat.base;
  flat.finger_2 = flat.base;
  flat.tcp_offset = Eigen::Vector3d::Zero();

  auto fcl = make_fcl_checker(flat, primary_);
  fcl->add_ground_plane(Eigen::Vector3d(0.0, 0.0, 1.0), 0.0);

  GroundConfig loose;
  loose.collision_tolerance = 0.010;
  GroundConstraint loose_constraint{loose};
  loose_constraint.set_fcl_checker(fcl);
  EXPECT_TRUE(loose_constraint.intersects_ground(at_height(0.005), 0.03));

  GroundConfig tight;
  tight.collision_tolerance = 0.001;
  GroundConstraint tight_constraint{tight};
  tight_constraint.set_fcl_checker(fcl);
  EXPECT_FALSE(tight_constraint.intersects_ground(at_height(0.005), 0.03));
}

TEST(GroundConstraintNameTest, ReturnsGroundConstraint)
{
  EXPECT_EQ(GroundConstraint{GroundConfig{}}.get_name(), "GroundConstraint");
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
