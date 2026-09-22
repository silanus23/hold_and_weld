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

#include <algorithm>
#include <cmath>
#include <memory>
#include <vector>

#include <BRepPrimAPI_MakeBox.hxx>

#include <BRepBuilderAPI_Transform.hxx>
#include <gp_Ax1.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>
#include <gp_Trsf.hxx>
#include <gp_Vec.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS_Shape.hxx>
#include <TopoDS_Wire.hxx>

#include "hold_and_weld_gripper_sampler/core/gripper.hpp"
#include "hold_and_weld_gripper_sampler/constraints/kissing_surface_constraint.hpp"
#include "hold_and_weld_gripper_sampler/collision/fcl_collision_checker.hpp"
#include "hold_and_weld_gripper_sampler/geometry/geometry_mapper.hpp"
#include "test_helpers.hpp"


using namespace hold_and_weld_gripper_sampler;  // NOLINT
using namespace hold_and_weld_gripper_sampler::constraints;  // NOLINT
using namespace hold_and_weld_gripper_sampler::geometry;  // NOLINT
using namespace hold_and_weld_gripper_sampler::core;  // NOLINT


class KissingSurfaceConstraintTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    mapper_ = std::make_shared<GeometryMapper>();
    gripper_ = create_mock_gripper();
  }

  // Build and wire an FCLCollisionChecker into a constraint so that
  // intersects_secondary() can be called.
  void wire_fcl(
    KissingSurfaceConstraint & constraint,
    const std::vector<TopoDS_Shape> & secondaries,
    const TopoDS_Shape & primary_for_fcl)
  {
    auto fcl = make_fcl_checker(gripper_, primary_for_fcl);
    fcl->add_secondary_shapes(secondaries);
    constraint.set_fcl_checker(fcl);
  }

  std::shared_ptr<GeometryMapper> mapper_;
  ParsedGripper gripper_;
};

// Secondary 1 m away from primary must not ban any surfaces.
TEST_F(KissingSurfaceConstraintTest, NoContactWithDistantSecondary)
{
  gp_Trsf primary_pos;
  primary_pos.SetTranslation(gp_Vec(0.0, 0.0, 0.5));
  TopoDS_Shape primary = BRepBuilderAPI_Transform(
    BRepPrimAPI_MakeBox(0.1, 0.1, 0.1).Shape(),
    primary_pos, Standard_True).Shape();

  gp_Trsf far_pos;
  far_pos.SetTranslation(gp_Vec(1.0, 0.0, 0.5));
  TopoDS_Shape far_secondary = BRepBuilderAPI_Transform(
    BRepPrimAPI_MakeBox(0.1, 0.1, 0.1).Shape(),
    far_pos, Standard_True).Shape();

  std::vector<TopoDS_Shape> secondaries = {far_secondary};

  KissingSurfaceConstraint constraint(
    mapper_, gripper_, secondaries);

  Topology topology = mapper_->load_from_shape(primary);
  constraint.analyze_constraints(topology);

  EXPECT_TRUE(constraint.get_banned_surface_ids().empty());
  EXPECT_TRUE(constraint.get_sample_areas().empty());
}

// TODO(@silanus23): intersects_secondary returns false even when the gripper is
// placed inside the secondary shape. Needs investigation into how BVH volumes
// are built for the secondary and how the gripper geometry is represented.
TEST_F(KissingSurfaceConstraintTest, DISABLED_CollisionWhenInsideSecondary)
{
  TopoDS_Shape primary = BRepPrimAPI_MakeBox(0.1, 0.1, 0.1).Shape();

  // Large secondary that encloses the origin.
  gp_Trsf secondary_pos;
  secondary_pos.SetTranslation(gp_Vec(-0.1, -0.1, -0.1));
  TopoDS_Shape large_secondary = BRepBuilderAPI_Transform(
    BRepPrimAPI_MakeBox(0.3, 0.3, 0.3).Shape(), secondary_pos, Standard_True).Shape();

  std::vector<TopoDS_Shape> secondaries = {large_secondary};

  KissingSurfaceConstraint constraint(
    mapper_, gripper_, secondaries, 0.8, 0.001);

  Topology topology = mapper_->load_from_shape(primary);
  constraint.analyze_constraints(topology);
  wire_fcl(constraint, secondaries, primary);

  EXPECT_TRUE(constraint.intersects_secondary(0.03, gp_Trsf()));
}

// 10 mm tolerance detects a 5 mm gap; 1 mm tolerance does not.
TEST_F(KissingSurfaceConstraintTest, CollisionToleranceAffectsDetectionDistance)
{
  gp_Trsf secondary_pos;
  secondary_pos.SetTranslation(gp_Vec(-0.5, -0.5, -0.01));
  TopoDS_Shape secondary = BRepPrimAPI_MakeBox(1.0, 1.0, 0.01).Shape();
  secondary = BRepBuilderAPI_Transform(secondary, secondary_pos, Standard_True).Shape();

  std::vector<TopoDS_Shape> secondaries = {secondary};

  // Flat gripper: bottom face at z=0, top face at z=0.01.
  ParsedGripper flat_gripper;
  flat_gripper.base = BRepPrimAPI_MakeBox(
    gp_Pnt(-0.03, -0.03, 0.0), 0.06, 0.06, 0.01).Shape();
  flat_gripper.finger_1 = flat_gripper.base;
  flat_gripper.finger_2 = flat_gripper.base;
  flat_gripper.finger_1_axis = Eigen::Vector3d(0.0, 1.0, 0.0);
  flat_gripper.finger_2_axis = Eigen::Vector3d(0.0, -1.0, 0.0);
  flat_gripper.max_opening = 0.06;
  flat_gripper.gripper_type = "parallel";
  flat_gripper.tcp_offset = Eigen::Vector3d::Zero();
  flat_gripper.tcp_rpy = Eigen::Vector3d::Zero();
  flat_gripper.base_link_name = "flat_base";
  flat_gripper.finger_1_link_name = "flat_f1";
  flat_gripper.finger_2_link_name = "flat_f2";
  flat_gripper.finger_1_joint_name = "flat_j1";
  flat_gripper.finger_2_joint_name = "flat_j2";

  // Gripper bottom face at z=0; translate 5 mm above secondary top face.
  constexpr double kGap = 0.005;
  gp_Trsf pose;
  pose.SetTranslation(gp_Vec(0.0, 0.0, kGap));

  TopoDS_Shape primary = BRepPrimAPI_MakeBox(0.1, 0.1, 0.1).Shape();
  Topology topology = mapper_->load_from_shape(primary);

  // Loose tolerance (10 mm) > gap (5 mm) -> collision must be reported.
  KissingSurfaceConstraint constraint_loose(
    mapper_, flat_gripper, secondaries, 0.8, 0.010);
  constraint_loose.analyze_constraints(topology);
  {
    auto fcl_loose = make_fcl_checker(flat_gripper, primary);
    fcl_loose->add_secondary_shapes(secondaries);
    constraint_loose.set_fcl_checker(fcl_loose);
  }
  EXPECT_TRUE(constraint_loose.intersects_secondary(0.03, pose))
    << "10 mm tolerance must detect gripper whose bottom face is 5 mm above secondary";

  // Tight tolerance (1 mm) < gap (5 mm) -> no collision must be reported.
  KissingSurfaceConstraint constraint_tight(
    mapper_, flat_gripper, secondaries, 0.8, 0.001);
  constraint_tight.analyze_constraints(topology);
  {
    auto fcl_tight = make_fcl_checker(flat_gripper, primary);
    fcl_tight->add_secondary_shapes(secondaries);
    constraint_tight.set_fcl_checker(fcl_tight);
  }
  EXPECT_FALSE(constraint_tight.intersects_secondary(0.03, pose))
    << "1 mm tolerance must not fire when gripper bottom face is 5 mm above secondary";
}

// Gripper intersects any of three secondaries; clear space at (0.5, 0.5, 0.5) does not.
TEST_F(KissingSurfaceConstraintTest, CollisionWithAnySecondary)
{
  TopoDS_Shape primary = BRepPrimAPI_MakeBox(0.1, 0.1, 0.1).Shape();

  gp_Trsf bottom_pos;
  bottom_pos.SetTranslation(gp_Vec(-0.5, -0.5, -0.01));
  TopoDS_Shape bottom_secondary = BRepPrimAPI_MakeBox(1.0, 1.0, 0.01).Shape();
  bottom_secondary = BRepBuilderAPI_Transform(bottom_secondary, bottom_pos, Standard_True).Shape();

  gp_Trsf left_pos;
  left_pos.SetTranslation(gp_Vec(-0.1, 0.0, 0.0));
  TopoDS_Shape left_fixture = BRepPrimAPI_MakeBox(0.05, 0.1, 0.1).Shape();
  left_fixture = BRepBuilderAPI_Transform(left_fixture, left_pos, Standard_True).Shape();

  gp_Trsf right_pos;
  right_pos.SetTranslation(gp_Vec(0.15, 0.0, 0.0));
  TopoDS_Shape right_fixture = BRepPrimAPI_MakeBox(0.05, 0.1, 0.1).Shape();
  right_fixture = BRepBuilderAPI_Transform(right_fixture, right_pos, Standard_True).Shape();

  std::vector<TopoDS_Shape> secondaries = {bottom_secondary, left_fixture, right_fixture};

  KissingSurfaceConstraint constraint(
    mapper_, gripper_, secondaries);

  Topology topology = mapper_->load_from_shape(primary);
  constraint.analyze_constraints(topology);
  wire_fcl(constraint, secondaries, primary);

  gp_Trsf at_bottom;
  at_bottom.SetTranslation(gp_Vec(0.0, 0.0, 0.0));
  EXPECT_TRUE(constraint.intersects_secondary(0.03, at_bottom));

  gp_Trsf at_left;
  at_left.SetTranslation(gp_Vec(-0.075, 0.05, 0.05));
  EXPECT_TRUE(constraint.intersects_secondary(0.03, at_left))
    << "Gripper inside left fixture volume must report collision";

  gp_Trsf clear_space;
  clear_space.SetTranslation(gp_Vec(0.5, 0.5, 0.5));
  EXPECT_FALSE(constraint.intersects_secondary(0.03, clear_space));
}

// No secondary shapes must produce no banned IDs, no sample areas, and no collisions.
TEST_F(KissingSurfaceConstraintTest, EmptySecondaryShapesVectorProducesNoResults)
{
  std::vector<TopoDS_Shape> no_secondaries;
  KissingSurfaceConstraint constraint(mapper_, gripper_, no_secondaries, 0.5);

  TopoDS_Shape primary = BRepPrimAPI_MakeBox(0.1, 0.1, 0.1).Shape();
  Topology topology = mapper_->load_from_shape(primary);
  constraint.analyze_constraints(topology);

  auto fcl = make_fcl_checker(gripper_, primary);
  constraint.set_fcl_checker(fcl);

  EXPECT_TRUE(constraint.get_banned_surface_ids().empty());
  EXPECT_TRUE(constraint.get_sample_areas().empty());

  gp_Trsf identity;
  EXPECT_FALSE(constraint.intersects_secondary(0.03, identity));
}

// The ground belongs to GroundConstraint. With no fixtures, a pose through the
// FCL ground must not be reported as a secondary collision, or ground rejections
// get counted under the wrong reason.
TEST_F(KissingSurfaceConstraintTest, FclGroundIsNotASecondary)
{
  TopoDS_Shape primary = create_box_at_helper(0.1, 0.1, 0.1, 0.0, 0.0, 1.0);

  std::vector<TopoDS_Shape> no_secondaries;
  KissingSurfaceConstraint constraint(mapper_, gripper_, no_secondaries);
  constraint.analyze_constraints(mapper_->load_from_shape(primary));

  auto fcl = make_fcl_checker(gripper_, primary);
  fcl->add_ground_plane(Eigen::Vector3d(0.0, 0.0, 1.0), 0.0);
  constraint.set_fcl_checker(fcl);

  gp_Trsf below_ground;
  below_ground.SetTranslation(gp_Vec(0.0, 0.0, -0.2));
  ASSERT_TRUE(fcl->collides_with_ground(below_ground, 0.03, 1e-6))
    << "sanity: the pose really is through the ground";

  EXPECT_FALSE(constraint.intersects_secondary(0.03, below_ground));
}

// A surface that is fully banned must not also appear in partial-exclusion sample areas.
TEST_F(KissingSurfaceConstraintTest, BannedSurfaceIdsNotInSampleAreas)
{
  TopoDS_Shape primary = BRepPrimAPI_MakeBox(0.1, 0.1, 0.1).Shape();

  gp_Trsf secondary_pos;
  secondary_pos.SetTranslation(gp_Vec(-0.5, -0.5, -0.01));
  TopoDS_Shape secondary = BRepPrimAPI_MakeBox(1.0, 1.0, 0.01).Shape();
  secondary = BRepBuilderAPI_Transform(secondary, secondary_pos, Standard_True).Shape();

  std::vector<TopoDS_Shape> secondaries = {secondary};

  KissingSurfaceConstraint constraint(
    mapper_, gripper_, secondaries);

  Topology topology = mapper_->load_from_shape(primary);
  constraint.analyze_constraints(topology);

  std::vector<int> banned = constraint.get_banned_surface_ids();
  auto areas = constraint.get_sample_areas();

  for (const auto & area : areas) {
    bool is_banned = std::find(banned.begin(), banned.end(), area.surface_id) != banned.end();
    EXPECT_FALSE(is_banned) << "Surface " << area.surface_id <<
      " is both banned and has sample area";
  }
}

// A higher contact threshold bans the same or fewer surfaces than a lower threshold.
TEST_F(KissingSurfaceConstraintTest, HighThresholdBansFewerSurfaces)
{
  TopoDS_Shape primary = BRepPrimAPI_MakeBox(0.1, 0.1, 0.1).Shape();

  gp_Trsf secondary_pos;
  secondary_pos.SetTranslation(gp_Vec(-0.5, -0.5, -0.01));
  TopoDS_Shape secondary = BRepPrimAPI_MakeBox(1.0, 1.0, 0.01).Shape();
  secondary = BRepBuilderAPI_Transform(secondary, secondary_pos, Standard_True).Shape();

  std::vector<TopoDS_Shape> secondaries = {secondary};

  // Low threshold (50% contact bans surface)
  KissingSurfaceConstraint constraint_low(
    mapper_, gripper_, secondaries, 0.5);

  // High threshold (95% contact required to ban)
  KissingSurfaceConstraint constraint_high(
    mapper_, gripper_, secondaries, 0.95);

  Topology topology = mapper_->load_from_shape(primary);
  constraint_low.analyze_constraints(topology);
  constraint_high.analyze_constraints(topology);

  EXPECT_GE(constraint_low.get_banned_surface_ids().size(), 1u)
    << "Low threshold (50%) must ban at least the bottom face touching the ground secondary";
  EXPECT_LE(
    constraint_high.get_banned_surface_ids().size(),
    constraint_low.get_banned_surface_ids().size());
}

// Re-running analyze_constraints replaces previous results rather than accumulating them.
TEST_F(KissingSurfaceConstraintTest, ReanalysisReplacesNotAccumulates)
{
  gp_Trsf secondary_pos;
  secondary_pos.SetTranslation(gp_Vec(-0.5, -0.5, -0.01));
  TopoDS_Shape secondary = BRepPrimAPI_MakeBox(1.0, 1.0, 0.01).Shape();
  secondary = BRepBuilderAPI_Transform(secondary, secondary_pos, Standard_True).Shape();

  std::vector<TopoDS_Shape> secondaries = {secondary};

  KissingSurfaceConstraint constraint(mapper_, gripper_, secondaries, 0.5);

  TopoDS_Shape touching_box = BRepPrimAPI_MakeBox(0.1, 0.1, 0.1).Shape();
  Topology touching_topology = mapper_->load_from_shape(touching_box);
  constraint.analyze_constraints(touching_topology);

  size_t first_banned_count = constraint.get_banned_surface_ids().size();
  EXPECT_GE(first_banned_count, 1u) << "First analysis must ban the touching face";

  gp_Trsf elevated_pos;
  elevated_pos.SetTranslation(gp_Vec(0.0, 0.0, 1.0));
  TopoDS_Shape elevated_box = BRepBuilderAPI_Transform(
    BRepPrimAPI_MakeBox(0.1, 0.1, 0.1).Shape(), elevated_pos, Standard_True).Shape();
  Topology elevated_topology = mapper_->load_from_shape(elevated_box);
  constraint.analyze_constraints(elevated_topology);

  EXPECT_EQ(constraint.get_banned_surface_ids().size(), 0u);
}

// --- Partial contact exclusion wires -----------------------------------------
//
// A surface whose contact ratio falls below contact_threshold is not banned;
// instead the region in contact becomes a partial-exclusion SampleArea. That
// wire is later fed to is_point_inside_wire(), which ray-casts and therefore
// requires a *closed* wire to mean anything.

namespace
{

// Number of edges in a wire.
int count_wire_edges(const TopoDS_Wire & wire)
{
  int n = 0;
  for (TopExp_Explorer exp(wire, TopAbs_EDGE); exp.More(); exp.Next()) {
    ++n;
  }
  return n;
}

}  // namespace

// A bottom face only half covered by the ground sits below contact_threshold,
// so it must survive as a usable partial exclusion rather than being banned.
TEST_F(KissingSurfaceConstraintTest, PartialGroundContactYieldsClosedExclusionWire)
{
  // Primary occupies x in [0, 0.1], y in [0, 0.1], z in [0, 0.1].
  TopoDS_Shape primary = BRepPrimAPI_MakeBox(0.1, 0.1, 0.1).Shape();

  // Ground covers only x in [0, 0.05] — half the bottom face — top face at z=0.
  gp_Trsf ground_pos;
  ground_pos.SetTranslation(gp_Vec(0.0, -0.05, -0.01));
  TopoDS_Shape ground = BRepBuilderAPI_Transform(
    BRepPrimAPI_MakeBox(0.05, 0.2, 0.01).Shape(), ground_pos, Standard_True).Shape();

  std::vector<TopoDS_Shape> secondaries = {ground};

  KissingSurfaceConstraint constraint(mapper_, gripper_, secondaries, 0.8);

  Topology topology = mapper_->load_from_shape(primary);
  constraint.analyze_constraints(topology);

  const auto banned = constraint.get_banned_surface_ids();
  const auto areas = constraint.get_sample_areas();

  EXPECT_TRUE(banned.empty())
    << "~50% contact is below the 80% threshold, so nothing should be banned outright";

  ASSERT_FALSE(areas.empty())
    << "Half-covered bottom face must produce a partial-exclusion area; "
    << "an empty result means the contact region is not excluded from sampling at all";

  for (const auto & area : areas) {
    EXPECT_TRUE(area.is_exclusion) << "Contact regions are keep-out, not keep-in";
    ASSERT_FALSE(area.wire.IsNull()) << "Surface " << area.surface_id << ": null wire";
    EXPECT_TRUE(is_wire_closed(area.wire))
      << "Surface " << area.surface_id << ": wire has " << count_wire_edges(area.wire)
      << " edge(s) and is not closed — is_point_inside_wire() ray casting is "
      << "undefined on an open wire";
  }
}

// A box resting on one edge has no fully supported face: the two faces meeting
// at the resting edge graze the ground along a line. This is the case the
// roadmap's "topology aware" ground rejection has to get right.
//
// This failed while measure_contact_ratio sampled triangle centroids: both
// grazing faces had their lowest node exactly on the ground (z = 0.00000) but
// their lowest centroid 23.6 mm above it, so contact ratio read 0% on all six
// faces and the part looked airborne. It passes now that the ratio comes from
// sampling::sample_face_region, which measures the 7.07% strip directly.
TEST_F(KissingSurfaceConstraintTest, EdgeRestingPartYieldsClosedExclusionWire)
{
  // Box centred on x, spanning y in [-0.05, 0.05], z in [0, 0.1].
  TopoDS_Shape box = BRepPrimAPI_MakeBox(
    gp_Pnt(-0.05, -0.05, 0.0), 0.1, 0.1, 0.1).Shape();

  // Roll 45 degrees about X so the part balances on its y=-0.05, z=0 edge.
  gp_Trsf roll;
  roll.SetRotation(gp_Ax1(gp_Pnt(0.0, 0.0, 0.0), gp_Dir(1.0, 0.0, 0.0)), M_PI / 4.0);

  // Lift so the lowest corner lands exactly on z=0.
  constexpr double kDrop = 0.05 * M_SQRT1_2;  // |min z| after the roll
  gp_Trsf lift;
  lift.SetTranslation(gp_Vec(0.0, 0.0, kDrop));

  TopoDS_Shape primary = BRepBuilderAPI_Transform(
    BRepBuilderAPI_Transform(box, roll, Standard_True).Shape(), lift, Standard_True).Shape();

  gp_Trsf ground_pos;
  ground_pos.SetTranslation(gp_Vec(-0.2, -0.2, -0.01));
  TopoDS_Shape ground = BRepBuilderAPI_Transform(
    BRepPrimAPI_MakeBox(0.4, 0.4, 0.01).Shape(), ground_pos, Standard_True).Shape();

  std::vector<TopoDS_Shape> secondaries = {ground};

  KissingSurfaceConstraint constraint(mapper_, gripper_, secondaries, 0.8);

  Topology topology = mapper_->load_from_shape(primary);
  constraint.analyze_constraints(topology);

  const auto banned = constraint.get_banned_surface_ids();
  const auto areas = constraint.get_sample_areas();

  EXPECT_TRUE(banned.empty())
    << "No face of an edge-resting box is supported, so none should be banned";

  ASSERT_FALSE(areas.empty())
    << "The two faces meeting at the resting edge graze the ground and must "
    << "produce partial-exclusion areas; an empty result means contact points "
    << "can still be sampled right down at floor level";

  for (const auto & area : areas) {
    ASSERT_FALSE(area.wire.IsNull()) << "Surface " << area.surface_id << ": null wire";
    EXPECT_TRUE(is_wire_closed(area.wire))
      << "Surface " << area.surface_id << ": wire has " << count_wire_edges(area.wire)
      << " edge(s) and is not closed — is_point_inside_wire() ray casting is "
      << "undefined on an open wire";
  }
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
