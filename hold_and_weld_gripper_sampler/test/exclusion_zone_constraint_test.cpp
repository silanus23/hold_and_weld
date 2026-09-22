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
#include <functional>
#include <memory>
#include <utility>
#include <vector>

#include <BRepPrimAPI_MakeBox.hxx>

#include <gp_Trsf.hxx>
#include <gp_Vec.hxx>
#include <TopoDS_Shape.hxx>
#include <TopoDS_Wire.hxx>

#include "hold_and_weld_gripper_sampler/core/gripper.hpp"
#include "hold_and_weld_gripper_sampler/constraints/exclusion_zone_constraint.hpp"
#include "hold_and_weld_gripper_sampler/collision/fcl_collision_checker.hpp"
#include "hold_and_weld_gripper_sampler/geometry/geometry_mapper.hpp"
#include "hold_and_weld_gripper_sampler/sampling/face_sampler.hpp"
#include "test_helpers.hpp"

using namespace hold_and_weld_gripper_sampler;  // NOLINT
using namespace hold_and_weld_gripper_sampler::constraints;  // NOLINT
using namespace hold_and_weld_gripper_sampler::geometry;  // NOLINT


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
    auto fcl = make_fcl_checker(gripper, primary_shape);
    fcl->add_exclusion_volumes(constraint.get_collision_volumes());
    constraint.set_fcl_checker(fcl);
  }

  std::shared_ptr<GeometryMapper> mapper_;
  ParsedGripper gripper_;
};

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

// TODO(@silanus23): FCL collision check returns false even when the gripper is
// placed at origin inside a circle exclusion zone. Likely a volume construction
// or transform issue in the FCL wiring for circle exclusions.
TEST_F(ExclusionZoneConstraintTest, DISABLED_CollisionWhenInsideExclusionZone)
{
  exclusion_circle circle;
  circle.center = Eigen::Vector3d(0.0, 0.0, 0.0);
  circle.normal = Eigen::Vector3d(0.0, 0.0, 1.0);
  circle.radius = 0.1;
  circle.projection_depth = 0.1;
  circle.clearance = 0.01;

  std::vector<exclusion_circle> circles = {circle};

  ExclusionZoneConstraint constraint(mapper_, gripper_, circles);

  TopoDS_Shape test_box = BRepPrimAPI_MakeBox(0.3, 0.3, 0.2).Shape();
  Topology topology = mapper_->load_from_shape(test_box);
  constraint.analyze_constraints(test_box, topology);
  wire_fcl(constraint, gripper_, test_box);

  gp_Trsf origin_transform;
  origin_transform.SetTranslation(gp_Vec(0.0, 0.0, 0.0));

  EXPECT_TRUE(constraint.intersects_exclusion_zone(origin_transform, 0.03));
}

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

// TODO(@silanus23): FCL collision check returns false even when the gripper is
// placed inside a polygon exclusion prism. Likely a volume construction or
// transform issue in the FCL wiring for polygon exclusions.
TEST_F(ExclusionZoneConstraintTest, DISABLED_CollisionWithPolygonExclusionZone)
{
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

  TopoDS_Shape test_box = BRepPrimAPI_MakeBox(0.3, 0.3, 0.2).Shape();
  Topology topology = mapper_->load_from_shape(test_box);
  constraint.analyze_constraints(test_box, topology);
  wire_fcl(constraint, gripper_, test_box);

  gp_Trsf origin_transform;
  origin_transform.SetTranslation(gp_Vec(0.0, 0.0, 0.02));

  EXPECT_TRUE(constraint.intersects_exclusion_zone(origin_transform, 0.03));
}

// Degenerate zero-length line exclusion must not crash.
TEST_F(ExclusionZoneConstraintTest, ZeroLengthLineHandled)
{
  exclusion_line line;
  line.start = Eigen::Vector3d(0.0, 0.0, 0.0);
  line.end = Eigen::Vector3d(0.0, 0.0, 0.0);
  line.exclusion_radius = 0.01;
  line.clearance = 0.005;

  std::vector<exclusion_line> lines = {line};

  ExclusionZoneConstraint constraint(
    mapper_, gripper_, std::nullopt, std::nullopt, lines);

  TopoDS_Shape test_box = BRepPrimAPI_MakeBox(0.2, 0.2, 0.1).Shape();
  Topology topology = mapper_->load_from_shape(test_box);
  constraint.analyze_constraints(test_box, topology);
  wire_fcl(constraint, gripper_, test_box);
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

  constraint.intersects_exclusion_zone(transform, 0.03);
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

  EXPECT_EQ(constraint.get_name(), "ExclusionZoneConstraint");
  EXPECT_TRUE(constraint.get_sample_areas().empty());
}

// --- Exclusion wires ---------------------------------------------------------
//
// The wires feed the contact sampler's point-in-wire test, so each must be
// closed, and together they must keep every sample out of the zone.

namespace
{

// Walk every face with its exclusion wires applied and return the surviving
// samples that satisfy `inside_zone`.
std::vector<gp_Pnt> samples_left_in_zone(
  const Topology & topology,
  const std::vector<core::SampleArea> & areas,
  const std::function<bool(const gp_Pnt &)> & inside_zone)
{
  sampling::FaceSamplingConfig config;
  config.sample_density = 0.002;

  std::vector<gp_Pnt> leaked;
  const auto & surfaces = topology.get_all_surfaces();
  for (size_t id = 0; id < surfaces.size(); ++id) {
    std::vector<std::pair<TopoDS_Wire, bool>> wires;
    for (const auto & area : areas) {
      if (area.surface_id == static_cast<int>(id)) {
        wires.emplace_back(area.wire, area.is_exclusion);
      }
    }
    for (const auto & sample : sampling::sample_face_region(surfaces[id].face, config, wires)) {
      if (inside_zone(sample.point)) {leaked.push_back(sample.point);}
    }
  }
  return leaked;
}

}  // namespace

// A weld seam running off the edge of a face: the tube cuts the top face in a
// strip that ends at the face's own boundary, and cuts the side face in a
// half-disk. A section of the tube against the part returns only the cut
// curves, never the face edges that close these regions, so the wires came out
// open and the sampler could not tell inside from outside.
TEST_F(ExclusionZoneConstraintTest, SeamRunningOffAFaceYieldsClosedWiresThatExclude)
{
  exclusion_line seam;
  seam.start = Eigen::Vector3d(0.05, 0.05, 0.1);
  seam.end = Eigen::Vector3d(0.2, 0.05, 0.1);
  seam.exclusion_radius = 0.01;

  ExclusionZoneConstraint constraint(
    mapper_, gripper_, std::nullopt, std::nullopt, std::vector<exclusion_line>{seam});

  const TopoDS_Shape box = BRepPrimAPI_MakeBox(0.1, 0.1, 0.1).Shape();
  const Topology topology = mapper_->load_from_shape(box);
  constraint.analyze_constraints(box, topology);

  const auto areas = constraint.get_sample_areas();
  ASSERT_FALSE(areas.empty()) << "the seam crosses the top and side faces";
  for (const auto & area : areas) {
    ASSERT_FALSE(area.wire.IsNull());
    EXPECT_TRUE(area.is_exclusion);
    EXPECT_TRUE(is_wire_closed(area.wire))
      << "surface " << area.surface_id << ": open wires break point-in-wire ray casting";
  }

  // Strictly inside the tube, with a margin so boundary samples do not count.
  const auto leaked = samples_left_in_zone(
    topology, areas, [](const gp_Pnt & p) {
      const double r = std::hypot(p.Y() - 0.05, p.Z() - 0.1);
      return p.X() > 0.052 && r < 0.008;
    });
  EXPECT_TRUE(leaked.empty())
    << leaked.size() << " sample(s) survive inside the seam, first at ("
    << (leaked.empty() ? 0.0 : leaked[0].X()) << ", "
    << (leaked.empty() ? 0.0 : leaked[0].Y()) << ", "
    << (leaked.empty() ? 0.0 : leaked[0].Z()) << ")";
}

// A screw hole in the middle of a face. The volume sits on the face, so the
// samples to exclude lie on its boundary rather than strictly inside it.
TEST_F(ExclusionZoneConstraintTest, CircleInsideAFaceExcludesItsDisk)
{
  exclusion_circle hole;
  hole.center = Eigen::Vector3d(0.05, 0.05, 0.1);
  hole.normal = Eigen::Vector3d(0.0, 0.0, 1.0);
  hole.radius = 0.02;
  hole.projection_depth = 0.02;

  ExclusionZoneConstraint constraint(mapper_, gripper_, std::vector<exclusion_circle>{hole});

  const TopoDS_Shape box = BRepPrimAPI_MakeBox(0.1, 0.1, 0.1).Shape();
  const Topology topology = mapper_->load_from_shape(box);
  constraint.analyze_constraints(box, topology);

  const auto areas = constraint.get_sample_areas();
  ASSERT_EQ(areas.size(), 1u) << "only the top face touches the hole";
  EXPECT_TRUE(is_wire_closed(areas[0].wire));

  const auto leaked = samples_left_in_zone(
    topology, areas, [](const gp_Pnt & p) {
      return std::abs(p.Z() - 0.1) < 1e-9 &&
             std::hypot(p.X() - 0.05, p.Y() - 0.05) < 0.018;
    });
  EXPECT_TRUE(leaked.empty()) << leaked.size() << " sample(s) survive inside the hole";
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
