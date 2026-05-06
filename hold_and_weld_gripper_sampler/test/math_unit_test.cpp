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
#include <cmath>
#include <vector>

#include <BRepPrimAPI_MakeBox.hxx>
#include <gp_Ax1.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>
#include <gp_Quaternion.hxx>
#include <gp_Trsf.hxx>
#include <gp_Vec.hxx>
#include <TopoDS_Shape.hxx>

#include "hold_and_weld_gripper_sampler/angle_finding/grasp_orientation_finder.hpp"
#include "hold_and_weld_gripper_sampler/geometry/geometry_mapper.hpp"
#include "hold_and_weld_gripper_sampler/geometry/occt_utils.hpp"
#include "hold_and_weld_gripper_sampler/sampling/contact_point_sampler.hpp"

using namespace hold_and_weld_gripper_sampler;           // NOLINT
using namespace hold_and_weld_gripper_sampler::geometry;  // NOLINT
using namespace hold_and_weld_gripper_sampler::sampling;  // NOLINT
using namespace hold_and_weld_gripper_sampler::angle_finding;  // NOLINT

namespace
{

// ---------------------------------------------------------------------------
// Local copy of build_tangent_frame (mirrors the static implementation in
// grasp_orientation_finder.cpp exactly, so we can test its math properties).
// ---------------------------------------------------------------------------
void build_tangent_frame_local(const gp_Vec & normal, gp_Vec & out_lx, gp_Vec & out_ly)
{
  const double sign = (normal.Z() >= 0.0) ? 1.0 : -1.0;
  const double inv_denom = -1.0 / (sign + normal.Z());
  const double xy_cross_term = normal.X() * normal.Y() * inv_denom;
  out_lx = gp_Vec(
    1.0 + sign * normal.X() * normal.X() * inv_denom,
    sign * xy_cross_term,
    -sign * normal.X());
  out_ly = gp_Vec(
    xy_cross_term,
    sign + normal.Y() * normal.Y() * inv_denom,
    -normal.Y());
}

}  // namespace

// ---------------------------------------------------------------------------
// Test 1 — BuildTangentFrame_KnownNormal
//
// Call build_tangent_frame with a unit normal (0, 0, 1).  The three vectors
// {normal, lx, ly} must form an orthonormal basis:
//   * each must have magnitude ≈ 1  (tol 1e-10)
//   * every pair must be orthogonal (dot ≈ 0, tol 1e-10)
// ---------------------------------------------------------------------------
TEST(MathUnit, BuildTangentFrame_KnownNormal)
{
  const gp_Vec normal(0.0, 0.0, 1.0);
  gp_Vec lx, ly;
  build_tangent_frame_local(normal, lx, ly);

  // Unit-length check
  EXPECT_NEAR(normal.Magnitude(), 1.0, 1e-10) << "Input normal should already be unit length";
  EXPECT_NEAR(lx.Magnitude(), 1.0, 1e-10) << "lx must be unit length";
  EXPECT_NEAR(ly.Magnitude(), 1.0, 1e-10) << "ly must be unit length";

  // Orthogonality checks
  EXPECT_NEAR(normal.Dot(lx), 0.0, 1e-10) << "normal · lx must be 0";
  EXPECT_NEAR(normal.Dot(ly), 0.0, 1e-10) << "normal · ly must be 0";
  EXPECT_NEAR(lx.Dot(ly), 0.0, 1e-10) << "lx · ly must be 0";
}

// The same properties must hold for a non-axis-aligned normal.
TEST(MathUnit, BuildTangentFrame_ArbitraryNormal)
{
  // A unit normal at 45° in XZ plane
  const double inv_sqrt2 = 1.0 / std::sqrt(2.0);
  const gp_Vec normal(inv_sqrt2, 0.0, inv_sqrt2);
  gp_Vec lx, ly;
  build_tangent_frame_local(normal, lx, ly);

  EXPECT_NEAR(lx.Magnitude(), 1.0, 1e-10) << "lx must be unit length";
  EXPECT_NEAR(ly.Magnitude(), 1.0, 1e-10) << "ly must be unit length";

  EXPECT_NEAR(normal.Dot(lx), 0.0, 1e-10) << "normal · lx must be 0";
  EXPECT_NEAR(normal.Dot(ly), 0.0, 1e-10) << "normal · ly must be 0";
  EXPECT_NEAR(lx.Dot(ly), 0.0, 1e-10) << "lx · ly must be 0";
}

// Negative-Z normal (sign = -1 branch)
TEST(MathUnit, BuildTangentFrame_NegativeZNormal)
{
  const gp_Vec normal(0.0, 0.0, -1.0);
  gp_Vec lx, ly;
  build_tangent_frame_local(normal, lx, ly);

  EXPECT_NEAR(lx.Magnitude(), 1.0, 1e-10) << "lx must be unit length";
  EXPECT_NEAR(ly.Magnitude(), 1.0, 1e-10) << "ly must be unit length";

  EXPECT_NEAR(normal.Dot(lx), 0.0, 1e-10) << "normal · lx must be 0";
  EXPECT_NEAR(normal.Dot(ly), 0.0, 1e-10) << "normal · ly must be 0";
  EXPECT_NEAR(lx.Dot(ly), 0.0, 1e-10) << "lx · ly must be 0";
}

// ---------------------------------------------------------------------------
// Test 2 — ExtractQuaternion_90DegZRotation
//
// Build a gp_Trsf that rotates 90° around the world Z axis.
// extract_quaternion must return a quaternion with:
//   norm ≈ 1.0
//   x ≈ 0, y ≈ 0, |z| ≈ sin(π/4), |w| ≈ cos(π/4)  (either sign is valid)
// ---------------------------------------------------------------------------
TEST(MathUnit, ExtractQuaternion_90DegZRotation)
{
  gp_Trsf trsf;
  trsf.SetRotation(gp_Ax1(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1)), M_PI / 2.0);

  const Eigen::Quaterniond q = extract_quaternion(trsf);

  constexpr double tol = 1e-6;
  const double expected_zw = std::sin(M_PI / 4.0);  // ≈ 0.70710678

  EXPECT_NEAR(q.norm(), 1.0, tol) << "Quaternion must be unit length";
  EXPECT_NEAR(q.x(), 0.0, tol) << "x component must be 0";
  EXPECT_NEAR(q.y(), 0.0, tol) << "y component must be 0";

  // Both (z, w) = (+sin45, +cos45) and (-sin45, -cos45) represent the same rotation.
  const bool positive_convention =
    std::abs(q.z() - expected_zw) < tol && std::abs(q.w() - expected_zw) < tol;
  const bool negative_convention =
    std::abs(q.z() + expected_zw) < tol && std::abs(q.w() + expected_zw) < tol;

  EXPECT_TRUE(positive_convention || negative_convention)
    << "Expected z≈±" << expected_zw << " w≈±" << expected_zw
    << " but got z=" << q.z() << " w=" << q.w();
}

// ---------------------------------------------------------------------------
// Test 3 — RpyToQuaternion_HalfPiZ
//
// rpy_to_quaternion(0, 0, π/2) must produce the same 90° Z-rotation quaternion
// as the one verified in ExtractQuaternion_90DegZRotation.
// ---------------------------------------------------------------------------
TEST(MathUnit, RpyToQuaternion_HalfPiZ)
{
  const gp_Quaternion q_occt = rpy_to_quaternion(0.0, 0.0, M_PI / 2.0);

  // Convert to Eigen for easier norm check
  const Eigen::Quaterniond q(q_occt.W(), q_occt.X(), q_occt.Y(), q_occt.Z());

  constexpr double tol = 1e-6;
  const double expected_zw = std::sin(M_PI / 4.0);

  EXPECT_NEAR(q.norm(), 1.0, tol) << "Quaternion must be unit length";
  EXPECT_NEAR(q.x(), 0.0, tol) << "x must be 0 for pure Z rotation";
  EXPECT_NEAR(q.y(), 0.0, tol) << "y must be 0 for pure Z rotation";

  const bool positive_convention =
    std::abs(q.z() - expected_zw) < tol && std::abs(q.w() - expected_zw) < tol;
  const bool negative_convention =
    std::abs(q.z() + expected_zw) < tol && std::abs(q.w() + expected_zw) < tol;

  EXPECT_TRUE(positive_convention || negative_convention)
    << "Expected z≈±" << expected_zw << " w≈±" << expected_zw
    << " but got z=" << q.z() << " w=" << q.w();
}

// ---------------------------------------------------------------------------
// Test 4 — ToGrasp_QuaternionNormAndTranslation
//
// Build a GraspCandidate with two known contact points and an identity
// gripper transform.  to_grasp must return:
//   * tcp_orientation.norm() ≈ 1.0
//   * tcp_position ≈ midpoint of contact_1 and contact_2
// ---------------------------------------------------------------------------
TEST(MathUnit, ToGrasp_QuaternionNormAndTranslation)
{
  GraspCandidate candidate;
  candidate.contact_1 = gp_Pnt(0.0, 0.0, 0.0);
  candidate.contact_2 = gp_Pnt(0.0, 0.0, 0.05);
  candidate.surface_id_1 = 0;
  candidate.surface_id_2 = 1;
  candidate.grip_distance = 0.05;
  candidate.quality_score = 1.0;

  // Identity transform — no rotation, no translation
  candidate.gripper_transform = gp_Trsf();

  // Needed by to_grasp but not asserted here
  candidate.approach_direction = gp_Vec(1.0, 0.0, 0.0);
  candidate.base_position = gp_Pnt(0.0, 0.0, 0.0);

  const Grasp grasp = to_grasp(candidate);

  constexpr double tol = 1e-6;

  // Orientation quaternion must be unit-length
  EXPECT_NEAR(grasp.tcp_orientation.norm(), 1.0, tol)
    << "Grasp orientation quaternion must be unit length";

  // TCP position must be the midpoint of the two contact points
  const Eigen::Vector3d expected_mid(0.0, 0.0, 0.025);
  EXPECT_NEAR(grasp.tcp_position.x(), expected_mid.x(), tol);
  EXPECT_NEAR(grasp.tcp_position.y(), expected_mid.y(), tol);
  EXPECT_NEAR(grasp.tcp_position.z(), expected_mid.z(), tol);
}

// ---------------------------------------------------------------------------
// Test 5 — ContactPointSampler_TightBox_FindsOpposingFaces
//
// Build a 0.04 m × 0.10 m × 0.10 m box (4 cm gap in X).
// Configure sampler with min_opening=0.038, max_opening=0.041.
// generate_contact_pairs must find at least one pair and every pair must
// have grip_distance ∈ [0.038, 0.041].
// ---------------------------------------------------------------------------
TEST(MathUnit, ContactPointSampler_TightBox_FindsOpposingFaces)
{
  // Box: 4 cm (X) × 10 cm (Y) × 10 cm (Z)
  const double box_x = 0.04;
  const double box_y = 0.10;
  const double box_z = 0.10;
  const TopoDS_Shape box = BRepPrimAPI_MakeBox(box_x, box_y, box_z).Shape();

  GeometryMapper mapper;
  const Topology topology = mapper.load_from_shape(box);

  // All surface IDs are eligible
  std::vector<int> all_ids;
  all_ids.reserve(topology.num_surfaces());
  for (size_t i = 0; i < topology.num_surfaces(); ++i) {
    all_ids.push_back(static_cast<int>(i));
  }

  SamplingConfig config;
  config.min_gripper_opening = 0.038;
  config.max_gripper_opening = 0.041;
  config.sample_density = 0.01;

  ContactPointSampler sampler(config);
  const std::vector<ContactPair> pairs =
    sampler.generate_contact_pairs(topology, all_ids, {});

  EXPECT_GT(pairs.size(), 0u)
    << "Sampler must find at least one pair on a box with a 4 cm X-gap "
       "when the gripper opening range is 38–41 mm";

  for (const auto & pair : pairs) {
    EXPECT_GE(pair.grip_distance, 0.038 - 1e-6)
      << "grip_distance below min_gripper_opening";
    EXPECT_LE(pair.grip_distance, 0.041 + 1e-6)
      << "grip_distance above max_gripper_opening";
  }
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
