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
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>

#include "hold_and_weld_application/kinematics/ceres_ik_solver.hpp"
#include "hold_and_weld_application/kinematics/configuration_finder.hpp"
#include "hold_and_weld_application/kinematics/kinematics_solver.hpp"
#include "hold_and_weld_application/kinematics/urdf_parser.hpp"

using hold_and_weld::kinematics::Candidate;
using hold_and_weld::kinematics::CeresIKSolver;
using hold_and_weld::kinematics::ConfigurationFinder;
using hold_and_weld::kinematics::ConfigurationFinderParams;
using hold_and_weld::kinematics::KinematicsSolver;
using hold_and_weld::kinematics::ParsedChain;
using hold_and_weld::kinematics::URDFParser;
using hold_and_weld::kinematics::WalkFailure;
using hold_and_weld::kinematics::WalkResult;
using Vector6d = ConfigurationFinder::Vector6d;

namespace
{
constexpr double kApproachOffset = 0.1;

std::vector<double> to_vec(const Vector6d & q)
{
  return std::vector<double>(q.data(), q.data() + 6);
}

double max_abs_diff(const Vector6d & a, const Vector6d & b)
{
  return (a - b).cwiseAbs().maxCoeff();
}
}  // namespace

class ConfigurationFinderTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }

    std::string pkg_share = ament_index_cpp::get_package_share_directory(
      "hold_and_weld_description");
    URDFParser parser;
    ParsedChain chain = parser.extract_joint_chain(
      pkg_share + "/urdf/dual_robot.xacro", "robot2_base_link", "robot2_wire_tip");

    kin_ = std::make_shared<KinematicsSolver>(chain);
    ik_ = std::make_shared<CeresIKSolver>(kin_);
    finder_ = std::make_unique<ConfigurationFinder>(kin_, ik_, ConfigurationFinderParams{});

    // welding.yaml safety_pose
    q_home_ << -0.0287, 0.4555, 2.0266, -0.0287, -1.5711, 0.0;
  }

  void TearDown() override
  {
    finder_.reset();
    ik_.reset();
    kin_.reset();
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }

  Eigen::Isometry3d fk(const Vector6d & q) const {return kin_->compute_fk(to_vec(q));}

  void expect_same_pose(const Eigen::Isometry3d & a, const Eigen::Isometry3d & b, double tol)
  {
    EXPECT_LT((a.translation() - b.translation()).norm(), tol);
    EXPECT_LT(Eigen::AngleAxisd(a.rotation().transpose() * b.rotation()).angle(), tol);
  }

  bool within_limits(const Vector6d & q) const
  {
    const auto & lim = kin_->joint_limits();
    for (int i = 0; i < 6; ++i) {
      if (q(i) < lim[i].first || q(i) > lim[i].second) {return false;}
    }
    return true;
  }

  std::shared_ptr<KinematicsSolver> kin_;
  std::shared_ptr<CeresIKSolver> ik_;
  std::unique_ptr<ConfigurationFinder> finder_;
  Vector6d q_home_;
};

// A line seam that rotates J6 by 2.6 rad. From the J6 copy near +2*pi the weld runs
// into the +455 deg limit; from the base copy it is fine. The finder must not pick
// the bad copy.
TEST_F(ConfigurationFinderTest, RanksFeasibleJ6CopyFirstWhenAnotherCopyRunsOutOfRange)
{
  Vector6d q_ref;
  q_ref << 0.0, 0.4, 0.3, 0.0, -1.0, 1.0;
  Vector6d q_end = q_ref;
  q_end(5) += 2.6;

  const std::vector<Eigen::Isometry3d> seam = {fk(q_ref), fk(q_end)};
  const Eigen::Isometry3d approach =
    hold_and_weld::kinematics::standoff_pose(seam.front(), kApproachOffset);
  const auto path = finder_->pilz_path(seam, "line", approach);
  ASSERT_GT(path.size(), 2u);

  Vector6d q_near;
  ASSERT_TRUE(ik_->solve(approach, q_ref, q_near, 1e-5, 1e-4, 0.0));
  Vector6d q_bad = q_near;
  q_bad(5) += 2.0 * M_PI;
  ASSERT_TRUE(within_limits(q_bad));
  expect_same_pose(fk(q_bad), approach, 1e-4);

  const WalkResult bad = finder_->walk(q_bad, path);
  EXPECT_FALSE(bad.feasible);
  const WalkResult good = finder_->walk(q_near, path);
  EXPECT_TRUE(good.feasible) << "failure: " << to_string(good.failure);

  const auto ranked = finder_->find(seam, "line", kApproachOffset, q_home_);
  ASSERT_FALSE(ranked.empty());
  EXPECT_TRUE(ranked.front().walk.feasible);
  EXPECT_GT(std::abs(ranked.front().q_start(5) - q_bad(5)), 1.0);
  EXPECT_TRUE(finder_->walk(ranked.front().q_start, path).feasible);

  bool bad_copy_listed_infeasible = false;
  for (const auto & c : ranked) {
    if (max_abs_diff(c.q_start, q_bad) < 1e-2) {
      bad_copy_listed_infeasible = !c.walk.feasible;
    }
  }
  EXPECT_TRUE(bad_copy_listed_infeasible);
}

TEST_F(ConfigurationFinderTest, ExpansionReproducesPoseWithinLimitsAndDistinct)
{
  Vector6d q;
  q << 0.2, 0.3, 0.2, 0.5, -0.8, 1.0;
  const auto expanded = finder_->expand_solution(q);

  // q4: 0.5 and flip -2.64 (one copy each within +-200 deg);
  // q6: 1.0, 1.0 +- 2*pi (three copies) and flip 4.14, 4.14 - 2*pi (two copies).
  EXPECT_EQ(expanded.size(), 5u);

  bool has_original = false;
  bool has_flip = false;
  for (size_t i = 0; i < expanded.size(); ++i) {
    EXPECT_TRUE(within_limits(expanded[i]));
    expect_same_pose(fk(expanded[i]), fk(q), 1e-9);
    has_original |= max_abs_diff(expanded[i], q) < 1e-12;
    has_flip |= std::abs(expanded[i](4) + q(4)) < 1e-12;
    for (size_t j = i + 1; j < expanded.size(); ++j) {
      EXPECT_GT(max_abs_diff(expanded[i], expanded[j]), 1.0);
    }
  }
  EXPECT_TRUE(has_original);
  EXPECT_TRUE(has_flip);
}

TEST_F(ConfigurationFinderTest, GeneratedCandidatesReachPoseAndAreDeduplicated)
{
  Vector6d q;
  q << 0.2, 0.3, 0.2, 0.5, -0.8, 1.0;
  const Eigen::Isometry3d pose = fk(q);
  const auto candidates = finder_->generate_candidates(pose, q_home_);
  ASSERT_FALSE(candidates.empty());

  bool contains_q = false;
  for (size_t i = 0; i < candidates.size(); ++i) {
    EXPECT_TRUE(within_limits(candidates[i]));
    expect_same_pose(fk(candidates[i]), pose, 1e-3);
    contains_q |= max_abs_diff(candidates[i], q) < 1e-2;
    for (size_t j = i + 1; j < candidates.size(); ++j) {
      EXPECT_GT(max_abs_diff(candidates[i], candidates[j]), finder_->params().dedupe_epsilon);
    }
  }
  EXPECT_TRUE(contains_q);
}

TEST_F(ConfigurationFinderTest, SameInputGivesIdenticalRanking)
{
  Vector6d q_a;
  q_a << 0.1, 0.4, 0.3, 0.2, -1.0, 0.5;
  Vector6d q_b = q_a;
  q_b(0) += 0.15;
  q_b(5) += 0.4;
  const std::vector<Eigen::Isometry3d> seam = {fk(q_a), fk(q_b)};

  const auto first = finder_->find(seam, "line", kApproachOffset, q_home_);
  const auto second = finder_->find(seam, "line", kApproachOffset, q_home_);
  ASSERT_FALSE(first.empty());
  ASSERT_EQ(first.size(), second.size());
  for (size_t i = 0; i < first.size(); ++i) {
    EXPECT_EQ(first[i].index, second[i].index);
    EXPECT_EQ(first[i].q_start, second[i].q_start);
    EXPECT_EQ(first[i].walk.feasible, second[i].walk.feasible);
    EXPECT_EQ(first[i].score, second[i].score);
  }
}

// Line that passes 5 mrad from the wrist singularity (q5 = 0): the orientation barely
// changes but J4/J6 must swing ~pi, which Pilz rejects as a joint-velocity violation.
TEST_F(ConfigurationFinderTest, ReportsJointStepNearWristSingularity)
{
  const double tilt = 0.05;
  const double miss = 0.005;
  auto wrist = [](double q4, double q5) {
      Vector6d q;
      q << 0.0, 0.4, 0.3, q4, q5, -q4;
      return q;
    };
  const Vector6d q_a = wrist(std::atan2(tilt, miss), std::hypot(tilt, miss));
  const Vector6d q_b = wrist(std::atan2(-tilt, miss), std::hypot(tilt, miss));
  const std::vector<Eigen::Isometry3d> seam = {fk(q_a), fk(q_b)};

  // Approach pose == seam start, so the path is only the LIN weld.
  const auto path = finder_->pilz_path(seam, "line", seam.front());
  ASSERT_GT(path.size(), 2u);

  const WalkResult result = finder_->walk(q_a, path);
  EXPECT_FALSE(result.feasible);
  EXPECT_EQ(result.failure, WalkFailure::kJointStep) << to_string(result.failure);
  EXPECT_GT(result.max_joint_step, finder_->params().max_joint_step);
}

TEST_F(ConfigurationFinderTest, PilzPathFollowsLineAndArcGeometry)
{
  Vector6d q_a;
  q_a << 0.1, 0.4, 0.3, 0.2, -1.0, 0.5;
  Vector6d q_b = q_a;
  q_b(0) += 0.2;
  const Eigen::Isometry3d start = fk(q_a);
  const Eigen::Isometry3d end = fk(q_b);
  const Eigen::Isometry3d approach =
    hold_and_weld::kinematics::standoff_pose(start, kApproachOffset);
  const double step = finder_->params().path_step;

  // Standoff is along the seam start's local +Z (same maths as the welder).
  EXPECT_NEAR(
    (approach.translation() - start.translation()).dot(start.rotation().col(2)),
    kApproachOffset, 1e-12);

  const auto line = finder_->pilz_path({start, end}, "line", approach);
  expect_same_pose(line.front(), approach, 1e-12);
  expect_same_pose(line.back(), end, 1e-9);
  const Eigen::Vector3d dir = (end.translation() - start.translation()).normalized();
  bool reached_start = false;
  for (size_t i = 1; i < line.size(); ++i) {
    EXPECT_LE((line[i].translation() - line[i - 1].translation()).norm(), step + 1e-9);
    if (reached_start) {
      const Eigen::Vector3d off = line[i].translation() - start.translation();
      EXPECT_LT((off - off.dot(dir) * dir).norm(), 1e-9);
    }
    reached_start |= (line[i].translation() - start.translation()).norm() < 1e-9;
  }
  EXPECT_TRUE(reached_start);

  // Arc through three points on a circle of radius 0.2 m about c.
  const Eigen::Vector3d c(1.0, 0.0, 0.8);
  auto on_circle = [&](double angle) {
      Eigen::Isometry3d p = start;
      p.translation() = c + 0.2 * Eigen::Vector3d(std::cos(angle), std::sin(angle), 0.0);
      return p;
    };
  const std::vector<Eigen::Isometry3d> arc_seam = {on_circle(0.0), on_circle(0.5),
    on_circle(1.0), on_circle(1.5), on_circle(2.0)};
  const auto arc = finder_->pilz_path(arc_seam, "arc", arc_seam.front());
  expect_same_pose(arc.back(), arc_seam.back(), 1e-9);
  for (const auto & p : arc) {
    EXPECT_NEAR((p.translation() - c).norm(), 0.2, 1e-9);
  }
  EXPECT_NEAR(arc[arc.size() / 2].translation().x(), c.x() + 0.2 * std::cos(1.0), 0.01);
}

TEST_F(ConfigurationFinderTest, WalkRejectsNonFiniteStart)
{
  Vector6d q;
  q << 0.1, 0.4, 0.3, 0.2, -1.0, 0.5;
  const std::vector<Eigen::Isometry3d> path = {fk(q)};
  q(3) = std::numeric_limits<double>::quiet_NaN();

  const WalkResult result = finder_->walk(q, path);
  EXPECT_FALSE(result.feasible);
  EXPECT_EQ(result.failure, WalkFailure::kJointLimit) << to_string(result.failure);
}

TEST_F(ConfigurationFinderTest, WalkRejectsEmptyPathAndStartOffPath)
{
  Vector6d q;
  q << 0.1, 0.4, 0.3, 0.2, -1.0, 0.5;
  EXPECT_THROW(finder_->walk(q, {}), std::invalid_argument);

  Eigen::Isometry3d elsewhere = fk(q);
  elsewhere.translation().z() += 0.05;
  EXPECT_THROW(finder_->walk(q, {elsewhere}), std::invalid_argument);
}

TEST_F(ConfigurationFinderTest, ConstructorRejectsInvalidParams)
{
  auto expect_rejected = [this](auto mutate) {
      ConfigurationFinderParams p;
      mutate(p);
      EXPECT_THROW(ConfigurationFinder(kin_, ik_, p), std::invalid_argument);
    };
  expect_rejected([](ConfigurationFinderParams & p) {p.path_step = 0.0;});
  expect_rejected([](ConfigurationFinderParams & p) {p.path_step_rot = -0.01;});
  expect_rejected([](ConfigurationFinderParams & p) {p.max_joint_step = 0.0;});
  expect_rejected([](ConfigurationFinderParams & p) {p.tol_pos = 0.0;});
  expect_rejected([](ConfigurationFinderParams & p) {p.tol_rot = 0.0;});
  expect_rejected([](ConfigurationFinderParams & p) {p.walk_seed_weight = -1.0;});
  expect_rejected([](ConfigurationFinderParams & p) {p.dedupe_epsilon = -1e-3;});
  expect_rejected([](ConfigurationFinderParams & p) {p.manipulability_threshold = -1.0;});
  expect_rejected([](ConfigurationFinderParams & p) {
      p.w_home = std::numeric_limits<double>::quiet_NaN();
    });

  EXPECT_THROW(
    ConfigurationFinder(nullptr, ik_, ConfigurationFinderParams{}), std::invalid_argument);
  EXPECT_THROW(
    ConfigurationFinder(kin_, nullptr, ConfigurationFinderParams{}), std::invalid_argument);
}
