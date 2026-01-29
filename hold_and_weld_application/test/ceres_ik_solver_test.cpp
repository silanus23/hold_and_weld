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

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>

#include "hold_and_weld_application/kinematics/ceres_ik_solver.hpp"
#include "hold_and_weld_application/kinematics/kinematics_solver.hpp"
#include "hold_and_weld_application/kinematics/urdf_parser.hpp"

using hold_and_weld::kinematics::CeresIKSolver;
using hold_and_weld::kinematics::KinematicsSolver;
using hold_and_weld::kinematics::URDFParser;
using hold_and_weld::kinematics::ParsedChain;

namespace
{
constexpr double GP25_REACH_MIN = 0.5;    // meters - minimum reach from base
constexpr double GP25_REACH_MAX = 2.5;    // meters - maximum reach from base
constexpr double WIRE_TIP_OFFSET = 0.15;  // meters - typical tool offset
}  // namespace

class CeresIKSolverTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // Initialize ROS 2
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }

    // Load URDF and create solvers
    std::string pkg_share = ament_index_cpp::get_package_share_directory(
      "hold_and_weld_description");
    std::string urdf_path = pkg_share + "/urdf/dual_robot.xacro";

    URDFParser parser;
    ParsedChain chain = parser.extract_joint_chain(
      urdf_path,
      "robot2_base_link",
      "robot2_wire_tip");

    fk_solver_ = std::make_shared<KinematicsSolver>(chain);
    ik_solver_ = std::make_unique<CeresIKSolver>(fk_solver_);
  }

  void TearDown() override
  {
    ik_solver_.reset();
    fk_solver_.reset();
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }

  std::shared_ptr<KinematicsSolver> fk_solver_;
  std::unique_ptr<CeresIKSolver> ik_solver_;
};

// ============================================================================
// Basic Functionality Tests
// ============================================================================

TEST_F(CeresIKSolverTest, SolvesIdentityPose)
{
  // Start from a known configuration
  std::vector<double> q_start = {0.1, 0.2, 0.3, 0.1, 0.2, 0.1};

  // Compute its pose
  Eigen::Isometry3d target_pose = fk_solver_->compute_fk(q_start);

  // Solve IK with same config as seed (should converge immediately)
  std::vector<double> q_solution;
  bool success = ik_solver_->solve(target_pose, q_start, q_solution);

  EXPECT_TRUE(success);

  // Solution should be very close to start
  for (size_t i = 0; i < 6; ++i) {
    EXPECT_NEAR(q_solution[i], q_start[i], 1e-3);
  }
}

TEST_F(CeresIKSolverTest, SolvesWithGoodSeed)
{
  // Target configuration
  std::vector<double> q_target = {0.5, -0.3, 0.4, 0.2, -0.1, 0.3};
  Eigen::Isometry3d target_pose = fk_solver_->compute_fk(q_target);

  // Seed close to target
  std::vector<double> q_seed = {0.52, -0.28, 0.38, 0.22, -0.08, 0.32};

  std::vector<double> q_solution;
  bool success = ik_solver_->solve(target_pose, q_seed, q_solution);

  EXPECT_TRUE(success);

  // Verify achieved pose matches target
  Eigen::Isometry3d achieved_pose = fk_solver_->compute_fk(q_solution);

  Eigen::Vector3d pos_error = achieved_pose.translation() - target_pose.translation();
  EXPECT_LT(pos_error.norm(), 1e-3);  // Less than 1mm position error

  Eigen::Matrix3d R_error = target_pose.rotation() * achieved_pose.rotation().transpose();
  Eigen::AngleAxisd aa(R_error);
  EXPECT_LT(std::abs(aa.angle()), 1e-2);  // Less than ~0.57 degrees orientation error
}

TEST_F(CeresIKSolverTest, SolvesMultipleConsecutivePoses)
{
  // Simulate walking along a Cartesian path with warm starts
  std::vector<double> q_current = {0.2, 0.3, 0.4, 0.1, 0.2, 0.1};

  // Generate 10 poses with small increments
  for (int i = 0; i < 10; ++i) {
    // Compute current pose
    Eigen::Isometry3d current_pose = fk_solver_->compute_fk(q_current);

    // Small translation step (10mm in Z direction)
    Eigen::Isometry3d target_pose = current_pose;
    target_pose.translate(Eigen::Vector3d(0.0, 0.0, 0.01));

    // Solve IK with warm start
    std::vector<double> q_solution;
    bool success = ik_solver_->solve(target_pose, q_current, q_solution);

    EXPECT_TRUE(success) << "Failed at iteration " << i;

    // Verify solution
    Eigen::Isometry3d achieved_pose = fk_solver_->compute_fk(q_solution);
    Eigen::Vector3d pos_error = achieved_pose.translation() - target_pose.translation();
    EXPECT_LT(pos_error.norm(), 1e-3) << "Position error too large at iteration " << i;

    // Use this solution as seed for next iteration
    q_current = q_solution;
  }
}

// ============================================================================
// Convergence and Tolerance Tests
// ============================================================================

TEST_F(CeresIKSolverTest, RespectsPositionTolerance)
{
  std::vector<double> q_target = {0.3, 0.2, 0.5, 0.1, 0.3, 0.2};
  Eigen::Isometry3d target_pose = fk_solver_->compute_fk(q_target);

  std::vector<double> q_seed = {0.35, 0.25, 0.45, 0.15, 0.25, 0.25};
  std::vector<double> q_solution;

  // Tight tolerance
  double tight_tolerance = 1e-4;  // 0.1mm
  bool success = ik_solver_->solve(
    target_pose, q_seed, q_solution,
    tight_tolerance, 1e-2);

  if (success) {
    Eigen::Isometry3d achieved_pose = fk_solver_->compute_fk(q_solution);
    Eigen::Vector3d pos_error = achieved_pose.translation() - target_pose.translation();
    EXPECT_LT(pos_error.norm(), tight_tolerance);
  }
}

TEST_F(CeresIKSolverTest, RespectsOrientationTolerance)
{
  std::vector<double> q_target = {0.4, -0.2, 0.6, 0.2, -0.1, 0.3};
  Eigen::Isometry3d target_pose = fk_solver_->compute_fk(q_target);

  std::vector<double> q_seed = {0.42, -0.18, 0.58, 0.22, -0.08, 0.32};
  std::vector<double> q_solution;

  // Tight orientation tolerance
  double tight_orientation = 1e-3;  // ~0.057 degrees
  bool success = ik_solver_->solve(
    target_pose, q_seed, q_solution,
    1e-3, tight_orientation);

  if (success) {
    Eigen::Isometry3d achieved_pose = fk_solver_->compute_fk(q_solution);
    Eigen::Matrix3d R_error = target_pose.rotation() * achieved_pose.rotation().transpose();
    Eigen::AngleAxisd aa(R_error);
    EXPECT_LT(std::abs(aa.angle()), tight_orientation);
  }
}

// ============================================================================
// Joint Limit Tests
// ============================================================================

TEST_F(CeresIKSolverTest, RespectsJointLimits)
{
  // Target pose that requires joint near limits
  std::vector<double> q_target = {0.3, 0.4, 0.5, 0.2, 0.3, 0.1};
  Eigen::Isometry3d target_pose = fk_solver_->compute_fk(q_target);

  std::vector<double> q_seed = {0.35, 0.45, 0.45, 0.25, 0.35, 0.15};
  std::vector<double> q_solution;

  bool success = ik_solver_->solve(target_pose, q_seed, q_solution);

  if (success) {
    // Verify all joints within limits
    const auto & limits = fk_solver_->joint_limits();
    for (size_t i = 0; i < 6; ++i) {
      EXPECT_GE(q_solution[i], limits[i].first)
        << "Joint " << i << " below lower limit";
      EXPECT_LE(q_solution[i], limits[i].second)
        << "Joint " << i << " above upper limit";
    }
  }
}

// ============================================================================
// Edge Case Tests
// ============================================================================

TEST_F(CeresIKSolverTest, HandlesInvalidSeedSize)
{
  Eigen::Isometry3d target_pose = Eigen::Isometry3d::Identity();
  std::vector<double> q_seed_wrong_size = {0.1, 0.2, 0.3};  // Only 3 joints
  std::vector<double> q_solution;

  bool success = ik_solver_->solve(target_pose, q_seed_wrong_size, q_solution);

  EXPECT_FALSE(success);
}

TEST_F(CeresIKSolverTest, HandlesPoorSeed)
{
  // Target configuration
  std::vector<double> q_target = {0.5, 0.3, 0.4, 0.2, 0.3, 0.1};
  Eigen::Isometry3d target_pose = fk_solver_->compute_fk(q_target);

  // Very poor seed (far from target)
  std::vector<double> q_seed = {-0.5, -0.3, -0.4, -0.2, -0.3, -0.1};
  std::vector<double> q_solution;

  // May or may not converge, but should not crash
  bool success = ik_solver_->solve(target_pose, q_seed, q_solution);

  // If it converges, verify solution is valid
  if (success) {
    Eigen::Isometry3d achieved_pose = fk_solver_->compute_fk(q_solution);
    Eigen::Vector3d pos_error = achieved_pose.translation() - target_pose.translation();
    EXPECT_LT(pos_error.norm(), 1e-3);
  }
}

// ============================================================================
// Configuration Tests
// ============================================================================

TEST_F(CeresIKSolverTest, RotationWeightAffectsSolution)
{
  std::vector<double> q_target = {0.4, 0.2, 0.5, 0.1, 0.3, 0.2};
  Eigen::Isometry3d target_pose = fk_solver_->compute_fk(q_target);

  std::vector<double> q_seed = {0.45, 0.25, 0.45, 0.15, 0.25, 0.25};

  // Solve with default rotation weight
  std::vector<double> q_solution1;
  ik_solver_->set_rotation_weight(1.0);
  bool success1 = ik_solver_->solve(target_pose, q_seed, q_solution1);

  // Solve with high rotation weight (prioritize orientation)
  std::vector<double> q_solution2;
  ik_solver_->set_rotation_weight(10.0);
  bool success2 = ik_solver_->solve(target_pose, q_seed, q_solution2);

  EXPECT_TRUE(success1);
  EXPECT_TRUE(success2);

  // Solutions may differ slightly due to different weighting
  // Just verify both are valid
  if (success1) {
    Eigen::Isometry3d pose1 = fk_solver_->compute_fk(q_solution1);
    EXPECT_LT((pose1.translation() - target_pose.translation()).norm(), 1e-3);
  }

  if (success2) {
    Eigen::Isometry3d pose2 = fk_solver_->compute_fk(q_solution2);
    EXPECT_LT((pose2.translation() - target_pose.translation()).norm(), 1e-3);
  }
}

TEST_F(CeresIKSolverTest, MaxIterationsLimitsComputation)
{
  std::vector<double> q_target = {0.5, 0.3, 0.4, 0.2, 0.3, 0.1};
  Eigen::Isometry3d target_pose = fk_solver_->compute_fk(q_target);

  std::vector<double> q_seed = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  std::vector<double> q_solution;

  // Very few iterations (may not converge)
  ik_solver_->set_max_iterations(5);
  [[maybe_unused]] bool success = ik_solver_->solve(target_pose, q_seed, q_solution);

  // Should either converge or fail gracefully (not crash)
  // No assertion here, just verify it doesn't hang or crash
}

// ============================================================================
// Singularity Tests
// ============================================================================

TEST_F(CeresIKSolverTest, HandlesNearSingularity)
{
  // Configuration near wrist singularity (joints 4, 5, 6 aligned)
  std::vector<double> q_singular = {0.0, -M_PI / 4, M_PI / 2, 0.0, 0.0, 0.0};
  Eigen::Isometry3d target_pose = fk_solver_->compute_fk(q_singular);

  // Small perturbation to move slightly away
  std::vector<double> q_seed = {0.01, -M_PI / 4 + 0.01, M_PI / 2 - 0.01, 0.01, 0.01, 0.01};
  std::vector<double> q_solution;

  // IK may fail or converge slowly near singularities - verify it doesn't crash
  [[maybe_unused]] bool success = ik_solver_->solve(target_pose, q_seed, q_solution);

  // Verify manipulability is low at this configuration
  double yoshikawa = fk_solver_->compute_yoshikawa_index(q_singular);
  EXPECT_LT(yoshikawa, 0.1) << "Expected low manipulability near singularity";
}

// ============================================================================
// Performance Test (Informational)
// ============================================================================

TEST_F(CeresIKSolverTest, DISABLED_PerformanceTest)
{
  // This test is disabled by default (use --gtest_also_run_disabled_tests to run)
  // Measures average solve time with warm starts

  std::vector<double> q_current = {0.2, 0.3, 0.4, 0.1, 0.2, 0.1};
  int num_solves = 100;
  double total_time = 0.0;
  int successes = 0;

  for (int i = 0; i < num_solves; ++i) {
    Eigen::Isometry3d current_pose = fk_solver_->compute_fk(q_current);

    // Small step
    Eigen::Isometry3d target_pose = current_pose;
    target_pose.translate(Eigen::Vector3d(0.001, 0.001, 0.005));

    auto start = std::chrono::high_resolution_clock::now();

    std::vector<double> q_solution;
    bool success = ik_solver_->solve(target_pose, q_current, q_solution);

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> elapsed = end - start;
    total_time += elapsed.count();

    if (success) {
      successes++;
      q_current = q_solution;
    }
  }

  double avg_time = total_time / num_solves;
  double success_rate = 100.0 * successes / num_solves;

  std::cout << "\n=== IK Performance ===" << std::endl;
  std::cout << "Average solve time: " << avg_time << " ms" << std::endl;
  std::cout << "Success rate: " << success_rate << "%" << std::endl;
  std::cout << "Target: < 5ms per solve" << std::endl;

  // Informational - no strict assertion
  EXPECT_LT(avg_time, 10.0) << "IK solving too slow (target < 5ms)";
}

// ============================================================================
// Main
// ============================================================================

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
