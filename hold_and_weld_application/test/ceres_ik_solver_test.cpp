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

using hold_and_weld_application::kinematics::CeresIKSolver;
using hold_and_weld_application::kinematics::KinematicsSolver;
using hold_and_weld_application::kinematics::URDFParser;
using hold_and_weld_application::kinematics::ParsedChain;

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
  std::vector<double> q_start_vec = {0.1, 0.2, 0.3, 0.1, 0.2, 0.1};

  // Compute its pose
  Eigen::Isometry3d target_pose = fk_solver_->compute_fk(q_start_vec);

  // Convert to Eigen for IK solver
  Eigen::Matrix<double, 6, 1> q_start;
  for (size_t i = 0; i < 6; ++i) {
    q_start[i] = q_start_vec[i];
  }

  // Solve IK with same config as seed (should converge immediately)
  Eigen::Matrix<double, 6, 1> q_solution;
  bool success = ik_solver_->solve(target_pose, q_start, q_solution);

  EXPECT_TRUE(success);

  // Solution should be very close to start
  for (size_t i = 0; i < 6; ++i) {
    EXPECT_NEAR(q_solution[i], q_start_vec[i], 1e-3);
  }
}

TEST_F(CeresIKSolverTest, SolvesWithGoodSeed)
{
  // Target configuration
  std::vector<double> q_target_vec = {0.5, -0.3, 0.4, 0.2, -0.1, 0.3};
  Eigen::Isometry3d target_pose = fk_solver_->compute_fk(q_target_vec);

  // Seed close to target
  Eigen::Matrix<double, 6, 1> q_seed;
  q_seed << 0.52, -0.28, 0.38, 0.22, -0.08, 0.32;

  Eigen::Matrix<double, 6, 1> q_solution;
  bool success = ik_solver_->solve(target_pose, q_seed, q_solution);

  EXPECT_TRUE(success);

  // Verify achieved pose matches target
  std::vector<double> q_solution_vec(6);
  for (size_t i = 0; i < 6; ++i) {
    q_solution_vec[i] = q_solution[i];
  }
  Eigen::Isometry3d achieved_pose = fk_solver_->compute_fk(q_solution_vec);

  Eigen::Vector3d pos_error = achieved_pose.translation() - target_pose.translation();
  EXPECT_LT(pos_error.norm(), 1e-3);  // Less than 1mm position error

  Eigen::Matrix3d R_error = target_pose.rotation() * achieved_pose.rotation().transpose();
  Eigen::AngleAxisd aa(R_error);
  EXPECT_LT(std::abs(aa.angle()), 1e-2);  // Less than ~0.57 degrees orientation error
}

TEST_F(CeresIKSolverTest, SolvesMultipleConsecutivePoses)
{
  // Simulate walking along a Cartesian path with warm starts
  Eigen::Matrix<double, 6, 1> q_current;
  q_current << 0.2, 0.3, 0.4, 0.1, 0.2, 0.1;

  // Generate 10 poses with small increments
  for (int i = 0; i < 10; ++i) {
    // Compute current pose
    std::vector<double> q_current_vec(6);
    for (size_t j = 0; j < 6; ++j) {
      q_current_vec[j] = q_current[j];
    }
    Eigen::Isometry3d current_pose = fk_solver_->compute_fk(q_current_vec);

    // Small translation step (10mm in Z direction)
    Eigen::Isometry3d target_pose = current_pose;
    target_pose.translate(Eigen::Vector3d(0.0, 0.0, 0.01));

    // Solve IK with warm start
    Eigen::Matrix<double, 6, 1> q_solution;
    bool success = ik_solver_->solve(target_pose, q_current, q_solution);

    EXPECT_TRUE(success) << "Failed at iteration " << i;

    // Verify solution
    std::vector<double> q_solution_vec(6);
    for (size_t j = 0; j < 6; ++j) {
      q_solution_vec[j] = q_solution[j];
    }
    Eigen::Isometry3d achieved_pose = fk_solver_->compute_fk(q_solution_vec);
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
  std::vector<double> q_target_vec = {0.3, 0.2, 0.5, 0.1, 0.3, 0.2};
  Eigen::Isometry3d target_pose = fk_solver_->compute_fk(q_target_vec);

  Eigen::Matrix<double, 6, 1> q_seed;
  q_seed << 0.35, 0.25, 0.45, 0.15, 0.25, 0.25;
  Eigen::Matrix<double, 6, 1> q_solution;

  // Tight tolerance
  double tight_tolerance = 1e-4;  // 0.1mm
  bool success = ik_solver_->solve(
    target_pose, q_seed, q_solution,
    tight_tolerance, 1e-2);

  if (success) {
    std::vector<double> q_solution_vec(6);
    for (size_t i = 0; i < 6; ++i) {
      q_solution_vec[i] = q_solution[i];
    }
    Eigen::Isometry3d achieved_pose = fk_solver_->compute_fk(q_solution_vec);
    Eigen::Vector3d pos_error = achieved_pose.translation() - target_pose.translation();
    EXPECT_LT(pos_error.norm(), tight_tolerance);
  }
}

TEST_F(CeresIKSolverTest, RespectsOrientationTolerance)
{
  std::vector<double> q_target_vec = {0.4, -0.2, 0.6, 0.2, -0.1, 0.3};
  Eigen::Isometry3d target_pose = fk_solver_->compute_fk(q_target_vec);

  Eigen::Matrix<double, 6, 1> q_seed;
  q_seed << 0.42, -0.18, 0.58, 0.22, -0.08, 0.32;
  Eigen::Matrix<double, 6, 1> q_solution;

  // Tight orientation tolerance
  double tight_orientation = 1e-3;  // ~0.057 degrees
  bool success = ik_solver_->solve(
    target_pose, q_seed, q_solution,
    1e-3, tight_orientation);

  if (success) {
    std::vector<double> q_solution_vec(6);
    for (size_t i = 0; i < 6; ++i) {
      q_solution_vec[i] = q_solution[i];
    }
    Eigen::Isometry3d achieved_pose = fk_solver_->compute_fk(q_solution_vec);
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
  std::vector<double> q_target_vec = {0.3, 0.4, 0.5, 0.2, 0.3, 0.1};
  Eigen::Isometry3d target_pose = fk_solver_->compute_fk(q_target_vec);

  Eigen::Matrix<double, 6, 1> q_seed;
  q_seed << 0.35, 0.45, 0.45, 0.25, 0.35, 0.15;
  Eigen::Matrix<double, 6, 1> q_solution;

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
  // This test is no longer applicable since we use fixed-size Eigen vectors
  // The compiler enforces correct size at compile time
  SUCCEED() << "Test skipped: Eigen fixed-size vectors enforce correct dimensions at compile time";
}

TEST_F(CeresIKSolverTest, HandlesPoorSeed)
{
  // Target configuration
  std::vector<double> q_target_vec = {0.5, 0.3, 0.4, 0.2, 0.3, 0.1};
  Eigen::Isometry3d target_pose = fk_solver_->compute_fk(q_target_vec);

  // Very poor seed (far from target)
  Eigen::Matrix<double, 6, 1> q_seed;
  q_seed << -0.5, -0.3, -0.4, -0.2, -0.3, -0.1;
  Eigen::Matrix<double, 6, 1> q_solution;

  // May or may not converge, but should not crash
  bool success = ik_solver_->solve(target_pose, q_seed, q_solution);

  // If it converges, verify solution is valid
  if (success) {
    std::vector<double> q_solution_vec(6);
    for (size_t i = 0; i < 6; ++i) {
      q_solution_vec[i] = q_solution[i];
    }
    Eigen::Isometry3d achieved_pose = fk_solver_->compute_fk(q_solution_vec);
    Eigen::Vector3d pos_error = achieved_pose.translation() - target_pose.translation();
    EXPECT_LT(pos_error.norm(), 1e-3);
  }
}

// ============================================================================
// Configuration Tests
// ============================================================================

TEST_F(CeresIKSolverTest, RotationWeightAffectsSolution)
{
  std::vector<double> q_target_vec = {0.4, 0.2, 0.5, 0.1, 0.3, 0.2};
  Eigen::Isometry3d target_pose = fk_solver_->compute_fk(q_target_vec);

  Eigen::Matrix<double, 6, 1> q_seed;
  q_seed << 0.45, 0.25, 0.45, 0.15, 0.25, 0.25;

  // Solve with default rotation weight
  Eigen::Matrix<double, 6, 1> q_solution1;
  ik_solver_->set_rotation_weight(1.0);
  bool success1 = ik_solver_->solve(target_pose, q_seed, q_solution1);

  // Solve with high rotation weight (prioritize orientation)
  Eigen::Matrix<double, 6, 1> q_solution2;
  ik_solver_->set_rotation_weight(10.0);
  bool success2 = ik_solver_->solve(target_pose, q_seed, q_solution2);

  EXPECT_TRUE(success1);
  EXPECT_TRUE(success2);

  // Solutions may differ slightly due to different weighting
  // Just verify both are valid
  if (success1) {
    std::vector<double> q_solution1_vec(6);
    for (size_t i = 0; i < 6; ++i) {
      q_solution1_vec[i] = q_solution1[i];
    }
    Eigen::Isometry3d pose1 = fk_solver_->compute_fk(q_solution1_vec);
    EXPECT_LT((pose1.translation() - target_pose.translation()).norm(), 1e-3);
  }

  if (success2) {
    std::vector<double> q_solution2_vec(6);
    for (size_t i = 0; i < 6; ++i) {
      q_solution2_vec[i] = q_solution2[i];
    }
    Eigen::Isometry3d pose2 = fk_solver_->compute_fk(q_solution2_vec);
    EXPECT_LT((pose2.translation() - target_pose.translation()).norm(), 1e-3);
  }
}

TEST_F(CeresIKSolverTest, MaxIterationsLimitsComputation)
{
  std::vector<double> q_target_vec = {0.5, 0.3, 0.4, 0.2, 0.3, 0.1};
  Eigen::Isometry3d target_pose = fk_solver_->compute_fk(q_target_vec);

  Eigen::Matrix<double, 6, 1> q_seed;
  q_seed << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
  Eigen::Matrix<double, 6, 1> q_solution;

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
  std::vector<double> q_singular_vec = {0.0, -M_PI / 4, M_PI / 2, 0.0, 0.0, 0.0};
  Eigen::Isometry3d target_pose = fk_solver_->compute_fk(q_singular_vec);

  // Small perturbation to move slightly away
  Eigen::Matrix<double, 6, 1> q_seed;
  q_seed << 0.01, -M_PI / 4 + 0.01, M_PI / 2 - 0.01, 0.01, 0.01, 0.01;
  Eigen::Matrix<double, 6, 1> q_solution;

  // IK may fail or converge slowly near singularities - verify it doesn't crash
  [[maybe_unused]] bool success = ik_solver_->solve(target_pose, q_seed, q_solution);

  // Verify manipulability is low at this configuration
  double yoshikawa = fk_solver_->compute_yoshikawa_index(q_singular_vec);
  EXPECT_LT(yoshikawa, 0.1) << "Expected low manipulability near singularity";
}

// ============================================================================
// Performance Test (Informational)
// ============================================================================

TEST_F(CeresIKSolverTest, DISABLED_PerformanceTest)
{
  // This test is disabled by default (use --gtest_also_run_disabled_tests to run)
  // Measures average solve time with warm starts

  Eigen::Matrix<double, 6, 1> q_current;
  q_current << 0.2, 0.3, 0.4, 0.1, 0.2, 0.1;
  int num_solves = 100;
  double total_time = 0.0;
  int successes = 0;

  for (int i = 0; i < num_solves; ++i) {
    std::vector<double> q_current_vec(6);
    for (size_t j = 0; j < 6; ++j) {
      q_current_vec[j] = q_current[j];
    }
    Eigen::Isometry3d current_pose = fk_solver_->compute_fk(q_current_vec);

    // Small step
    Eigen::Isometry3d target_pose = current_pose;
    target_pose.translate(Eigen::Vector3d(0.001, 0.001, 0.005));

    auto start = std::chrono::high_resolution_clock::now();

    Eigen::Matrix<double, 6, 1> q_solution;
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
// Seed Penalty and Configuration Continuity Tests
// ============================================================================

TEST_F(CeresIKSolverTest, SeedPenaltyPreventsConfigurationFlips)
{
  // Test that seed penalty keeps solution near seed configuration
  // Start with a known configuration
  std::vector<double> q_start_vec = {0.3, 0.4, 0.5, 0.2, 0.3, 0.1};
  Eigen::Isometry3d target_pose = fk_solver_->compute_fk(q_start_vec);

  // Use a slightly different seed (small perturbation)
  Eigen::Matrix<double, 6, 1> q_seed;
  q_seed << 0.31, 0.41, 0.51, 0.21, 0.31, 0.11;

  Eigen::Matrix<double, 6, 1> q_solution;
  bool success = ik_solver_->solve(target_pose, q_seed, q_solution);

  ASSERT_TRUE(success);

  // Solution should stay close to seed (not flip to another configuration)
  for (size_t i = 0; i < 6; ++i) {
    EXPECT_NEAR(q_solution[i], q_seed[i], 0.2)
      << "Joint " << i << " deviated too far from seed";
  }

  // Verify pose accuracy is maintained
  std::vector<double> q_solution_vec(6);
  for (size_t i = 0; i < 6; ++i) {
    q_solution_vec[i] = q_solution[i];
  }
  Eigen::Isometry3d achieved_pose = fk_solver_->compute_fk(q_solution_vec);
  Eigen::Vector3d pos_error = achieved_pose.translation() - target_pose.translation();
  EXPECT_LT(pos_error.norm(), 1e-3);
}

TEST_F(CeresIKSolverTest, ConfigurationContinuityAlongTrajectory)
{
  // Test that warm-starting maintains configuration continuity
  // along a trajectory (no sudden joint flips)
  Eigen::Matrix<double, 6, 1> q_current;
  q_current << 0.3, 0.4, 0.5, 0.2, 0.3, 0.1;

  std::vector<Eigen::Matrix<double, 6, 1>> trajectory;
  trajectory.push_back(q_current);

  // Walk along a straight line path
  for (int i = 0; i < 20; ++i) {
    std::vector<double> q_current_vec(6);
    for (size_t j = 0; j < 6; ++j) {
      q_current_vec[j] = q_current[j];
    }
    Eigen::Isometry3d current_pose = fk_solver_->compute_fk(q_current_vec);

    // Small step (5mm)
    Eigen::Isometry3d target_pose = current_pose;
    target_pose.translate(Eigen::Vector3d(0.005, 0.0, 0.0));

    Eigen::Matrix<double, 6, 1> q_solution;
    bool success = ik_solver_->solve(target_pose, q_current, q_solution);

    ASSERT_TRUE(success) << "IK failed at step " << i;

    // Check for configuration continuity (no large jumps)
    for (size_t j = 0; j < 6; ++j) {
      double joint_change = std::abs(q_solution[j] - q_current[j]);
      EXPECT_LT(joint_change, 0.1)
        << "Large joint change at step " << i << ", joint " << j
        << ": " << joint_change << " rad";
    }

    trajectory.push_back(q_solution);
    q_current = q_solution;
  }

  // Verify smooth trajectory (no discontinuities)
  EXPECT_EQ(trajectory.size(), 21);
}

TEST_F(CeresIKSolverTest, ComparesMultipleIKSolutions)
{
  // Test that different seeds lead to different (but valid) IK solutions
  std::vector<double> q_target_vec = {0.5, 0.3, 0.4, 0.2, 0.3, 0.1};
  Eigen::Isometry3d target_pose = fk_solver_->compute_fk(q_target_vec);

  // Seed 1: Close to target
  Eigen::Matrix<double, 6, 1> q_seed1;
  q_seed1 << 0.51, 0.31, 0.41, 0.21, 0.31, 0.11;

  // Seed 2: Different configuration (if reachable)
  Eigen::Matrix<double, 6, 1> q_seed2;
  q_seed2 << 0.45, 0.25, 0.35, 0.15, 0.25, 0.05;

  Eigen::Matrix<double, 6, 1> q_solution1, q_solution2;
  bool success1 = ik_solver_->solve(target_pose, q_seed1, q_solution1);
  bool success2 = ik_solver_->solve(target_pose, q_seed2, q_solution2);

  ASSERT_TRUE(success1);
  ASSERT_TRUE(success2);

  // Both solutions should achieve the target pose
  std::vector<double> q_sol1_vec(6), q_sol2_vec(6);
  for (size_t i = 0; i < 6; ++i) {
    q_sol1_vec[i] = q_solution1[i];
    q_sol2_vec[i] = q_solution2[i];
  }

  Eigen::Isometry3d pose1 = fk_solver_->compute_fk(q_sol1_vec);
  Eigen::Isometry3d pose2 = fk_solver_->compute_fk(q_sol2_vec);

  Eigen::Vector3d pos_error1 = pose1.translation() - target_pose.translation();
  Eigen::Vector3d pos_error2 = pose2.translation() - target_pose.translation();

  EXPECT_LT(pos_error1.norm(), 1e-3);
  EXPECT_LT(pos_error2.norm(), 1e-3);

  // Solutions should stay near their respective seeds
  double dist1_to_seed1 = (q_solution1 - q_seed1).norm();
  double dist2_to_seed2 = (q_solution2 - q_seed2).norm();

  EXPECT_LT(dist1_to_seed1, 0.5);
  EXPECT_LT(dist2_to_seed2, 0.5);
}

TEST_F(CeresIKSolverTest, SeedPenaltyBalancesPoseAccuracy)
{
  // Test that seed penalty doesn't prevent reaching target pose
  std::vector<double> q_target_vec = {0.4, 0.3, 0.5, 0.15, 0.25, 0.2};
  Eigen::Isometry3d target_pose = fk_solver_->compute_fk(q_target_vec);

  // Moderately distant seed
  Eigen::Matrix<double, 6, 1> q_seed;
  q_seed << 0.5, 0.4, 0.4, 0.1, 0.2, 0.15;

  Eigen::Matrix<double, 6, 1> q_solution;
  bool success = ik_solver_->solve(target_pose, q_seed, q_solution);

  ASSERT_TRUE(success);

  // Pose accuracy should still be high despite seed penalty
  std::vector<double> q_solution_vec(6);
  for (size_t i = 0; i < 6; ++i) {
    q_solution_vec[i] = q_solution[i];
  }
  Eigen::Isometry3d achieved_pose = fk_solver_->compute_fk(q_solution_vec);

  Eigen::Vector3d pos_error = achieved_pose.translation() - target_pose.translation();
  EXPECT_LT(pos_error.norm(), 1e-4) << "Position error too large with seed penalty";

  Eigen::Matrix3d R_error = target_pose.rotation() * achieved_pose.rotation().transpose();
  Eigen::AngleAxisd aa(R_error);
  EXPECT_LT(std::abs(aa.angle()), 1e-3) << "Orientation error too large with seed penalty";
}

TEST_F(CeresIKSolverTest, SmallStepsConvergeFast)
{
  // Test that small incremental steps converge quickly (for trajectory validation)
  Eigen::Matrix<double, 6, 1> q_current;
  q_current << 0.3, 0.4, 0.5, 0.2, 0.3, 0.1;

  std::vector<double> q_current_vec(6);
  for (size_t i = 0; i < 6; ++i) {
    q_current_vec[i] = q_current[i];
  }
  Eigen::Isometry3d current_pose = fk_solver_->compute_fk(q_current_vec);

  // Very small step (1mm) - typical for seam validation
  Eigen::Isometry3d target_pose = current_pose;
  target_pose.translate(Eigen::Vector3d(0.001, 0.0, 0.0));

  auto start = std::chrono::high_resolution_clock::now();

  Eigen::Matrix<double, 6, 1> q_solution;
  bool success = ik_solver_->solve(target_pose, q_current, q_solution);

  auto end = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double, std::milli> elapsed = end - start;

  ASSERT_TRUE(success);
  EXPECT_LT(elapsed.count(), 10.0) << "Small step took too long: " << elapsed.count() << "ms";

  // Joint changes should be minimal
  for (size_t i = 0; i < 6; ++i) {
    EXPECT_LT(std::abs(q_solution[i] - q_current[i]), 0.01);
  }
}

TEST_F(CeresIKSolverTest, SeamLikeTrajectoryValidation)
{
  // Simulate approach validation scenario: walking along a seam trajectory
  // Use a well-conditioned starting configuration
  Eigen::Matrix<double, 6, 1> q_approach;
  q_approach << 0.2, 0.3, 0.4, 0.1, 0.2, 0.1;

  const int num_segments = 10;
  const double segment_length = 0.010;  // 10mm steps (realistic for seam validation)

  Eigen::Matrix<double, 6, 1> q_current = q_approach;
  int convergence_count = 0;

  for (int i = 0; i < num_segments; ++i) {
    std::vector<double> q_current_vec(6);
    for (size_t j = 0; j < 6; ++j) {
      q_current_vec[j] = q_current[j];
    }
    Eigen::Isometry3d current_pose = fk_solver_->compute_fk(q_current_vec);

    // Next seam point - move in Z direction (more typical for vertical seams)
    Eigen::Isometry3d target_pose = current_pose;
    target_pose.translate(Eigen::Vector3d(0.0, 0.0, segment_length));

    Eigen::Matrix<double, 6, 1> q_solution;
    bool success = ik_solver_->solve(
      target_pose, q_current, q_solution,
      1e-4,  // 0.1mm tolerance
      1e-3   // ~0.057 deg tolerance
    );

    if (success) {
      convergence_count++;

      // Verify pose accuracy
      std::vector<double> q_solution_vec(6);
      for (size_t j = 0; j < 6; ++j) {
        q_solution_vec[j] = q_solution[j];
      }
      Eigen::Isometry3d achieved_pose = fk_solver_->compute_fk(q_solution_vec);
      Eigen::Vector3d pos_error = achieved_pose.translation() - target_pose.translation();

      EXPECT_LT(pos_error.norm(), 1e-3) << "Position error at segment " << i;

      q_current = q_solution;
    } else {
      // It's okay if some segments fail (might be near limits or singularities)
      break;
    }
  }

  // Should successfully walk through most segments
  // This validates the warm-starting and configuration continuity features
  EXPECT_GE(convergence_count, static_cast<int>(num_segments * 0.7))
    << "Only converged " << convergence_count << " out of " << num_segments << " segments";
}

// ============================================================================
// Main
// ============================================================================

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
