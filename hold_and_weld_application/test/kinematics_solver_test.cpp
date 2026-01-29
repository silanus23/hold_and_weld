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

#include <rclcpp/rclcpp.hpp>

#include "hold_and_weld_application/kinematics/kinematics_solver.hpp"
#include "hold_and_weld_application/kinematics/urdf_parser.hpp"

using hold_and_weld::kinematics::KinematicsSolver;
using hold_and_weld::kinematics::URDFParser;

namespace
{
constexpr double GP25_REACH_MIN = 0.5;   // meters - minimum reach from base
constexpr double GP25_REACH_MAX = 2.5;   // meters - maximum reach from base
}  // namespace

class KinematicsSolverTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);

    // Parse robot URDF
    URDFParser parser;
    auto chain = parser.extract_joint_chain(
      "package://hold_and_weld_description/urdf/dual_robot.xacro",
      "robot2_base_link",
      "robot2_wire_tip"
    );

    // Create solver
    solver_ = std::make_unique<KinematicsSolver>(chain);
  }

  void TearDown() override
  {
    rclcpp::shutdown();
  }

  std::unique_ptr<KinematicsSolver> solver_;
};

TEST_F(KinematicsSolverTest, HasCorrectDOF)
{
  EXPECT_EQ(solver_->dof(), 6);
}

TEST_F(KinematicsSolverTest, HasJointLimits)
{
  const auto & limits = solver_->joint_limits();
  EXPECT_EQ(limits.size(), 6);

  // Check all limits are valid (min < max)
  for (size_t i = 0; i < 6; ++i) {
    EXPECT_LT(limits[i].first, limits[i].second)
      << "Joint " << i << " has invalid limits";
  }
}

TEST_F(KinematicsSolverTest, FK_ZeroConfiguration)
{
  std::vector<double> q_zero = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  Eigen::Isometry3d T = solver_->compute_fk(q_zero);

  // At zero config, TCP should be at some reasonable position
  EXPECT_GT(T.translation().norm(), GP25_REACH_MIN);
  EXPECT_LT(T.translation().norm(), GP25_REACH_MAX);

  // Rotation should be valid (determinant = 1)
  double det = T.rotation().determinant();
  EXPECT_NEAR(det, 1.0, 1e-6);
}

TEST_F(KinematicsSolverTest, FK_InvalidJointVectorSize)
{
  std::vector<double> q_invalid = {0.0, 0.0, 0.0};  // Only 3 joints

  EXPECT_THROW(solver_->compute_fk(q_invalid), std::invalid_argument);
}

TEST_F(KinematicsSolverTest, FK_DifferentConfigurations)
{
  std::vector<double> q1 = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  std::vector<double> q2 = {0.5, 0.0, 0.0, 0.0, 0.0, 0.0};
  std::vector<double> q3 = {0.0, 0.5, 0.0, 0.0, 0.0, 0.0};

  Eigen::Isometry3d T1 = solver_->compute_fk(q1);
  Eigen::Isometry3d T2 = solver_->compute_fk(q2);
  Eigen::Isometry3d T3 = solver_->compute_fk(q3);

  // Different joint angles should give different TCP positions
  EXPECT_FALSE(T1.translation().isApprox(T2.translation(), 1e-3));
  EXPECT_FALSE(T1.translation().isApprox(T3.translation(), 1e-3));
  EXPECT_FALSE(T2.translation().isApprox(T3.translation(), 1e-3));
}

TEST_F(KinematicsSolverTest, Jacobian_CorrectDimensions)
{
  std::vector<double> q = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  Eigen::MatrixXd J = solver_->compute_jacobian(q);

  EXPECT_EQ(J.rows(), 6);  // [linear_velocity; angular_velocity]
  EXPECT_EQ(J.cols(), 6);  // 6 DOF
}

TEST_F(KinematicsSolverTest, Jacobian_ChangesWithConfiguration)
{
  std::vector<double> q1 = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  std::vector<double> q2 = {0.5, 0.3, 0.2, 0.0, 0.0, 0.0};

  Eigen::MatrixXd J1 = solver_->compute_jacobian(q1);
  Eigen::MatrixXd J2 = solver_->compute_jacobian(q2);

  // Jacobians should be different for different configurations
  EXPECT_FALSE(J1.isApprox(J2, 1e-3));
}

TEST_F(KinematicsSolverTest, Jacobian_NumericalCheck)
{
  // Verify Jacobian using numerical differentiation
  std::vector<double> q = {0.1, 0.2, 0.3, 0.1, 0.2, 0.1};
  double epsilon = 1e-6;

  Eigen::MatrixXd J_analytical = solver_->compute_jacobian(q);
  Eigen::MatrixXd J_numerical(6, 6);

  Eigen::Isometry3d T0 = solver_->compute_fk(q);

  // Numerical differentiation for each joint
  for (size_t i = 0; i < 6; ++i) {
    std::vector<double> q_plus = q;
    q_plus[i] += epsilon;

    Eigen::Isometry3d T_plus = solver_->compute_fk(q_plus);

    // Linear velocity component (straightforward)
    Eigen::Vector3d dp = (T_plus.translation() - T0.translation()) / epsilon;
    J_numerical.block<3, 1>(0, i) = dp;

    // Angular velocity component (use rotation vector difference)
    Eigen::Matrix3d R_diff = T_plus.rotation() * T0.rotation().transpose();
    Eigen::AngleAxisd aa(R_diff);
    Eigen::Vector3d omega = (aa.angle() / epsilon) * aa.axis();
    J_numerical.block<3, 1>(3, i) = omega;
  }

  // Check if analytical and numerical Jacobians are close
  // Use larger tolerance since numerical differentiation has errors
  EXPECT_TRUE(J_analytical.isApprox(J_numerical, 1e-2))
    << "Analytical Jacobian:\n" << J_analytical
    << "\nNumerical Jacobian:\n" << J_numerical;
}

TEST_F(KinematicsSolverTest, YoshikawaIndex_IsPositive)
{
  std::vector<double> q = {0.1, 0.2, 0.3, 0.1, 0.2, 0.1};

  double yoshikawa = solver_->compute_yoshikawa_index(q);

  EXPECT_GE(yoshikawa, 0.0);
}

TEST_F(KinematicsSolverTest, YoshikawaIndex_ChangesWithConfiguration)
{
  std::vector<double> q1 = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  std::vector<double> q2 = {0.5, 0.3, 0.2, 0.0, 0.0, 0.0};

  double y1 = solver_->compute_yoshikawa_index(q1);
  double y2 = solver_->compute_yoshikawa_index(q2);

  // Different configurations should have different manipulability
  EXPECT_NE(y1, y2);
}

TEST_F(KinematicsSolverTest, ConditionNumber_IsValid)
{
  std::vector<double> q = {0.1, 0.2, 0.3, 0.1, 0.2, 0.1};

  double cond = solver_->compute_condition_number(q);

  // Condition number should be >= 1
  EXPECT_GE(cond, 1.0);

  // Should not be NaN or infinite for valid configuration
  EXPECT_TRUE(std::isfinite(cond));
}

TEST_F(KinematicsSolverTest, SingularityCheck_NonSingularConfiguration)
{
  // A well-conditioned configuration
  std::vector<double> q = {0.2, 0.3, 0.4, 0.1, 0.2, 0.1};

  EXPECT_FALSE(solver_->is_near_singularity(q, 0.01));
}

TEST_F(KinematicsSolverTest, SingularityCheck_CustomThreshold)
{
  std::vector<double> q = {0.1, 0.2, 0.3, 0.1, 0.2, 0.1};

  double yoshikawa = solver_->compute_yoshikawa_index(q);

  // If we set threshold above yoshikawa, it should detect singularity
  EXPECT_TRUE(solver_->is_near_singularity(q, yoshikawa + 0.01));

  // If we set threshold below yoshikawa, it should not
  EXPECT_FALSE(solver_->is_near_singularity(q, yoshikawa - 0.01));
}

TEST_F(KinematicsSolverTest, JointLimits_WithinBounds)
{
  // Configuration well within limits
  std::vector<double> q = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  EXPECT_TRUE(solver_->check_joint_limits(q, 0.05));
}

TEST_F(KinematicsSolverTest, JointLimits_NearBoundary)
{
  const auto & limits = solver_->joint_limits();

  // Set first joint near upper limit
  std::vector<double> q = {
    limits[0].second - 0.03,  // 3 degrees from limit
    0.0, 0.0, 0.0, 0.0, 0.0
  };

  // Should fail with 5 degree margin
  EXPECT_FALSE(solver_->check_joint_limits(q, 0.05));

  // Should pass with 1 degree margin
  EXPECT_TRUE(solver_->check_joint_limits(q, 0.01));
}

TEST_F(KinematicsSolverTest, JointLimits_OutOfBounds)
{
  const auto & limits = solver_->joint_limits();

  // Set first joint beyond limit
  std::vector<double> q = {
    limits[0].second + 0.1,  // Beyond limit
    0.0, 0.0, 0.0, 0.0, 0.0
  };

  EXPECT_FALSE(solver_->check_joint_limits(q, 0.05));
}

TEST_F(KinematicsSolverTest, JointLimits_AllJointsChecked)
{
  const auto & limits = solver_->joint_limits();

  // Test each joint at its limit
  for (size_t i = 0; i < 6; ++i) {
    std::vector<double> q_min = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    std::vector<double> q_max = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    q_min[i] = limits[i].first + 0.03;   // Near lower limit
    q_max[i] = limits[i].second - 0.03;  // Near upper limit

    // Both should fail with large margin
    EXPECT_FALSE(solver_->check_joint_limits(q_min, 0.05))
      << "Joint " << i << " lower limit check failed";
    EXPECT_FALSE(solver_->check_joint_limits(q_max, 0.05))
      << "Joint " << i << " upper limit check failed";
  }
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
