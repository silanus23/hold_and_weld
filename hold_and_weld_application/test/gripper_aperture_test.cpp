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

#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "hold_and_weld_application/action_servers/gripper_aperture.hpp"

using hold_and_weld::FingerJointBounds;
using hold_and_weld::resolve_gripper_apertures;

namespace
{
/// Finger limits of a 2F-140 stand-in: 0.140 m stroke split across two fingers.
std::vector<FingerJointBounds> robotiq_2f140_fingers()
{
  return {
    {"robot1_left_finger_joint", 0.0, 0.07},
    {"robot1_right_finger_joint", 0.0, 0.07}};
}
}  // namespace

// A gripper swap must not need a code change: with no open_position configured,
// "open" is whatever the URDF says the fingers can reach.
TEST(GripperAperture, DefaultsToUrdfLimitsWhenNotConfigured)
{
  const auto apertures =
    resolve_gripper_apertures(robotiq_2f140_fingers(), std::nullopt, std::nullopt);
  EXPECT_DOUBLE_EQ(apertures.open, 0.07);
  EXPECT_DOUBLE_EQ(apertures.close, 0.0);
}

TEST(GripperAperture, UsesConfiguredOpenPositionWithinLimits)
{
  const auto apertures = resolve_gripper_apertures(robotiq_2f140_fingers(), 0.05, std::nullopt);
  EXPECT_DOUBLE_EQ(apertures.open, 0.05);
}

// The old hardcoded 0.15 against a 0.07-stroke finger: the controller would be
// commanded past the joint limit. This must be caught before any motion.
TEST(GripperAperture, RejectsOpenPositionBeyondUpperLimit)
{
  try {
    resolve_gripper_apertures(robotiq_2f140_fingers(), 0.15, std::nullopt);
    FAIL() << "0.15 m exceeds the 0.07 m finger limit and must be rejected";
  } catch (const std::invalid_argument & e) {
    EXPECT_NE(std::string(e.what()).find("robot1_left_finger_joint"), std::string::npos)
      << "error should name the offending joint, got: " << e.what();
  }
}

TEST(GripperAperture, RejectsOpenPositionBelowLowerLimit)
{
  EXPECT_THROW(
    resolve_gripper_apertures(robotiq_2f140_fingers(), -0.01, std::nullopt),
    std::invalid_argument);
}

// A configured close_position lets the fingers stop on the object (partial grip)
// instead of always driving to the joint's fully-closed lower bound.
TEST(GripperAperture, UsesConfiguredClosePositionWithinLimits)
{
  const auto apertures =
    resolve_gripper_apertures(robotiq_2f140_fingers(), std::nullopt, 0.03);
  EXPECT_DOUBLE_EQ(apertures.close, 0.03);
  EXPECT_DOUBLE_EQ(apertures.open, 0.07);
}

TEST(GripperAperture, RejectsClosePositionBeyondUpperLimit)
{
  try {
    resolve_gripper_apertures(robotiq_2f140_fingers(), std::nullopt, 0.15);
    FAIL() << "0.15 m exceeds the 0.07 m finger limit and must be rejected";
  } catch (const std::invalid_argument & e) {
    EXPECT_NE(std::string(e.what()).find("robot1_left_finger_joint"), std::string::npos)
      << "error should name the offending joint, got: " << e.what();
  }
}

TEST(GripperAperture, RejectsClosePositionBelowLowerLimit)
{
  EXPECT_THROW(
    resolve_gripper_apertures(robotiq_2f140_fingers(), std::nullopt, -0.01),
    std::invalid_argument);
}

// Both fingers get the same command, so the tightest finger bounds the aperture.
TEST(GripperAperture, AsymmetricLimitsUseTightestFinger)
{
  const std::vector<FingerJointBounds> fingers = {
    {"left", 0.0, 0.10},
    {"right", 0.01, 0.08}};
  const auto apertures = resolve_gripper_apertures(fingers, std::nullopt, std::nullopt);
  EXPECT_DOUBLE_EQ(apertures.open, 0.08);
  EXPECT_DOUBLE_EQ(apertures.close, 0.01);
  EXPECT_THROW(
    resolve_gripper_apertures(fingers, 0.09, std::nullopt), std::invalid_argument);
  EXPECT_THROW(
    resolve_gripper_apertures(fingers, std::nullopt, 0.005), std::invalid_argument);
}

TEST(GripperAperture, RejectsEmptyFingerList)
{
  EXPECT_THROW(
    resolve_gripper_apertures({}, std::nullopt, std::nullopt), std::invalid_argument);
}
