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

#ifndef HOLD_AND_WELD_APPLICATION__ACTION_SERVERS__GRIPPER_APERTURE_HPP_
#define HOLD_AND_WELD_APPLICATION__ACTION_SERVERS__GRIPPER_APERTURE_HPP_

#include <optional>
#include <string>
#include <vector>

namespace hold_and_weld
{

/**
 * @brief Position bounds of one finger's driving joint, as read from the robot model.
 */
struct FingerJointBounds
{
  std::string joint_name;
  double lower;
  double upper;
};

/**
 * @brief Finger positions commanded for the open and closed gripper states.
 */
struct GripperApertures
{
  double open;
  double close;
};

/**
 * @brief Resolve the open/close finger commands against the gripper's joint limits.
 *
 * Every finger receives the same position command, so the tightest finger bounds
 * both states. Open is @p requested_open when given, otherwise the tightest upper
 * bound; close is @p requested_close when given, otherwise the tightest lower bound.
 *
 * @param fingers Bounds of every finger joint the gripper controller drives.
 * @param requested_open Configured open position [m], or std::nullopt to use the limit.
 * @param requested_close Configured close position [m], or std::nullopt to use the limit.
 * @return The open and close positions to command.
 */
GripperApertures resolve_gripper_apertures(
  const std::vector<FingerJointBounds> & fingers,
  const std::optional<double> & requested_open,
  const std::optional<double> & requested_close);

}  // namespace hold_and_weld

#endif  // HOLD_AND_WELD_APPLICATION__ACTION_SERVERS__GRIPPER_APERTURE_HPP_
