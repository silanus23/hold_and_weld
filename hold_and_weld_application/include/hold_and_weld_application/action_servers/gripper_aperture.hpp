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

#include <algorithm>
#include <optional>
#include <stdexcept>
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
 * @brief Finger joint positions [m] commanded for the open and closed gripper states.
 */
struct GripperApertures
{
  double open;
  double close;
};

namespace detail
{

/**
 * @brief Replace @p resolved with @p requested if given; throws if it is outside any
 * finger's limits. @p yaml_key names the request in the error message.
 */
inline void apply_requested_position(
  const std::vector<FingerJointBounds> & fingers,
  const std::optional<double> & requested,
  const char * yaml_key,
  double & resolved)
{
  if (!requested) {
    return;
  }
  for (const auto & finger : fingers) {
    // Written as "not inside" so NaN counts as out of limits.
    if (!(finger.lower <= *requested && *requested <= finger.upper)) {
      throw std::invalid_argument(
              std::string(yaml_key) + " " + std::to_string(*requested) + " m is outside joint '" +
              finger.joint_name + "' limits [" + std::to_string(finger.lower) + ", " +
              std::to_string(finger.upper) + "]");
    }
  }
  resolved = *requested;
}

}  // namespace detail

/**
 * @brief Resolve the open/close finger commands against the gripper's joint limits.
 *
 * Every finger receives the same position command, so the tightest finger bounds
 * both states. Open is @p requested_open when given, otherwise the tightest upper
 * bound; close is @p requested_close when given, otherwise the tightest lower bound.
 * Throws std::invalid_argument if a request is NaN or outside any finger's limits,
 * or if the resolved open position is not greater than the close position.
 *
 * @param fingers Bounds of every finger joint the gripper controller drives.
 * @param requested_open Configured open position [m], or std::nullopt to use the limit.
 * @param requested_close Configured close position [m], or std::nullopt to use the limit.
 * @return The open and close positions to command.
 */
inline GripperApertures resolve_gripper_apertures(
  const std::vector<FingerJointBounds> & fingers,
  const std::optional<double> & requested_open,
  const std::optional<double> & requested_close)
{
  if (fingers.empty()) {
    throw std::invalid_argument("No gripper finger joints given");
  }

  GripperApertures apertures{fingers.front().upper, fingers.front().lower};
  for (const auto & finger : fingers) {
    apertures.open = std::min(apertures.open, finger.upper);
    apertures.close = std::max(apertures.close, finger.lower);
  }

  detail::apply_requested_position(fingers, requested_open, "open_position", apertures.open);
  detail::apply_requested_position(fingers, requested_close, "close_position", apertures.close);
  if (!(apertures.open > apertures.close)) {
    throw std::invalid_argument(
            "open position " + std::to_string(apertures.open) +
            " m must be greater than close position " + std::to_string(apertures.close) +
            " m (check open_position/close_position and the finger joint limits)");
  }
  return apertures;
}

}  // namespace hold_and_weld

#endif  // HOLD_AND_WELD_APPLICATION__ACTION_SERVERS__GRIPPER_APERTURE_HPP_
