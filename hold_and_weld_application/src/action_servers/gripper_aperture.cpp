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

#include "hold_and_weld_application/action_servers/gripper_aperture.hpp"

#include <algorithm>
#include <stdexcept>
#include <string>
#include <vector>

namespace hold_and_weld
{

namespace
{

void apply_requested_position(
  const std::vector<FingerJointBounds> & fingers,
  const std::optional<double> & requested,
  const char * yaml_key,
  double & resolved)
{
  if (!requested) {
    return;
  }
  for (const auto & finger : fingers) {
    if (*requested < finger.lower || *requested > finger.upper) {
      throw std::invalid_argument(
              std::string(yaml_key) + " " + std::to_string(*requested) + " m is outside joint '" +
              finger.joint_name + "' limits [" + std::to_string(finger.lower) + ", " +
              std::to_string(finger.upper) + "]");
    }
  }
  resolved = *requested;
}

}  // namespace

GripperApertures resolve_gripper_apertures(
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

  apply_requested_position(fingers, requested_open, "open_position", apertures.open);
  apply_requested_position(fingers, requested_close, "close_position", apertures.close);
  return apertures;
}

}  // namespace hold_and_weld
