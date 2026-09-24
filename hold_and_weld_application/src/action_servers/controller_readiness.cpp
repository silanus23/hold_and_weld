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

#include "hold_and_weld_application/action_servers/controller_readiness.hpp"

#include <algorithm>
#include <string>
#include <vector>

namespace hold_and_weld
{

std::string controller_name_from_action_topic(const std::string & action_topic)
{
  const auto action_slash = action_topic.find_last_of('/');
  if (action_slash == std::string::npos || action_slash == 0) {
    return "";
  }
  const auto name_slash = action_topic.find_last_of('/', action_slash - 1);
  const auto start = name_slash == std::string::npos ? 0 : name_slash + 1;
  return action_topic.substr(start, action_slash - start);
}

bool is_controller_active(
  const std::vector<controller_manager_msgs::msg::ControllerState> & controllers,
  const std::string & name)
{
  return std::any_of(controllers.begin(), controllers.end(), [&name](const auto & controller) {
             return controller.name == name && controller.state == "active";
           });
}

}  // namespace hold_and_weld
