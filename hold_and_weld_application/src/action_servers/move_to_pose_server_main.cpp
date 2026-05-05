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

#include "hold_and_weld_application/action_servers/move_to_pose_action_server.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto move_to_pose_server = std::make_shared<hold_and_weld::application::MoveToPoseActionServer>();

  rclcpp::spin(move_to_pose_server);

  // manual_shutdown() must be called while the ROS context is still valid:
  // it calls stop() on all cached move groups so the controller can process
  // the cancel before the context is torn down by rclcpp::shutdown().
  move_to_pose_server->manual_shutdown();

  rclcpp::shutdown();
  return 0;
}
