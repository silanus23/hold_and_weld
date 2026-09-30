// Copyright 2025 Berkan Tali
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

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "hold_and_weld_application/action_servers/gripper_action_server.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<hold_and_weld::application::GripperActionServer>();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());

  // Ctrl-C shuts the context down, and only then does spin() return, so stopping the
  // arm after spin() would be too late for stop() to reach the controller. Pre-shutdown
  // callbacks run before the context is invalidated, while both executors still spin.
  std::weak_ptr<hold_and_weld::application::GripperActionServer> weak_node = node;
  rclcpp::contexts::get_global_default_context()->add_pre_shutdown_callback(
    [weak_node]() {
      if (auto locked = weak_node.lock()) {
        locked->manual_shutdown();
      }
    });

  executor.spin();

  // No-op after the pre-shutdown callback; covers spin() returning for another reason.
  node->manual_shutdown();

  rclcpp::shutdown();
  return 0;
}
