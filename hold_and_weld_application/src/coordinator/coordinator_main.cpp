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

#include <thread>

#include <rclcpp/rclcpp.hpp>
#include "hold_and_weld_application/coordinator/dual_robot_coordinator.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<hold_and_weld::DualRobotCoordinator>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);

  std::thread logic_thread([node, &executor]() {
        // Run the sequence
      node->run();
      executor.cancel();
    });

  executor.spin();

  if (logic_thread.joinable()) {
    logic_thread.join();
  }

  rclcpp::shutdown();
  return 0;
}
