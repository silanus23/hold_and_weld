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

#ifndef GP25_URDF_HPP_
#define GP25_URDF_HPP_

#include <unistd.h>

#include <cstdlib>
#include <filesystem>
#include <stdexcept>
#include <string>

#include <ament_index_cpp/get_package_share_directory.hpp>

/**
 * @brief Path to dual_robot.xacro expanded with the GP25 pair of fixtures/gp25_workcell.yaml.
 *
 * Expanded once per test process.
 *
 * @return Path to the generated URDF file
 */
inline std::string gp25_dual_robot_urdf()
{
  static const std::string urdf_path = [] {
      const std::string xacro_path =
        ament_index_cpp::get_package_share_directory("hold_and_weld_description") +
        "/urdf/dual_robot.xacro";
      const std::string path = (std::filesystem::temp_directory_path() /
        ("hold_and_weld_gp25_" + std::to_string(::getpid()) + ".urdf")).string();
      const std::string command = "xacro '" + xacro_path + "' workcell_config:='" +
        TEST_FIXTURES_DIR + "/gp25_workcell.yaml' -o '" + path + "'";
      if (std::system(command.c_str()) != 0) {
        throw std::runtime_error("xacro failed: " + command);
      }
      return path;
    }();
  return urdf_path;
}

#endif  // GP25_URDF_HPP_
