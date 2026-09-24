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

/**
 * @file gripper_catalog_test.cpp
 * @brief Every gripper in hold_and_weld_description's gripper_catalog.xacro must
 * pass GripperParser's finger convention, as mounted on robot1
 */

#include <gtest/gtest.h>
#include <tinyxml2.h>

#include <cstdio>
#include <fstream>
#include <regex>
#include <stdexcept>
#include <string>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include "hold_and_weld_gripper_sampler/io/gripper_parser.hpp"

using hold_and_weld_gripper_sampler::io::GripperParser;

namespace
{

std::string description_dir()
{
  return ament_index_cpp::get_package_share_directory("hold_and_weld_description");
}

/**
 * @brief Model names of the gripper_catalog property, "${['a', 'b']}"
 */
std::vector<std::string> catalog_models()
{
  const std::string path = description_dir() + "/urdf/end_effectors/gripper_catalog.xacro";
  tinyxml2::XMLDocument doc;
  if (doc.LoadFile(path.c_str()) != tinyxml2::XML_SUCCESS) {
    throw std::runtime_error("Cannot load " + path);
  }
  for (auto * prop = doc.RootElement()->FirstChildElement("xacro:property"); prop != nullptr;
    prop = prop->NextSiblingElement("xacro:property"))
  {
    const char * name = prop->Attribute("name");
    if (name && std::string(name) == "gripper_catalog") {
      const std::string value = prop->Attribute("value");
      const std::regex quoted("'([^']+)'");
      std::vector<std::string> models;
      for (auto it = std::sregex_iterator(value.begin(), value.end(), quoted);
        it != std::sregex_iterator(); ++it)
      {
        models.push_back((*it)[1]);
      }
      return models;
    }
  }
  throw std::runtime_error(path + " has no gripper_catalog property");
}

}  // namespace

TEST(GripperCatalogTest, EveryCatalogGripper_PassesTheFingerConvention)
{
  const std::vector<std::string> models = catalog_models();
  ASSERT_FALSE(models.empty());

  const std::string layout_path = ::testing::TempDir() + "gripper_catalog_test_layout.yaml";
  const std::string xacro_path = description_dir() + "/urdf/robot1_gripper.xacro";
  GripperParser parser;

  for (const auto & model : models) {
    SCOPED_TRACE(model);
    {
      std::ofstream layout(layout_path);
      layout << "robots:\n  robot1: {model: gp25, x: 0.0, y: 0.0, z: 0.0, yaw: 0.0}\n"
             << "gripper_model: " << model << "\n";
    }
    try {
      const auto gripper = parser.parse_from_xacro_file(
        xacro_path, "workcell_config:=" + layout_path + " controller_config_file:=");
      EXPECT_GT(gripper.max_opening, 0.0);
    } catch (const std::exception & e) {
      ADD_FAILURE() << e.what();
    }
  }
  std::remove(layout_path.c_str());
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
