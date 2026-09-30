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

#include <string>
#include <utility>
#include <vector>

#include "hold_and_weld_application/action_servers/weld_seam_loader.hpp"

using hold_and_weld::parse_weld_seams;

namespace
{
const char kPoses[] =
  R"("poses": [{"position": [1, 0, 1], "quaternion": [0, 0, 0, 1]},
               {"position": [1, 0.1, 1], "quaternion": [0, 0, 0, 1]}])";

std::string line_seam()
{
  return std::string(R"({"segment_type": "line", )") + kPoses + "}";
}

/**
 * {"seams": {<id>: <seam>, ...}} with the seams in the given order.
 */
std::string seams_json(const std::vector<std::pair<std::string, std::string>> & seams)
{
  std::string out = R"({"seams": {)";
  for (size_t i = 0; i < seams.size(); ++i) {
    out += (i ? ", \"" : "\"") + seams[i].first + "\": " + seams[i].second;
  }
  return out + "}}";
}
}  // namespace

// A bad seam is skipped and reported; the rest of the job still loads. An arc needs only
// its poses (CIRC uses the middle one as interim point), not center/radius.
TEST(WeldSeamLoader, SkipsBadSeamAndLoadsTheRest)
{
  const std::string no_poses = R"({"segment_type": "line"})";
  const std::string bare_arc =
    R"({"segment_type": "arc", "poses": [
         {"position": [1, 0, 1], "quaternion": [0, 0, 0, 1]},
         {"position": [1.1, 0.1, 1], "quaternion": [0, 0, 0, 1]},
         {"position": [1, 0.2, 1], "quaternion": [0, 0, 0, 1]}]})";
  const auto parsed = parse_weld_seams(
    seams_json({{"seam_0", line_seam()}, {"seam_1", no_poses}, {"seam_2", bare_arc}}));

  ASSERT_EQ(parsed.seams.size(), 2u);
  EXPECT_EQ(parsed.seams[0].seam_id, "seam_0");
  EXPECT_EQ(parsed.seams[1].seam_id, "seam_2");
  EXPECT_EQ(parsed.skipped, std::vector<std::string>{"seam_1"});
  EXPECT_TRUE(parsed.partial.empty());
}

// A bad pose (here a zero quaternion, which has no orientation) is dropped; the seam keeps
// its good poses and is reported as partial so a shortened weld is not taken as complete.
TEST(WeldSeamLoader, DropsBadPoseAndMarksSeamPartial)
{
  const std::string seam =
    R"({"segment_type": "line", "poses": [
         {"position": [1, 0, 1], "quaternion": [0, 0, 0, 1]},
         {"position": [1, 0.1, 1], "quaternion": [0, 0, 0, 0]},
         {"position": [1, 0.2, 1], "quaternion": [0, 0, 0, 1]}]})";
  const auto parsed = parse_weld_seams(seams_json({{"seam_0", seam}}));

  ASSERT_EQ(parsed.seams.size(), 1u);
  EXPECT_EQ(parsed.seams[0].poses.size(), 2u);
  EXPECT_DOUBLE_EQ(parsed.seams[0].poses.back().position.y, 0.2);
  EXPECT_EQ(parsed.partial, std::vector<std::string>{"seam_0"});
  EXPECT_TRUE(parsed.skipped.empty());
}
