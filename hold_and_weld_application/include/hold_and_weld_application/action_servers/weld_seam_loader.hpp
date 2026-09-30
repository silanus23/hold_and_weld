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

#ifndef HOLD_AND_WELD_APPLICATION__ACTION_SERVERS__WELD_SEAM_LOADER_HPP_
#define HOLD_AND_WELD_APPLICATION__ACTION_SERVERS__WELD_SEAM_LOADER_HPP_

#include <Eigen/Geometry>

#include <array>
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "hold_and_weld_application/utils.hpp"

namespace hold_and_weld
{

/**
 * @brief What one weld job welds, as loaded by parse_weld_seams(): the usable seams plus
 * what was left out.
 */
struct WeldJob
{
  std::vector<WeldSeam> seams;
  std::vector<std::string> skipped;
  std::vector<std::string> partial;
  std::vector<std::string> problems;
};

namespace detail
{

// Keys come back sorted as strings, so "seam_10" sorts before "seam_2".
using Json = nlohmann::json;

// Below this norm a quaternion has no usable direction to normalise.
constexpr double kMinQuaternionNorm = 1e-6;

/**
 * @brief @p value as a finite double; throws std::invalid_argument naming @p what otherwise.
 */
inline double finite_number(const Json & value, const std::string & what)
{
  if (!value.is_number()) {
    throw std::invalid_argument(what + " must be a number");
  }
  const double number = value.get<double>();
  if (!std::isfinite(number)) {
    throw std::invalid_argument(what + " must be finite");
  }
  return number;
}

/**
 * @brief @p value as three finite doubles; throws std::invalid_argument otherwise.
 */
inline std::array<double, 3> finite_vector3(const Json & value, const std::string & what)
{
  if (!value.is_array() || value.size() != 3) {
    throw std::invalid_argument(what + " must be an array of 3 numbers");
  }
  return {
    finite_number(value[0], what + "[0]"),
    finite_number(value[1], what + "[1]"),
    finite_number(value[2], what + "[2]")};
}

/**
 * @brief One JSON pose as an end-effector goal; throws std::invalid_argument if unusable.
 */
inline geometry_msgs::msg::Pose json_to_pose(const Json & pose_data, const std::string & what)
{
  if (!pose_data.is_object() || !pose_data.contains("position") ||
    !pose_data.contains("quaternion"))
  {
    throw std::invalid_argument(what + " needs 'position' and 'quaternion'");
  }
  const auto position = finite_vector3(pose_data["position"], what + ".position");

  const Json & quat = pose_data["quaternion"];
  if (!quat.is_array() || quat.size() != 4) {
    throw std::invalid_argument(what + ".quaternion must be an array of 4 numbers");
  }
  // JSON order is (x, y, z, w); Eigen's constructor takes (w, x, y, z).
  Eigen::Quaterniond q(
    finite_number(quat[3], what + ".quaternion[3]"),
    finite_number(quat[0], what + ".quaternion[0]"),
    finite_number(quat[1], what + ".quaternion[1]"),
    finite_number(quat[2], what + ".quaternion[2]"));
  if (q.norm() < kMinQuaternionNorm) {
    throw std::invalid_argument(what + ".quaternion has (near-)zero norm");
  }
  q.normalize();

  // The planner's pose Z points into the part (the torch's working direction), while
  // the torch TCP's +Z points back up the torch (the torch chain in
  // welding_torch_prefix.xacro runs along -Z). Turning 180° about X reverses Z and keeps
  // X, the travel direction. DO NOT MODIFY unless the torch or its mounting changes.
  const Eigen::Quaterniond flip_rotation(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()));
  q = q * flip_rotation;

  geometry_msgs::msg::Pose pose;
  pose.position.x = position[0];
  pose.position.y = position[1];
  pose.position.z = position[2];
  pose.orientation.x = q.x();
  pose.orientation.y = q.y();
  pose.orientation.z = q.z();
  pose.orientation.w = q.w();
  return pose;
}

/**
 * @brief Parse one seam, dropping bad poses and ignoring bad informational fields (both
 * recorded in @p parsed). Throws std::invalid_argument if the seam is unusable.
 */
inline WeldSeam parse_seam(
  const std::string & seam_id, const Json & seam_data, WeldJob & parsed)
{
  if (!seam_data.is_object()) {
    throw std::invalid_argument("must be an object");
  }
  if (!seam_data.contains("poses") || !seam_data["poses"].is_array()) {
    throw std::invalid_argument("'poses' must be an array");
  }

  WeldSeam seam;
  seam.seam_id = seam_id;
  const std::string prefix = "seam '" + seam_id + "': ";

  // segment_type picks the motion (LIN, CIRC or the dense fallback), so a bad one skips
  // the seam; start/end/length_m/center/radius are informational and are only dropped.
  if (seam_data.contains("segment_type")) {
    if (!seam_data["segment_type"].is_string()) {
      throw std::invalid_argument("segment_type must be a string");
    }
    seam.segment_type = seam_data["segment_type"].get<std::string>();
  }
  auto optional_field = [&](const char * key, auto && read) {
      if (!seam_data.contains(key)) {
        return;
      }
      try {
        read(seam_data[key]);
      } catch (const std::exception & e) {
        parsed.problems.push_back(prefix + "ignored " + e.what());
      }
    };
  optional_field("start", [&](const Json & v) {seam.start = finite_vector3(v, "start");});
  optional_field("end", [&](const Json & v) {seam.end = finite_vector3(v, "end");});
  optional_field("length_m", [&](const Json & v) {seam.length_m = finite_number(v, "length_m");});
  if (seam_data.contains("center") && seam_data.contains("radius")) {
    optional_field("radius", [&](const Json & v) {
        const auto center = finite_vector3(seam_data["center"], "center");
        const double radius = finite_number(v, "radius");
        if (!(radius > 0.0)) {
          throw std::invalid_argument("radius (must be positive)");
        }
        seam.center = center;
        seam.radius = radius;
        seam.has_arc_geometry = true;
      });
  }

  const Json & poses = seam_data["poses"];
  seam.poses.reserve(poses.size());
  for (size_t i = 0; i < poses.size(); ++i) {
    try {
      seam.poses.push_back(json_to_pose(poses[i], "poses[" + std::to_string(i) + "]"));
    } catch (const std::exception & e) {
      parsed.problems.push_back(prefix + "dropped " + e.what());
    }
  }
  seam.num_poses = seam.poses.size();

  // A CIRC needs start, interim and end; anything else needs at least a start and an end.
  // Welding an arc with fewer poses as a line would cut across the arc, off the seam.
  const size_t min_poses = seam.segment_type == "arc" ? 3 : 2;
  if (seam.poses.size() < min_poses) {
    throw std::invalid_argument(
            std::to_string(seam.poses.size()) + " usable of " + std::to_string(poses.size()) +
            " poses, needs " + std::to_string(min_poses));
  }
  if (seam.poses.size() < poses.size()) {
    parsed.partial.push_back(seam_id);
  }
  return seam;
}

}  // namespace detail

/**
 * @brief Parse a weld path JSON document (hold_and_weld_planning's weld_planner output).
 *
 * Best effort: a bad pose is dropped and its seam reported in WeldJob::partial;
 * a seam left without enough poses (2, or 3 for an arc) or with a bad segment_type is
 * reported in WeldJob::skipped; bad informational fields are ignored. Every
 * such decision gets a line in WeldJob::problems. Each pose's orientation is
 * normalised and gets the torch's 180° X flip (see WeldSeam::poses). Throws
 * std::invalid_argument only when the document itself is unusable (not JSON, or no
 * 'seams' object).
 *
 * @param json_text Contents of the JSON file.
 * @return Usable seams ordered by seam id (string order), plus what was left out.
 */
inline WeldJob parse_weld_seams(const std::string & json_text)
{
  using detail::Json;
  Json data;
  try {
    data = Json::parse(json_text);
  } catch (const Json::exception & e) {
    throw std::invalid_argument(std::string("weld JSON is not valid JSON: ") + e.what());
  }
  if (!data.is_object() || !data.contains("seams") || !data["seams"].is_object()) {
    throw std::invalid_argument("weld JSON needs a 'seams' object");
  }

  WeldJob parsed;
  parsed.seams.reserve(data["seams"].size());
  for (const auto & [seam_id, seam_data] : data["seams"].items()) {
    try {
      parsed.seams.push_back(detail::parse_seam(seam_id, seam_data, parsed));
    } catch (const std::exception & e) {
      parsed.skipped.push_back(seam_id);
      parsed.problems.push_back("seam '" + seam_id + "': skipped, " + e.what());
    }
  }
  return parsed;
}

}  // namespace hold_and_weld

#endif  // HOLD_AND_WELD_APPLICATION__ACTION_SERVERS__WELD_SEAM_LOADER_HPP_
