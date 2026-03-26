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

#include "hold_and_weld_gripper_sampler/io/result_writer.hpp"

#include <chrono>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <sstream>

#include <rclcpp/rclcpp.hpp>

namespace hold_and_weld_gripper_sampler
{
namespace io
{

using core::GraspFinderResult;

static const rclcpp::Logger logger_ = rclcpp::get_logger("gripper_sampler");

bool ResultWriter::write_to_file(
  const std::vector<Grasp> & grasps,
  const std::string & output_path,
  const ResultMetadata & metadata,
  const WriterOptions & options)
{
  try {
    std::string json_content = to_json_string(grasps, metadata, options);

    std::ofstream file(output_path);
    if (!file.is_open()) {
      set_error("Failed to open output file: " + output_path);
      return false;
    }

    file << json_content;
    file.close();

    if (file.fail()) {
      set_error("Failed to write to file: " + output_path);
      return false;
    }

    RCLCPP_INFO(logger_, "Wrote %zu grasps to %s", grasps.size(), output_path.c_str());
    return true;
  } catch (const std::exception & e) {
    set_error("Error writing to file '" + output_path + "': " + std::string(e.what()));
    return false;
  } catch (...) {
    set_error("Unknown error writing to file '" + output_path + "'");
    return false;
  }
}

bool ResultWriter::write_to_file(
  const GraspFinderResult & result,
  const std::string & output_path,
  const ResultMetadata & metadata,
  const WriterOptions & options)
{
  // Merge statistics from result into metadata
  ResultMetadata merged_metadata = metadata;
  merged_metadata.num_contact_pairs = result.num_contact_pairs;
  merged_metadata.num_surfaces_valid = result.num_valid_surfaces;
  merged_metadata.num_surfaces_banned = result.num_banned_surfaces;
  merged_metadata.num_candidates = result.num_candidates;
  merged_metadata.num_grasps_output = result.grasps.size();

  return write_to_file(result.grasps, output_path, merged_metadata, options);
}

std::string ResultWriter::to_json_string(
  const std::vector<Grasp> & grasps,
  const ResultMetadata & metadata,
  const WriterOptions & options)
{
  try {
    std::vector<Grasp> filtered_grasps = filter_grasps(grasps, options);

    std::ostringstream json;
    std::string indent = options.pretty_print ? std::string(options.indent_size, ' ') : "";
    std::string newline = options.pretty_print ? "\n" : "";
    std::string sep = options.pretty_print ? " " : "";

    json << std::fixed << std::setprecision(6);

    json << "{" << newline;

  // Metadata section
    if (options.include_metadata) {
      json << indent << "\"metadata\":" << sep << "{" << newline;

      std::string generated_at = metadata.generated_at.empty() ?
        generate_timestamp() : metadata.generated_at;
      json << indent << indent << "\"generated_at\":" << sep << "\"" << generated_at << "\"," <<
        newline;
      json << indent << indent << "\"coordinate_frame\":" << sep << "\"" <<
        metadata.coordinate_frame << "\"," << newline;

      if (!metadata.primary_source.empty()) {
        json << indent << indent << "\"primary_source\":" << sep << "\"" <<
          metadata.primary_source << "\"," << newline;
      }
      if (!metadata.gripper_source.empty()) {
        json << indent << indent << "\"gripper_source\":" << sep << "\"" <<
          metadata.gripper_source << "\"," << newline;
      }
      if (!metadata.config_source.empty()) {
        json << indent << indent << "\"config_source\":" << sep << "\"" <<
          metadata.config_source << "\"," << newline;
      }

    // Statistics
      if (options.include_debug) {
        json << indent << indent << "\"num_surfaces_total\":" << sep <<
          metadata.num_surfaces_total << "," << newline;
        json << indent << indent << "\"num_surfaces_valid\":" << sep <<
          metadata.num_surfaces_valid << "," << newline;
        json << indent << indent << "\"num_surfaces_banned\":" << sep <<
          metadata.num_surfaces_banned << "," << newline;
        json << indent << indent << "\"num_contact_pairs\":" << sep <<
          metadata.num_contact_pairs << "," << newline;
        json << indent << indent << "\"num_candidates\":" << sep <<
          metadata.num_candidates << "," << newline;

        if (metadata.total_time_seconds > 0) {
          json << indent << indent << "\"total_time_seconds\":" << sep <<
            metadata.total_time_seconds << "," << newline;
        }
        if (metadata.sampling_time_seconds > 0) {
          json << indent << indent << "\"sampling_time_seconds\":" << sep <<
            metadata.sampling_time_seconds << "," << newline;
        }
        if (metadata.orientation_time_seconds > 0) {
          json << indent << indent << "\"orientation_time_seconds\":" << sep <<
            metadata.orientation_time_seconds << "," << newline;
        }
      }

      json << indent << indent << "\"num_grasps_output\":" << sep << filtered_grasps.size() <<
        newline;
      json << indent << "}," << newline;
    }

  // Grasps array
    json << indent << "\"grasps\":" << sep << "[" << newline;

    for (size_t i = 0; i < filtered_grasps.size(); ++i) {
      const Grasp & grasp = filtered_grasps[i];

      json << indent << indent << "{" << newline;

    // ID
      json << indent << indent << indent << "\"id\":" << sep << i << "," << newline;

    // Quality score
      json << indent << indent << indent << "\"quality_score\":" << sep <<
        grasp.quality_score << "," << newline;

    // TCP pose
      json << indent << indent << indent << "\"tcp_pose\":" << sep << "{" << newline;
      json << indent << indent << indent << indent << "\"position\":" << sep << "[" <<
        grasp.tcp_position.x() << "," << sep <<
        grasp.tcp_position.y() << "," << sep <<
        grasp.tcp_position.z() << "]," << newline;
      json << indent << indent << indent << indent << "\"quaternion\":" << sep << "[" <<
        grasp.tcp_orientation.x() << "," << sep <<
        grasp.tcp_orientation.y() << "," << sep <<
        grasp.tcp_orientation.z() << "," << sep <<
        grasp.tcp_orientation.w() << "]" << newline;
      json << indent << indent << indent << "}," << newline;

    // Contact 1
      json << indent << indent << indent << "\"contact_1\":" << sep << "{" << newline;
      json << indent << indent << indent << indent << "\"position\":" << sep << "[" <<
        grasp.contact_point_1.x() << "," << sep <<
        grasp.contact_point_1.y() << "," << sep <<
        grasp.contact_point_1.z() << "]," << newline;
      json << indent << indent << indent << indent << "\"surface_id\":" << sep <<
        grasp.surface_id_1 << newline;
      json << indent << indent << indent << "}," << newline;

    // Contact 2
      json << indent << indent << indent << "\"contact_2\":" << sep << "{" << newline;
      json << indent << indent << indent << indent << "\"position\":" << sep << "[" <<
        grasp.contact_point_2.x() << "," << sep <<
        grasp.contact_point_2.y() << "," << sep <<
        grasp.contact_point_2.z() << "]," << newline;
      json << indent << indent << indent << indent << "\"surface_id\":" << sep <<
        grasp.surface_id_2 << newline;
      json << indent << indent << indent << "}," << newline;

    // Gripper opening
      json << indent << indent << indent << "\"gripper_opening\":" << sep <<
        grasp.gripper_opening << newline;

      json << indent << indent << "}";

      if (i < filtered_grasps.size() - 1) {
        json << ",";
      }
      json << newline;
    }

    json << indent << "]" << newline;
    json << "}" << newline;

    return json.str();
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger_, "Error generating JSON: %s", e.what());
    throw std::runtime_error("Failed to generate JSON: " + std::string(e.what()));
  } catch (...) {
    RCLCPP_ERROR(logger_, "Unknown error generating JSON");
    throw std::runtime_error("Failed to generate JSON: unknown error");
  }
}

std::string ResultWriter::to_json_string(
  const GraspFinderResult & result,
  const ResultMetadata & metadata,
  const WriterOptions & options)
{
  // Merge statistics from result into metadata
  ResultMetadata merged_metadata = metadata;
  merged_metadata.num_contact_pairs = result.num_contact_pairs;
  merged_metadata.num_surfaces_valid = result.num_valid_surfaces;
  merged_metadata.num_surfaces_banned = result.num_banned_surfaces;
  merged_metadata.num_candidates = result.num_candidates;
  merged_metadata.num_grasps_output = result.grasps.size();

  return to_json_string(result.grasps, merged_metadata, options);
}

std::string ResultWriter::generate_timestamp()
{
  try {
    auto now = std::chrono::system_clock::now();
    auto time_t_now = std::chrono::system_clock::to_time_t(now);

    std::tm tm_utc;
#ifdef _WIN32
    gmtime_s(&tm_utc, &time_t_now);
#else
    gmtime_r(&time_t_now, &tm_utc);
#endif

    std::ostringstream oss;
    oss << std::put_time(&tm_utc, "%Y-%m-%dT%H:%M:%SZ");
    return oss.str();
  } catch (...) {
    return "unknown";
  }
}

std::vector<Grasp> ResultWriter::filter_grasps(
  const std::vector<Grasp> & grasps,
  const WriterOptions & options) const
{
  std::vector<Grasp> filtered;
  filtered.reserve(grasps.size());

  for (const auto & grasp : grasps) {
    // Filter by quality
    if (grasp.quality_score < options.min_quality) {
      continue;
    }

    filtered.push_back(grasp);

    // Limit count
    if (options.max_grasps > 0 && filtered.size() >= options.max_grasps) {
      break;
    }
  }

  return filtered;
}

void ResultWriter::set_error(const std::string & message)
{
  last_error_ = message;
  RCLCPP_ERROR(logger_, "%s", message.c_str());
}

}  // namespace io
}  // namespace hold_and_weld_gripper_sampler
