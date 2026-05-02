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

#ifndef HOLD_AND_WELD_GRIPPER_SAMPLER__IO__RESULT_WRITER_HPP_
#define HOLD_AND_WELD_GRIPPER_SAMPLER__IO__RESULT_WRITER_HPP_

#include <string>
#include <vector>

#include "hold_and_weld_gripper_sampler/core/grasp.hpp"
#include "hold_and_weld_gripper_sampler/core/grasp_finder.hpp"

namespace hold_and_weld_gripper_sampler
{
namespace io
{

/**
 * @brief Metadata for grasp result output
 */
struct ResultMetadata
{
  std::string generated_at;
  std::string coordinate_frame = "world";
  std::string primary_source;
  std::string gripper_source;
  std::string config_source;

  // Pipeline statistics
  size_t num_surfaces_total = 0;
  size_t num_surfaces_valid = 0;
  size_t num_surfaces_banned = 0;
  size_t num_contact_pairs = 0;
  size_t num_candidates = 0;
  size_t num_grasps_output = 0;

  // Timing (optional)
  double total_time_seconds = 0.0;
  double sampling_time_seconds = 0.0;
  double orientation_time_seconds = 0.0;

  // Gripper geometry (for visualization)
  double finger_length = 0.0;
};

/**
 * @brief Options for result output
 */
struct WriterOptions
{
  bool pretty_print = true;
  int indent_size = 2;
  bool include_metadata = true;
  size_t max_grasps = 0;
  double min_quality = 0.0;
};

/**
 * @brief Writes grasp results to JSON format
 *
 * Output format matches the trajectory JSON used by magic_wand.py and other
 * visualization tools. Each grasp includes:
 * - TCP pose (position + quaternion)
 * - Contact points (both fingers)
 * - Quality score
 * - Surface IDs
 * - Gripper opening
 *
 * Example output:
 * @code{.json}
 * {
 *   "metadata": {
 *     "generated_at": "2026-01-15T10:30:00Z",
 *     "coordinate_frame": "world",
 *     "primary_source": "meshes/workpiece.step",
 *     "num_grasps_output": 25
 *   },
 *   "grasps": [
 *     {
 *       "id": 0,
 *       "quality_score": 0.85,
 *       "tcp_pose": {
 *         "position": [1.2, -0.5, 0.75],
 *         "quaternion": [0.0, 0.0, 0.0, 1.0]
 *       },
 *       "contact_1": {
 *         "position": [1.18, -0.5, 0.75],
 *         "surface_id": 3
 *       },
 *       "contact_2": {
 *         "position": [1.22, -0.5, 0.75],
 *         "surface_id": 7
 *       },
 *       "gripper_opening": 0.04
 *     }
 *   ]
 * }
 * @endcode
 */
class ResultWriter
{
public:
  ResultWriter() = default;
  ~ResultWriter() = default;

  /**
   * @brief Write grasp results to JSON file
   *
   * @param grasps Vector of grasps to write
   * @param output_path Path to output JSON file
   * @param metadata Optional metadata to include
   * @param options Writer options
   * @return true if write succeeded, false otherwise
   */
  bool write_to_file(
    const std::vector<Grasp> & grasps,
    const std::string & output_path,
    const ResultMetadata & metadata = ResultMetadata{},
    const WriterOptions & options = WriterOptions{});

  /**
   * @brief Write GraspFinderResult to JSON file
   *
   * Convenience method that extracts grasps and statistics from result.
   *
   * @param result GraspFinderResult from GraspFinder::find()
   * @param output_path Path to output JSON file
   * @param metadata Additional metadata (statistics will be merged from result)
   * @param options Writer options
   * @return true if write succeeded, false otherwise
   */
  bool write_to_file(
    const core::GraspFinderResult & result,
    const std::string & output_path,
    const ResultMetadata & metadata = ResultMetadata{},
    const WriterOptions & options = WriterOptions{});

  /**
   * @brief Convert grasps to JSON string
   *
   * @param grasps Vector of grasps to convert
   * @param metadata Optional metadata to include
   * @param options Writer options
   * @return JSON string
   */
  std::string to_json_string(
    const std::vector<Grasp> & grasps,
    const ResultMetadata & metadata = ResultMetadata{},
    const WriterOptions & options = WriterOptions{});

  /**
   * @brief Convert GraspFinderResult to JSON string
   *
   * @param result GraspFinderResult to convert
   * @param metadata Additional metadata
   * @param options Writer options
   * @return JSON string
   */
  std::string to_json_string(
    const core::GraspFinderResult & result,
    const ResultMetadata & metadata = ResultMetadata{},
    const WriterOptions & options = WriterOptions{});

  /**
   * @brief Get last error message
   *
   * @return Error message from last failed operation
   */
  std::string get_last_error() const {return last_error_;}

  /**
   * @brief Generate ISO 8601 timestamp for current time
   *
   * @return Timestamp string (e.g., "2026-01-15T10:30:00Z")
   */
  static std::string generate_timestamp();

private:
  std::string last_error_;

  /**
   * @brief Filter grasps by quality and limit count
   */
  std::vector<Grasp> filter_grasps(
    const std::vector<Grasp> & grasps,
    const WriterOptions & options) const;

  /**
   * @brief Set error message
   */
  void set_error(const std::string & message);
};

}  // namespace io
}  // namespace hold_and_weld_gripper_sampler

#endif  // HOLD_AND_WELD_GRIPPER_SAMPLER__IO__RESULT_WRITER_HPP_
