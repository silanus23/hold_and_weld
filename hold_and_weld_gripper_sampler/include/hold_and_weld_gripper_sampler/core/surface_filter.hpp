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

#ifndef HOLD_AND_WELD_GRIPPER_SAMPLER__CORE__SURFACE_FILTER_HPP_
#define HOLD_AND_WELD_GRIPPER_SAMPLER__CORE__SURFACE_FILTER_HPP_

#include <string>
#include <vector>

#include "hold_and_weld_gripper_sampler/geometry/topology.hpp"

namespace hold_and_weld_gripper_sampler
{
namespace core
{

/**
 * @brief Base class for surface filtering operations.
 *
 * Surface filters evaluate surfaces in a topology and return a list of
 * surface IDs that pass the filter criteria. This is used during pre-filtering
 * to eliminate unsuitable surfaces before grasp generation.
 */
class SurfaceFilter
{
public:
  /**
   * @brief Virtual destructor
   */
  virtual ~SurfaceFilter() = default;

  /**
   * @brief Evaluate topology and return list of valid surface IDs
   *
   * @param topology The object topology to evaluate
   * @return Vector of surface IDs (0-indexed) that pass the filter
   */
  virtual std::vector<int> evaluate(const geometry::Topology & topology) const = 0;

  /**
   * @brief Get human-readable name of the filter
   *
   * @return Filter name string (for logging/debugging)
   */
  virtual std::string get_name() const = 0;
};

}  // namespace core
}  // namespace hold_and_weld_gripper_sampler

#endif  // HOLD_AND_WELD_GRIPPER_SAMPLER__CORE__SURFACE_FILTER_HPP_
