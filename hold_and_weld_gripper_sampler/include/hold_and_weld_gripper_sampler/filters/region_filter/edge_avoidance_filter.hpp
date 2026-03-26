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

#ifndef HOLD_AND_WELD_GRIPPER_SAMPLER__FILTERS__REGION_FILTER__EDGE_AVOIDANCE_FILTER_HPP_
#define HOLD_AND_WELD_GRIPPER_SAMPLER__FILTERS__REGION_FILTER__EDGE_AVOIDANCE_FILTER_HPP_

#include <vector>
#include <string>

#include "hold_and_weld_gripper_sampler/core/region_filter.hpp"
#include "hold_and_weld_gripper_sampler/geometry/topology.hpp"

namespace hold_and_weld_gripper_sampler
{
namespace filters
{

/**
 * @brief Creates inset regions by offsetting surface boundaries inward
 *
 * For each valid surface:
 * 1. Builds wire from surface edges
 * 2. Applies negative offset (inward) to create inset boundary
 * 3. Extracts corner positions from inset wire
 * 4. Returns SampleArea with inset region
 *
 * This ensures grasp contact points stay away from sharp edges.
 */
class EdgeAvoidanceFilter : public core::RegionFilter
{
public:
  /**
   * @brief Constructor
   *
   * @param offset Inset distance in meters (positive value moves inward)
   */
  explicit EdgeAvoidanceFilter(double offset);

  std::vector<core::SampleArea> evaluate(
    const TopoDS_Shape & shape,
    const geometry::Topology & topology,
    const std::vector<int> & valid_surface_ids
  ) const override;

  std::string get_name() const override;

private:
  double offset_;
};

}  // namespace filters
}  // namespace hold_and_weld_gripper_sampler

#endif  // HOLD_AND_WELD_GRIPPER_SAMPLER__FILTERS__REGION_FILTER__EDGE_AVOIDANCE_FILTER_HPP_
