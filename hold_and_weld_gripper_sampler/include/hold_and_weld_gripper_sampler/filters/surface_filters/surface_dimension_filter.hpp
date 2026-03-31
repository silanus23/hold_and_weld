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

#ifndef HOLD_AND_WELD_GRIPPER_SAMPLER__FILTERS__SURFACE_FILTERS__SURFACE_DIMENSION_FILTER_HPP_
#define HOLD_AND_WELD_GRIPPER_SAMPLER__FILTERS__SURFACE_FILTERS__SURFACE_DIMENSION_FILTER_HPP_

#include <string>
#include <vector>

#include "hold_and_weld_gripper_sampler/core/surface_filter.hpp"
#include "hold_and_weld_gripper_sampler/geometry/topology.hpp"

namespace hold_and_weld_gripper_sampler
{
namespace filters
{

/**
 * @brief Filters surfaces based on minimum dimension using oriented bounding box
 *
 * For each surface:
 * 1. Creates local coordinate frame aligned with surface normal
 * 2. Transforms surface to local frame (centered at origin)
 * 3. Computes axis-aligned bounding box in local frame
 * 4. Checks if minimum dimension (width/height/depth) exceeds threshold
 *
 * This approach works for planar and curved surfaces.
 */
class SurfaceDimensionFilter : public core::SurfaceFilter
{
public:
  /**
   * @brief Constructor
   *
   * @param min_dimension Minimum allowed dimension in meters
   */
  explicit SurfaceDimensionFilter(double min_dimension);

  /**
   * @brief Evaluate which surfaces pass the minimum dimension threshold
   *
   * For each surface in the topology, computes the oriented bounding box in
   * the surface's local coordinate frame and checks whether its smallest
   * dimension (width, height, or depth) is at least min_dimension.
   *
   * @param topology Topology containing all surfaces to evaluate
   * @return Vector of surface IDs that satisfy the minimum dimension requirement
   */
  std::vector<int> evaluate(const geometry::Topology & topology) const override;

  /**
   * @brief Get the human-readable name of this filter
   *
   * @return Filter name string
   */
  std::string get_name() const override;

private:
  double min_dimension_;
};

}  // namespace filters
}  // namespace hold_and_weld_gripper_sampler

#endif  // HOLD_AND_WELD_GRIPPER_SAMPLER__FILTERS__SURFACE_FILTERS__SURFACE_DIMENSION_FILTER_HPP_
