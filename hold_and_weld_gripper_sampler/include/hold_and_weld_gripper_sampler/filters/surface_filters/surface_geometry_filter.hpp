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

#ifndef HOLD_AND_WELD_GRIPPER_SAMPLER__FILTERS__SURFACE_FILTERS__SURFACE_GEOMETRY_FILTER_HPP_
#define HOLD_AND_WELD_GRIPPER_SAMPLER__FILTERS__SURFACE_FILTERS__SURFACE_GEOMETRY_FILTER_HPP_

#include <string>
#include <vector>

#include "hold_and_weld_gripper_sampler/core/surface_filter.hpp"
#include "hold_and_weld_gripper_sampler/geometry/topology.hpp"

namespace hold_and_weld_gripper_sampler
{
namespace filters
{

/**
 * @brief Filters surfaces based on area and curvature
 *
 * Accepts surfaces that:
 * - Have sufficient area (>= min_area)
 * - Are not too curved (mean curvature <= max_mean_curvature)
 */
class SurfaceGeometryFilter : public core::SurfaceFilter
{
public:
  /**
   * @brief Constructor
   *
   * @param min_area Minimum surface area in m^2 (default: 0.001 = 10cm^2)
   * @param max_mean_curvature Maximum mean curvature in 1/m (default: 0.1 = radius >= 10m)
   */
  SurfaceGeometryFilter(
    double min_area = 0.001,
    double max_mean_curvature = 0.1
  );

  std::vector<int> evaluate(const geometry::Topology & topology) const override;

  std::string get_name() const override;

private:
  double min_area_;
  double max_mean_curvature_;
};

}  // namespace filters
}  // namespace hold_and_weld_gripper_sampler

#endif  // HOLD_AND_WELD_GRIPPER_SAMPLER__FILTERS__SURFACE_FILTERS__SURFACE_GEOMETRY_FILTER_HPP_
