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

#ifndef HOLD_AND_WELD_GRIPPER_SAMPLER__CORE__REGION_FILTER_HPP_
#define HOLD_AND_WELD_GRIPPER_SAMPLER__CORE__REGION_FILTER_HPP_

#include <string>
#include <vector>

#include <TopoDS_Wire.hxx>

#include "hold_and_weld_gripper_sampler/geometry/topology.hpp"

namespace hold_and_weld_gripper_sampler
{
namespace core
{

/**
 * @brief A wire on one surface that restricts where contact points may be sampled
 *
 * Today every SampleArea is an exclusion wire produced by a constraint
 * (exclusion zone, kissing surface, ground). Inclusion areas from
 * RegionFilter implementations are planned; none exist yet.
 */
struct SampleArea
{
  /** ID of the surface this area lies on. */
  int surface_id;

  /** Closed 3D boundary wire in the world frame, lying on the surface. */
  TopoDS_Wire wire;

  /** True: points inside the wire are rejected. False: points outside it are. */
  bool is_exclusion = false;
};

/**
 * @brief Base class for region filtering operations. Planned; no implementations yet.
 *
 * Region filters take a list of valid surface IDs (from surface filters)
 * and return a list of SampleArea objects defining where on each surface
 * grasps can be generated.
 */
class RegionFilter
{
public:
  RegionFilter() = default;
  virtual ~RegionFilter() = default;

  /**
   * @brief Evaluate surfaces and return valid sampling areas
   *
   * @param shape The primary shape the topology was extracted from
   * @param topology The object topology containing surface geometry
   * @param valid_surface_ids List of surface IDs to process (from surface filters)
   * @return Vector of SampleArea objects defining where grasps can be sampled
   */
  virtual std::vector<SampleArea> evaluate(
    const TopoDS_Shape & shape,
    const geometry::Topology & topology,
    const std::vector<int> & valid_surface_ids
  ) const = 0;

  /**
   * @brief Get human-readable name of the filter
   *
   * @return Filter name string (for logging/debugging)
   */
  virtual std::string get_name() const = 0;
};

}  // namespace core
}  // namespace hold_and_weld_gripper_sampler

#endif  // HOLD_AND_WELD_GRIPPER_SAMPLER__CORE__REGION_FILTER_HPP_
