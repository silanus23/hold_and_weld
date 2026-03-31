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

#include "hold_and_weld_gripper_sampler/filters/region_filter/edge_avoidance_filter.hpp"

#include <algorithm>
#include <string>
#include <unordered_set>
#include <vector>

#include <BRepOffsetAPI_MakeOffset.hxx>
#include <GeomAbs_JoinType.hxx>
#include <rclcpp/rclcpp.hpp>
#include <Standard_Failure.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Wire.hxx>
#include <TopExp_Explorer.hxx>

namespace hold_and_weld_gripper_sampler
{
namespace filters
{

static const rclcpp::Logger logger_ = rclcpp::get_logger("gripper_sampler");

EdgeAvoidanceFilter::EdgeAvoidanceFilter(double offset)
: offset_(offset)
{
}

std::vector<core::SampleArea> EdgeAvoidanceFilter::evaluate(
  const TopoDS_Shape & shape,
  const geometry::Topology & topology,
  const std::vector<int> & valid_surface_ids
) const
{
  // Shape unused here; kept for base class signature consistency with future plugin API.
  (void)shape;

  const std::unordered_set<int> valid_set(valid_surface_ids.begin(), valid_surface_ids.end());
  const auto & surfaces = topology.get_all_surfaces();
  std::vector<core::SampleArea> sample_areas;

  for (size_t i = 0; i < surfaces.size(); i++) {
    int surface_id = static_cast<int>(i);

    if (valid_set.find(surface_id) == valid_set.end()) {
      continue;
    }

    try {
      BRepOffsetAPI_MakeOffset offset_tool(surfaces[i].face, GeomAbs_Arc);
      offset_tool.Perform(-offset_);

      if (!offset_tool.IsDone()) {
        RCLCPP_DEBUG(logger_, "Offset failed for surface %d (may have collapsed)", surface_id);
        continue;
      }

      TopoDS_Shape offset_shape = offset_tool.Shape();
      if (offset_shape.IsNull()) {
        RCLCPP_DEBUG(logger_, "Offset produced null shape for surface %d", surface_id);
        continue;
      }

      for (TopExp_Explorer wire_exp(offset_shape, TopAbs_WIRE); wire_exp.More(); wire_exp.Next()) {
        TopoDS_Wire offset_wire = TopoDS::Wire(wire_exp.Current());

        if (offset_wire.IsNull()) {
          continue;
        }

        // Skip empty wires
        TopExp_Explorer edge_check(offset_wire, TopAbs_EDGE);
        if (!edge_check.More()) {
          continue;
        }

        offset_wire.Orientation(TopAbs_FORWARD);

        core::SampleArea sample_area;
        sample_area.surface_id = surface_id;
        sample_area.wire = offset_wire;
        sample_areas.push_back(sample_area);
      }
    } catch (Standard_Failure & e) {
      RCLCPP_WARN(logger_, "OCCT exception for surface %d: %s - skipping",
        surface_id, e.GetMessageString());
    } catch (...) {
      RCLCPP_WARN(logger_, "Edge offset failed for surface %d - skipping", surface_id);
    }
  }

  return sample_areas;
}

std::string EdgeAvoidanceFilter::get_name() const
{
  return "EdgeAvoidanceFilter";
}

}  // namespace filters
}  // namespace hold_and_weld_gripper_sampler
