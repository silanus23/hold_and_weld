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

#include <Eigen/Dense>

#include <algorithm>
#include <string>
#include <vector>

#include <BRepOffsetAPI_MakeOffset.hxx>
#include <BRep_Tool.hxx>
#include <GeomAbs_JoinType.hxx>
#include <gp_Pnt.hxx>
#include <rclcpp/rclcpp.hpp>
#include <Standard_Failure.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Vertex.hxx>
#include <TopoDS_Wire.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopTools_IndexedMapOfShape.hxx>

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
  const auto & surfaces = topology.get_all_surfaces();
  std::vector<core::SampleArea> sample_areas;

  for (size_t i = 0; i < surfaces.size(); i++) {
    int surface_id = static_cast<int>(i);

    if (std::find(valid_surface_ids.begin(), valid_surface_ids.end(), surface_id) ==
      valid_surface_ids.end())
    {
      continue;
    }

    const geometry::Surface & surface = surfaces[i];

    try {
      // Inward offset creates safe zone away from edges where gripper fingers can safely grip
      BRepOffsetAPI_MakeOffset offset_tool(surface.face, GeomAbs_Arc);
      offset_tool.Perform(-offset_);

      if (!offset_tool.IsDone()) {
        RCLCPP_DEBUG(logger_,
              "Offset operation failed for surface %d (geometry may have collapsed)",
          surface_id);
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
          RCLCPP_DEBUG(logger_, "Null wire in offset result for surface %d", surface_id);
          continue;
        }

        // Validate wire has edges
        TopExp_Explorer edge_check(offset_wire, TopAbs_EDGE);
        if (!edge_check.More()) {
          RCLCPP_DEBUG(logger_, "Empty wire (no edges) for surface %d", surface_id);
          continue;
        }

        offset_wire.Orientation(TopAbs_FORWARD);

        core::SampleArea sample_area;
        sample_area.surface_id = surface_id;
        sample_area.wire = offset_wire;

        sample_areas.push_back(sample_area);
      }
    } catch (const std::exception & e) {
      RCLCPP_DEBUG(logger_, "Edge offset failed for surface %d: %s - skipping",
        surface_id, e.what());
      continue;
    } catch (...) {
      RCLCPP_DEBUG(logger_, "Edge offset failed for surface %d (unknown error) - skipping",
        surface_id);
      continue;
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
