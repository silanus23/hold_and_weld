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

#include "hold_and_weld_gripper_sampler/filters/surface_filters/surface_dimension_filter.hpp"

#include <algorithm>
#include <cmath>
#include <Bnd_Box.hxx>
#include <BRepBndLib.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <gp_Ax3.hxx>
#include <gp_Dir.hxx>
#include <gp_Trsf.hxx>
#include <gp_Vec.hxx>
#include <rclcpp/rclcpp.hpp>
#include <Standard_Failure.hxx>
#include <TopoDS_Shape.hxx>

namespace hold_and_weld_gripper_sampler
{
namespace filters
{

static const rclcpp::Logger logger_ = rclcpp::get_logger("gripper_sampler");

SurfaceDimensionFilter::SurfaceDimensionFilter(double min_dimension)
: min_dimension_(min_dimension)
{
}

std::vector<int> SurfaceDimensionFilter::evaluate(const geometry::Topology & topology) const
{
  const auto & all_surfaces = topology.get_all_surfaces();
  std::vector<int> valid_surface_ids;

  for (size_t i = 0; i < all_surfaces.size(); i++) {
    const geometry::Surface & surface = all_surfaces[i];

    try {
      gp_Vec z_axis = surface.normal;

      if (z_axis.Magnitude() < 1e-6) {
        continue;
      }
      z_axis.Normalize();

      gp_Vec ref_vec(0, 0, 1);
      if (std::abs(z_axis.Dot(ref_vec)) > 0.9) {
        ref_vec = gp_Vec(1, 0, 0);
      }

      gp_Vec x_axis = ref_vec.Crossed(z_axis);
      if (x_axis.Magnitude() < 1e-6) {continue;}
      x_axis.Normalize();

      // Build a local frame aligned with the surface normal so the bounding box
      // measures true planar extents rather than world-axis projections.
      gp_Ax3 local_frame(surface.center,
        gp_Dir(z_axis.XYZ()),
        gp_Dir(x_axis.XYZ()));

      gp_Trsf world_to_local;
      world_to_local.SetTransformation(local_frame, gp_Ax3());

      BRepBuilderAPI_Transform transformer(surface.face, world_to_local, Standard_True);
      if (!transformer.IsDone()) {continue;}
      TopoDS_Shape transformed_face = transformer.Shape();

      if (transformed_face.IsNull()) {continue;}

      Bnd_Box bbox;
      BRepBndLib::Add(transformed_face, bbox);

      if (bbox.IsVoid()) {continue;}

      double xmin, ymin, zmin, xmax, ymax, zmax;
      bbox.Get(xmin, ymin, zmin, xmax, ymax, zmax);

      double dim_x = xmax - xmin;
      double dim_y = ymax - ymin;

      if (std::min(dim_x, dim_y) >= min_dimension_) {
        valid_surface_ids.push_back(static_cast<int>(i));
      }
    } catch (const Standard_Failure & e) {
      RCLCPP_WARN(logger_, "Filter failed for surface %zu: %s", i, e.GetMessageString());
    }
  }

  return valid_surface_ids;
}

std::string SurfaceDimensionFilter::get_name() const
{
  return "SurfaceDimensionFilter";
}

}  // namespace filters
}  // namespace hold_and_weld_gripper_sampler
