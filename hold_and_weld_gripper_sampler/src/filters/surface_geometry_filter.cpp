#include "hold_and_weld_gripper_sampler/filters/surface_filters/surface_geometry_filter.hpp"

#include <cmath>
#include <string>
#include <vector>
#include <BRepAdaptor_Surface.hxx>
#include <BRepGProp.hxx>
#include <BRepTools.hxx>
#include <BRep_Tool.hxx>
#include <GeomLProp_SLProps.hxx>
#include <GProp_GProps.hxx>
#include <rclcpp/rclcpp.hpp>
#include <Standard_Failure.hxx>
#include "hold_and_weld_gripper_sampler/core/surface_filter.hpp"
#include "hold_and_weld_gripper_sampler/geometry/topology.hpp"

namespace hold_and_weld_gripper_sampler
{
namespace filters
{

static const rclcpp::Logger logger_ = rclcpp::get_logger("gripper_sampler");

SurfaceGeometryFilter::SurfaceGeometryFilter(double min_area, double max_mean_curvature)
: min_area_(min_area), max_mean_curvature_(max_mean_curvature)
{
}

std::vector<int> SurfaceGeometryFilter::evaluate(const geometry::Topology & topology) const
{
  const auto & all_surfaces = topology.get_all_surfaces();
  std::vector<int> valid_surface_ids;

  for (size_t i = 0; i < all_surfaces.size(); i++) {
    const geometry::Surface & surface = all_surfaces[i];

    try {
      GProp_GProps props;
      BRepGProp::SurfaceProperties(surface.face, props);
      double area = props.Mass();

      if (area < min_area_) {continue;}

      Handle(Geom_Surface) geom_surface = BRep_Tool::Surface(surface.face);
      if (geom_surface.IsNull()) {continue;}

      double u_min, u_max, v_min, v_max;
      BRepTools::UVBounds(surface.face, u_min, u_max, v_min, v_max);

      double u_mid = (u_min + u_max) / 2.0;
      double v_mid = (v_min + v_max) / 2.0;

      // Note: Curvature is calculated at a single point.
      // For complex NURBS, this may not represent the whole face.
      GeomLProp_SLProps surface_props(geom_surface, u_mid, v_mid, 2, 1e-6);

      if (!surface_props.IsCurvatureDefined()) {continue;}

      double mean_curvature = std::abs(surface_props.MeanCurvature());

      if (mean_curvature <= max_mean_curvature_) {
        valid_surface_ids.push_back(static_cast<int>(i));
      }
    } catch (Standard_Failure & e) {
      RCLCPP_WARN(logger_, "Curvature analysis failed for surface %zu: %s", i,
            e.GetMessageString());
    }
  }

  return valid_surface_ids;
}

std::string SurfaceGeometryFilter::get_name() const
{
  return "SurfaceGeometryFilter";
}

} // namespace filters
} // namespace hold_and_weld_gripper_sampler
