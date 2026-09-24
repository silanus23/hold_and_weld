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
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <BRepAdaptor_Curve.hxx>
#include <gp_Ax3.hxx>
#include <gp_Dir.hxx>
#include <gp_Trsf.hxx>
#include <gp_Vec.hxx>
#include <rclcpp/rclcpp.hpp>
#include <Standard_Failure.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>

namespace hold_and_weld_gripper_sampler
{
namespace filters
{

static const rclcpp::Logger logger_ = rclcpp::get_logger("gripper_sampler");

namespace
{

using Point2 = std::pair<double, double>;

double cross(const Point2 & o, const Point2 & a, const Point2 & b)
{
  return (a.first - o.first) * (b.second - o.second) -
         (a.second - o.second) * (b.first - o.first);
}

// Andrew's monotone chain; returns the hull counter-clockwise without repeating the start.
std::vector<Point2> convex_hull(std::vector<Point2> points)
{
  std::sort(points.begin(), points.end());
  points.erase(std::unique(points.begin(), points.end()), points.end());
  if (points.size() < 3) {return points;}

  std::vector<Point2> hull(2 * points.size());
  size_t k = 0;
  for (const auto & p : points) {
    while (k >= 2 && cross(hull[k - 2], hull[k - 1], p) <= 0.0) {--k;}
    hull[k++] = p;
  }
  for (size_t i = points.size() - 1, lower = k + 1; i > 0; --i) {
    const auto & p = points[i - 1];
    while (k >= lower && cross(hull[k - 2], hull[k - 1], p) <= 0.0) {--k;}
    hull[k++] = p;
  }
  hull.resize(k - 1);
  return hull;
}

// Smallest extent of the hull over all in-plane directions. The minimum is reached with
// one side of the enclosing rectangle flush against a hull edge, so only those are tried.
double min_width(const std::vector<Point2> & hull)
{
  if (hull.size() < 3) {return 0.0;}
  double best = std::numeric_limits<double>::max();
  for (size_t i = 0; i < hull.size(); ++i) {
    const Point2 & a = hull[i];
    const Point2 & b = hull[(i + 1) % hull.size()];
    const double edge_len = std::hypot(b.first - a.first, b.second - a.second);
    if (edge_len < 1e-12) {continue;}
    double farthest = 0.0;
    for (const auto & p : hull) {
      farthest = std::max(farthest, std::abs(cross(a, b, p)) / edge_len);
    }
    best = std::min(best, farthest);
  }
  return best;
}

}  // namespace

SurfaceDimensionFilter::SurfaceDimensionFilter(double min_dimension)
: min_dimension_(min_dimension)
{
  if (!(min_dimension_ >= 0.0) || !std::isfinite(min_dimension_)) {
    throw std::invalid_argument("SurfaceDimensionFilter: min_dimension must be finite and >= 0");
  }
}

std::vector<int> SurfaceDimensionFilter::evaluate(const geometry::Topology & topology) const
{
  const auto & all_surfaces = topology.get_all_surfaces();
  std::vector<int> valid_surface_ids;

  // Boundary samples per edge; enough to follow arcs without costing much per face.
  constexpr int kSamplesPerEdge = 32;

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

      // Local frame with z along the surface normal. Its in-plane x axis is arbitrary,
      // so the extent is measured over every in-plane direction (min_width), not along x/y.
      gp_Ax3 local_frame(surface.center,
        gp_Dir(z_axis.XYZ()),
        gp_Dir(x_axis.XYZ()));

      gp_Trsf world_to_local;
      world_to_local.SetTransformation(local_frame, gp_Ax3());

      // Project boundary samples onto the local XY plane.
      std::vector<Point2> projected;
      for (TopExp_Explorer exp(surface.face, TopAbs_EDGE); exp.More(); exp.Next()) {
        BRepAdaptor_Curve curve(TopoDS::Edge(exp.Current()));
        const double t0 = curve.FirstParameter();
        const double t1 = curve.LastParameter();
        for (int s = 0; s <= kSamplesPerEdge; ++s) {
          gp_Pnt p = curve.Value(t0 + (t1 - t0) * s / kSamplesPerEdge);
          p.Transform(world_to_local);
          projected.emplace_back(p.X(), p.Y());
        }
      }
      if (projected.size() < 3) {continue;}

      if (min_width(convex_hull(std::move(projected))) >= min_dimension_) {
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
