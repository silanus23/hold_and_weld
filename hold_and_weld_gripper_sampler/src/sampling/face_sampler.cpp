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

#include "hold_and_weld_gripper_sampler/sampling/face_sampler.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepLib.hxx>
#include <BRepTools.hxx>
#include <BRepTopAdaptor_FClass2d.hxx>
#include <BRep_Tool.hxx>

#include <GCE2d_MakeSegment.hxx>
#include <Geom2d_TrimmedCurve.hxx>
#include <Geom_BSplineSurface.hxx>
#include <Geom_Surface.hxx>
#include <Standard_Failure.hxx>
#include <TopLoc_Location.hxx>
#include <TopoDS.hxx>

namespace hold_and_weld_gripper_sampler
{
namespace sampling
{

namespace
{

const rclcpp::Logger logger_ = rclcpp::get_logger("face_sampler");

constexpr int kMaxKnotSpans = 256;
constexpr int kTileProbeSteps = 3;
constexpr int kMaxRefineDepth = 4;
constexpr double kMinDerivative = 1e-9;

// Distinct knot values strictly inside (lo, hi); empty unless surf is a B-spline.
std::vector<double> interior_knots(
  const Handle(Geom_Surface) & surf, bool along_u, double lo, double hi)
{
  std::vector<double> knots;

  const auto * bspline = dynamic_cast<const Geom_BSplineSurface *>(surf.get());
  // Analytic surfaces carry no knot vector; subdivision alone handles them.
  if (bspline == nullptr) {
    return knots;
  }

  const int count = along_u ? bspline->NbUKnots() : bspline->NbVKnots();
  for (int i = 1; i <= count; ++i) {
    const double knot = along_u ? bspline->UKnot(i) : bspline->VKnot(i);
    if (knot > lo && knot < hi) {
      knots.push_back(knot);
    }
  }
  return knots;
}

// Largest |dS/du| (or |dS/dv|) in a span, probed across the other axis too.
double span_scale(
  const Handle(Geom_Surface) & surf, bool along_u,
  double lo, double hi, double other_min, double other_max)
{
  double peak = 0.0;

  for (int i = 0; i < kTileProbeSteps; ++i) {
    const double s = lo + (hi - lo) * (i + 0.5) / kTileProbeSteps;
    for (int j = 0; j < kTileProbeSteps; ++j) {
      const double o = other_min + (other_max - other_min) * (j + 0.5) / kTileProbeSteps;

      try {
        gp_Pnt p;
        gp_Vec d1u, d1v;
        surf->D1(along_u ? s : o, along_u ? o : s, p, d1u, d1v);
        peak = std::max(peak, (along_u ? d1u : d1v).Magnitude());
      } catch (const Standard_Failure &) {
        continue;
      }
    }
  }

  return (peak > kMinDerivative) ? peak : 1.0;
}

void refine_span(
  const Handle(Geom_Surface) & surf, bool along_u,
  double lo, double hi, double other_min, double other_max,
  double density, int max_cells, int depth,
  std::vector<std::pair<double, int>> * pieces)
{
  const double scale = span_scale(surf, along_u, lo, hi, other_min, other_max);
  const double wanted = (hi - lo) * scale / density;
  const int cells = std::max(1, static_cast<int>(std::ceil(wanted)));

  if (cells > max_cells && depth < kMaxRefineDepth) {
    const int splits = std::min(
      max_cells, static_cast<int>(std::ceil(static_cast<double>(cells) / max_cells)));
    for (int i = 0; i < splits; ++i) {
      refine_span(
        surf, along_u,
        lo + (hi - lo) * i / splits, lo + (hi - lo) * (i + 1) / splits,
        other_min, other_max, density, max_cells, depth + 1, pieces);
    }
    return;
  }

  // Clamped, so hitting the depth ceiling costs fidelity rather than memory.
  pieces->emplace_back(hi, std::min(cells, max_cells));
}

SampleAxis build_axis(
  const Handle(Geom_Surface) & surf, bool along_u,
  double lo, double hi, double other_min, double other_max,
  double density, int max_cells, bool cell_centres)
{
  const std::vector<double> knots = interior_knots(surf, along_u, lo, hi);

  // Thinned by an even stride rather than truncated, so an over-knotted surface
  // keeps boundaries spread across the face instead of only near its start.
  const size_t stride = 1 + knots.size() / kMaxKnotSpans;

  std::vector<double> span_ends;
  for (size_t i = 0; i < knots.size(); i += stride) {
    span_ends.push_back(knots[i]);
  }
  span_ends.push_back(hi);

  std::vector<std::pair<double, int>> pieces;
  double span_start = lo;
  for (const double span_end : span_ends) {
    refine_span(
      surf, along_u, span_start, span_end, other_min, other_max,
      density, max_cells, 0, &pieces);
    span_start = span_end;
  }

  SampleAxis axis;
  double start = lo;
  for (const auto & [end, cells] : pieces) {
    const double width = (end - start) / cells;
    for (int i = 0; i < cells; ++i) {
      axis.coords.push_back(cell_centres ? start + width * (i + 0.5) : start + width * i);
      axis.widths.push_back(width);
    }
    axis.cells += cells;
    axis.max_width = std::max(axis.max_width, width);
    start = end;
  }

  if (!cell_centres && !axis.widths.empty()) {
    // Nodes include the far endpoint; tile interiors are shared, not duplicated.
    axis.coords.push_back(hi);
    axis.widths.push_back(axis.widths.back());
  }

  return axis;
}

// Evenly spaced axis, for when the caller pins the step count itself.
SampleAxis uniform_axis(double lo, double hi, int steps, bool cell_centres)
{
  SampleAxis axis;
  const double width = (hi - lo) / steps;
  const int count = cell_centres ? steps : steps + 1;

  for (int i = 0; i < count; ++i) {
    axis.coords.push_back(cell_centres ? lo + width * (i + 0.5) : lo + width * i);
    axis.widths.push_back(width);
  }
  axis.cells = steps;
  axis.max_width = width;

  return axis;
}

bool passes_wire_restrictions(
  const gp_Pnt2d & point_2d,
  const std::vector<RegionClassifier> & wire_classifiers)
{
  for (const auto & entry : wire_classifiers) {
    const TopAbs_State state = entry.classifier->Perform(point_2d);
    const bool inside_wire = (state == TopAbs_IN || state == TopAbs_ON);

    if (entry.is_exclusion_zone == inside_wire) {
      // Inside a banned region, or outside a required one.
      return false;
    }
  }
  return true;
}

}  // namespace

std::vector<FaceSample> sample_face_region(
  const TopoDS_Face & face,
  const FaceSamplingConfig & config,
  const std::vector<std::pair<TopoDS_Wire, bool>> & wires_with_flags,
  FaceSamplingGrid * grid_out)
{
  std::vector<FaceSample> samples;

  Handle(Geom_Surface) surf = BRep_Tool::Surface(face);
  if (surf.IsNull()) {
    RCLCPP_WARN(logger_, "sample_face_region: face has no surface, skipping");
    return samples;
  }

  Standard_Real u_min, u_max, v_min, v_max;
  BRepTools::UVBounds(face, u_min, u_max, v_min, v_max);

  const double u_range = u_max - u_min;
  const double v_range = v_max - v_min;
  if (u_range <= 0.0 || v_range <= 0.0) {
    RCLCPP_WARN(logger_, "sample_face_region: degenerate UV bounds, skipping");
    return samples;
  }

  const bool cell_centres = (config.layout == GridLayout::kCellCentres);

  SampleAxis u_axis;
  SampleAxis v_axis;
  if (config.grid_steps > 0) {
    u_axis = uniform_axis(u_min, u_max, config.grid_steps, cell_centres);
    v_axis = uniform_axis(v_min, v_max, config.grid_steps, cell_centres);
  } else {
    const double density = std::max(config.sample_density, kMinDerivative);
    const int max_cells = std::max(1, config.max_cells_per_tile);

    u_axis = build_axis(
      surf, true, u_min, u_max, v_min, v_max, density, max_cells, cell_centres);
    v_axis = build_axis(
      surf, false, v_min, v_max, u_min, u_max, density, max_cells, cell_centres);
  }

  if (u_axis.coords.empty() || v_axis.coords.empty()) {
    RCLCPP_WARN(logger_, "sample_face_region: no sample positions produced, skipping");
    return samples;
  }

  if (grid_out != nullptr) {
    grid_out->du = u_axis.max_width;
    grid_out->dv = v_axis.max_width;
    grid_out->u_steps = u_axis.cells;
    grid_out->v_steps = v_axis.cells;
  }

  const int u_count = static_cast<int>(u_axis.coords.size());
  const int v_count = static_cast<int>(v_axis.coords.size());

  std::vector<RegionClassifier> wire_classifiers;
  wire_classifiers.reserve(wires_with_flags.size());
  for (const auto & [wire, is_excl] : wires_with_flags) {
    // An unusable wire rejects the face, for the reason given in the catch below.
    const TopoDS_Face wire_face = face_bounded_by_wire(face, wire);
    if (wire_face.IsNull()) {
      RCLCPP_WARN(logger_,
        "sample_face_region: failed to build face for region wire, "
        "rejecting this face's samples conservatively");
      return samples;
    }

    try {
      RegionClassifier entry;
      entry.classifier =
        std::make_unique<BRepTopAdaptor_FClass2d>(wire_face, config.classifier_tolerance);
      entry.is_exclusion_zone = is_excl;
      wire_classifiers.push_back(std::move(entry));
    } catch (const Standard_Failure & e) {
      // A region wire whose in/out state cannot be determined must not be
      // silently treated as "no restriction" — that would let samples through
      // an exclusion zone or contact area a bad wire happens to shadow. Reject
      // the whole face's samples instead, matching the other structural
      // early-outs above (no surface, degenerate UV bounds).
      RCLCPP_WARN(
        logger_,
        "sample_face_region: region classifier failed to build (%s), "
        "rejecting this face's samples conservatively",
        e.GetMessageString());
      return samples;
    }
  }

  std::unique_ptr<BRepTopAdaptor_FClass2d> face_classifier;
  try {
    face_classifier =
      std::make_unique<BRepTopAdaptor_FClass2d>(face, config.classifier_tolerance);
  } catch (const Standard_Failure & e) {
    RCLCPP_WARN(
      logger_, "sample_face_region: face classifier failed to build (%s), skipping",
      e.GetMessageString());
    return samples;
  }

  const bool reversed = (face.Orientation() == TopAbs_REVERSED);
  samples.reserve(static_cast<size_t>(u_count) * static_cast<size_t>(v_count));

  for (int i = 0; i < u_count; ++i) {
    for (int j = 0; j < v_count; ++j) {
      const double u = u_axis.coords[i];
      const double v = v_axis.coords[j];

      const gp_Pnt2d point_2d(u, v);

      const TopAbs_State face_state = face_classifier->Perform(point_2d);
      if (face_state != TopAbs_IN && face_state != TopAbs_ON) {
        continue;
      }

      if (!wire_classifiers.empty() &&
        !passes_wire_restrictions(point_2d, wire_classifiers))
      {
        continue;
      }

      FaceSample sample;
      sample.uv = point_2d;

      try {
        gp_Vec d1u, d1v;
        surf->D1(u, v, sample.point, d1u, d1v);

        // |dS/du x dS/dv| is the local area scale factor; exact per sample,
        // which is what makes area weighting correct on freeform surfaces.
        const gp_Vec cross = d1u.Crossed(d1v);
        const double jacobian = cross.Magnitude();
        sample.area_weight = jacobian * u_axis.widths[i] * v_axis.widths[j];
        sample.cell_du = u_axis.widths[i];
        sample.cell_dv = v_axis.widths[j];

        if (config.compute_normals && jacobian > kMinDerivative) {
          sample.normal = cross / jacobian;
          if (reversed) {
            sample.normal.Reverse();
          }
          sample.has_normal = true;
        }
      } catch (const Standard_Failure &) {
        // Degenerate parameter (cone apex, seam pole): no derivative, so the
        // sample carries no area and cannot vote.
        continue;
      }

      samples.push_back(sample);
    }
  }

  return samples;
}

double area_fraction(
  const std::vector<FaceSample> & samples,
  const std::function<bool(const FaceSample &)> & predicate)
{
  double total = 0.0;
  double matched = 0.0;

  for (const auto & sample : samples) {
    total += sample.area_weight;
    if (predicate(sample)) {
      matched += sample.area_weight;
    }
  }

  if (total <= kMinDerivative) {
    return 0.0;
  }
  return matched / total;
}

TopoDS_Face face_bounded_by_wire(const TopoDS_Face & face, const TopoDS_Wire & wire)
{
  // The untransformed surface, not BRep_Tool::Surface(face): on a located face
  // that returns a fresh transformed copy per call, and pcurves are looked up
  // by surface handle, so a wire built on one copy has no pcurves on another.
  TopLoc_Location location;
  const Handle(Geom_Surface) & surf = BRep_Tool::Surface(face, location);
  if (surf.IsNull()) {
    return TopoDS_Face();
  }

  // Wires arrive in world coordinates; bring them into the surface's own frame.
  // UV is unchanged by the move, and UV is all the classifier reads.
  const TopoDS_Wire local_wire = TopoDS::Wire(wire.Moved(location.Inverted()));

  try {
    BRepBuilderAPI_MakeFace maker(surf, local_wire, Standard_True);
    if (!maker.IsDone()) {
      return TopoDS_Face();
    }
    return maker.Face();
  } catch (const Standard_Failure & e) {
    RCLCPP_DEBUG(logger_, "face_bounded_by_wire: %s", e.GetMessageString());
    return TopoDS_Face();
  }
}

TopoDS_Wire bounding_wire_in_uv(
  const TopoDS_Face & face,
  const std::vector<FaceSample> & samples)
{
  if (samples.empty()) {
    return TopoDS_Wire();
  }

  // Built on the untransformed surface and moved into place at the end, so the
  // pcurves match what face_bounded_by_wire builds on; see there.
  TopLoc_Location location;
  const Handle(Geom_Surface) & surf = BRep_Tool::Surface(face, location);
  if (surf.IsNull()) {
    return TopoDS_Wire();
  }

  double u_lo = std::numeric_limits<double>::max();
  double u_hi = -std::numeric_limits<double>::max();
  double v_lo = std::numeric_limits<double>::max();
  double v_hi = -std::numeric_limits<double>::max();

  // Each sample stands for its own cell, so the region extends half that
  // cell past it. Per sample rather than the grid's widest cell: adaptive
  // cells can differ by orders of magnitude across one face.
  for (const auto & sample : samples) {
    u_lo = std::min(u_lo, sample.uv.X() - sample.cell_du * 0.5);
    u_hi = std::max(u_hi, sample.uv.X() + sample.cell_du * 0.5);
    v_lo = std::min(v_lo, sample.uv.Y() - sample.cell_dv * 0.5);
    v_hi = std::max(v_hi, sample.uv.Y() + sample.cell_dv * 0.5);
  }

  Standard_Real face_u_min, face_u_max, face_v_min, face_v_max;
  BRepTools::UVBounds(face, face_u_min, face_u_max, face_v_min, face_v_max);

  u_lo = std::max(u_lo, static_cast<double>(face_u_min));
  u_hi = std::min(u_hi, static_cast<double>(face_u_max));
  v_lo = std::max(v_lo, static_cast<double>(face_v_min));
  v_hi = std::min(v_hi, static_cast<double>(face_v_max));

  if (u_hi - u_lo < kMinDerivative || v_hi - v_lo < kMinDerivative) {
    RCLCPP_DEBUG(logger_, "bounding_wire_in_uv: region is degenerate in UV");
    return TopoDS_Wire();
  }

  const gp_Pnt2d corners[4] = {
    gp_Pnt2d(u_lo, v_lo),
    gp_Pnt2d(u_hi, v_lo),
    gp_Pnt2d(u_hi, v_hi),
    gp_Pnt2d(u_lo, v_hi)
  };

  try {
    BRepBuilderAPI_MakeWire wire_builder;
    for (int i = 0; i < 4; ++i) {
      Handle(Geom2d_TrimmedCurve) segment =
        GCE2d_MakeSegment(corners[i], corners[(i + 1) % 4]).Value();
      BRepBuilderAPI_MakeEdge edge_maker(segment, surf);
      if (!edge_maker.IsDone()) {
        RCLCPP_WARN(logger_, "bounding_wire_in_uv: edge %d could not be built", i);
        return TopoDS_Wire();
      }
      wire_builder.Add(edge_maker.Edge());
    }

    if (!wire_builder.IsDone()) {
      RCLCPP_WARN(logger_, "bounding_wire_in_uv: MakeWire failed");
      return TopoDS_Wire();
    }

    TopoDS_Wire wire = wire_builder.Wire();
    // Edges so far carry only pcurves; downstream consumers read 3D geometry.
    BRepLib::BuildCurves3d(wire);
    return TopoDS::Wire(wire.Moved(location));
  } catch (const Standard_Failure & e) {
    RCLCPP_ERROR(logger_, "bounding_wire_in_uv: %s", e.GetMessageString());
    return TopoDS_Wire();
  }
}

double sampled_area(const std::vector<FaceSample> & samples)
{
  double total = 0.0;
  for (const auto & sample : samples) {
    total += sample.area_weight;
  }
  return total;
}

}  // namespace sampling
}  // namespace hold_and_weld_gripper_sampler
