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

#include "hold_and_weld_gripper_sampler/geometry/shape_refiner.hpp"

#include <algorithm>
#include <cmath>
#include <deque>
#include <stdexcept>

#include <BRepAdaptor_Curve.hxx>
#include <BRep_Tool.hxx>
#include <Geom2d_Curve.hxx>
#include <GeomAdaptor_Curve.hxx>
#include <gp_Pnt2d.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepAlgoAPI_Defeaturing.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepFeat_SplitShape.hxx>
#include <BRepGProp.hxx>
#include <BRepTools.hxx>
#include <GCPnts_AbscissaPoint.hxx>
#include <GeomLProp_SLProps.hxx>
#include <Precision.hxx>
#include <GProp_GProps.hxx>
#include <rclcpp/rclcpp.hpp>
#include <ShapeFix_Shape.hxx>
#include <ShapeUpgrade_ShapeDivideClosed.hxx>
#include <ShapeUpgrade_UnifySameDomain.hxx>
#include <Standard_Failure.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <TopoDS.hxx>

// TODO(@silanus23): Add a splitter that splits based on sudden normal trend changes.
// TODO(@silanus23): Make heuristic approches adaptive instead strict sample based

namespace hold_and_weld_gripper_sampler
{
namespace geometry
{

static const rclcpp::Logger logger_ = rclcpp::get_logger("gripper_sampler");

namespace
{

// Adds one iso-parameter split edge, or logs and skips it (UIso/VIso/MakeEdge
// can fail without throwing, e.g. at a pole) so one bad candidate doesn't abort
// the whole face's split pass.
bool add_iso_split_edge(
  BRepFeat_SplitShape & splitter,
  const Handle(Geom_Surface) & surf,
  bool along_u,
  double param,
  double other_min,
  double other_max,
  const TopoDS_Face & face)
{
  try {
    Handle(Geom_Curve) iso = along_u ? surf->UIso(param) : surf->VIso(param);
    if (iso.IsNull()) {
      RCLCPP_WARN(logger_, "%sIso(%.6f) returned a null curve - skipping this split",
        along_u ? "U" : "V", param);
      return false;
    }

    BRepBuilderAPI_MakeEdge edge_maker(iso, other_min, other_max);
    if (!edge_maker.IsDone()) {
      RCLCPP_WARN(logger_, "MakeEdge failed for %sIso(%.6f) - skipping this split",
        along_u ? "U" : "V", param);
      return false;
    }

    splitter.Add(edge_maker.Edge(), face);
    return true;
  } catch (const Standard_Failure & e) {
    RCLCPP_WARN(logger_, "%sIso(%.6f) split failed: %s - skipping this split",
      along_u ? "U" : "V", param, e.GetMessageString());
    return false;
  }
}

// A face on a periodic surface reports IsUPeriodic() even when it only covers part of
// the period (a half-pipe, a fillet), so the face's own parameter span decides closure.
bool covers_full_u_period(const BRepAdaptor_Surface & surface)
{
  return surface.IsUPeriodic() &&
         (surface.LastUParameter() - surface.FirstUParameter()) >=
         surface.UPeriod() - Precision::PConfusion() * 1000.0;
}

bool covers_full_v_period(const BRepAdaptor_Surface & surface)
{
  return surface.IsVPeriodic() &&
         (surface.LastVParameter() - surface.FirstVParameter()) >=
         surface.VPeriod() - Precision::PConfusion() * 1000.0;
}

// Split parameters for one face, collected before any splitting is done.
struct FaceSplits
{
  TopoDS_Face face;
  std::vector<double> u_splits;
  std::vector<double> v_splits;
};

// Adds every iso edge of one face to the splitter. Returns true if at least one was added.
bool add_face_splits(BRepFeat_SplitShape & splitter, const FaceSplits & fs)
{
  // BRep_Tool::Surface applies the face location, so the edges land where the face is.
  Handle(Geom_Surface) surf = BRep_Tool::Surface(fs.face);
  if (surf.IsNull()) {return false;}
  BRepAdaptor_Surface adaptor(fs.face);
  const double u_min = adaptor.FirstUParameter();
  const double u_max = adaptor.LastUParameter();
  const double v_min = adaptor.FirstVParameter();
  const double v_max = adaptor.LastVParameter();

  bool added = false;
  for (double u : fs.u_splits) {
    added |= add_iso_split_edge(splitter, surf, true, u, v_min, v_max, fs.face);
  }
  for (double v : fs.v_splits) {
    added |= add_iso_split_edge(splitter, surf, false, v, u_min, u_max, fs.face);
  }
  return added;
}

// Returns the split shape, or a null shape if nothing was added or the split did not build.
TopoDS_Shape try_split(const TopoDS_Shape & shape, const std::vector<const FaceSplits *> & plan)
{
  try {
    BRepFeat_SplitShape splitter(shape);
    bool added = false;
    for (const FaceSplits * fs : plan) {
      added |= add_face_splits(splitter, *fs);
    }
    if (!added) {return TopoDS_Shape();}
    splitter.Build();
    return splitter.IsDone() ? splitter.Shape() : TopoDS_Shape();
  } catch (const Standard_Failure & e) {
    RCLCPP_DEBUG(logger_, "BRepFeat_SplitShape failed: %s", e.GetMessageString());
    return TopoDS_Shape();
  }
}

// Splits all faces in one BRepFeat_SplitShape pass. If that pass fails, retries face by
// face so one bad split edge only costs its own face instead of every face's splits.
TopoDS_Shape apply_face_splits(
  const TopoDS_Shape & shape, const std::vector<FaceSplits> & plan, const char * phase)
{
  if (plan.empty()) {return shape;}

  std::vector<const FaceSplits *> all;
  for (const auto & fs : plan) {all.push_back(&fs);}
  TopoDS_Shape combined = try_split(shape, all);
  if (!combined.IsNull()) {return combined;}

  RCLCPP_WARN(logger_, "%s: combined split failed - retrying face by face", phase);
  TopoDS_Shape result = shape;
  size_t skipped = 0;
  for (const auto & fs : plan) {
    // Faces untouched by earlier splits keep their identity, so this finds them.
    TopTools_IndexedMapOfShape current_faces;
    TopExp::MapShapes(result, TopAbs_FACE, current_faces);
    if (!current_faces.Contains(fs.face)) {
      ++skipped;
      continue;
    }
    TopoDS_Shape split = try_split(result, {&fs});
    if (split.IsNull()) {
      ++skipped;
    } else {
      result = split;
    }
  }
  if (skipped > 0) {
    RCLCPP_WARN(logger_, "%s: %zu of %zu face(s) could not be split", phase, skipped, plan.size());
  }
  return result;
}

}  // namespace

ShapeRefiner::ShapeRefiner(
  double max_cylinder_radius,
  double max_arc_length,
  double enclave_area_ratio,
  double enclave_angle_threshold,
  double max_face_area_ratio,
  double planarity_tolerance_deg)
: max_cylinder_radius_(max_cylinder_radius),
  max_arc_length_(max_arc_length),
  enclave_area_ratio_(enclave_area_ratio),
  enclave_angle_threshold_(enclave_angle_threshold),
  max_face_area_ratio_(max_face_area_ratio),
  planarity_tolerance_deg_(planarity_tolerance_deg)
{
  // Negated comparisons so NaN is rejected too. max_arc_length divides split counts,
  // so 0 or infinity would turn into an undefined int conversion.
  if (!(max_arc_length_ > 0.0) || !std::isfinite(max_arc_length_)) {
    throw std::invalid_argument("ShapeRefiner: max_arc_length must be finite and > 0");
  }
  if (!(enclave_area_ratio_ >= 0.0 && enclave_area_ratio_ <= 1.0)) {
    throw std::invalid_argument("ShapeRefiner: enclave_area_ratio must be in [0, 1]");
  }
  if (!(enclave_angle_threshold_ >= 0.0 && enclave_angle_threshold_ <= 90.0)) {
    throw std::invalid_argument("ShapeRefiner: enclave_angle_threshold must be in [0, 90]");
  }
  if (!(max_face_area_ratio_ > 0.0 && max_face_area_ratio_ <= 1.0)) {
    throw std::invalid_argument("ShapeRefiner: max_face_area_ratio must be in (0, 1]");
  }
  if (!(planarity_tolerance_deg_ > 0.0 && planarity_tolerance_deg_ < 90.0)) {
    throw std::invalid_argument("ShapeRefiner: planarity_tolerance_deg must be in (0, 90)");
  }
}

TopoDS_Shape ShapeRefiner::refine(const TopoDS_Shape & raw_shape) const
{
  try {
    ShapeFix_Shape healer(raw_shape);
    healer.Perform();
    TopoDS_Shape current_shape = healer.Shape();

    if (current_shape.IsNull()) {
      return raw_shape;
    }

    GProp_GProps global_props;
    BRepGProp::SurfaceProperties(current_shape, global_props);
    double global_total_area = global_props.Mass();
    // Every later ratio divides by this; a zero-area shape has nothing to refine.
    if (!(global_total_area > 0.0)) {
      RCLCPP_WARN(logger_, "Shape has no surface area - skipping refinement");
      return raw_shape;
    }

    // TODO(@silanus23): Add enable_enclave_removal bool config parameter
    TopTools_ListOfShape faces_to_remove;
    identify_enclave_features(current_shape, global_total_area, faces_to_remove);

    if (!faces_to_remove.IsEmpty()) {
      try {
        BRepAlgoAPI_Defeaturing eraser;
        eraser.SetShape(current_shape);
        eraser.AddFacesToRemove(faces_to_remove);
        eraser.Build();
        if (!eraser.IsDone()) {
          RCLCPP_WARN(logger_, "Defeaturing IsDone() false - skipping enclave removal");
        } else {
          current_shape = eraser.Shape();
          ShapeUpgrade_UnifySameDomain mid_healer(current_shape, true, true);
          mid_healer.Build();
          current_shape = mid_healer.Shape();
        }
      } catch (const Standard_Failure & e) {
        RCLCPP_WARN(logger_, "Defeaturing failed: %s - continuing without enclave removal",
          e.GetMessageString());
      } catch (const std::exception & e) {
        RCLCPP_WARN(logger_, "Defeaturing exception: %s - continuing without enclave removal",
          e.what());
      }
    }

    TopoDS_Shape s = refine_phase1_periodic_split(current_shape);
    s = refine_phase2_arc_length_split(s, global_total_area);
    s = refine_phase3_area_ratio_split(s, global_total_area);
    return refine_phase4_unify(s);
  } catch (const Standard_Failure & e) {
    RCLCPP_ERROR(logger_, "OCCT error in shape refinement: %s - returning original shape",
      e.GetMessageString());
    return raw_shape;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger_, "Exception in shape refinement: %s - returning original shape",
      e.what());
    return raw_shape;
  }
}

TopoDS_Shape ShapeRefiner::refine_phase1_periodic_split(const TopoDS_Shape & shape) const
{
  // Pre-split closed faces: BRepFeat_SplitShape silently fails on them because it
  // cannot insert ISO edges without an existing seam. ShapeUpgrade_ShapeDivideClosed opens it.
  int oversized_closed_faces = 0;
  for (TopExp_Explorer periodic_exp(shape, TopAbs_FACE); periodic_exp.More();
    periodic_exp.Next())
  {
    const TopoDS_Face & face = TopoDS::Face(periodic_exp.Current());
    BRepAdaptor_Surface adaptor(face);
    if (!covers_full_u_period(adaptor)) {continue;}

    // Estimate full U-period arc length:
    // exact formula for analytics, the V = mid isocurve (which runs along U) for BSpline/Bezier.
    double u_arc_length = 0.0;
    const GeomAbs_SurfaceType stype = adaptor.GetType();
    if (stype == GeomAbs_Cylinder) {
      u_arc_length = 2.0 * M_PI * adaptor.Cylinder().Radius();
    } else if (stype == GeomAbs_Cone) {
      // Approximate via the reference radius of the cone
      u_arc_length = 2.0 * M_PI * adaptor.Cone().RefRadius();
    } else if (stype == GeomAbs_Sphere) {
      u_arc_length = 2.0 * M_PI * adaptor.Sphere().Radius();
    } else if (stype == GeomAbs_Torus) {
      u_arc_length = 2.0 * M_PI * (adaptor.Torus().MajorRadius() +
        adaptor.Torus().MinorRadius());
    } else {
      // BSpline, Bezier or other — sample the mid-V isocurve length as an estimate
      try {
        Handle(Geom_Surface) surf = BRep_Tool::Surface(face);
        Handle(Geom_Curve) iso = surf.IsNull() ? Handle(Geom_Curve)() : surf->VIso(
          (adaptor.FirstVParameter() + adaptor.LastVParameter()) / 2.0);
        if (!iso.IsNull()) {
          GeomAdaptor_Curve iso_adaptor(iso,
            adaptor.FirstUParameter(), adaptor.LastUParameter());
          u_arc_length = GCPnts_AbscissaPoint::Length(iso_adaptor);
        }
      } catch (const Standard_Failure & e) {
        RCLCPP_DEBUG(logger_,
          "VIso arc length estimation failed: %s - face deferred to area ratio pass",
          e.GetMessageString());
      }
    }

    if (u_arc_length > max_arc_length_) {
      ++oversized_closed_faces;
    }
  }

  TopoDS_Shape result = shape;
  if (oversized_closed_faces > 0) {
    RCLCPP_INFO(logger_,
      "%d closed face(s) exceed arc length limit - "
      "opening seams before surface splitter.",
      oversized_closed_faces);
    try {
      // One split point halves every closed face. The count applies to the whole shape,
      // so a larger value would over-split small holes; phase 2 splits each half further
      // by its own arc length.
      ShapeUpgrade_ShapeDivideClosed divider(shape);
      divider.SetNbSplitPoints(1);
      divider.Perform();
      TopoDS_Shape divided = divider.Result();
      if (!divided.IsNull()) {
        result = divided;
      }
    } catch (const Standard_Failure & e) {
      RCLCPP_WARN(logger_,
        "ShapeDivideClosed failed: %s - continuing without pre-split",
        e.GetMessageString());
    } catch (const std::exception & e) {
      RCLCPP_WARN(logger_, "ShapeDivideClosed exception: %s - continuing without pre-split",
        e.what());
    }
  }
  RCLCPP_DEBUG(logger_, "Phase 1 (periodic pre-split) complete: %d oversized closed face(s)",
    oversized_closed_faces);
  return result;
}

TopoDS_Shape ShapeRefiner::refine_phase2_arc_length_split(
  const TopoDS_Shape & shape, double global_total_area) const
{
  (void)global_total_area;  // reserved for future diagnostics

  std::vector<FaceSplits> plan;

  TopExp_Explorer face_exp(shape, TopAbs_FACE);
  for (; face_exp.More(); face_exp.Next()) {
    const TopoDS_Face & face = TopoDS::Face(face_exp.Current());

    if (is_physically_planar(face)) {continue;}

    BRepAdaptor_Surface adaptor(face);
    GeomAbs_SurfaceType type = adaptor.GetType();

    // Skip faces still closed after phase 1 — BRepFeat_SplitShape cannot split them.
    // Partial faces on periodic surfaces (half-pipes, fillets) are split here.
    if (covers_full_u_period(adaptor) || covers_full_v_period(adaptor)) {continue;}
    std::vector<double> u_splits, v_splits;

    // For freeform surfaces, detect inflection points by sampling curvature
    if (type == GeomAbs_BSplineSurface || type == GeomAbs_BezierSurface) {
      find_inflections(adaptor, true, u_splits);
      find_inflections(adaptor, false, v_splits);
    }

    // Always check boundary arc lengths — handles all surface types including
    // distorted/exported cylinders and edgeless closed surfaces
    std::vector<double> edge_u_splits, edge_v_splits;
    check_edge_arc_lengths(face, edge_u_splits, edge_v_splits);
    u_splits.insert(u_splits.end(), edge_u_splits.begin(), edge_u_splits.end());
    v_splits.insert(v_splits.end(), edge_v_splits.begin(), edge_v_splits.end());

    if (u_splits.empty() && v_splits.empty()) {continue;}
    plan.push_back({face, std::move(u_splits), std::move(v_splits)});
  }

  TopoDS_Shape result = apply_face_splits(shape, plan, "Phase 2 surface split");
  RCLCPP_DEBUG(logger_, "Phase 2 (arc length + inflection split) complete");
  return result;
}

TopoDS_Shape ShapeRefiner::refine_phase3_area_ratio_split(
  const TopoDS_Shape & shape, double global_total_area) const
{
  std::vector<FaceSplits> plan;

  TopExp_Explorer final_exp(shape, TopAbs_FACE);
  for (; final_exp.More(); final_exp.Next()) {
    const TopoDS_Face & face = TopoDS::Face(final_exp.Current());

    GProp_GProps face_props;
    try {
      BRepGProp::SurfaceProperties(face, face_props);
    } catch (const Standard_Failure &) {
      continue;
    }

    double area_ratio = face_props.Mass() / global_total_area;
    if (area_ratio <= max_face_area_ratio_) {continue;}

    BRepAdaptor_Surface adaptor(face);
    const char * type_str = "Other";
    switch (adaptor.GetType()) {
      case GeomAbs_Plane:          type_str = "Plane"; break;
      case GeomAbs_Cylinder:       type_str = "Cylinder"; break;
      case GeomAbs_Cone:           type_str = "Cone"; break;
      case GeomAbs_Sphere:         type_str = "Sphere"; break;
      case GeomAbs_Torus:          type_str = "Torus"; break;
      case GeomAbs_BSplineSurface: type_str = "BSpline"; break;
      case GeomAbs_BezierSurface:  type_str = "Bezier"; break;
      default: break;
    }

    RCLCPP_WARN(logger_,
      "Face still exceeds area ratio (%.1f%% > %.1f%%) after splitting - "
      "forcing edge-based split. Surface type: %s",
      area_ratio * 100.0, max_face_area_ratio_ * 100.0, type_str);

    std::vector<double> u_splits, v_splits;
    check_edge_arc_lengths(face, u_splits, v_splits);
    if (u_splits.empty() && v_splits.empty()) {continue;}
    plan.push_back({face, std::move(u_splits), std::move(v_splits)});
  }

  TopoDS_Shape result = apply_face_splits(shape, plan, "Phase 3 area ratio split");
  RCLCPP_DEBUG(logger_, "Phase 3 (area ratio split) complete");
  return result;
}

TopoDS_Shape ShapeRefiner::refine_phase4_unify(const TopoDS_Shape & shape) const
{
  // true/false = unify edges but NOT faces — merging faces would undo the splits above.
  ShapeUpgrade_UnifySameDomain final_unifier(shape, true, false);
  final_unifier.Build();
  RCLCPP_DEBUG(logger_, "Phase 4 (edge unification) complete");
  return final_unifier.Shape();
}

void ShapeRefiner::identify_enclave_features(
  const TopoDS_Shape & shape,
  double global_total_area,
  TopTools_ListOfShape & kill_list) const
{
  TopTools_IndexedDataMapOfShapeListOfShape edge_to_faces;
  TopExp::MapShapesAndAncestors(shape, TopAbs_EDGE, TopAbs_FACE, edge_to_faces);

  // Tracks faces already added to kill_list —
  // prevents double-adding faces shared by multiple enclaves.
  TopTools_MapOfShape processed_faces;

  TopExp_Explorer face_exp(shape, TopAbs_FACE);
  for (; face_exp.More(); face_exp.Next()) {
    const TopoDS_Face & parent_face = TopoDS::Face(face_exp.Current());
    if (processed_faces.Contains(parent_face)) {continue;}

    TopoDS_Wire outer_wire = BRepTools::OuterWire(parent_face);

    TopExp_Explorer wire_exp(parent_face, TopAbs_WIRE);
    for (; wire_exp.More(); wire_exp.Next()) {
      const TopoDS_Wire & wire = TopoDS::Wire(wire_exp.Current());

      // Skip the outer wire — only process inner wires (holes)
      if (!outer_wire.IsNull() && wire.IsSame(outer_wire)) {
        continue;
      }

      TopTools_ListOfShape enclave_faces;
      // BFS from inner wire boundary
      collect_enclave_faces(wire, parent_face, edge_to_faces, enclave_faces);

      if (should_suppress_enclave(parent_face, enclave_faces, global_total_area)) {
        for (TopTools_ListIteratorOfListOfShape it(enclave_faces); it.More(); it.Next()) {
          kill_list.Append(it.Value());
          processed_faces.Add(it.Value());
        }
      }
    }
  }
}

void ShapeRefiner::collect_enclave_faces(
  const TopoDS_Wire & footprint,
  const TopoDS_Face & parent_face,
  const TopTools_IndexedDataMapOfShapeListOfShape & edge_map,
  TopTools_ListOfShape & enclave_faces) const
{
  TopTools_MapOfShape visited;
  std::deque<TopoDS_Face> hop_queue;

  for (TopExp_Explorer exp(footprint, TopAbs_EDGE); exp.More(); exp.Next()) {
    const TopoDS_Shape & edge = exp.Current();
    if (edge_map.Contains(edge)) {
      const TopTools_ListOfShape & neighbors = edge_map.FindFromKey(edge);
      for (TopTools_ListIteratorOfListOfShape it(neighbors); it.More(); it.Next()) {
        const TopoDS_Face & face = TopoDS::Face(it.Value());
        if (!face.IsSame(parent_face) && visited.Add(face)) {
          hop_queue.push_back(face);
          enclave_faces.Append(face);
        }
      }
    }
  }

  while (!hop_queue.empty()) {
    TopoDS_Face current = hop_queue.front();
    hop_queue.pop_front();

    for (TopExp_Explorer exp(current, TopAbs_EDGE); exp.More(); exp.Next()) {
      const TopoDS_Shape & edge = exp.Current();
      if (edge_map.Contains(edge)) {
        const TopTools_ListOfShape & neighbors = edge_map.FindFromKey(edge);
        for (TopTools_ListIteratorOfListOfShape it(neighbors); it.More(); it.Next()) {
          const TopoDS_Face & face = TopoDS::Face(it.Value());
          if (!face.IsSame(parent_face) && visited.Add(face)) {
            hop_queue.push_back(face);
            enclave_faces.Append(face);
          }
        }
      }
    }
  }
}

bool ShapeRefiner::should_suppress_enclave(
  const TopoDS_Face & parent_face,
  const TopTools_ListOfShape & enclave_faces,
  double global_total_area) const
{
  if (enclave_faces.IsEmpty()) {return false;}

  double enclave_area = 0.0;
  GProp_GProps area_props;
  for (TopTools_ListIteratorOfListOfShape it(enclave_faces); it.More(); it.Next()) {
    try {
      BRepGProp::SurfaceProperties(TopoDS::Face(it.Value()), area_props);
      enclave_area += area_props.Mass();
    } catch (const Standard_Failure & e) {
      RCLCPP_DEBUG(logger_, "Area computation failed for enclave face: %s - skipping face",
        e.GetMessageString());
    }
  }

  if ((enclave_area / global_total_area) > enclave_area_ratio_) {return false;}

  gp_Dir n_parent = calculate_safe_normal(parent_face);

  for (TopTools_ListIteratorOfListOfShape it(enclave_faces); it.More(); it.Next()) {
    gp_Dir n_wall = calculate_safe_normal(TopoDS::Face(it.Value()));
    double angle_deg = n_parent.Angle(n_wall) * (180.0 / M_PI);

    // Steep walls indicate a real feature — keep the enclave
    if (angle_deg > enclave_angle_threshold_ && angle_deg < (180.0 - enclave_angle_threshold_)) {
      return false;
    }
  }

  return true;
}

void ShapeRefiner::find_inflections(
  const BRepAdaptor_Surface & surface,
  bool scan_u,
  std::vector<double> & splits) const
{
  double start = scan_u ? surface.FirstUParameter() : surface.FirstVParameter();
  double end = scan_u ? surface.LastUParameter() : surface.LastVParameter();
  double other_mid = scan_u ?
    (surface.FirstVParameter() + surface.LastVParameter()) / 2.0 :
    (surface.FirstUParameter() + surface.LastUParameter()) / 2.0;

  const int num_samples = 25;
  double step = (end - start) / num_samples;
  double prev_k = 0.0;

  for (int i = 0; i <= num_samples; ++i) {
    double current_p = start + (i * step);
    double u = scan_u ? current_p : other_mid;
    double v = scan_u ? other_mid : current_p;

    // Gaussian curvature is unchanged by the face location, so the bare surface is fine here.
    GeomLProp_SLProps props(surface.Surface().Surface(), u, v, 2, 1e-6);
    if (props.IsCurvatureDefined()) {
      double current_k = props.GaussianCurvature();
      if (i > 0 && (prev_k * current_k) < 0.0) {
        // Curvature sign change, interpolate split position
        double denom = std::abs(prev_k) + std::abs(current_k);
        if (denom > 1e-6) {
          splits.push_back((start + (i - 1) * step) + (step * std::abs(prev_k) / denom));
        }
      }
      prev_k = current_k;
    } else {
      prev_k = 0.0;
    }
  }
}

void ShapeRefiner::check_edge_arc_lengths(
  const TopoDS_Face & face,
  std::vector<double> & u_splits,
  std::vector<double> & v_splits) const
{
  BRepAdaptor_Surface surface(face);
  double u_min = surface.FirstUParameter();
  double u_max = surface.LastUParameter();
  double v_min = surface.FirstVParameter();
  double v_max = surface.LastVParameter();

  // Conservative fallback: use max boundary edge length for both U and V.
  // May create extra splits but guarantees no needed splits are missed.
  double max_u_edge_length = 0.0;
  double max_v_edge_length = 0.0;
  bool has_edges = false;

  for (TopExp_Explorer edge_exp(face, TopAbs_EDGE); edge_exp.More(); edge_exp.Next()) {
    has_edges = true;
    try {
      const TopoDS_Edge & edge = TopoDS::Edge(edge_exp.Current());
      BRepAdaptor_Curve curve(edge);
      double length = GCPnts_AbscissaPoint::Length(curve);
      Standard_Real p_first, p_last;
      Handle(Geom2d_Curve) pcurve = BRep_Tool::CurveOnSurface(edge, face, p_first, p_last);
      if (!pcurve.IsNull()) {
        gp_Pnt2d uv1 = pcurve->Value(p_first);
        gp_Pnt2d uv2 = pcurve->Value(p_last);
        if (std::abs(uv2.X() - uv1.X()) >= std::abs(uv2.Y() - uv1.Y())) {
          max_u_edge_length = std::max(max_u_edge_length, length);
        } else {
          max_v_edge_length = std::max(max_v_edge_length, length);
        }
      } else {
        // No pcurve available, conservatively assign to U only.
        // Assigning to both would trigger unnecessary splits in both directions.
        max_u_edge_length = std::max(max_u_edge_length, length);
      }
    } catch (const Standard_Failure & e) {
      RCLCPP_DEBUG(logger_, "Failed to compute edge length: %s", e.GetMessageString());
    } catch (const std::exception & e) {
      RCLCPP_DEBUG(logger_, "Exception computing edge length: %s", e.what());
    }
  }

  // Fallback for edgeless surfaces (e.g. closed spheres)
  if (!has_edges || (max_u_edge_length < 1e-6 && max_v_edge_length < 1e-6)) {
    GeomAbs_SurfaceType type = surface.GetType();
    if (type == GeomAbs_Cylinder) {
      max_u_edge_length = 2.0 * M_PI * surface.Cylinder().Radius();
    } else if (type == GeomAbs_Sphere) {
      max_u_edge_length = 2.0 * M_PI * surface.Sphere().Radius();
      max_v_edge_length = M_PI * surface.Sphere().Radius();
    } else {
      return;
    }

    // Scale the full-range estimate down to the actual span of this face.
    // U spans 2π; a sphere's V spans only π (pole to pole).
    max_u_edge_length *= (u_max - u_min) / (2.0 * M_PI);
    max_v_edge_length *= (v_max - v_min) / M_PI;
  }

  if (max_u_edge_length > max_arc_length_) {
    int num_pieces = static_cast<int>(std::ceil(max_u_edge_length / max_arc_length_));
    double step = (u_max - u_min) / num_pieces;
    for (int i = 1; i < num_pieces; ++i) {
      u_splits.push_back(u_min + (i * step));
    }
  }

  if (max_v_edge_length > max_arc_length_) {
    int num_pieces = static_cast<int>(std::ceil(max_v_edge_length / max_arc_length_));
    double step = (v_max - v_min) / num_pieces;
    for (int i = 1; i < num_pieces; ++i) {
      v_splits.push_back(v_min + (i * step));
    }
  }
}

bool ShapeRefiner::is_physically_planar(const TopoDS_Face & face) const
{
  BRepAdaptor_Surface surface(face);
  if (surface.GetType() == GeomAbs_Plane) {return true;}

  Handle(Geom_Surface) surf = BRep_Tool::Surface(face);
  if (surf.IsNull()) {return false;}

  gp_Dir ref_normal = calculate_safe_normal(face);
  const bool reversed = (face.Orientation() == TopAbs_REVERSED);
  const double tolerance_rad = planarity_tolerance_deg_ * M_PI / 180.0;

  double u_params[] = {surface.FirstUParameter(), surface.LastUParameter()};
  double v_params[] = {surface.FirstVParameter(), surface.LastVParameter()};

  // Checks 4 corners only — a saddle point in the interior could be missed!
  for (double u : u_params) {
    for (double v : v_params) {
      GeomLProp_SLProps props(surf, u, v, 1, 1e-6);
      if (props.IsNormalDefined()) {
        // ref_normal is orientation-corrected, so the corner normals must be too.
        gp_Dir corner_normal = props.Normal();
        if (reversed) {corner_normal.Reverse();}
        if (ref_normal.Angle(corner_normal) > tolerance_rad) {return false;}
      }
    }
  }
  return true;
}

gp_Dir ShapeRefiner::calculate_safe_normal(const TopoDS_Face & face) const
{
  try {
    BRepAdaptor_Surface surface(face);
    double u_mid = (surface.FirstUParameter() + surface.LastUParameter()) / 2.0;
    double v_mid = (surface.FirstVParameter() + surface.LastVParameter()) / 2.0;

    // BRep_Tool::Surface applies the face location, so the normal is in the shape's frame.
    Handle(Geom_Surface) surf = BRep_Tool::Surface(face);
    if (surf.IsNull()) {
      RCLCPP_WARN(logger_, "Face has no surface, using Z-up fallback");
      return gp_Dir(0, 0, 1);
    }
    GeomLProp_SLProps props(surf, u_mid, v_mid, 1, 1e-6);

    if (!props.IsNormalDefined()) {
      // Midpoint can land on a degenerate pole (sphere apex, cone tip) — retry at 10% offset.
      u_mid = surface.FirstUParameter() +
        (surface.LastUParameter() - surface.FirstUParameter()) * 0.1;
      v_mid = surface.FirstVParameter() +
        (surface.LastVParameter() - surface.FirstVParameter()) * 0.1;
      props.SetParameters(u_mid, v_mid);
    }

    if (!props.IsNormalDefined()) {
      RCLCPP_WARN(logger_, "Unable to compute face normal, using Z-up fallback");
      return gp_Dir(0, 0, 1);
    }

    gp_Dir normal = props.Normal();
    // geometric ≠ topological direction
    if (face.Orientation() == TopAbs_REVERSED) {normal.Reverse();}
    return normal;
  } catch (const Standard_Failure &) {
    RCLCPP_DEBUG(logger_, "Failed to calculate normal - using fallback");
    return gp_Dir(0, 0, 1);
  }
}

}  // namespace geometry
}  // namespace hold_and_weld_gripper_sampler
