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
#include <GProp_GProps.hxx>
#include <rclcpp/rclcpp.hpp>
#include <ShapeFix_Shape.hxx>
#include <ShapeUpgrade_ShapeDivideClosed.hxx>
#include <ShapeUpgrade_UnifySameDomain.hxx>
#include <Standard_Failure.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>

// TODO(@silanus23): Add a splitter that splits based on sudden normal trend changes.
// TODO(@silanus23): Make heuristic approches adaptive instead strict sample based

namespace hold_and_weld_gripper_sampler
{
namespace geometry
{

static const rclcpp::Logger logger_ = rclcpp::get_logger("gripper_sampler");

ShapeRefiner::ShapeRefiner(
  double max_cylinder_radius,
  double max_arc_length,
  double enclave_area_ratio,
  double enclave_angle_threshold,
  double max_face_area_ratio)
: max_cylinder_radius_(max_cylinder_radius),
  max_arc_length_(max_arc_length),
  enclave_area_ratio_(enclave_area_ratio),
  enclave_angle_threshold_(enclave_angle_threshold),
  max_face_area_ratio_(max_face_area_ratio) {}

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
  // Pre-split U-periodic faces: BRepFeat_SplitShape silently fails on them because it
  // cannot insert ISO edges without an existing seam. ShapeUpgrade_ShapeDivideClosed opens it.
  int nb_split_points = 0;
  for (TopExp_Explorer periodic_exp(shape, TopAbs_FACE); periodic_exp.More();
    periodic_exp.Next())
  {
    BRepAdaptor_Surface adaptor(TopoDS::Face(periodic_exp.Current()));
    if (!adaptor.IsUPeriodic()) {continue;}

    // Estimate full U-period arc length:
    // exact formula for analytics, mid-V isocurve for BSpline/Bezier.
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
        Handle(Geom_Curve) iso = adaptor.Surface().Surface()->UIso(
          (adaptor.FirstUParameter() + adaptor.LastUParameter()) / 2.0);
        if (!iso.IsNull()) {
          GeomAdaptor_Curve iso_adaptor(iso,
            adaptor.FirstVParameter(), adaptor.LastVParameter());
          u_arc_length = GCPnts_AbscissaPoint::Length(iso_adaptor);
        }
      } catch (const Standard_Failure & e) {
        RCLCPP_DEBUG(logger_,
              "UIso arc length estimation failed: %s - face deferred to area ratio pass",
          e.GetMessageString());
      }
    }

    if (u_arc_length > max_arc_length_) {
      int pieces = static_cast<int>(std::ceil(u_arc_length / max_arc_length_));
      nb_split_points = std::max(nb_split_points, pieces - 1);
    }
  }

  TopoDS_Shape result = shape;
  if (nb_split_points > 0) {
    RCLCPP_INFO(logger_,
      "U-periodic face(s) exceed arc length limit - "
      "pre-splitting to open seam before surface splitter (%d point(s)).",
      nb_split_points);
    try {
      ShapeUpgrade_ShapeDivideClosed divider(shape);
      divider.SetNbSplitPoints(nb_split_points);
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
  RCLCPP_DEBUG(logger_, "Phase 1 (periodic pre-split) complete: %d split point(s)",
        nb_split_points);
  return result;
}

TopoDS_Shape ShapeRefiner::refine_phase2_arc_length_split(
  const TopoDS_Shape & shape, double global_total_area) const
{
  (void)global_total_area;  // reserved for future diagnostics

  BRepFeat_SplitShape splitter(shape);
  bool needs_split = false;

  TopExp_Explorer face_exp(shape, TopAbs_FACE);
  for (; face_exp.More(); face_exp.Next()) {
    const TopoDS_Face & face = TopoDS::Face(face_exp.Current());

    if (is_physically_planar(face)) {continue;}

    BRepAdaptor_Surface adaptor(face);
    GeomAbs_SurfaceType type = adaptor.GetType();

    // Skip U-periodic faces — already handled by ShapeUpgrade_ShapeDivideClosed above
    if (adaptor.IsUPeriodic()) {continue;}
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

    Handle(Geom_Surface) surf = adaptor.Surface().Surface();
    double u_min = adaptor.FirstUParameter();
    double u_max = adaptor.LastUParameter();
    double v_min = adaptor.FirstVParameter();
    double v_max = adaptor.LastVParameter();

    for (double u : u_splits) {
      Handle(Geom_Curve) u_iso = surf->UIso(u);
      BRepBuilderAPI_MakeEdge edge_maker(u_iso, v_min, v_max);
      splitter.Add(edge_maker.Edge(), face);
      needs_split = true;
    }

    for (double v : v_splits) {
      Handle(Geom_Curve) v_iso = surf->VIso(v);
      BRepBuilderAPI_MakeEdge edge_maker(v_iso, u_min, u_max);
      splitter.Add(edge_maker.Edge(), face);
      needs_split = true;
    }
  }

  TopoDS_Shape result = shape;
  if (needs_split) {
    splitter.Build();
    if (!splitter.IsDone()) {
      RCLCPP_WARN(logger_, "BRepFeat_SplitShape IsDone() false - skipping surface split");
    } else {
      result = splitter.Shape();
    }
  }
  RCLCPP_DEBUG(logger_, "Phase 2 (arc length + inflection split) complete");
  return result;
}

TopoDS_Shape ShapeRefiner::refine_phase3_area_ratio_split(
  const TopoDS_Shape & shape, double global_total_area) const
{
  BRepFeat_SplitShape final_splitter(shape);
  bool needs_final_split = false;

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

    Handle(Geom_Surface) surf = adaptor.Surface().Surface();
    double u_min = adaptor.FirstUParameter();
    double u_max = adaptor.LastUParameter();
    double v_min = adaptor.FirstVParameter();
    double v_max = adaptor.LastVParameter();

    for (double u : u_splits) {
      Handle(Geom_Curve) u_iso = surf->UIso(u);
      BRepBuilderAPI_MakeEdge edge_maker(u_iso, v_min, v_max);
      final_splitter.Add(edge_maker.Edge(), face);
      needs_final_split = true;
    }

    for (double v : v_splits) {
      Handle(Geom_Curve) v_iso = surf->VIso(v);
      BRepBuilderAPI_MakeEdge edge_maker(v_iso, u_min, u_max);
      final_splitter.Add(edge_maker.Edge(), face);
      needs_final_split = true;
    }
  }

  TopoDS_Shape result = shape;
  if (needs_final_split) {
    final_splitter.Build();
    if (!final_splitter.IsDone()) {
      RCLCPP_WARN(logger_, "BRepFeat_SplitShape (final) IsDone() false - skipping final split");
    } else {
      result = final_splitter.Shape();
    }
  }
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

    GeomLProp_SLProps props(surface.Surface().Surface(), u, v, 2, 1e-6);
    if (props.IsCurvatureDefined()) {
      double current_k = props.GaussianCurvature();
      if (i > 0 && (prev_k * current_k) < 0.0) {
        // Curvature sign change — interpolate split position
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
        // No pcurve available — conservatively assign to U only.
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

    // Scale full-circumference estimate down to the actual arc span of this face.
    max_u_edge_length *= (u_max - u_min) / (2.0 * M_PI);
    max_v_edge_length *= (v_max - v_min) / (2.0 * M_PI);
  }

  // Full cylinders arrive via the area-ratio pass; partial ones are handled here.
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

  gp_Dir ref_normal = calculate_safe_normal(face);

  double u_params[] = {surface.FirstUParameter(), surface.LastUParameter()};
  double v_params[] = {surface.FirstVParameter(), surface.LastVParameter()};

  // Checks 4 corners only — a saddle point in the interior could be missed!
  for (double u : u_params) {
    for (double v : v_params) {
      GeomLProp_SLProps props(surface.Surface().Surface(), u, v, 1, 1e-6);
      if (props.IsNormalDefined()) {
        if (ref_normal.Angle(props.Normal()) > (M_PI / 180.0)) {return false;}
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

    GeomLProp_SLProps props(surface.Surface().Surface(), u_mid, v_mid, 1, 1e-6);

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
