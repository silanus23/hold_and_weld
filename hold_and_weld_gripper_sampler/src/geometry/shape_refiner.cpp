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
#include <ShapeUpgrade_UnifySameDomain.hxx>
#include <Standard_Failure.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>

namespace hold_and_weld_gripper_sampler
{
namespace geometry
{

static const rclcpp::Logger logger_ = rclcpp::get_logger("gripper_sampler");

ShapeRefiner::ShapeRefiner(
  double max_cylinder_radius,
  double max_arc_length,
  double enclave_area_ratio,
  double enclave_angle_threshold)
: max_cylinder_radius_(max_cylinder_radius),
  max_arc_length_(max_arc_length),
  enclave_area_ratio_(enclave_area_ratio),
  enclave_angle_threshold_(enclave_angle_threshold) {}

TopoDS_Shape ShapeRefiner::refine(const TopoDS_Shape & raw_shape) const
{
  // Surface refinement strategy for gripper sampling:
  // 1. Heal/fix topology issues
  // 2. Identify and remove small enclave features (shallow pockets, holes)
  // 3. Split large curved surfaces into smaller patches for better sampling coverage
  //    - Cylinders: split based on radius and arc length
  //    - BSpline/Bezier: detect inflection points by sampling curvature
  //    - Other surfaces: conservative edge-based splitting (may over-split, won't under-split)
  // 4. Unify adjacent coplanar faces to reduce unnecessary patch count
  try {
    ShapeFix_Shape healer(raw_shape);
    healer.Perform();
    TopoDS_Shape current_shape = healer.Shape();

    GProp_GProps global_props;
    BRepGProp::SurfaceProperties(current_shape, global_props);
    double global_total_area = global_props.Mass();

    TopTools_ListOfShape faces_to_remove;
    identify_enclave_features(current_shape, global_total_area, faces_to_remove);

    if (!faces_to_remove.IsEmpty()) {
      try {
        BRepAlgoAPI_Defeaturing eraser;
        eraser.SetShape(current_shape);
        eraser.AddFacesToRemove(faces_to_remove);
        eraser.Build();
        if (eraser.IsDone()) {
          current_shape = eraser.Shape();

          ShapeUpgrade_UnifySameDomain mid_healer(current_shape, true, true);
          mid_healer.Build();
          current_shape = mid_healer.Shape();
        }
      } catch (const std::exception & e) {
        RCLCPP_WARN(logger_, "Defeaturing failed: %s - continuing without enclave removal",
          e.what());
      } catch (...) {
        RCLCPP_WARN(logger_,
              "Defeaturing failed with unknown error - continuing without enclave removal");
      }
    }

    BRepFeat_SplitShape splitter(current_shape);
    bool needs_split = false;

    // Split large curved surfaces based on their type:
    // - Analytical surfaces (cylinder, cone): use geometric properties (radius, angle)
    // - Freeform surfaces (BSpline, Bezier): sample interior to detect inflection points
    // - Generic/unknown surfaces: conservative edge-based estimation (may over-split)
    TopExp_Explorer face_exp(current_shape, TopAbs_FACE);
    for (; face_exp.More(); face_exp.Next()) {
      const TopoDS_Face & face = TopoDS::Face(face_exp.Current());

      if (is_physically_planar(face)) {continue;}

      BRepAdaptor_Surface adaptor(face);
      GeomAbs_SurfaceType type = adaptor.GetType();
      std::vector<double> u_splits, v_splits;

      if (type == GeomAbs_Cylinder) {
        // Cylinders: split based on radius and circumference
        get_cylinder_splits(face, u_splits);
      } else if (type == GeomAbs_Cone) {
        // Cones: analytical edge-based splitting
        get_analytical_splits(face, u_splits, v_splits);
      } else if (type == GeomAbs_BSplineSurface || type == GeomAbs_BezierSurface) {
        // Freeform surfaces: detect curvature changes by sampling interior
        find_inflections(adaptor, true, u_splits);
        find_inflections(adaptor, false, v_splits);
      } else {
        // Fallback for other surface types: conservative edge measurement
        // (may create extra splits, but ensures large curved areas get subdivided)
        check_edge_arc_lengths(face, u_splits, v_splits);
      }

      if (u_splits.empty() && v_splits.empty()) {continue;}

      Handle(Geom_Surface) surf = adaptor.Surface().Surface();
      double u_min = adaptor.FirstUParameter();
      double u_max = adaptor.LastUParameter();
      double v_min = adaptor.FirstVParameter();
      double v_max = adaptor.LastVParameter();

      for (double u : u_splits) {
        Handle(Geom_Curve) u_iso = surf->UIso(u);
        BRepBuilderAPI_MakeEdge edge_maker(u_iso, v_min, v_max);
        if (edge_maker.IsDone()) {
          splitter.Add(edge_maker.Edge(), face);
          needs_split = true;
        }
      }

      for (double v : v_splits) {
        Handle(Geom_Curve) v_iso = surf->VIso(v);
        BRepBuilderAPI_MakeEdge edge_maker(v_iso, u_min, u_max);
        if (edge_maker.IsDone()) {
          splitter.Add(edge_maker.Edge(), face);
          needs_split = true;
        }
      }
    }

    if (needs_split) {
      splitter.Build();
      if (splitter.IsDone()) {
        current_shape = splitter.Shape();
      }
    }

    TopExp_Explorer sanity_exp(current_shape, TopAbs_FACE);
    for (; sanity_exp.More(); sanity_exp.Next()) {
      const TopoDS_Face & face = TopoDS::Face(sanity_exp.Current());
      GProp_GProps face_props;
      BRepGProp::SurfaceProperties(face, face_props);
      double face_area = face_props.Mass();

      if (face_area > (global_total_area * 0.3)) {
        BRepAdaptor_Surface adaptor(face);
        const char * type_str = "Other";
        switch (adaptor.GetType()) {
          case GeomAbs_Plane: type_str = "Plane"; break;
          case GeomAbs_Cylinder: type_str = "Cylinder"; break;
          case GeomAbs_Cone: type_str = "Cone"; break;
          case GeomAbs_Sphere: type_str = "Sphere"; break;
          case GeomAbs_Torus: type_str = "Torus"; break;
          case GeomAbs_BSplineSurface: type_str = "BSpline"; break;
          case GeomAbs_BezierSurface: type_str = "Bezier"; break;
          default: break;
        }
        RCLCPP_WARN(logger_,
          "Large unsplit surface detected (%.1f%% of total area). "
          "This may reduce gripper sampling quality. Surface type: %s",
          (face_area / global_total_area * 100.0), type_str);
      }
    }

    ShapeUpgrade_UnifySameDomain final_unifier(current_shape, true, false);
    final_unifier.Build();

    return final_unifier.Shape();
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger_, "Error in shape refinement: %s - returning original shape",
      e.what());
    return raw_shape;
  } catch (...) {
    RCLCPP_ERROR(logger_, "Unknown error in shape refinement - returning original shape");
    return raw_shape;
  }
}

void ShapeRefiner::identify_enclave_features(
  const TopoDS_Shape & shape,
  double global_total_area,
  TopTools_ListOfShape & kill_list) const
{
  TopTools_IndexedDataMapOfShapeListOfShape edge_to_faces;
  TopExp::MapShapesAndAncestors(shape, TopAbs_EDGE, TopAbs_FACE, edge_to_faces);

  TopTools_MapOfShape processed_faces;

  TopExp_Explorer face_exp(shape, TopAbs_FACE);
  for (; face_exp.More(); face_exp.Next()) {
    const TopoDS_Face & parent_face = TopoDS::Face(face_exp.Current());
    if (processed_faces.Contains(parent_face)) {continue;}

    TopoDS_Wire outer_wire = BRepTools::OuterWire(parent_face);

    TopExp_Explorer wire_exp(parent_face, TopAbs_WIRE);
    for (; wire_exp.More(); wire_exp.Next()) {
      const TopoDS_Wire & wire = TopoDS::Wire(wire_exp.Current());

            // Skip the outer wire - only process inner wires (holes)
      if (!outer_wire.IsNull() && wire.IsSame(outer_wire)) {
        continue;
      }

      TopTools_ListOfShape enclave_faces;
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

    // Check total area of enclave
  double enclave_area = 0.0;
  GProp_GProps area_props;
  for (TopTools_ListIteratorOfListOfShape it(enclave_faces); it.More(); it.Next()) {
    BRepGProp::SurfaceProperties(TopoDS::Face(it.Value()), area_props);
    enclave_area += area_props.Mass();
  }

    // If enclave is too large (occupies significant chunk), keep it
  if ((enclave_area / global_total_area) > enclave_area_ratio_) {return false;}

    // Check angle between parent and enclave walls
    // Small angle (< threshold) = shallow pocket (nearly parallel) → REMOVE
    // Large angle (> threshold) = steep walls (poking out) → KEEP
  gp_Dir n_parent = calculate_safe_normal(parent_face);

  for (TopTools_ListIteratorOfListOfShape it(enclave_faces); it.More(); it.Next()) {
    gp_Dir n_wall = calculate_safe_normal(TopoDS::Face(it.Value()));
    double angle_deg = n_parent.Angle(n_wall) * (180.0 / M_PI);

    // If angle is steep (not near 0° or 180°), walls are poking out significantly - keep it
    // Angles near 0° or 180° mean walls are nearly parallel/anti-parallel (shallow pocket)
    // Only keep enclave if angle is in the "steep" middle range
    if (angle_deg > enclave_angle_threshold_ && angle_deg < (180.0 - enclave_angle_threshold_)) {
      return false;  // Steep walls - keep the enclave
    }
  }

    // All enclave faces have shallow angles - safe to remove
  return true;
}

void ShapeRefiner::find_inflections(
  const BRepAdaptor_Surface & surface,
  bool scan_u,
  std::vector<double> & splits) const
{
  double start = scan_u ? surface.FirstUParameter() : surface.FirstVParameter();
  double end = scan_u ? surface.LastUParameter() : surface.LastVParameter();
  double other_mid = scan_u ? (surface.FirstVParameter() + surface.LastVParameter()) / 2.0 :
    (surface.FirstUParameter() + surface.LastUParameter()) / 2.0;

  const int num_samples = 25;
  double step = (end - start) / num_samples;
  double prev_k = 0.0;

  for (int i = 0; i <= num_samples; ++i) {
    double current_p = start + (i * step);
    double u = scan_u ? current_p : other_mid;
    double v = scan_u ? other_mid : current_p;

    GeomLProp_SLProps props(surface.Surface().Surface(), u, v, 2, 1e-7);
    if (props.IsCurvatureDefined()) {
      double current_k = props.GaussianCurvature();
      if (i > 0 && (prev_k * current_k) < 0.0) {
        // Inflection point detected (curvature sign change)
        // Interpolate exact position using linear approximation
        // Check that curvatures are significant to avoid division by near-zero
        // (both curvatures near zero could be numerical noise, not real inflection)
        double denom = std::abs(prev_k) + std::abs(current_k);
        if (denom > 1e-10) {
          double ratio = std::abs(prev_k) / denom;
          splits.push_back((start + (i - 1) * step) + (step * ratio));
        }
      }
      prev_k = current_k;
    }
  }
}

void ShapeRefiner::get_cylinder_splits(
  const TopoDS_Face & face,
  std::vector<double> & u_splits) const
{
  BRepAdaptor_Surface surface(face);
  gp_Cylinder cylinder = surface.Cylinder();
  double radius = cylinder.Radius();

    // Check radius limit for perfect cylinders
  if (radius > max_cylinder_radius_) {
    double u_min = surface.FirstUParameter();
    double u_max = surface.LastUParameter();
    double circumference = 2.0 * M_PI * radius;

    int num_pieces = std::ceil(circumference / max_arc_length_);
    if (num_pieces > 1) {
      double step = (u_max - u_min) / num_pieces;
      for (int i = 1; i < num_pieces; ++i) {
        u_splits.push_back(u_min + (i * step));
      }
      return;
    }
  }

    // Check arc length using edge measurement
  std::vector<double> v_splits_unused;
  check_edge_arc_lengths(face, u_splits, v_splits_unused);
}

void ShapeRefiner::get_analytical_splits(
  const TopoDS_Face & face,
  std::vector<double> & u_splits,
  std::vector<double> & v_splits) const
{
  check_edge_arc_lengths(face, u_splits, v_splits);
}

// Estimates surface arc lengths by measuring boundary edges.
// This is a conservative fallback for generic surfaces where interior curvature
// cannot be analytically determined. Uses max edge length for both U and V directions
// to avoid missing needed splits (at the cost of potentially creating extra splits).
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

    // Measure all boundary edges to estimate surface arc lengths.
    // This is a fallback method for generic surfaces where we can't analyze interior curvature.
  TopExp_Explorer edge_exp(face, TopAbs_EDGE);
  double max_u_edge_length = 0.0;
  double max_v_edge_length = 0.0;
  bool has_edges = false;

  for (; edge_exp.More(); edge_exp.Next()) {
    has_edges = true;
    const TopoDS_Edge & edge = TopoDS::Edge(edge_exp.Current());

    try {
      BRepAdaptor_Curve curve(edge);
      double length = GCPnts_AbscissaPoint::Length(curve);

      // CONSERVATIVE STRATEGY FOR SAMPLING:
      // We assign the maximum edge length to BOTH U and V directions.
      //
      // Why not classify edges by direction?
      // - Boundary edges don't reliably represent interior surface arc length
      // - For doubly curved surfaces, interior can curve more than edges suggest
      // - Edge classification heuristics can fail on complex topology
      //
      // Conservative approach (current):
      // - Use max(all edges) for both directions
      // - May create extra unnecessary splits → acceptable for sampling
      // - Guarantees we won't MISS needed splits → critical for coverage
      //
      // For surfaces where we can analyze curvature (BSpline/Bezier), we use
      // find_inflections() instead which samples the interior properly.
      max_u_edge_length = std::max(max_u_edge_length, length);
      max_v_edge_length = std::max(max_v_edge_length, length);
    } catch (const std::exception & e) {
            // Skip degenerate or problematic edges
      RCLCPP_DEBUG(logger_, "Failed to compute edge length: %s", e.what());
      continue;
    } catch (...) {
      RCLCPP_DEBUG(logger_, "Failed to compute edge length (unknown error)");
      continue;
    }
  }

    // Fallback for surfaces without edges (closed surfaces like spheres)
  if (!has_edges || (max_u_edge_length < 1e-6 && max_v_edge_length < 1e-6)) {
    GeomAbs_SurfaceType type = surface.GetType();
    if (type == GeomAbs_Cylinder) {
      max_u_edge_length = 2.0 * M_PI * surface.Cylinder().Radius();
    } else if (type == GeomAbs_Sphere) {
      max_u_edge_length = 2.0 * M_PI * surface.Sphere().Radius();
      max_v_edge_length = M_PI * surface.Sphere().Radius();
    } else {
      return;       // Can't estimate arc length
    }

        // Adjust for partial parametric range
    double u_range_ratio = (u_max - u_min) / (2.0 * M_PI);
    double v_range_ratio = (v_max - v_min) / (2.0 * M_PI);
    max_u_edge_length *= u_range_ratio;
    max_v_edge_length *= v_range_ratio;
  }

    // Split U direction if edge exceeds limit
  if (max_u_edge_length > max_arc_length_) {
    int num_pieces = std::ceil(max_u_edge_length / max_arc_length_);
    double step = (u_max - u_min) / num_pieces;
    for (int i = 1; i < num_pieces; ++i) {
      u_splits.push_back(u_min + (i * step));
    }
  }

    // Split V direction if edge exceeds limit
  if (max_v_edge_length > max_arc_length_) {
    int num_pieces = std::ceil(max_v_edge_length / max_arc_length_);
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

    // Check corners for deviation
  double u_params[] = {surface.FirstUParameter(), surface.LastUParameter()};
  double v_params[] = {surface.FirstVParameter(), surface.LastVParameter()};

  for (double u : u_params) {
    for (double v : v_params) {
      GeomLProp_SLProps props(surface.Surface().Surface(), u, v, 1, 1e-7);
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

    GeomLProp_SLProps props(surface.Surface().Surface(), u_mid, v_mid, 1, 1e-7);
    if (!props.IsNormalDefined()) {
      // Try offset point
      u_mid = surface.FirstUParameter() + (surface.LastUParameter() - surface.FirstUParameter()) *
        0.1;
      v_mid = surface.FirstVParameter() + (surface.LastVParameter() - surface.FirstVParameter()) *
        0.1;
      props.SetParameters(u_mid, v_mid);
    }

    if (!props.IsNormalDefined()) {
      // Last resort: unable to compute actual normal, use default Z-up
      RCLCPP_WARN(logger_, "Unable to compute face normal, using default Z-up direction");
      return gp_Dir(0, 0, 1);
    }

    gp_Dir normal = props.Normal();
    if (face.Orientation() == TopAbs_REVERSED) {normal.Reverse();}
    return normal;
  } catch (Standard_Failure &) {
    RCLCPP_DEBUG(logger_, "Failed to calculate normal - using fallback");
    return gp_Dir(0, 0, 1);
  }
}

}  // namespace geometry
}  // namespace hold_and_weld_gripper_sampler
