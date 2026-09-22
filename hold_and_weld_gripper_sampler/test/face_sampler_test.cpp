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

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

#include <BRepAlgoAPI_Cut.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakePolygon.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <BRepFilletAPI_MakeFillet.hxx>
#include <BRepGProp.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeCone.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRepPrimAPI_MakeRevol.hxx>
#include <BRepPrimAPI_MakeSphere.hxx>
#include <BRepPrimAPI_MakeTorus.hxx>
#include <BRepTools.hxx>
#include <BRep_Tool.hxx>
#include <GProp_GProps.hxx>
#include <GeomAPI_PointsToBSpline.hxx>
#include <Geom_BSplineCurve.hxx>
#include <Geom_BSplineSurface.hxx>
#include <Geom_Plane.hxx>
#include <Geom_Surface.hxx>
#include <Precision.hxx>
#include <TColStd_Array1OfInteger.hxx>
#include <TColStd_Array1OfReal.hxx>
#include <TColgp_Array1OfPnt.hxx>
#include <TColgp_Array2OfPnt.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopLoc_Location.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Shape.hxx>
#include <TopoDS_Wire.hxx>
#include <gp_Ax1.hxx>
#include <gp_Ax2.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>
#include <gp_Trsf.hxx>
#include <gp_Vec.hxx>

#include "hold_and_weld_gripper_sampler/sampling/face_sampler.hpp"

using hold_and_weld_gripper_sampler::sampling::FaceSample;
using hold_and_weld_gripper_sampler::sampling::FaceSamplingConfig;
using hold_and_weld_gripper_sampler::sampling::FaceSamplingGrid;
using hold_and_weld_gripper_sampler::sampling::GridLayout;
using hold_and_weld_gripper_sampler::sampling::area_fraction;
using hold_and_weld_gripper_sampler::sampling::bounding_wire_in_uv;
using hold_and_weld_gripper_sampler::sampling::sample_face_region;
using hold_and_weld_gripper_sampler::sampling::sampled_area;

namespace
{

// Exact face area straight from OCCT, used as the reference everywhere below.
double occt_face_area(const TopoDS_Face & face)
{
  GProp_GProps props;
  BRepGProp::SurfaceProperties(face, props);
  return props.Mass();
}

std::vector<TopoDS_Face> faces_of(const TopoDS_Shape & shape)
{
  std::vector<TopoDS_Face> faces;
  for (TopExp_Explorer exp(shape, TopAbs_FACE); exp.More(); exp.Next()) {
    faces.push_back(TopoDS::Face(exp.Current()));
  }
  return faces;
}

// The planar face whose centroid sits highest in Z.
TopoDS_Face top_planar_face(const TopoDS_Shape & shape)
{
  TopoDS_Face best;
  double best_z = -std::numeric_limits<double>::max();

  for (const auto & face : faces_of(shape)) {
    Handle(Geom_Surface) surf = BRep_Tool::Surface(face);
    if (Handle(Geom_Plane) ::DownCast(surf).IsNull()) {continue;}

    GProp_GProps props;
    BRepGProp::SurfaceProperties(face, props);
    const double z = props.CentreOfMass().Z();
    if (z > best_z) {
      best_z = z;
      best = face;
    }
  }
  return best;
}

FaceSamplingConfig area_config(double density)
{
  FaceSamplingConfig config;
  config.sample_density = density;
  // Cell centres: every sample owns exactly one cell, so weights sum to area.
  config.layout = GridLayout::kCellCentres;
  config.compute_normals = true;
  return config;
}

// Gently bulged B-spline strip with one interior U knot at @p interior_knot.
// Poles/degree/shape are fixed, so moving the knot only changes how the
// parameter range is distributed -- isolates parameterisation as the
// variable under test, apart from curvature, degree, or trimming.
TopoDS_Face knotted_nurbs_strip(double interior_knot)
{
  constexpr double kWidth = 0.2;
  const double xs[5] = {0.0, 0.10, 0.20, 0.30, 0.40};
  const double zs[5] = {0.0, 0.02, 0.03, 0.02, 0.0};

  TColgp_Array2OfPnt poles(1, 5, 1, 2);
  for (int i = 1; i <= 5; ++i) {
    poles.SetValue(i, 1, gp_Pnt(xs[i - 1], 0.0, zs[i - 1]));
    poles.SetValue(i, 2, gp_Pnt(xs[i - 1], kWidth, zs[i - 1]));
  }

  TColStd_Array1OfReal u_knots(1, 3);
  u_knots.SetValue(1, 0.0);
  u_knots.SetValue(2, interior_knot);
  u_knots.SetValue(3, 1.0);

  TColStd_Array1OfInteger u_mults(1, 3);
  u_mults.SetValue(1, 4);
  u_mults.SetValue(2, 1);
  u_mults.SetValue(3, 4);

  TColStd_Array1OfReal v_knots(1, 2);
  v_knots.SetValue(1, 0.0);
  v_knots.SetValue(2, 1.0);

  TColStd_Array1OfInteger v_mults(1, 2);
  v_mults.SetValue(1, 2);
  v_mults.SetValue(2, 2);

  Handle(Geom_BSplineSurface) surf =
    new Geom_BSplineSurface(poles, u_knots, v_knots, u_mults, v_mults, 3, 1);

  return BRepBuilderAPI_MakeFace(surf, Precision::Confusion()).Face();
}

// Doubly curved freeform patch, 0.4 m square, evenly knotted in both
// directions. The "ordinary NURBS" case a weldment is mostly made of.
TopoDS_Face bumpy_nurbs_patch()
{
  TColgp_Array2OfPnt poles(1, 5, 1, 5);
  for (int i = 1; i <= 5; ++i) {
    for (int j = 1; j <= 5; ++j) {
      poles.SetValue(i, j, gp_Pnt(
          0.1 * (i - 1),
          0.1 * (j - 1),
          0.02 * std::sin(1.7 * (i - 1)) * std::cos(1.3 * (j - 1))));
    }
  }

  TColStd_Array1OfReal knots(1, 3);
  knots.SetValue(1, 0.0);
  knots.SetValue(2, 0.5);
  knots.SetValue(3, 1.0);

  TColStd_Array1OfInteger mults(1, 3);
  mults.SetValue(1, 4);
  mults.SetValue(2, 1);
  mults.SetValue(3, 4);

  Handle(Geom_BSplineSurface) surf =
    new Geom_BSplineSurface(poles, knots, knots, mults, mults, 3, 3);

  return BRepBuilderAPI_MakeFace(surf, Precision::Confusion()).Face();
}

// The one non-planar face of a shape, by first match.
TopoDS_Face first_curved_face(const TopoDS_Shape & shape)
{
  for (const auto & face : faces_of(shape)) {
    if (Handle(Geom_Plane) ::DownCast(BRep_Tool::Surface(face)).IsNull()) {
      return face;
    }
  }
  return TopoDS_Face();
}

// Closed wire through the given points, built in 3D.
TopoDS_Wire polygon_wire(const std::vector<gp_Pnt> & points)
{
  BRepBuilderAPI_MakePolygon poly;
  for (const auto & p : points) {
    poly.Add(p);
  }
  poly.Close();
  return poly.Wire();
}

}  // namespace

// A box face is 2 triangles no matter the mesh deflection, so sample count has
// to come from sample_density, not from triangulation. Also covers grid_steps
// overriding density outright.
TEST(FaceSamplerTest, SampleCountScalesWithDensityOnPlanarFace)
{
  const TopoDS_Shape box = BRepPrimAPI_MakeBox(0.1, 0.1, 0.1).Shape();
  const TopoDS_Face face = top_planar_face(box);
  ASSERT_FALSE(face.IsNull());

  const size_t coarse = sample_face_region(face, area_config(0.02)).size();
  const size_t fine = sample_face_region(face, area_config(0.005)).size();

  EXPECT_EQ(coarse, 25u);    // 0.1 m / 0.02 m = 5 cells per axis
  EXPECT_EQ(fine, 400u);     // 0.1 m / 0.005 m = 20 cells per axis
  EXPECT_GT(fine, coarse);

  FaceSamplingConfig steps_config;
  steps_config.sample_density = 0.0001;
  steps_config.grid_steps = 7;
  steps_config.layout = GridLayout::kCellCentres;
  EXPECT_EQ(sample_face_region(face, steps_config).size(), 49u);
}

// The ground predicate: area fraction of a face lying within a Z band. This is
// the measurement that replaces triangle-centroid contact ratio.
TEST(FaceSamplerTest, AreaFractionMeasuresPartialGroundContact)
{
  // Box rolled 45 degrees about X, balanced on one edge at z = 0. The old
  // centroid sampler reported 0% contact here; the dense reference is 7.00%.
  const double drop = 0.05 * M_SQRT1_2;
  gp_Trsf roll;
  roll.SetRotation(gp_Ax1(gp_Pnt(0, 0, 0), gp_Dir(1, 0, 0)), M_PI / 4.0);
  gp_Trsf lift;
  lift.SetTranslation(gp_Vec(0, 0, drop));

  const TopoDS_Shape box =
    BRepPrimAPI_MakeBox(gp_Pnt(-0.05, -0.05, 0.0), 0.1, 0.1, 0.1).Shape();
  const TopoDS_Shape rolled =
    BRepBuilderAPI_Transform(BRepBuilderAPI_Transform(box, roll).Shape(), lift).Shape();

  constexpr double kContactBand = 0.005;
  const auto in_contact = [](const FaceSample & s) {
      return s.point.Z() <= kContactBand;
    };

  std::vector<double> fractions;
  for (const auto & face : faces_of(rolled)) {
    fractions.push_back(area_fraction(sample_face_region(face, area_config(0.001)),
      in_contact));
  }
  ASSERT_EQ(fractions.size(), 6u);

  // Exactly two faces graze the ground, each over a 5mm/sin45 = 7.07mm strip
  // of a 100mm face. The other four are clear of the band entirely.
  int grazing = 0;
  for (double f : fractions) {
    if (f > 0.01) {
      ++grazing;
      EXPECT_NEAR(f, 0.0707, 0.005);
    } else {
      EXPECT_LT(f, 0.01);
    }
  }
  EXPECT_EQ(grazing, 2) << "expected exactly two faces in the ground band";
}

TEST(FaceSamplerTest, AreaFractionIsZeroForEmptySamples)
{
  EXPECT_EQ(area_fraction({}, [](const FaceSample &) {return true;}), 0.0);
}

// kNodes must keep the legacy grid so repointing the existing point samplers at
// this function is a refactor, not a behaviour change.
TEST(FaceSamplerTest, NodeLayoutIncludesUvEndpoints)
{
  const TopoDS_Shape box = BRepPrimAPI_MakeBox(0.1, 0.1, 0.1).Shape();
  const TopoDS_Face face = top_planar_face(box);
  ASSERT_FALSE(face.IsNull());

  FaceSamplingConfig config;
  config.sample_density = 0.02;
  config.layout = GridLayout::kNodes;

  const auto samples = sample_face_region(face, config);
  // 5 steps per axis -> 6 nodes per axis, endpoints included.
  EXPECT_EQ(samples.size(), 36u);

  double u_min = std::numeric_limits<double>::max();
  double u_max = -std::numeric_limits<double>::max();
  for (const auto & s : samples) {
    u_min = std::min(u_min, s.uv.X());
    u_max = std::max(u_max, s.uv.X());
  }
  EXPECT_NEAR(u_max - u_min, 0.1, 1e-9);
}

TEST(FaceSamplerTest, ClusteredKnotNurbsAreaIsIndependentOfParameterisation)
{
  const double reference =
    sampled_area(sample_face_region(knotted_nurbs_strip(0.5), area_config(0.002)));
  ASSERT_GT(reference, 0.0);

  for (const double knot : {0.1, 0.01, 0.004}) {
    const TopoDS_Face face = knotted_nurbs_strip(knot);
    ASSERT_FALSE(face.IsNull());

    const double measured = sampled_area(sample_face_region(face, area_config(0.002)));
    const double rel_error = std::abs(measured - reference) / reference;

    EXPECT_LT(rel_error, 0.01)
      << "interior knot " << knot << ": evenly knotted " << reference
      << " m^2, measured " << measured << " m^2, relative error " << rel_error;
  }
}

TEST(FaceSamplerTest, ClusteredKnotNurbsHonoursDensityInStretchedRegion)
{
  const TopoDS_Face face = knotted_nurbs_strip(0.004);
  ASSERT_FALSE(face.IsNull());

  constexpr double kDensity = 0.002;
  const auto samples = sample_face_region(face, area_config(kDensity));
  ASSERT_GT(samples.size(), 4u);

  // One row at constant V, ordered along U, so consecutive entries really are
  // neighbours on the grid.
  const double v_row = samples.front().uv.Y();
  std::vector<gp_Pnt> row;
  for (const auto & s : samples) {
    if (std::abs(s.uv.Y() - v_row) < 1e-12) {row.push_back(s.point);}
  }
  ASSERT_GT(row.size(), 4u);

  Handle(Geom_Surface) surf = BRep_Tool::Surface(face);
  Standard_Real u_min, u_max, v_min, v_max;
  BRepTools::UVBounds(face, u_min, u_max, v_min, v_max);
  row.insert(row.begin(), surf->Value(u_min, v_row));
  row.push_back(surf->Value(u_max, v_row));

  double max_gap = 0.0;
  for (size_t i = 1; i < row.size(); ++i) {
    max_gap = std::max(max_gap, row[i].Distance(row[i - 1]));
  }

  // sample_density is a metric spacing in metres, so it has to hold across the
  // clustered span too, not only on average over the face. Cell centres sit
  // half a cell in from each edge, so 2x the density is already generous.
  EXPECT_LT(max_gap, 2.0 * kDensity)
    << "largest coverage gap " << max_gap << " m at requested density "
    << kDensity << " m, over " << row.size() << " points in the row";
}

// Doubly curved freeform patch. Curvature varies in both directions at once, so
// neither axis can be sized from a single probe, but the parameterisation is
// even -- this is the shape a real STEP weldment panel is closest to.
TEST(FaceSamplerTest, BumpyNurbsPatchAreaMatchesOcct)
{
  const TopoDS_Face face = bumpy_nurbs_patch();
  ASSERT_FALSE(face.IsNull());

  const double reference = occt_face_area(face);
  ASSERT_GT(reference, 0.16);   // 0.4 m square plus the bulges

  const auto samples = sample_face_region(face, area_config(0.004));
  ASSERT_GT(samples.size(), 5000u);

  EXPECT_NEAR(sampled_area(samples), reference, reference * 0.01)
    << "reference " << reference << " measured " << sampled_area(samples);

  for (const auto & s : samples) {
    ASSERT_TRUE(s.has_normal);
    EXPECT_NEAR(s.normal.Magnitude(), 1.0, 1e-9);
  }
}

// Torus: periodic in both U and V, and |dS/du| = R + r cos(v) varies around the
// tube, so the inner and outer halves genuinely want different step counts.
TEST(FaceSamplerTest, TorusAreaMatchesAnalytic)
{
  constexpr double kMajor = 0.10;
  constexpr double kMinor = 0.03;
  const TopoDS_Shape torus = BRepPrimAPI_MakeTorus(kMajor, kMinor).Shape();
  const auto faces = faces_of(torus);
  ASSERT_EQ(faces.size(), 1u);

  const auto samples = sample_face_region(faces[0], area_config(0.002));
  ASSERT_GT(samples.size(), 1000u);

  const double analytic = 4.0 * M_PI * M_PI * kMajor * kMinor;
  EXPECT_NEAR(sampled_area(samples), analytic, analytic * 0.01);

  // Confirm the weights really do span the inner/outer range, i.e. this would
  // not pass with one weight per sample. (R+r)/(R-r) = 1.857 here.
  double min_w = std::numeric_limits<double>::max();
  double max_w = 0.0;
  for (const auto & s : samples) {
    if (s.area_weight <= 0.0) {continue;}
    min_w = std::min(min_w, s.area_weight);
    max_w = std::max(max_w, s.area_weight);
  }
  EXPECT_GT(max_w / min_w, 1.5) << "expected tube-position-dependent weights";
}

// Cone apex: a genuine parametric singularity, where |dS/du| -> 0 and the normal
// is undefined. The guard in sample_face_region must drop or de-weight those
// samples without poisoning the total with NaN.
TEST(FaceSamplerTest, ConeLateralAreaSurvivesDegenerateApex)
{
  constexpr double kRadius = 0.05;
  constexpr double kHeight = 0.10;
  const TopoDS_Shape cone =
    BRepPrimAPI_MakeCone(kRadius, 0.0, kHeight).Shape();

  const TopoDS_Face lateral = first_curved_face(cone);
  ASSERT_FALSE(lateral.IsNull());

  const auto samples = sample_face_region(lateral, area_config(0.002));
  ASSERT_FALSE(samples.empty());

  for (const auto & s : samples) {
    ASSERT_TRUE(std::isfinite(s.area_weight)) << "non-finite area weight near apex";
    ASSERT_GE(s.area_weight, 0.0);
    ASSERT_TRUE(std::isfinite(s.point.X()) && std::isfinite(s.point.Z()));
    if (s.has_normal) {
      EXPECT_NEAR(s.normal.Magnitude(), 1.0, 1e-9);
    }
  }

  const double analytic = M_PI * kRadius * std::hypot(kRadius, kHeight);
  EXPECT_NEAR(sampled_area(samples), analytic, analytic * 0.01);
}

// Surface of revolution over a B-spline profile: freeform in one direction,
// periodic in the other, with a seam. Closest thing here to a turned part.
TEST(FaceSamplerTest, RevolvedSplineAreaMatchesOcct)
{
  TColgp_Array1OfPnt profile(1, 5);
  profile.SetValue(1, gp_Pnt(0.05, 0.0, 0.00));
  profile.SetValue(2, gp_Pnt(0.07, 0.0, 0.03));
  profile.SetValue(3, gp_Pnt(0.04, 0.0, 0.06));
  profile.SetValue(4, gp_Pnt(0.06, 0.0, 0.09));
  profile.SetValue(5, gp_Pnt(0.05, 0.0, 0.12));

  Handle(Geom_BSplineCurve) curve = GeomAPI_PointsToBSpline(profile).Curve();
  ASSERT_FALSE(curve.IsNull());

  const TopoDS_Shape revolved = BRepPrimAPI_MakeRevol(
    BRepBuilderAPI_MakeEdge(curve).Edge(),
    gp_Ax1(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1))).Shape();

  const auto faces = faces_of(revolved);
  ASSERT_EQ(faces.size(), 1u);

  const double reference = occt_face_area(faces[0]);
  ASSERT_GT(reference, 0.0);

  const double measured = sampled_area(sample_face_region(faces[0], area_config(0.002)));
  EXPECT_NEAR(measured, reference, reference * 0.01)
    << "reference " << reference << " measured " << measured;
}

// A filleted box is the cheapest way to get many surface types in one shape:
// planes, cylindrical edge blends and spherical corner patches, all trimmed
// against each other. Total area over every face must still close.
TEST(FaceSamplerTest, FilletedBoxTotalAreaMatchesOcct)
{
  const TopoDS_Shape box = BRepPrimAPI_MakeBox(0.1, 0.1, 0.1).Shape();

  BRepFilletAPI_MakeFillet fillet(box);
  TopTools_IndexedMapOfShape edges;
  TopExp::MapShapes(box, TopAbs_EDGE, edges);
  for (int i = 1; i <= edges.Extent(); ++i) {
    fillet.Add(0.01, TopoDS::Edge(edges(i)));
  }
  const TopoDS_Shape rounded = fillet.Shape();

  const auto faces = faces_of(rounded);
  ASSERT_GT(faces.size(), 20u) << "expected planes plus edge and corner blends";

  GProp_GProps props;
  BRepGProp::SurfaceProperties(rounded, props);
  const double reference = props.Mass();

  double measured = 0.0;
  for (const auto & face : faces) {
    measured += sampled_area(sample_face_region(face, area_config(0.001)));
  }

  EXPECT_NEAR(measured, reference, reference * 0.02)
    << "reference " << reference << " measured " << measured
    << " over " << faces.size() << " faces";
}

// Three holes, so the face carries three inner wires at once rather than one.
TEST(FaceSamplerTest, MultipleInnerHolesAreAllExcluded)
{
  constexpr double kHoleRadius = 0.02;
  TopoDS_Shape plate = BRepPrimAPI_MakeBox(0.2, 0.2, 0.05).Shape();

  const gp_Pnt centres[3] = {
    gp_Pnt(0.05, 0.05, -0.01),
    gp_Pnt(0.15, 0.05, -0.01),
    gp_Pnt(0.10, 0.15, -0.01)
  };
  for (const auto & c : centres) {
    plate = BRepAlgoAPI_Cut(
      plate,
      BRepPrimAPI_MakeCylinder(gp_Ax2(c, gp_Dir(0, 0, 1)), kHoleRadius, 0.07).Shape()).Shape();
  }

  const TopoDS_Face face = top_planar_face(plate);
  ASSERT_FALSE(face.IsNull());

  const double reference = occt_face_area(face);
  const double solid = 0.2 * 0.2;
  const double holes = 3.0 * M_PI * kHoleRadius * kHoleRadius;
  ASSERT_NEAR(reference, solid - holes, solid * 0.01) << "cut did not make three holes";

  const double measured = sampled_area(sample_face_region(face, area_config(0.002)));
  EXPECT_NEAR(measured, reference, reference * 0.02);
  EXPECT_LT(measured, solid - holes * 0.8) << "at least one hole was not excluded";
}

namespace
{

TopoDS_Face l_notched_plate()
{
  // Notch corner at non-round (0.127, 0.113), so any alignment between a notch
  // edge and a cell edge is incidental rather than built into the shape.
  return BRepBuilderAPI_MakeFace(polygon_wire({
      gp_Pnt(0.0, 0.0, 0.0), gp_Pnt(0.2, 0.0, 0.0), gp_Pnt(0.2, 0.113, 0.0),
      gp_Pnt(0.127, 0.113, 0.0), gp_Pnt(0.127, 0.2, 0.0), gp_Pnt(0.0, 0.2, 0.0)})).Face();
}

TopoDS_Face skewed_triangle()
{
  return BRepBuilderAPI_MakeFace(polygon_wire({
      gp_Pnt(0.0, 0.0, 0.0), gp_Pnt(0.2, 0.0, 0.0), gp_Pnt(0.07, 0.15, 0.0)})).Face();
}

TopoDS_Face plate_with_hole()
{
  const TopoDS_Shape box = BRepPrimAPI_MakeBox(0.2, 0.2, 0.1).Shape();
  const TopoDS_Shape drill =
    BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(0.1, 0.1, -0.01), gp_Dir(0, 0, 1)),
      0.03, 0.12).Shape();
  return top_planar_face(BRepAlgoAPI_Cut(box, drill).Shape());
}

TopoDS_Face cross_drilled_cylinder()
{
  const TopoDS_Shape tube = BRepPrimAPI_MakeCylinder(0.08, 0.2).Shape();
  const TopoDS_Shape drill = BRepPrimAPI_MakeCylinder(
    gp_Ax2(gp_Pnt(0.0, 0.0, 0.087), gp_Dir(1, 0, 0)), 0.025, 0.12).Shape();
  const TopoDS_Shape cut = BRepAlgoAPI_Cut(tube, drill).Shape();

  // The drill's own wall is cylindrical too, so pick the tube by area.
  TopoDS_Face best;
  double best_area = 0.0;
  for (const auto & face : faces_of(cut)) {
    if (Handle(Geom_Plane) ::DownCast(BRep_Tool::Surface(face)).IsNull()) {
      const double area = occt_face_area(face);
      if (area > best_area) {
        best_area = area;
        best = face;
      }
    }
  }
  return best;
}

double perimeter_of(const TopoDS_Face & face)
{
  GProp_GProps props;
  BRepGProp::LinearProperties(face, props);
  return props.Mass();
}

}  // namespace

TEST(FaceSamplerTest, TrimBoundaryErrorStaysWithinFirstOrderBound)
{
  struct TrimCase
  {
    const char * name;
    TopoDS_Face face;
  };
  const TrimCase cases[] = {
    {"L-notch", l_notched_plate()},
    {"skewed triangle", skewed_triangle()},
    {"circular hole", plate_with_hole()},
    {"cross-drilled cylinder", cross_drilled_cylinder()},
  };

  // Roughly log-spaced, deliberately not round numbers, and including 0.005:
  // the default for both area-fraction consumers (GroundConstraint and
  // KissingSurfaceConstraint), which is the density the decision is about.
  const std::vector<double> densities = {
    0.008, 0.0065, 0.005, 0.0041, 0.0033, 0.0027, 0.0021, 0.0017, 0.0013, 0.001};

  for (const auto & c : cases) {
    ASSERT_FALSE(c.face.IsNull()) << c.name;

    // Adaptive integration to 1e-9: the reference must be far tighter than the
    // smallest error measured, or the fitted order is the oracle's, not ours.
    GProp_GProps props;
    BRepGProp::SurfaceProperties(c.face, props, 1e-9);
    const double reference = props.Mass();
    const double perimeter = perimeter_of(c.face);
    ASSERT_GT(reference, 0.0) << c.name;

    double sum_x = 0.0, sum_y = 0.0, sum_xx = 0.0, sum_xy = 0.0;
    int fitted = 0;
    double worst_fill = 0.0;

    for (const double h : densities) {
      const double measured = sampled_area(sample_face_region(c.face, area_config(h)));
      const double error = std::abs(measured - reference);
      const double bound = h * perimeter / std::sqrt(2.0);

      GTEST_LOG_(INFO) << c.name << ": density " << h
                       << " m -> relative error " << error / reference
                       << " (signed " << (measured - reference) / reference
                       << "), " << error / bound << " of bound";

      EXPECT_LE(error, bound)
        << c.name << " at density " << h << ": |dA| " << error
        << " exceeds the documented h*P/sqrt(2) = " << bound;

      worst_fill = std::max(worst_fill, error / bound);
      // Densities where the boundary cells cancel exactly (the triangle does
      // this at several) measure round-off, not trim error, and would drag the
      // fit anywhere. Leave them out of it.
      if (error > 1e-9 * reference) {
        sum_x += std::log(h);
        sum_y += std::log(error);
        sum_xx += std::log(h) * std::log(h);
        sum_xy += std::log(h) * std::log(error);
        ++fitted;
      }
    }

    // Least-squares slope of log(error) against log(h). First order is 1; the
    // oscillation of individual points is why this is a fit and not a ratio of
    // two densities. Logged, not asserted: this is the measurement item 4's
    // decision is recorded against.
    const double order = fitted > 1 ?
      (fitted * sum_xy - sum_x * sum_y) / (fitted * sum_xx - sum_x * sum_x) : 0.0;
    GTEST_LOG_(INFO) << c.name << ": perimeter " << perimeter << " m, area "
                     << reference << " m^2, fitted order " << order
                     << ", worst error " << worst_fill << " of bound";
  }
}

// sample_face_region reverses the normal on TopAbs_REVERSED faces, so an
// inverted branch would flip the sign without changing magnitude or direction
// spread. On a convex solid every outward normal has a positive dot with the
// vector from the centroid to the sample.
TEST(FaceSamplerTest, NormalsPointOutwardOnEveryBoxFace)
{
  const TopoDS_Shape box = BRepPrimAPI_MakeBox(0.1, 0.2, 0.3).Shape();
  const gp_Pnt centre(0.05, 0.1, 0.15);

  const auto faces = faces_of(box);
  ASSERT_EQ(faces.size(), 6u);

  for (size_t i = 0; i < faces.size(); ++i) {
    const auto samples = sample_face_region(faces[i], area_config(0.01));
    ASSERT_FALSE(samples.empty()) << "face " << i;

    for (const auto & s : samples) {
      ASSERT_TRUE(s.has_normal);
      EXPECT_GT(s.normal.Dot(gp_Vec(centre, s.point)), 0.0)
        << "face " << i << " normal points into the solid";
    }
  }
}

// Same check on a curved face, where the normal comes from the cross product
// rather than from a plane's own axis. Outward at every sample also means the
// normals are per sample, not one average for the whole face.
TEST(FaceSamplerTest, NormalsPointOutwardOnCylinderAndSphere)
{
  const TopoDS_Face lateral =
    first_curved_face(BRepPrimAPI_MakeCylinder(0.15, 0.25).Shape());
  ASSERT_FALSE(lateral.IsNull());

  const auto cylinder_samples = sample_face_region(lateral, area_config(0.01));
  ASSERT_FALSE(cylinder_samples.empty());
  for (const auto & s : cylinder_samples) {
    ASSERT_TRUE(s.has_normal);
    EXPECT_NEAR(s.normal.Magnitude(), 1.0, 1e-9);
    // Axis is Z, so outward is the radial direction from the axis.
    EXPECT_NEAR(s.normal.Z(), 0.0, 1e-6);
    const gp_Vec radial(s.point.X(), s.point.Y(), 0.0);
    EXPECT_GT(s.normal.Dot(radial), 0.0) << "cylinder normal points inward";
  }

  const auto sphere_faces = faces_of(BRepPrimAPI_MakeSphere(0.1).Shape());
  ASSERT_EQ(sphere_faces.size(), 1u);
  for (const auto & s : sample_face_region(sphere_faces[0], area_config(0.01))) {
    if (!s.has_normal) {continue;}
    EXPECT_GT(s.normal.Dot(gp_Vec(gp_Pnt(0, 0, 0), s.point)), 0.0)
      << "sphere normal points inward";
  }
}

// The wires_with_flags path is how every constraint and exclusion zone reaches
// the sampler. Exclusion wires are covered by BoundingWireRoundTripsAsAnExclusion.
TEST(FaceSamplerTest, InclusionWireKeepsOnlyItsInterior)
{
  const TopoDS_Shape box = BRepPrimAPI_MakeBox(0.2, 0.2, 0.1).Shape();
  const TopoDS_Face face = top_planar_face(box);
  ASSERT_FALSE(face.IsNull());

  const TopoDS_Wire square = polygon_wire({
    gp_Pnt(0.05, 0.05, 0.1), gp_Pnt(0.15, 0.05, 0.1),
    gp_Pnt(0.15, 0.15, 0.1), gp_Pnt(0.05, 0.15, 0.1)});

  const std::vector<std::pair<TopoDS_Wire, bool>> include{{square, false}};
  const auto samples = sample_face_region(face, area_config(0.002), include);

  EXPECT_NEAR(sampled_area(samples), 0.01, 0.01 * 0.02);
  for (const auto & s : samples) {
    EXPECT_GE(s.point.X(), 0.05 - 1e-6);
    EXPECT_LE(s.point.X(), 0.15 + 1e-6);
  }
}

// The documented consumer pattern end to end: measure a region, grow it back
// into a wire, hand that wire back as an exclusion. It is the only path that
// writes into the SampleArea the samplers will read.
TEST(FaceSamplerTest, BoundingWireRoundTripsAsAnExclusion)
{
  const TopoDS_Shape box = BRepPrimAPI_MakeBox(0.2, 0.2, 0.1).Shape();
  const TopoDS_Face face = top_planar_face(box);
  ASSERT_FALSE(face.IsNull());

  FaceSamplingGrid grid;
  const auto samples = sample_face_region(face, area_config(0.002), {}, &grid);
  ASSERT_FALSE(samples.empty());
  ASSERT_GT(grid.u_steps, 0);

  // Half the face, split on a grid line so the wire lands on cell boundaries.
  std::vector<FaceSample> half;
  for (const auto & s : samples) {
    if (s.point.X() < 0.1) {half.push_back(s);}
  }
  ASSERT_FALSE(half.empty());

  const TopoDS_Wire wire = bounding_wire_in_uv(face, half);
  ASSERT_FALSE(wire.IsNull()) << "bounding_wire_in_uv produced nothing";

  const std::vector<std::pair<TopoDS_Wire, bool>> exclude{{wire, true}};
  const double remaining =
    sampled_area(sample_face_region(face, area_config(0.002), exclude));

  EXPECT_NEAR(remaining, 0.02, 0.04 * 0.03)
    << "excluding the measured half left " << remaining << " m^2";
}

// The wire is built from pcurves so it follows the surface. On a cylinder a
// 3D-line wire between the same corners would cut through the solid, so this is
// where that choice actually matters.
TEST(FaceSamplerTest, BoundingWireIsBuiltOnCurvedSurface)
{
  const TopoDS_Face lateral =
    first_curved_face(BRepPrimAPI_MakeCylinder(0.15, 0.25).Shape());
  ASSERT_FALSE(lateral.IsNull());

  const auto samples = sample_face_region(lateral, area_config(0.005));
  ASSERT_FALSE(samples.empty());

  std::vector<FaceSample> band;
  for (const auto & s : samples) {
    if (s.point.Z() < 0.1) {band.push_back(s);}
  }
  ASSERT_FALSE(band.empty());

  const TopoDS_Wire wire = bounding_wire_in_uv(lateral, band);
  ASSERT_FALSE(wire.IsNull()) << "no wire on a cylindrical face";

  // Every vertex of the wire must sit on the cylinder, not on a chord.
  TopTools_IndexedMapOfShape verts;
  TopExp::MapShapes(wire, TopAbs_VERTEX, verts);
  ASSERT_GT(verts.Extent(), 0);
  for (int i = 1; i <= verts.Extent(); ++i) {
    const gp_Pnt p = BRep_Tool::Pnt(TopoDS::Vertex(verts(i)));
    EXPECT_NEAR(std::hypot(p.X(), p.Y()), 0.15, 1e-6)
      << "wire vertex left the cylinder";
  }
}

// STEP assemblies and the loader's transform leave faces located, and
// BRep_Tool::Surface hands back a fresh transformed copy per call. The wire and
// the face it is classified against must still agree on which surface the
// pcurves live on, or the exclusion silently removes nothing.
TEST(FaceSamplerTest, BoundingWireRoundTripsOnLocatedCurvedFace)
{
  gp_Trsf rotation;
  rotation.SetRotation(gp_Ax1(gp_Pnt(0.0, 0.0, 0.0), gp_Dir(0.0, 0.0, 1.0)), 0.3);
  const TopoDS_Shape cylinder =
    BRepPrimAPI_MakeCylinder(0.15, 0.25).Shape().Moved(TopLoc_Location(rotation));
  const TopoDS_Face lateral = first_curved_face(cylinder);
  ASSERT_FALSE(lateral.IsNull());
  ASSERT_FALSE(lateral.Location().IsIdentity());

  const auto samples = sample_face_region(lateral, area_config(0.005));
  ASSERT_FALSE(samples.empty());

  std::vector<FaceSample> band;
  for (const auto & s : samples) {
    if (s.point.Z() < 0.1) {band.push_back(s);}
  }
  ASSERT_FALSE(band.empty());

  const TopoDS_Wire wire = bounding_wire_in_uv(lateral, band);
  ASSERT_FALSE(wire.IsNull());

  // The wire is handed on as 3D geometry too, so it has to sit where the
  // located face is: on the cylinder, within half a cell of the band.
  TopTools_IndexedMapOfShape verts;
  TopExp::MapShapes(wire, TopAbs_VERTEX, verts);
  ASSERT_GT(verts.Extent(), 0);
  for (int i = 1; i <= verts.Extent(); ++i) {
    const gp_Pnt p = BRep_Tool::Pnt(TopoDS::Vertex(verts(i)));
    EXPECT_NEAR(std::hypot(p.X(), p.Y()), 0.15, 1e-6) << "wire vertex left the cylinder";
    EXPECT_LE(p.Z(), 0.1 + 0.005) << "wire vertex left the band";
  }

  const double full = sampled_area(samples);
  const double band_area = sampled_area(band);

  const std::vector<std::pair<TopoDS_Wire, bool>> exclude{{wire, true}};
  const double remaining =
    sampled_area(sample_face_region(lateral, area_config(0.005), exclude));
  EXPECT_NEAR(remaining, full - band_area, full * 0.01)
    << "excluding the band left " << remaining << " m^2 of " << full;

  const std::vector<std::pair<TopoDS_Wire, bool>> include{{wire, false}};
  const double kept =
    sampled_area(sample_face_region(lateral, area_config(0.005), include));
  EXPECT_NEAR(kept, band_area, full * 0.01)
    << "including only the band kept " << kept << " m^2";
}

// Padding has to follow the cells the edge samples actually came from. Growing
// by the face-wide widest cell lets a region at the fine end of a clustered-knot
// span swallow the stretched cells next to it.
TEST(FaceSamplerTest, BoundingWireHugsRegionOnClusteredKnotStrip)
{
  const TopoDS_Face face = knotted_nurbs_strip(0.004);
  ASSERT_FALSE(face.IsNull());

  constexpr double kDensity = 0.005;
  const auto samples = sample_face_region(face, area_config(kDensity));
  ASSERT_FALSE(samples.empty());

  for (const double cut : {0.02, 0.05, 0.1}) {
    std::vector<FaceSample> left;
    for (const auto & s : samples) {
      if (s.point.X() < cut) {left.push_back(s);}
    }
    ASSERT_FALSE(left.empty());

    const TopoDS_Wire wire = bounding_wire_in_uv(face, left);
    ASSERT_FALSE(wire.IsNull());

    // Extent rather than area, so an overshoot cannot hide behind a
    // compensating undershoot elsewhere.
    TopTools_IndexedMapOfShape verts;
    TopExp::MapShapes(wire, TopAbs_VERTEX, verts);
    double wire_x_max = -std::numeric_limits<double>::max();
    for (int i = 1; i <= verts.Extent(); ++i) {
      wire_x_max = std::max(wire_x_max, BRep_Tool::Pnt(TopoDS::Vertex(verts(i))).X());
    }

    // The outermost sample is under the cut, and its cell reaches at most
    // half a cell -- about kDensity / 2 -- past it.
    EXPECT_LT(wire_x_max, cut + kDensity)
      << "region x < " << cut << " grew to x = " << wire_x_max
      << ", overshoot " << wire_x_max - cut << " m at density " << kDensity << " m";
  }
}

TEST(FaceSamplerTest, BoundingWireIsNullForEmptySamples)
{
  const TopoDS_Shape box = BRepPrimAPI_MakeBox(0.1, 0.1, 0.1).Shape();
  const TopoDS_Face face = top_planar_face(box);
  ASSERT_FALSE(face.IsNull());

  EXPECT_TRUE(bounding_wire_in_uv(face, {}).IsNull());
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
