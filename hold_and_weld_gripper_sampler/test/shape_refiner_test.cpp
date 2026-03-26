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
#include "hold_and_weld_gripper_sampler/geometry/shape_refiner.hpp"

#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRepPrimAPI_MakeCone.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepAlgoAPI_Cut.hxx>
#include <BRepAlgoAPI_Fuse.hxx>
#include <TopoDS.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <gp_Pnt.hxx>
#include <gp_Dir.hxx>
#include <gp_Ax2.hxx>
#include <gp_Elips.hxx>
#include <Geom_BSplineSurface.hxx>
#include <GeomAPI_PointsToBSplineSurface.hxx>
#include <TColgp_Array2OfPnt.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>

using namespace hold_and_weld_gripper_sampler::geometry;  // NOLINT

namespace test_constants
{
// Default ShapeRefiner configuration
constexpr double kMaxCylinderRadius = 0.100;
constexpr double kMaxArcLength = 0.200;
constexpr double kEnclaveAreaRatio = 0.005;
constexpr double kEnclaveAngleThreshold = 45.0;

// Expected face counts for common primitives
constexpr int kBoxFaceCount = 6;
constexpr int kCylinderFaceCount = 3;
}  // namespace test_constants

int count_faces(const TopoDS_Shape & shape)
{
  int count = 0;
  for (TopExp_Explorer exp(shape, TopAbs_FACE); exp.More(); exp.Next()) {
    count++;
  }
  return count;
}

int count_cylindrical_faces(const TopoDS_Shape & shape)
{
  int count = 0;
  for (TopExp_Explorer exp(shape, TopAbs_FACE); exp.More(); exp.Next()) {
    const TopoDS_Face & face = TopoDS::Face(exp.Current());
    BRepAdaptor_Surface adaptor(face);
    if (adaptor.GetType() == GeomAbs_Cylinder) {
      count++;
    }
  }
  return count;
}

int count_planar_faces(const TopoDS_Shape & shape)
{
  int count = 0;
  for (TopExp_Explorer exp(shape, TopAbs_FACE); exp.More(); exp.Next()) {
    const TopoDS_Face & face = TopoDS::Face(exp.Current());
    BRepAdaptor_Surface adaptor(face);
    if (adaptor.GetType() == GeomAbs_Plane) {
      count++;
    }
  }
  return count;
}

TopoDS_Shape create_plate_with_hole(double plate_size, double hole_radius)
{
  TopoDS_Shape plate = BRepPrimAPI_MakeBox(plate_size, plate_size, 0.01).Shape();
  gp_Ax2 axis(gp_Pnt(plate_size / 2, plate_size / 2, -0.01), gp_Dir(0, 0, 1));
  TopoDS_Shape hole = BRepPrimAPI_MakeCylinder(axis, hole_radius, 0.03).Shape();
  BRepAlgoAPI_Cut cutter(plate, hole);
  return cutter.Shape();
}

TopoDS_Shape create_plate_with_pocket(double plate_size, double pocket_radius, double pocket_depth)
{
  TopoDS_Shape plate = BRepPrimAPI_MakeBox(plate_size, plate_size, 0.05).Shape();
  gp_Ax2 axis(gp_Pnt(plate_size / 2, plate_size / 2, 0.05 - pocket_depth), gp_Dir(0, 0, 1));
  TopoDS_Shape pocket = BRepPrimAPI_MakeCylinder(axis, pocket_radius, pocket_depth + 0.01).Shape();
  BRepAlgoAPI_Cut cutter(plate, pocket);
  return cutter.Shape();
}


TopoDS_Shape create_saddle_surface(double size)
{
  int u_count = 4, v_count = 4;
  TColgp_Array2OfPnt points(1, u_count, 1, v_count);

  for (int i = 1; i <= u_count; i++) {
    for (int j = 1; j <= v_count; j++) {
      double u = (i - 1) / 3.0 * size - size / 2;
      double v = (j - 1) / 3.0 * size - size / 2;
      double z = (u * u - v * v) / (size * size) * size / 4;
      points(i, j) = gp_Pnt(u, v, z);
    }
  }

  GeomAPI_PointsToBSplineSurface builder(points);
  Handle(Geom_BSplineSurface) surface = builder.Surface();
  TopoDS_Face face = BRepBuilderAPI_MakeFace(surface, 1e-6).Face();
  return face;
}


TopoDS_Shape create_complex_part()
{
  TopoDS_Shape base = BRepPrimAPI_MakeBox(0.2, 0.2, 0.05).Shape();
  gp_Ax2 cyl_axis(gp_Pnt(0.1, 0.1, 0.05), gp_Dir(0, 0, 1));
  TopoDS_Shape cylinder = BRepPrimAPI_MakeCylinder(cyl_axis, 0.08, 0.1).Shape();
  BRepAlgoAPI_Fuse fuser1(base, cylinder);
  TopoDS_Shape part = fuser1.Shape();

  gp_Ax2 hole1_axis(gp_Pnt(0.02, 0.02, -0.01), gp_Dir(0, 0, 1));
  TopoDS_Shape hole1 = BRepPrimAPI_MakeCylinder(hole1_axis, 0.003, 0.07).Shape();
  BRepAlgoAPI_Cut cutter1(part, hole1);
  part = cutter1.Shape();

  gp_Ax2 hole2_axis(gp_Pnt(0.18, 0.02, -0.01), gp_Dir(0, 0, 1));
  TopoDS_Shape hole2 = BRepPrimAPI_MakeCylinder(hole2_axis, 0.003, 0.07).Shape();
  BRepAlgoAPI_Cut cutter2(part, hole2);
  part = cutter2.Shape();

  return part;
}

class ShapeRefinerTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    refiner_ = std::make_unique<ShapeRefiner>(
      test_constants::kMaxCylinderRadius,
      test_constants::kMaxArcLength,
      test_constants::kEnclaveAreaRatio,
      test_constants::kEnclaveAngleThreshold);
  }

  std::unique_ptr<ShapeRefiner> refiner_;
};

TEST_F(ShapeRefinerTest, Refine_WithSimpleThroughHole_PreservesMostFaces)
{
  TopoDS_Shape input = create_plate_with_hole(0.1, 0.025);
  int input_faces = count_faces(input);

  TopoDS_Shape output = refiner_->refine(input);
  int output_faces = count_faces(output);

  EXPECT_GE(output_faces, input_faces - 2);
}

TEST_F(ShapeRefinerTest, Refine_WithDeepPocketSteepWalls_PreservesFaces)
{
  ShapeRefiner strict_refiner(0.1, 0.2, 0.01, 30.0);
  TopoDS_Shape input = create_plate_with_pocket(0.1, 0.005, 0.02);
  int input_faces = count_faces(input);

  TopoDS_Shape output = strict_refiner.refine(input);
  int output_faces = count_faces(output);

  EXPECT_GE(output_faces, input_faces - 2);
}

TEST_F(ShapeRefinerTest, Refine_WithComplexPocketFillets_ReducesOrMaintainsFaces)
{
  TopoDS_Shape base = create_plate_with_pocket(0.1, 0.01, 0.003);
  int input_faces = count_faces(base);
  EXPECT_GT(input_faces, test_constants::kBoxFaceCount);

  TopoDS_Shape output = refiner_->refine(base);
  int output_faces = count_faces(output);

  EXPECT_LE(output_faces, input_faces);
}

TEST_F(ShapeRefinerTest, Refine_WithLargeRadiusCylinder_SplitsIntoMoreFaces)
{
  TopoDS_Shape input = BRepPrimAPI_MakeCylinder(0.150, 0.1).Shape();
  int input_cyl_faces = count_cylindrical_faces(input);
  EXPECT_EQ(input_cyl_faces, 1);

  TopoDS_Shape output = refiner_->refine(input);
  int output_faces = count_faces(output);

  EXPECT_GT(output_faces, test_constants::kCylinderFaceCount);
}

TEST_F(ShapeRefinerTest, Refine_WithSmallCylinderLargeArc_SplitsDueToArcLength)
{
  // Cylinder with radius 50mm < threshold 100mm
  // BUT circumference = 2pi×50 ≈ 314mm > 200mm arc length limit
  // So it WILL split due to arc length check
  TopoDS_Shape input = BRepPrimAPI_MakeCylinder(0.05, 0.1).Shape();
  int input_faces = count_faces(input);
  EXPECT_EQ(input_faces, test_constants::kCylinderFaceCount);

  TopoDS_Shape output = refiner_->refine(input);
  int output_faces = count_faces(output);

  EXPECT_GT(output_faces, input_faces) << "314mm circumference exceeds 200mm limit";
}

TEST_F(ShapeRefinerTest, Refine_WithPartialCylinder180Degrees_SplitsFaces)
{
  double angle_rad = 180.0 * M_PI / 180.0;
  TopoDS_Shape input = BRepPrimAPI_MakeCylinder(0.08, 0.1, angle_rad).Shape();
  int input_faces = count_faces(input);

  TopoDS_Shape output = refiner_->refine(input);
  int output_faces = count_faces(output);

  EXPECT_GT(output_faces, input_faces);
}

TEST_F(ShapeRefinerTest, Refine_WithTubeGeometry_SplitsBothCylinders)
{
  TopoDS_Shape outer = BRepPrimAPI_MakeCylinder(0.12, 0.1).Shape();
  TopoDS_Shape inner = BRepPrimAPI_MakeCylinder(0.08, 0.1).Shape();
  BRepAlgoAPI_Cut cutter(outer, inner);
  TopoDS_Shape input = cutter.Shape();
  int input_faces = count_faces(input);

  TopoDS_Shape output = refiner_->refine(input);
  int output_faces = count_faces(output);

  EXPECT_GT(output_faces, input_faces);
}

TEST_F(ShapeRefinerTest, Refine_WithConeGeometry_MaintainsOrIncreasesFaces)
{
  TopoDS_Shape input = BRepPrimAPI_MakeCone(0.08, 0.04, 0.15).Shape();
  int input_faces = count_faces(input);

  TopoDS_Shape output = refiner_->refine(input);
  int output_faces = count_faces(output);

  EXPECT_GE(output_faces, input_faces);
}

TEST_F(ShapeRefinerTest, Refine_WithSaddleSurface_HandlesNegativeGaussianCurvature)
{
  TopoDS_Shape input = create_saddle_surface(0.1);

  EXPECT_NO_THROW({
    TopoDS_Shape output = refiner_->refine(input);
    count_faces(output);
  });
}

TEST_F(ShapeRefinerTest, Refine_WithFlatPlate_NoRefinementNeeded)
{
  TopoDS_Shape input = BRepPrimAPI_MakeBox(0.1, 0.08, 0.005).Shape();
  int input_faces = count_faces(input);
  EXPECT_EQ(input_faces, test_constants::kBoxFaceCount);

  TopoDS_Shape output = refiner_->refine(input);
  int output_faces = count_faces(output);

  EXPECT_LE(output_faces, input_faces + 1);
}

TEST_F(ShapeRefinerTest, Refine_WithBoxAllPlanar_PreservesAllPlanarFaces)
{
  TopoDS_Shape input = BRepPrimAPI_MakeBox(0.05, 0.06, 0.07).Shape();
  int input_planar = count_planar_faces(input);
  EXPECT_EQ(input_planar, test_constants::kBoxFaceCount);

  TopoDS_Shape output = refiner_->refine(input);
  int output_planar = count_planar_faces(output);

  EXPECT_GE(output_planar, test_constants::kBoxFaceCount);
}

TEST_F(ShapeRefinerTest, Refine_WithNearlyPlanarLargeCylinder_SplitsIntoManyPatches)
{
  // Very large radius cylinder (2m radius, appears flat locally)
  // Circumference = 2π×2000mm = 12,566mm >> 200mm limit
  TopoDS_Shape input = BRepPrimAPI_MakeCylinder(2.0, 0.05).Shape();
  TopoDS_Shape output = refiner_->refine(input);
  int output_faces = count_faces(output);

  EXPECT_GT(output_faces, 10) << "12.5m circumference should create many splits";
}

TEST_F(ShapeRefinerTest, Refine_WithDegenerateGeometry_HandlesGracefully)
{
  TopoDS_Shape input = BRepPrimAPI_MakeBox(0.1, 0.1, 0.0001).Shape();

  EXPECT_NO_THROW({
    TopoDS_Shape output = refiner_->refine(input);
    count_faces(output);
  });
}

TEST_F(ShapeRefinerTest, Refine_WithComplexPart_ProducesReasonableFaceCount)
{
  TopoDS_Shape input = create_complex_part();
  int input_faces = count_faces(input);

  TopoDS_Shape output = refiner_->refine(input);
  int output_faces = count_faces(output);

  EXPECT_GT(output_faces, 0);
  EXPECT_LT(output_faces, input_faces * 3);
}

TEST_F(ShapeRefinerTest, Refine_WithSmallRadiusLongArc_MustSplit)
{
  TopoDS_Shape input = BRepPrimAPI_MakeCylinder(0.05, 0.5).Shape();
  int input_faces = count_faces(input);
  EXPECT_EQ(input_faces, test_constants::kCylinderFaceCount);

  TopoDS_Shape output = refiner_->refine(input);
  int output_faces = count_faces(output);

  EXPECT_GT(output_faces, input_faces) << "Cylinder with 314mm arc should split (limit 200mm)";
}

TEST_F(ShapeRefinerTest, Refine_WithPartialCylinderArcLength_SplitsCorrectly)
{
  double angle_rad = 180.0 * M_PI / 180.0;
  TopoDS_Shape input = BRepPrimAPI_MakeCylinder(0.08, 0.1, angle_rad).Shape();
  int input_cyl_faces = count_cylindrical_faces(input);

  TopoDS_Shape output = refiner_->refine(input);
  int output_faces = count_faces(output);

  EXPECT_GT(output_faces, input_cyl_faces + 2) << "251mm arc must split";
}

TEST_F(ShapeRefinerTest, Refine_WithConeDifferentRadii_SplitsBothEdges)
{
  TopoDS_Shape input = BRepPrimAPI_MakeCone(0.10, 0.05, 0.20).Shape();
  int input_faces = count_faces(input);

  TopoDS_Shape output = refiner_->refine(input);
  int output_faces = count_faces(output);

  EXPECT_GT(output_faces, input_faces) << "Cone with 628mm and 314mm edges must split";
}

TEST_F(ShapeRefinerTest, Refine_WithMediumCylinder_SplitsMultipleTimes)
{
  TopoDS_Shape input = BRepPrimAPI_MakeCylinder(0.08, 0.1).Shape();
  int input_faces = count_faces(input);
  EXPECT_EQ(input_faces, test_constants::kCylinderFaceCount);

  TopoDS_Shape output = refiner_->refine(input);
  int output_faces = count_faces(output);

  EXPECT_GE(output_faces, 5) << "502mm arc should split into 3 patches (3+2 caps)";
}

TEST_F(ShapeRefinerTest, Refine_WithDefeaturingFailureCase_RecoversGracefully)
{
  TopoDS_Shape input = BRepPrimAPI_MakeBox(0.1, 0.1, 0.01).Shape();

  EXPECT_NO_THROW({
    TopoDS_Shape output = refiner_->refine(input);
    EXPECT_GT(count_faces(output), 0);
  });
}

TEST_F(ShapeRefinerTest, Refine_WithLargeUnsplitSurface_TriggersOrSplits)
{
  testing::internal::CaptureStderr();

  TopoDS_Shape input = BRepPrimAPI_MakeCylinder(2.0, 0.05).Shape();
  TopoDS_Shape output = refiner_->refine(input);

  std::string error_output = testing::internal::GetCapturedStderr();

  // Should have split the large cylinder
  EXPECT_GT(count_faces(output), test_constants::kCylinderFaceCount);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
