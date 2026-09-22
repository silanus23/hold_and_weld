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

#include <stdexcept>
#include <string>

#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakePolygon.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <gp_Pnt.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Shape.hxx>

#include "hold_and_weld_gripper_sampler/geometry/occt_utils.hpp"

using hold_and_weld_gripper_sampler::geometry::validate_shape_or_throw;

namespace
{

// Bowtie / figure-eight wire: MakeFace builds it fine (only checks closure,
// not self-intersection) but BRepCheck_Analyzer flags it. Tested as a shape,
// not a round-tripped STEP file, because OCCT's STEP reader silently heals
// this into two valid faces on import.
TopoDS_Face make_self_intersecting_face()
{
  BRepBuilderAPI_MakePolygon bowtie;
  bowtie.Add(gp_Pnt(0, 0, 0));
  bowtie.Add(gp_Pnt(10, 10, 0));
  bowtie.Add(gp_Pnt(10, 0, 0));
  bowtie.Add(gp_Pnt(0, 10, 0));
  bowtie.Close();

  return BRepBuilderAPI_MakeFace(bowtie.Wire()).Face();
}

}  // namespace

TEST(OcctUtilsTest, ValidateShapeOrThrow_WithValidShape_DoesNotThrow)
{
  const TopoDS_Shape box = BRepPrimAPI_MakeBox(0.1, 0.1, 0.1).Shape();
  EXPECT_NO_THROW(validate_shape_or_throw(box, "test box"));
}

// Exercised against a broken shape directly, not a STEP file, since a broken
// file gets healed before this check ever sees it (see
// make_self_intersecting_face()).
TEST(OcctUtilsTest, ValidateShapeOrThrow_WithInvalidShape_ThrowsWithContext)
{
  const TopoDS_Face face = make_self_intersecting_face();
  ASSERT_FALSE(face.IsNull());
  ASSERT_FALSE(BRepCheck_Analyzer(face).IsValid())
    << "premise: the constructed face must actually be invalid";

  try {
    validate_shape_or_throw(face, "unit test shape");
    FAIL() << "expected validate_shape_or_throw to reject an invalid shape";
  } catch (const std::runtime_error & e) {
    const std::string message = e.what();
    EXPECT_NE(message.find("unit test shape"), std::string::npos)
      << "error should name the context it was given: " << message;
    EXPECT_NE(message.find("invalid"), std::string::npos)
      << "error should say what kind of problem this is: " << message;
  }
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
